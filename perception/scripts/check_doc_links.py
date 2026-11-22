#!/usr/bin/env python3
"""Check local Markdown links and repository-root paths in project docs.

By default this checks local references in ``docs/learning`` and ``ops``.
HTTP checks are optional because remote servers can rate-limit or block probes.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
import ipaddress
from pathlib import Path
import re
import subprocess
import sys
from urllib.error import HTTPError, URLError
from urllib.parse import unquote, urlsplit
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIRS = ("docs/learning", "ops")
REPO_PREFIXES = ("docs/", "ops/", "sim/", "scripts/", "tests/", "ros2/",
                 "experiments/", "research/", "calibration/", "lab/", "tools/",
                 "patches/", "queue/", "data/", "docker/", "third_party/")
LINK_RE = re.compile(r"!?(?:\[[^\]]*\])\(\s*(?:<([^>]+)>|([^\s)]+))(?:\s+[^)]*)?\)")
REF_DEF_RE = re.compile(r"^[ \t]*\[([^\]]+)\]:[ \t]*(?:<([^>]+)>|(\S+))", re.MULTILINE)
REF_USE_RE = re.compile(r"!?(?:\[[^\]]*\])\[([^\]]*)\]")
INLINE_CODE_RE = re.compile(r"(`+).*?\1", re.DOTALL)
FENCE_RE = re.compile(r"^\s*(```+|~~~+).*?^\s*\1\s*$", re.MULTILINE | re.DOTALL)
ROOT_PATH_RE = re.compile(
    r"(?<![\w./~:-])((?:" + "|".join(re.escape(p) for p in REPO_PREFIXES)
    + r")[A-Za-z0-9_./{}<>+*?-]+)"
)
URL_RE = re.compile(r"https?://[^\s<>\])]+", re.IGNORECASE)
URL_TRAILING_PUNCTUATION = ".,;:!?\"'`"
HOST_LABEL_RE = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?$")


@dataclass(frozen=True)
class Finding:
    file: Path
    line: int
    target: str
    kind: str
    status: str


def markdown_files(directories: tuple[str, ...]) -> list[Path]:
    files: list[Path] = []
    for directory in directories:
        base = ROOT / directory
        if base.exists():
            files.extend(p for p in base.rglob("*.md") if p.is_file())
    return sorted(set(files))


def _clean_destination(value: str) -> str:
    value = value.strip().strip("\"'")
    # Markdown permits a title after whitespace; regex already excludes it.
    return unquote(value)


def _resolve_local(source: Path, target: str) -> Path | None:
    target = _clean_destination(target)
    if not target or target.startswith("#"):
        return None
    parsed = urlsplit(target)
    if parsed.scheme or parsed.netloc or target.startswith("/"):
        return None
    path_part = parsed.path
    if not path_part:
        return None
    # Markdown destinations are relative to the source document, including
    # hrefs that happen to begin with a repository directory name.
    return (source.parent / path_part).resolve()


def _is_historical(source: Path, text: str) -> bool:
    relative = source.relative_to(ROOT).as_posix()
    return (relative.startswith("ops/worklog/sessions/")
            or relative.startswith("ops/worklog/reviews/")
            or "<!-- doc-link-checker: historical -->" in text.lower())


def scan_local(directories: tuple[str, ...]) -> list[Finding]:
    findings: list[Finding] = []
    for source in markdown_files(directories):
        text = source.read_text(encoding="utf-8", errors="replace")
        masked = FENCE_RE.sub(lambda m: "\n" * m.group(0).count("\n"), text)
        # Keep inline links outside code spans; definitions are inspected separately.
        no_inline = INLINE_CODE_RE.sub(lambda m: " " * len(m.group(0)), masked)
        destinations: list[tuple[int, str, str]] = []
        for match in LINK_RE.finditer(no_inline):
            dest = match.group(1) or match.group(2)
            destinations.append((no_inline.count("\n", 0, match.start()) + 1,
                                 _clean_destination(dest), "markdown"))
        link_destination_spans = [
            match.span(1) if match.group(1) is not None else match.span(2)
            for match in LINK_RE.finditer(no_inline)
        ]
        definitions: dict[str, tuple[int, str]] = {}
        for match in REF_DEF_RE.finditer(no_inline):
            dest = match.group(2) or match.group(3)
            definitions[match.group(1).strip().lower()] = (
                no_inline.count("\n", 0, match.start()) + 1, _clean_destination(dest))
        for label, (line, dest) in definitions.items():
            destinations.append((line, dest, "reference"))

        # Flag used full or collapsed reference labels without a definition.
        # Shortcut references are ambiguous with ordinary bracketed prose.
        defined_labels = set(definitions)
        for match in REF_USE_RE.finditer(no_inline):
            following = no_inline[match.end():]
            if following.startswith("("):
                continue
            explicit = re.match(r"\[([^\]]*)\]", following)
            label = (explicit.group(1) if explicit and explicit.group(1)
                     else match.group(1)) or no_inline[match.start() + 1:match.end() - 1]
            label = label.strip().lower()
            if label and label not in defined_labels:
                line = no_inline.count("\n", 0, match.start()) + 1
                findings.append(Finding(source, line, f"[{label}]", "reference", "BROKEN"))

        seen: set[tuple[int, str]] = set()
        for line, target, kind in destinations:
            if (line, target) in seen:
                continue
            seen.add((line, target))
            resolved = _resolve_local(source, target)
            if resolved is None:
                continue
            if not resolved.exists():
                status = "HISTORICAL-MISSING" if _is_historical(source, text) else "BROKEN"
                findings.append(Finding(source, line, target, kind, status))

        # Explicit root-prefixed paths are often written in inline code, so scan
        # those too. Fenced blocks stay excluded; globs and machine paths are
        # filtered below.
        for match in ROOT_PATH_RE.finditer(masked):
            # A Markdown href always resolves from source.parent, even when it
            # starts with a repository directory name. Handle it above.
            if any(start <= match.start(1) < end
                   for start, end in link_destination_spans):
                continue
            target = match.group(1).rstrip(".,;:!?)\"")
            if any(char in target for char in "{}*?"):
                continue
            if "<" in target or ">" in target:
                continue
            # Avoid treating search descriptions such as ``docs/config/scripts``
            # as concrete file paths. Markdown links still support directories.
            if not Path(target).suffix and not target.endswith("/"):
                continue
            line = masked.count("\n", 0, match.start()) + 1
            if (line, target) in seen:
                continue
            # Link destinations were already handled above.
            resolved = (ROOT / target).resolve()
            if not resolved.exists():
                status = "HISTORICAL-MISSING" if _is_historical(source, text) else "BROKEN"
                findings.append(Finding(source, line, target, "repo-path", status))
    return findings


def _http_status(url: str, timeout: float, backend: str = "python") -> tuple[str, str]:
    if backend == "curl":
        try:
            response = subprocess.run(
                ["curl", "--head", "--silent", "--show-error", "--location",
                 "--output", "/dev/null", "--write-out", "%{http_code}",
                 "--max-time", str(timeout), url],
                capture_output=True, text=True, timeout=timeout + 1, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return url, f"UNVERIFIED ({type(exc).__name__}: {exc})"
        if response.returncode and not response.stdout.strip():
            detail = response.stderr.strip() or f"curl exit {response.returncode}"
            return url, f"UNVERIFIED ({detail})"
        try:
            code = int(response.stdout.strip())
        except ValueError:
            detail = response.stderr.strip() or "curl returned no HTTP status"
            return url, f"UNVERIFIED ({detail})"
        if not 100 <= code <= 599:
            detail = response.stderr.strip() or f"curl returned invalid HTTP status {code}"
            return url, f"UNVERIFIED ({detail})"
    else:
        request = Request(url, method="HEAD", headers={"User-Agent": "doc-link-checker/1.0"})
        try:
            with urlopen(request, timeout=timeout) as response:
                code = response.status
        except HTTPError as exc:
            code = exc.code
        except (URLError, TimeoutError, OSError) as exc:
            reason = getattr(exc, "reason", None)
            detail = f": {reason}" if reason is not None else f": {exc}"
            return url, f"UNVERIFIED ({type(exc).__name__}{detail})"
    if code in (403, 429):
        return url, f"UNVERIFIED (HTTP {code})"
    return url, "OK" if 200 <= code < 400 else f"HTTP {code}"


def _valid_http_url(url: str) -> bool:
    """Reject URL-shaped code fragments whose authority is not a hostname."""
    try:
        parsed = urlsplit(url)
        host = parsed.hostname
        # Accessing port also catches malformed bracket/port authorities.
        _ = parsed.port
    except ValueError:
        return False
    if parsed.scheme.lower() not in ("http", "https") or not host:
        return False
    try:
        ipaddress.ip_address(host)
        return True
    except ValueError:
        return all(HOST_LABEL_RE.fullmatch(label)
                   for label in host.rstrip(".").split("."))


def scan_http(directories: tuple[str, ...], timeout: float, jobs: int,
              max_urls: int, backend: str = "curl") -> list[tuple[str, str]]:
    urls: set[str] = set()
    for source in markdown_files(directories):
        text = FENCE_RE.sub("", source.read_text(encoding="utf-8", errors="replace"))
        urls.update(cleaned for url in URL_RE.findall(text)
                    if (cleaned := url.rstrip(URL_TRAILING_PUNCTUATION))
                    and _valid_http_url(cleaned))
    selected = sorted(urls)[:max_urls]
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(_http_status, url, timeout, backend) for url in selected]
        results = [future.result() for future in as_completed(futures)]
    return sorted(results)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", default=list(DEFAULT_DIRS),
                        help="directories to scan (default: docs/learning ops)")
    parser.add_argument("--check-http", action="store_true", help="probe HTTP(S) links")
    parser.add_argument("--timeout", type=float, default=5.0, help="per-request timeout (max 10s)")
    parser.add_argument("--jobs", type=int, default=4, help="HTTP concurrency (max 8)")
    parser.add_argument("--max-urls", type=int, default=100, help="maximum unique URLs to probe")
    parser.add_argument("--http-backend", choices=("curl", "python"), default="curl",
                        help="curl uses the system trust store; python uses urllib's trust store")
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 10 or not 1 <= args.jobs <= 8 or not 1 <= args.max_urls <= 100:
        parser.error("timeout must be 0–10s, jobs 1–8, and max-urls 1–100")
    directories = tuple(args.paths)
    findings = scan_local(directories)
    for finding in findings:
        rel = finding.file.relative_to(ROOT)
        print(f"{finding.status} {rel}:{finding.line}: {finding.target} ({finding.kind})")
    broken = sum(f.status == "BROKEN" for f in findings)
    historical = sum(f.status == "HISTORICAL-MISSING" for f in findings)
    print(f"Local scan: {len(findings)} missing reference(s): {broken} broken, "
          f"{historical} historical warning(s).")
    if args.check_http:
        for url, status in scan_http(directories, args.timeout, args.jobs,
                                     args.max_urls, args.http_backend):
            print(f"HTTP {status}: {url}")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
