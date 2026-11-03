"""Reject private or prohibited content in this publish tree."""
from pathlib import Path
import gzip
import re
import sys

ROOT = Path(__file__).resolve().parent
# Split tokens so the scanner does not flag its own policy strings.
blocked_words = ("con" + "ductor", "work" + "er", "tick" + "et")
blocked_names = ("thesis" + "_text.txt", "Thuyet_minh_Binh" + ".pdf")
blocked_path = "hazard" + "-frames"
blocked_patterns = [
    re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b"),
    re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}"),
    re.compile(r"(?i)(?:api[_-]?key|access[_-]?token|secret)\s*[:=]\s*['\"]?[A-Za-z0-9/_+=.-]{12,}"),
    re.compile(r"(?i)(?:/" + "home" + r"/|/" + "Users" + r"/)[^/\s]+"),
    re.compile(r"\bT\d{1,2}\b"),
]

def scan():
    problems = []
    for path in ROOT.rglob("*"):
        if not path.is_file() or ".git" in path.parts:
            continue
        rel = path.relative_to(ROOT).as_posix()
        if blocked_path.lower() in rel.lower():
            problems.append(f"prohibited path: {rel}")
        # Plots made by this project live in media/figures; a photograph anywhere else is still blocked.
        is_figure = rel.startswith("media/figures/") and path.suffix.lower() == ".png"
        if path.suffix.lower() in {".npz", ".jpg", ".jpeg", ".png", ".webp"} and not is_figure:
            problems.append(f"prohibited binary data or image: {rel}")
        if "__pycache__" in path.parts or path.suffix.lower() == ".pyc":
            continue
        if path.suffix.lower() in {".pyc"}:
            problems.append(f"generated interpreter bytecode: {rel}")
            continue
        raw = path.read_bytes()
        if path.suffix.lower() == ".gz":
            try:
                raw = gzip.decompress(raw)
            except OSError:
                problems.append(f"unreadable compressed trace: {rel}")
                continue
        is_video = path.suffix.lower() in {".mp4", ".gif"} or is_figure
        if path.suffix.lower() == ".gif":
            # A GIF made here is all pixels, whose random bytes can look like text. Only its comment blocks hold text.
            text = "\n".join(value.decode("ascii", errors="ignore") for value in re.findall(rb"\x21\xfe[\x01-\xff]([\x20-\x7e]+)", raw))
        elif is_video:
            # Scan printable container strings without treating compressed video frames as text.
            text = "\n".join(
                value.decode("ascii", errors="ignore")
                for value in re.findall(rb"[\x20-\x7e]{5,}", raw)
            )
        elif path.suffix.lower() in {".png", ".jpg", ".jpeg", ".webp"}:
            problems.append(f"prohibited raster image: {rel}")
            continue
        else:
            text = raw.decode("utf-8", errors="ignore")
        for token in blocked_words:
            if re.search(r"(?i)\b" + re.escape(token) + r"\b", text):
                problems.append(f"prohibited internal term in {rel}")
        for token in blocked_names:
            if token.lower() in text.lower() or token.lower() in rel.lower():
                problems.append(f"prohibited source document in {rel}")
        # Codec bytes can coincidentally contain short workflow labels. Those
        # are checked in source and metadata text files instead.
        patterns_to_check = blocked_patterns[:-1] if is_video else blocked_patterns
        for pattern in patterns_to_check:
            if pattern.search(text):
                problems.append(f"private or secret-like text in {rel}")
                break
        if ("D435" + "i") in text and "136" in text:
            problems.append(f"prohibited camera-frame reference in {rel}")
    return sorted(set(problems))

if __name__ == "__main__":
    findings = scan()
    if findings:
        print("\n".join(findings), file=sys.stderr)
        raise SystemExit(1)
    print("Publish scan passed.")
