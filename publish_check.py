"""Reject private or prohibited content in this publish tree (not a history scan)."""
from pathlib import Path, PurePosixPath, PureWindowsPath
import gzip
import hashlib
import ipaddress
import json
import os
import re
import sys
import zlib

ROOT = Path(__file__).resolve().parent
MAX_TEXT_BYTES = 10 * 1024 * 1024
MAX_HARDWARE_KICAD_BYTES = 24 * 1024 * 1024
HARDWARE_KICAD_HEADERS = {
    ".kicad_pcb": "(kicad_pcb",
    ".kicad_sch": "(kicad_sch",
    ".kicad_sym": "(kicad_symbol_lib",
}
PDF_ALLOWED_METADATA = {
    "title": {"", "WBR power board schematic", "WBR control board schematic"},
    "author": {""},
    "creator": {"", "Eeschema-PDF"},
    "producer": {"", "KiCad PDF"},
    "subject": {""},
    "keywords": {""},
}
# Split sample signatures so policy code and tests are scanned too, without exemptions.
blocked_words = ("con" + "ductor", "work" + "er", "tick" + "et")
blocked_names = ("thesis" + "_text.txt", "Thuyet_minh_Binh" + ".pdf")
blocked_path = "hazard" + "-frames"
# Central inventory: model families observed in inputs plus common spelling variants.
IMU_MODEL_PATTERNS = (
    r"(?<![A-Za-z0-9])BMI[ _-]?(?:088|085|160|270|323)(?![A-Za-z0-9])",
    r"(?<![A-Za-z0-9])MPU[ _-]?(?:6000|6050|6500|9250)(?![A-Za-z0-9])",
    r"(?<![A-Za-z0-9])ICM[ _-]?(?:20602|20608|20948|42605|42688)(?:[ _-]?P)?(?![A-Za-z0-9])",
    r"(?<![A-Za-z0-9])BNO[ _-]?(?:055|080|085|086)(?![A-Za-z0-9])",
    r"(?<![A-Za-z0-9])LSM6(?:DS3|DSL|DSO|DSOX|DSV)(?![A-Za-z0-9])",
    r"(?<![A-Za-z0-9])ADIS[ _-]?16\d{3}(?![A-Za-z0-9])",
)
IMU_MODELS = re.compile("|".join(IMU_MODEL_PATTERNS), re.I)
blocked_patterns = [
    re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}"),
    re.compile(r"(?i)(?:api[_-]?key|access[_-]?token|secret)\s*[:=]\s*['\"]?[A-Za-z0-9/_+=.-]{12,}"),
    re.compile(r"(?i)(?:/" + "home" + r"/|/" + "Users" + r"/)[^/\s]+"),
    re.compile(r"(?i)[A-Z]:(?!//)[\\/]+[^\s\"<>]+"),
    re.compile(r"(?i)/" + r"mnt/[a-z]/[^\s\"'<>]+"),
    re.compile(r"\b(?:gh[pousr]_" + r"[A-Za-z0-9]{20,}|github_pat_" + r"[A-Za-z0-9_]{20,})\b"),
    re.compile(r"\bsk-" + r"(?:proj-|svcacct-|ant-(?:api\d+-)?)?[A-Za-z0-9_-]{20,}\b"),
    re.compile(r"(?i)\b(?:student[ _-]?(?:id|number)|matriculation|mssv|mã\s+số\s+sinh\s+viên)\s*[:=#-]?\s*\d{8,10}\b"),
    re.compile(r"(?i)\b\d{8,10}\s*\(?\s*(?:student[ _-]?id|mssv)\b"),
    re.compile(r"(?i)\b(?:co" + r"dex|cl" + r"aude)[_-]+[a-z0-9]+\b|\ba" + r"gy\b"),
    re.compile(r"(?i)\b(?:licen[cs]e|serial)[ _-]?(?:number|no\.?|key)\s*[:=#-]\s*[A-Z0-9][A-Z0-9-]{5,}\b"),
    re.compile(r"\bT\d{1,2}\b"),
]
FORBIDDEN_SUFFIXES = {
    ".npz", ".jpg", ".jpeg", ".webp", ".lic", ".step", ".stp", ".stl", ".obj", ".dae", ".slx",
    ".pdf", ".docx", ".bag", ".db3", ".pt", ".onnx", ".pem", ".pyc",
}
FORBIDDEN_DIRS = {".venv", "venv", ".env", ".aws", ".agents", ".co" + "dex", ".cl" + "aude", "__pycache__", ".pytest_cache"}
# Only these reviewed PNG paths and exact bytes are accepted.
PNG_ALLOWLIST = {
    'media/figures/gains.png': 'e6b34ddc3e9ba2e1b4b6874e0585fbeedba911e84eb9a5d72ed62c37a79f8838',
    'media/figures/hip_boundary.png': 'cc5e50c3498a0178e7b09aec53bb983d433552909bee83a8157d43685b620054',
    'media/figures/margins.png': '04f1bf66efb606056efd0bffa4311c93a75572b3d564dc59ba2785ac33593b3d',
    'media/figures/poles.png': 'd145498949cbe4f10d180c89d089b32ba8bdacc63f00a9958fcf9bcfa58d76da',
    'media/figures/simscape.png': 'd7bc6c9025fd26e37c2f83c1be014c8ec2933821a2b8607478f3ecff9be8cc57',
    'media/figures/simulink-dmbots-model.png': '924d5dd1ae0d05f28ca5bf19521d1edd3bdf42caaddd2652753ea007f0de3b05',
    'media/figures/m14-gate-history.png': '6342b5a89470958e66cb164ccfe1c39baea1863011eaace4c665e5d390993870',
    'media/figures/m14-nominal-mission.png': '59c66a64954e926f128d1a1b708dc3e8ad647d98c6fe12ef92a8df32664defac',
    'media/figures/m14-pitch-all-fixtures.png': '7f0613d86da149014150c42880fbbdd4d7171c9ac5f5a583d64c111a6bfa4ba7',
    'media/figures/m14-startup-fix.png': '8feb0eb5d78c8dd7a72ae13f553ddc6976b835242862729aafa09c4dc29e175b',
    'media/hardware/power-board.png': '897a09959997d5b980775a0e2131905ae06afdd97524f9a3161a64ff0f43871d',
    'media/hardware/control-board.png': '0ef5a8d4c2ee4f3b7dc30d8476895a6dc1aca29125494d6c355392a41e56082b',
}
# No compressed additions are approved in this P3-only batch.
GZIP_ALLOWLIST = frozenset()


def is_relative_path(value):
    return (isinstance(value, str) and bool(value) and "\\" not in value
            and not PureWindowsPath(value).drive and not PurePosixPath(value).is_absolute()
            and ".." not in PurePosixPath(value).parts and "://" not in value)


def receipt_paths_valid(value, key=""):
    if isinstance(value, dict):
        return all(receipt_paths_valid(v, k) for k, v in value.items())
    if isinstance(value, list):
        return all(receipt_paths_valid(v, key) for v in value)
    if isinstance(value, str) and (key in {"path", "file", "input", "output", "log", "executable"}
                                  or key.endswith(("_path", "_file"))):
        return is_relative_path(value)
    return True


def json_context(value):
    """Expose JSON scalar key/value context to the same text rules."""
    if isinstance(value, dict):
        return "\n".join((f"{key}: {item}" if not isinstance(item, (dict, list))
                           else json_context(item)) for key, item in value.items())
    if isinstance(value, list):
        return "\n".join(json_context(item) for item in value)
    return str(value)


def has_ip_address(text):
    for match in re.finditer(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", text):
        try:
            ipaddress.IPv4Address(match.group())
        except ValueError:
            continue
        # Explicit version labels are not network endpoints.
        prefix = text[max(0, match.start() - 40):match.start()]
        if re.search(r"(?i)(?:version|python|matlab|numpy|scipy|release|firmware)\s*[:=v]?\s*$", prefix):
            continue
        return True
    return False


def png_metadata(raw):
    """Read PNG metadata, including compressed text, without inspecting pixel data."""
    if not raw.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("invalid PNG")
    chunks = []
    offset = 8
    while offset + 12 <= len(raw):
        size = int.from_bytes(raw[offset:offset + 4], "big")
        kind = raw[offset + 4:offset + 8]
        if offset + size + 12 > len(raw):
            raise ValueError("truncated PNG")
        data = raw[offset + 8:offset + 8 + size]
        if kind == b"zTXt":
            keyword, rest = data.split(b"\0", 1)
            data = keyword + b" " + zlib.decompressobj().decompress(rest[1:], MAX_TEXT_BYTES + 1)
        elif kind == b"iTXt":
            keyword, rest = data.split(b"\0", 1)
            compressed = rest[0]
            language, translated, payload = rest[2:].split(b"\0", 2)
            if compressed:
                payload = zlib.decompressobj().decompress(payload, MAX_TEXT_BYTES + 1)
            data = b" ".join((keyword, language, translated, payload))
        if kind not in {b"IDAT", b"IHDR", b"IEND", b"PLTE"}:
            if len(data) > MAX_TEXT_BYTES:
                raise ValueError("oversize PNG metadata")
            chunks.append(data)
        offset += size + 12
    return b"\n".join(chunks)


def mp4_metadata(raw):
    """Inspect container atoms, excluding compressed samples in mdat atoms."""
    chunks = []
    offset = 0
    while offset + 8 <= len(raw):
        size = int.from_bytes(raw[offset:offset + 4], "big")
        kind = raw[offset + 4:offset + 8]
        header = 8
        if size == 1:
            if offset + 16 > len(raw):
                return raw  # Malformed: conservatively inspect all bytes.
            size = int.from_bytes(raw[offset + 8:offset + 16], "big")
            header = 16
        elif size == 0:
            size = len(raw) - offset
        if size < header or offset + size > len(raw):
            return raw
        if kind != b"mdat":
            chunks.append(raw[offset:offset + size])
        offset += size
    return b"".join(chunks) + raw[offset:]


def hardware_pdf_text(raw):
    """Inspect PDF metadata and visible text with a PDF parser."""
    import pymupdf

    with pymupdf.open(stream=raw, filetype="pdf") as pdf:
        if not pdf.is_pdf or pdf.is_repaired or pdf.needs_pass or not pdf.page_count:
            raise ValueError("invalid or unreadable PDF")
        if pdf.get_xml_metadata() or pdf.embfile_count():
            raise ValueError("unexpected XMP metadata or embedded files")
        metadata = pdf.metadata
        if any((metadata.get(key) or "") not in allowed
               for key, allowed in PDF_ALLOWED_METADATA.items()):
            raise ValueError("unapproved PDF metadata")
        return "\n".join(page.get_text() for page in pdf)


def scan(root=None):
    root = Path(root) if root is not None else ROOT
    problems = []
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in list(dirs):
            path = Path(directory) / name
            rel = path.relative_to(root).as_posix()
            if path.is_symlink():
                problems.append(f"prohibited symlink: {rel}")
                dirs.remove(name)
            elif name == ".git":
                dirs.remove(name)
            elif name.lower() in FORBIDDEN_DIRS or name.lower().startswith('.venv'):
                problems.append(f"prohibited environment or generated directory: {rel}")
                dirs.remove(name)
        for name in files:
            path = Path(directory) / name
            rel = path.relative_to(root).as_posix()
            if path.is_symlink():
                problems.append(f"prohibited symlink: {rel}")
                continue
            suffix = path.suffix.lower()
            under_hardware = len(path.relative_to(root).parts) > 1 and rel.startswith("hardware/")
            is_hardware_pdf = under_hardware and suffix == ".pdf"
            kicad_header = HARDWARE_KICAD_HEADERS.get(suffix) if under_hardware else None
            if ((suffix in FORBIDDEN_SUFFIXES and not is_hardware_pdf)
                    or name.lower().startswith(("trial_", ".env", "id_"))
                    or name.lower() == ".git" or blocked_path.lower() in rel.lower()):
                problems.append(f"prohibited file: {rel}")
            if any(n.lower() in rel.lower() for n in blocked_names) or IMU_MODELS.search(rel):
                problems.append(f"prohibited source or sensor filename: {rel}")
            is_figure = suffix == ".png" and rel in PNG_ALLOWLIST
            is_video = suffix in {".mp4", ".gif"} or is_figure
            if suffix == ".png" and not is_figure:
                problems.append(f"unreviewed PNG: {rel}")
                continue
            if suffix == ".gz" and rel not in GZIP_ALLOWLIST:
                problems.append(f"unapproved compressed data: {rel}")
                continue
            size_limit = MAX_HARDWARE_KICAD_BYTES if kicad_header else MAX_TEXT_BYTES
            if not is_video and path.stat().st_size > size_limit:
                problems.append(f"oversize text or opaque data: {rel}")
                continue
            raw = path.read_bytes()
            if is_figure and hashlib.sha256(raw).hexdigest() != PNG_ALLOWLIST[rel]:
                problems.append(f"changed allow-listed PNG: {rel}")
            if suffix == ".gz":
                try:
                    with gzip.open(path, "rb") as stream:
                        raw = stream.read(MAX_TEXT_BYTES + 1)
                    if len(raw) > MAX_TEXT_BYTES:
                        problems.append(f"oversize decompressed data: {rel}")
                        continue
                except (OSError, EOFError):
                    problems.append(f"unreadable compressed data: {rel}")
                    continue
            if is_figure:
                try:
                    raw = png_metadata(raw)
                except (ValueError, IndexError, zlib.error):
                    problems.append(f"unreadable PNG metadata: {rel}")
                    continue
            if is_hardware_pdf:
                try:
                    text = hardware_pdf_text(raw)
                except ImportError:
                    problems.append(f"PDF inspection requires PyMuPDF: {rel}")
                    continue
                except (ValueError, RuntimeError):
                    problems.append(f"unreadable or unapproved PDF metadata: {rel}")
                    continue
            elif kicad_header:
                try:
                    text = raw.decode("utf-8-sig")
                except UnicodeDecodeError:
                    problems.append(f"unreadable KiCad text: {rel}")
                    continue
                if not re.match(re.escape(kicad_header) + r"\s", text.lstrip()) or "\0" in text:
                    problems.append(f"unreadable KiCad text: {rel}")
                    continue
            elif suffix == ".gif":
                text = "\n".join(v.decode("ascii", errors="ignore") for v in re.findall(rb"\x21\xfe[\x01-\xff]([\x20-\x7e]+)", raw))
            elif is_video:
                if suffix == ".mp4":
                    raw = mp4_metadata(raw)
                text = "\n".join(v.decode("ascii", errors="ignore") for v in re.findall(rb"[\x20-\x7e]{5,}", raw))
            else:
                text = raw.decode("utf-8", errors="ignore")
            if suffix == ".json":
                try:
                    text += "\n" + json_context(json.loads(raw))
                except (ValueError, UnicodeDecodeError):
                    pass  # Raw text still receives every content check.
            # Normalize serialized backslashes before inspecting JSON and text paths.
            text = text.replace("\\\\", "\\")
            if any(re.search(r"(?i)\b" + re.escape(t) + r"\b", text) for t in blocked_words):
                problems.append(f"prohibited internal term in {rel}")
            if any(t.lower() in text.lower() for t in blocked_names):
                problems.append(f"prohibited source document in {rel}")
            patterns = blocked_patterns[:-1] if is_video else blocked_patterns
            # The legacy internal-label rule must not reject the documented MIT
            # packet's bit-width field sequence. Other labels remain prohibited.
            workflow_text = re.sub(r"P16\s*\|\s*V12\s*\|\s*Kp12\s*\|\s*Kd12\s*" + r"\|\s*T" + r"12\b", "MIT frame fields", text)
            if has_ip_address(text) or IMU_MODELS.search(text) or any(
                    p.search(workflow_text if p is blocked_patterns[-1] else text) for p in patterns):
                problems.append(f"private or secret-like text in {rel}")
            if ("D435" + "i") in text and "136" in text:
                problems.append(f"prohibited camera-frame reference in {rel}")
            if suffix == ".json" and ("receipt" in name.lower() or name in {"SOURCE_MANIFEST.json", "verification.json"}):
                try:
                    if not receipt_paths_valid(json.loads(raw)):
                        problems.append(f"non-relative receipt path in {rel}")
                except (ValueError, UnicodeDecodeError):
                    problems.append(f"unreadable receipt in {rel}")
    return sorted(set(problems))


if __name__ == "__main__":
    findings = scan()
    if findings:
        print("\n".join(findings), file=sys.stderr)
        raise SystemExit(1)
    print("Publish scan passed.")
