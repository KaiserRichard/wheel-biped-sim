#!/usr/bin/env python3
"""Generate a print-ready Kalibr AprilGrid PDF and matching target YAML (no dependencies).

Layout reproduces Kalibr's kalibr_create_target_pdf (ethz-asl/kalibr commit 1f60227442d25e36365ef5f72cd80b9666d73467):
tag family t36h11, borderBits=2, rotation=2 (np.rot90 twice), symmetric corner squares, tag ID = n_cols*y + x
with (x, y) counted from the bottom-left tag. tagSize is the full black square edge (incl. the 2-bit border);
tagSpacing is the gap as a fraction of tagSize.

Usage: python3 make_aprilgrid.py
"""
import os

# First 24 t36h11 codes as listed in Kalibr's generator (AprilTags C++ library, Kaess).
T36H11 = [0xd5d628584, 0xd97f18b49, 0xdd280910e, 0xe479e9c98, 0xebcbca822, 0xf31dab3ac,
          0x056a5d085, 0x10652e1d4, 0x22b1dfead, 0x265ad0472, 0x34fe91b86, 0x3ff962cd5,
          0x43a25329a, 0x474b4385f, 0x4e9d243e9, 0x5246149ae, 0x5997f5538, 0x683bb6c4c,
          0x6be4a7211, 0x7e3158eea, 0x81da494af, 0x858339a74, 0x8cd51a5fe, 0x9f21cc2d7]

MM = 72.0 / 25.4  # PDF points per millimetre
PAPERS = {"A3": (420.0, 297.0), "A4": (297.0, 210.0)}  # landscape, mm

# name: (paper, cols, rows, tag size mm, spacing fraction)
TARGETS = {
    "aprilgrid_6x4_45mm_A3": ("A3", 6, 4, 45.0, 0.3),
    "aprilgrid_6x4_30mm_A4": ("A4", 6, 4, 30.0, 0.3),
}


def code_matrix(code):
    """Kalibr: cell (i, j) is black when bit 6*i+j is 0, then np.rot90(m, 2)."""
    m = [[0 if code & (1 << (6 * i + j)) else 1 for j in range(6)] for i in range(6)]
    return [row[::-1] for row in m[::-1]]


def tag_rects(x0, y0, size, spacing, code, border_bits=2):
    """Black rectangles (x, y, w, h) in mm, y up, exactly as Kalibr's generateAprilTag."""
    b = size / (6 + 2 * border_bits)
    bs = border_bits * b
    rects = [(x0, y0, size, bs), (x0, y0 + size - bs, size, bs),
             (x0 + size - bs, y0, bs, size), (x0, y0, bs, size)]
    m = code_matrix(code)
    for i in range(6):
        for j in range(6):
            if m[i][j]:
                rects.append((x0 + (j + border_bits) * b, y0 + ((border_bits - 1) + 6 - i) * b, b, b))
    sq = spacing * size
    for cx, cy in ((x0 - sq, y0 - sq), (x0 + size, y0 - sq), (x0 + size, y0 + size), (x0 - sq, y0 + size)):
        rects.append((cx, cy, sq, sq))
    return rects


def grid(cols, rows, size, spacing):
    rects = []
    for y in range(rows):
        for x in range(cols):
            rects += tag_rects(x * (1 + spacing) * size, y * (1 + spacing) * size, size, spacing,
                               T36H11[cols * y + x])
    return rects


def pdf_bytes(paper_mm, ops):
    w, h = paper_mm[0] * MM, paper_mm[1] * MM
    content = "\n".join(ops).encode()
    objs = [b"<< /Type /Catalog /Pages 2 0 R >>",
            b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
            ("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Contents 4 0 R "
             "/Resources << /Font << /F1 5 0 R >> >> >>" % (w, h)).encode(),
            b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream",
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"]
    out, offs = b"%PDF-1.4\n", []
    for n, o in enumerate(objs, 1):
        offs.append(len(out))
        out += b"%d 0 obj\n" % n + o + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    out += b"".join(b"%010d 00000 n \n" % o for o in offs)
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)
    return out


def text(x, y, pt, s):
    s = s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    return "BT /F1 %.1f Tf %.3f %.3f Td (%s) Tj ET" % (pt, x * MM, y * MM, s)


def build(name, paper, cols, rows, size, spacing, outdir):
    pw, ph = PAPERS[paper]
    sq = spacing * size
    gw = cols * size + (cols - 1) * sq  # tag-to-tag extent
    gh = rows * size + (rows - 1) * sq
    ox = (pw - gw) / 2.0
    oy = (ph - gh) / 2.0 + 8.0  # leave room for the scale bar and caption below
    ops = ["0 0 0 rg"]
    for x, y, w, h in grid(cols, rows, size, spacing):
        ops.append("%.4f %.4f %.4f %.4f re f" % ((ox + x) * MM, (oy + y) * MM, w * MM, h * MM))
    # 100 mm scale bar with 10 mm ticks, for checking print scale
    bx, by = ox, oy - sq - 14.0
    ops.append("0.4 w %.3f %.3f m %.3f %.3f l S" % (bx * MM, by * MM, (bx + 100) * MM, by * MM))
    for k in range(11):
        t = 3.0 if k % 5 == 0 else 1.5
        ops.append("%.3f %.3f m %.3f %.3f l S" % ((bx + 10 * k) * MM, by * MM, (bx + 10 * k) * MM, (by + t) * MM))
    ops.append(text(bx + 103, by - 1, 7, "100 mm: must measure 100.0 mm (print at 100%, no fit-to-page)"))
    span = (cols - 1) * (1 + spacing) * size
    ops.append(text(bx, by - 7, 7,
                    "Kalibr aprilgrid t36h11  %dx%d tags  tagSize=%.3f m  tagSpacing=%.2f  IDs 0-%d from bottom-left  "
                    "check: left edge tag 0 to left edge tag %d = %.1f mm" % (cols, rows, size / 1000, spacing,
                                                                         cols * rows - 1, cols - 1, span)))
    with open(os.path.join(outdir, name + ".pdf"), "wb") as f:
        f.write(pdf_bytes((pw, ph), ops))
    with open(os.path.join(outdir, name + ".yaml"), "w") as f:
        f.write("# Kalibr target for %s.pdf (%s landscape). NOMINAL values: replace tagSize with the\n"
                "# measured value after printing (see README.md). tagSpacing is a ratio and does not change\n"
                "# under uniform print scaling.\n" % (name, paper))
        f.write("target_type: 'aprilgrid'\ntagCols: %d\ntagRows: %d\ntagSize: %.4f  # [m] NOMINAL\n"
                "tagSpacing: %.2f  # [-] gap / tagSize\n" % (cols, rows, size / 1000, spacing))


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for name, spec in TARGETS.items():
        build(name, *spec, outdir=here)
        print("wrote", name)
