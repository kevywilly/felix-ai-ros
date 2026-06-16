#!/usr/bin/env python3
"""Generate a print-ready checkerboard SVG for camera calibration.

The defaults match felix_perception's calibration workflow:
`ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 ...`

OpenCV's `--size WxH` counts INTERIOR corners (where four squares meet), so an
8x6 board needs 9x7 *squares*. SVG is used because it prints at an exact physical
size -- print at 100% / "actual size" (NOT "fit to page"), then MEASURE one
square with a ruler and pass that measured value to `--square`. Printer scaling
is the classic silent calibration error.

    python3 make_checkerboard.py                       # 9x7 @ 25mm -> checkerboard_8x6_25mm.svg
    python3 make_checkerboard.py --square-mm 20 --out board.svg
"""
import argparse


def make_svg(cols_squares, rows_squares, square_mm, margin_mm):
    w = cols_squares * square_mm + 2 * margin_mm
    h = rows_squares * square_mm + 2 * margin_mm
    inner_corners = f"{cols_squares - 1}x{rows_squares - 1}"
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}">',
        f'<rect x="0" y="0" width="{w}" height="{h}" fill="white"/>',
    ]
    for j in range(rows_squares):
        for i in range(cols_squares):
            if (i + j) % 2 == 0:
                x = margin_mm + i * square_mm
                y = margin_mm + j * square_mm
                parts.append(
                    f'<rect x="{x}" y="{y}" width="{square_mm}" '
                    f'height="{square_mm}" fill="black"/>')
    caption = (f"{inner_corners} interior corners "
               f"({cols_squares}x{rows_squares} squares), {square_mm} mm. "
               f"Print at 100% (actual size); measure a square and pass it to --square.")
    parts.append(
        f'<text x="{margin_mm}" y="{h - margin_mm / 3:.1f}" '
        f'font-family="sans-serif" font-size="3.5" fill="black">{caption}</text>')
    parts.append('</svg>')
    return "\n".join(parts)


_PAGES_PT = {  # landscape page sizes in PDF points (1 pt = 1/72 in)
    "letter": (792.0, 612.0),   # 11 x 8.5 in
    "a4": (841.89, 595.28),     # 297 x 210 mm
}
_MM_PT = 72.0 / 25.4


def make_pdf(cols_squares, rows_squares, square_mm, page="letter"):
    """A single-page landscape PDF with the pattern centered, sized in real
    points so it prints 1:1 at 100% scale from Preview/Acrobat (no browser
    fit-to-page scaling). Returns raw PDF bytes."""
    pw, ph = _PAGES_PT[page]
    sq = square_mm * _MM_PT
    patt_w, patt_h = cols_squares * sq, rows_squares * sq
    if patt_w > pw or patt_h > ph:
        raise SystemExit(
            f"pattern {patt_w/_MM_PT:.0f}x{patt_h/_MM_PT:.0f}mm does not fit "
            f"{page} landscape; use a smaller --square-mm or a bigger page.")
    ox, oy = (pw - patt_w) / 2.0, (ph - patt_h) / 2.0

    body = ["0 0 0 rg"]
    for j in range(rows_squares):
        for i in range(cols_squares):
            if (i + j) % 2 == 0:
                body.append(f"{ox + i*sq:.3f} {oy + j*sq:.3f} "
                            f"{sq:.3f} {sq:.3f} re f")
    content = ("\n".join(body) + "\n").encode("latin-1")

    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw:.2f} {ph:.2f}] "
        f"/Contents 4 0 R >>".encode("latin-1"),
        b"<< /Length " + str(len(content)).encode() +
        b" >>\nstream\n" + content + b"endstream",
    ]
    out = bytearray(b"%PDF-1.4\n")
    offsets = []
    for n, obj in enumerate(objs, start=1):
        offsets.append(len(out))
        out += f"{n} 0 obj\n".encode() + obj + b"\nendobj\n"
    xref_at = len(out)
    out += f"xref\n0 {len(objs)+1}\n".encode() + b"0000000000 65535 f \n"
    for off in offsets:
        out += f"{off:010d} 00000 n \n".encode()
    out += (f"trailer\n<< /Size {len(objs)+1} /Root 1 0 R >>\n"
            f"startxref\n{xref_at}\n%%EOF\n").encode()
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--size", default="8x6",
                    help="interior corners WxH (cameracalibrator --size); default 8x6")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--margin-mm", type=float, default=15.0)
    ap.add_argument("--format", choices=["svg", "pdf"], default="svg")
    ap.add_argument("--page", choices=list(_PAGES_PT), default="letter",
                    help="PDF page size (landscape); default letter")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    cw, ch = (int(v) for v in a.size.lower().split("x"))
    cols, rows = cw + 1, ch + 1  # interior corners -> squares
    out = a.out or f"checkerboard_{a.size}_{int(a.square_mm)}mm.{a.format}"
    if a.format == "pdf":
        with open(out, "wb") as f:
            f.write(make_pdf(cols, rows, a.square_mm, a.page))
        extra = f" on {a.page} landscape"
    else:
        with open(out, "w") as f:
            f.write(make_svg(cols, rows, a.square_mm, a.margin_mm))
        extra = ""
    print(f"wrote {out}: {cols}x{rows} squares ({a.size} interior corners), "
          f"{a.square_mm:.0f} mm -> pattern {cols*a.square_mm:.0f}x"
          f"{rows*a.square_mm:.0f} mm{extra}")


if __name__ == "__main__":
    main()
