#!/usr/bin/env python3
"""Generate a print-ready checkerboard SVG for stereo calibration.

Defaults match what MATLAB's Stereo Camera Calibrator and OpenCV's
findChessboardCornersSB expect for this rig: 9x6 INNER corners (= 10x7
squares) at 25 mm, which is 250x175 mm and fits A4 LANDSCAPE with margins.

"Inner corners" are the interior vertices where four squares meet -- the
board has one more square than corners on each axis. Give MATLAB the SQUARE
SIZE in mm (25), not the corner count; it infers the grid from the images.

The page carries a 100 mm reference line. Measure it with a ruler after
printing: if it is not exactly 100 mm, the printer scaled the page and every
extrinsic distance (baseline included) is wrong by that same ratio.

Usage:
    python3 make_checkerboard.py                       # 9x6 @ 25mm, A4 landscape
    python3 make_checkerboard.py --cols 9 --rows 6 --square 30 --page a3
"""
import argparse

PAGES = {  # width, height in mm
    'a4': (297.0, 210.0),        # landscape
    'a4-portrait': (210.0, 297.0),
    'a3': (420.0, 297.0),
    'letter': (279.4, 215.9),
}


def build(cols, rows, square, page):
    """cols/rows are INNER CORNER counts; squares = cols+1 by rows+1."""
    nx, ny = cols + 1, rows + 1
    bw, bh = nx * square, ny * square
    pw, ph = PAGES[page]

    if bw > pw or bh > ph:
        raise SystemExit(
            f"board {bw:.0f}x{bh:.0f} mm does not fit {page} ({pw:.0f}x{ph:.0f} mm).\n"
            f"reduce --square (try {min(pw / nx, ph / ny) * 0.9:.0f}) or use --page a3")

    ox = (pw - bw) / 2.0
    oy = 12.0
    if oy + bh > ph - 14.0:            # keep room for the reference line
        oy = max(6.0, (ph - bh - 14.0) / 2.0)
    ruler_y = min(oy + bh + 12.0, ph - 5.0)
    rx = (pw - 100.0) / 2.0

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{pw}mm" height="{ph}mm" '
        f'viewBox="0 0 {pw} {ph}">',
        '<rect width="100%" height="100%" fill="#fff"/>',
        '<g shape-rendering="crispEdges">',
    ]
    # Black squares only; the white ones are the page. A quiet white border all
    # the way around the board is required -- detectors find the outer squares
    # by contrast against it, so never crop to the board edge.
    for r in range(ny):
        for c in range(nx):
            if (r + c) % 2 == 0:
                out.append(f'<rect x="{ox + c * square:.4f}" y="{oy + r * square:.4f}" '
                           f'width="{square}" height="{square}" fill="#000"/>')
    out.append('</g>')

    # 100 mm print-scale check: end caps plus the label.
    out += [
        f'<g stroke="#000" stroke-width="0.3">',
        f'<line x1="{rx:.4f}" y1="{ruler_y:.4f}" x2="{rx + 100:.4f}" y2="{ruler_y:.4f}"/>',
        f'<line x1="{rx:.4f}" y1="{ruler_y - 2.5:.4f}" x2="{rx:.4f}" y2="{ruler_y + 2.5:.4f}"/>',
        f'<line x1="{rx + 100:.4f}" y1="{ruler_y - 2.5:.4f}" x2="{rx + 100:.4f}" y2="{ruler_y + 2.5:.4f}"/>',
        '</g>',
        f'<text x="{pw / 2:.4f}" y="{ruler_y - 4:.4f}" font-family="Helvetica,Arial" '
        f'font-size="4" text-anchor="middle" fill="#000">'
        f'100 mm &#8212; measure after printing; must be exact</text>',
        f'<text x="{pw / 2:.4f}" y="{ruler_y + 7:.4f}" font-family="Helvetica,Arial" '
        f'font-size="3.2" text-anchor="middle" fill="#000">'
        f'{cols}x{rows} inner corners &#183; {square:g} mm squares &#183; '
        f'print at 100% / "actual size", NO fit-to-page</text>',
        '</svg>',
    ]
    return '\n'.join(out), (bw, bh)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cols', type=int, default=9, help='inner corners across')
    p.add_argument('--rows', type=int, default=6, help='inner corners down')
    p.add_argument('--square', type=float, default=25.0, help='square size in mm')
    p.add_argument('--page', default='a4', choices=sorted(PAGES))
    p.add_argument('-o', '--out', default=None)
    a = p.parse_args()

    svg, (bw, bh) = build(a.cols, a.rows, a.square, a.page)
    out = a.out or f'checkerboard_{a.cols}x{a.rows}_{a.square:g}mm.svg'
    with open(out, 'w') as f:
        f.write(svg + '\n')
    print(f'{out}: {a.cols}x{a.rows} inner corners, {a.square:g} mm squares, '
          f'board {bw:.0f}x{bh:.0f} mm on {a.page}')


if __name__ == '__main__':
    main()
