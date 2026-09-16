#!/usr/bin/env python3
"""Generate print-ready ArUco markers as vector SVG at an exact physical size.

Vector, not a scaled bitmap: a resampled PNG puts grey fringes on the cell edges,
and the detector locates corners from exactly those edges.

DICT_4X4_50, IDs 0 and 1 -- two rovers need two IDs, and 4x4 gives the largest
cells for a given physical size, which is what buys detection range and blur
tolerance. Whitelist these two IDs in the detector and reject everything else;
that removes most of the false-positive risk that 4x4 dictionaries are criticised
for.

SIZE CONVENTION -- this is the one that goes wrong. `--size` is the side of the
full BLACK SQUARE, including its one-cell black border. That is also what OpenCV
means by markerLength when estimating pose. A 4x4 marker is 6x6 cells, so at
50 mm each cell is 50/6 = 8.333 mm. Do not measure just the inner 4x4 data area
and feed that number to the pose code.

Each marker gets a white quiet zone of at least one cell all round. Printing a
marker edge-to-edge is the most common reason detection silently fails.

Usage:
    python3 make_markers.py                    # 50 mm, IDs 0 and 1, plus spares
    python3 make_markers.py --size 40 --ids 0,1,2,3
"""
import argparse

import cv2
import numpy as np

PAGE_W, PAGE_H = 210.0, 297.0        # A4 portrait, mm


def marker_cells(dict_id, marker_id):
    """6x6 grid of 0/1 for a 4x4 marker including its black border."""
    d = cv2.aruco.getPredefinedDictionary(dict_id)
    img = cv2.aruco.generateImageMarker(d, marker_id, 6, 1)
    return (img // 255).astype(int)


def build(ids, size_mm, quiet_mm, dict_name, dict_id, spares):
    cells = 6
    cell = size_mm / cells
    tile = size_mm + 2 * quiet_mm

    rows = 2 if spares else 1
    cols = len(ids)
    grid_w, grid_h = cols * tile, rows * tile
    if grid_w > PAGE_W - 20 or grid_h > PAGE_H - 60:
        raise SystemExit(f'{cols}x{rows} tiles of {tile:.0f} mm do not fit A4; '
                         f'reduce --size or drop --spares')

    ox = (PAGE_W - grid_w) / 2.0
    oy = 18.0

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{PAGE_W}mm" '
           f'height="{PAGE_H}mm" viewBox="0 0 {PAGE_W} {PAGE_H}">',
           '<rect width="100%" height="100%" fill="#fff"/>']

    for r in range(rows):
        for c, mid in enumerate(ids):
            tx, ty = ox + c * tile, oy + r * tile
            mx, my = tx + quiet_mm, ty + quiet_mm

            # cut guide at the tile edge -- light grey, and a full quiet zone
            # away from the marker so it cannot affect detection
            out.append(f'<rect x="{tx:.3f}" y="{ty:.3f}" width="{tile:.3f}" '
                       f'height="{tile:.3f}" fill="none" stroke="#c8c8c8" '
                       f'stroke-width="0.2"/>')

            grid = marker_cells(dict_id, mid)
            out.append('<g shape-rendering="crispEdges">')
            for iy in range(cells):
                for ix in range(cells):
                    if grid[iy, ix] == 0:          # 0 = black
                        out.append(
                            f'<rect x="{mx + ix * cell:.4f}" '
                            f'y="{my + iy * cell:.4f}" '
                            f'width="{cell:.4f}" height="{cell:.4f}" '
                            f'fill="#000"/>')
            out.append('</g>')

            label = f'ID {mid}' + ('  (spare)' if r == 1 else '')
            out.append(
                f'<text x="{tx + tile / 2:.3f}" y="{ty + tile - 2.2:.3f}" '
                f'font-family="Helvetica,Arial" font-size="3.2" '
                f'text-anchor="middle" fill="#000">{label}</text>')

    # print-scale check
    ry = oy + grid_h + 16.0
    rx = (PAGE_W - 100.0) / 2.0
    out += [
        '<g stroke="#000" stroke-width="0.3">',
        f'<line x1="{rx:.3f}" y1="{ry:.3f}" x2="{rx + 100:.3f}" y2="{ry:.3f}"/>',
        f'<line x1="{rx:.3f}" y1="{ry - 2.5:.3f}" x2="{rx:.3f}" y2="{ry + 2.5:.3f}"/>',
        f'<line x1="{rx + 100:.3f}" y1="{ry - 2.5:.3f}" x2="{rx + 100:.3f}" '
        f'y2="{ry + 2.5:.3f}"/>',
        '</g>',
        f'<text x="{PAGE_W / 2:.3f}" y="{ry - 4:.3f}" font-family="Helvetica,Arial" '
        f'font-size="4" text-anchor="middle" fill="#000">'
        f'100 mm &#8212; measure after printing; must be exact</text>',
    ]

    for i, line in enumerate([
            f'{dict_name} &#183; marker side {size_mm:g} mm INCLUDING the black '
            f'border &#183; cell {cell:.3f} mm',
            f'Print at 100% / &#8220;actual size&#8221;, fit-to-page OFF. '
            f'Matte paper, not glossy.',
            f'Cut on the grey line. Keep the whole white margin &#8212; it is the '
            f'quiet zone and detection needs it.',
            f'Measure a printed marker edge to edge and use THAT number as '
            f'markerLength in the pose code.']):
        out.append(
            f'<text x="{PAGE_W / 2:.3f}" y="{ry + 9 + i * 4.6:.3f}" '
            f'font-family="Helvetica,Arial" font-size="3.3" text-anchor="middle" '
            f'fill="#000">{line}</text>')

    out.append('</svg>')
    return '\n'.join(out), tile, cell


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--size', type=float, default=50.0,
                   help='side of the black square in mm, border included')
    p.add_argument('--quiet', type=float, default=12.0,
                   help='white margin per side in mm (one cell is the minimum)')
    p.add_argument('--ids', default='0,1')
    p.add_argument('--spares', action='store_true', default=True)
    p.add_argument('--no-spares', dest='spares', action='store_false')
    p.add_argument('-o', '--out', default=None)
    a = p.parse_args()

    ids = [int(v) for v in a.ids.split(',')]
    min_quiet = a.size / 6.0
    if a.quiet < min_quiet:
        raise SystemExit(f'--quiet {a.quiet} mm is under one cell '
                         f'({min_quiet:.2f} mm); detection will be unreliable')

    svg, tile, cell = build(ids, a.size, a.quiet, 'DICT_4X4_50',
                            cv2.aruco.DICT_4X4_50, a.spares)
    out = a.out or f'aruco_4x4_{a.size:g}mm.svg'
    with open(out, 'w') as f:
        f.write(svg + '\n')
    print(f'{out}: ids {ids}, marker {a.size:g} mm (cell {cell:.3f} mm), '
          f'tile {tile:g} mm, quiet {a.quiet:g} mm')


if __name__ == '__main__':
    main()
