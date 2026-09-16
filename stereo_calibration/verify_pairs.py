#!/usr/bin/env python3
"""Check a captured stereo set at FULL resolution before feeding it to MATLAB.

MATLAB's Stereo Camera Calibrator silently drops any pair it cannot detect in
both eyes, so a set that "looks fine" can quietly calibrate on 6 of 20 pairs
and still report a flattering RMS. Run this first: it reports per-pair
detection, sharpness, and the spatial/scale coverage of the board across the
frame -- coverage is what actually constrains distortion and the baseline.

Usage:
    python3 verify_pairs.py captures/round3
    python3 verify_pairs.py captures/round3 --grid 9x6
"""
import argparse
import os
import sys

import cv2
import numpy as np


def detect(gray, grid):
    """PLAIN detection -- this is the pass/fail criterion, because it is what
    MATLAB does. MATLAB runs its own corner finder on the pixels you hand it and
    applies no contrast enhancement, so a pair that only detects after a CLAHE
    boost is a pair MATLAB will drop.

    An earlier version of this script returned the CLAHE result as success. It
    reported 21/22 usable on captures/round3; the true plain-detection figure
    was 0/22. Do not reintroduce that fallback here."""
    return cv2.findChessboardCornersSB(gray, grid)


def detect_boosted(gray, grid):
    """CLAHE-assisted, reported for DIAGNOSIS only: plain-fail + boosted-pass
    means the board is present and sharp but under-contrasted, which is a
    capture-exposure problem, not a bad pose."""
    return cv2.findChessboardCornersSB(
        cv2.createCLAHE(3.0, (8, 8)).apply(gray), grid)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('dir', help='directory holding left/ and right/')
    p.add_argument('--grid', default='9x6', help='inner corners, e.g. 9x6')
    p.add_argument('--sharp-min', type=float, default=45.0,
                   help='Laplacian-variance floor (on a 768x480 downscale)')
    a = p.parse_args()
    grid = tuple(int(v) for v in a.grid.lower().split('x'))

    ldir, rdir = os.path.join(a.dir, 'left'), os.path.join(a.dir, 'right')
    if not os.path.isdir(ldir) or not os.path.isdir(rdir):
        sys.exit(f'need {ldir} and {rdir}')

    names = sorted(set(os.listdir(ldir)) & set(os.listdir(rdir)))
    names = [n for n in names if not n.startswith('.')]
    lonely = sorted(set(os.listdir(ldir)) ^ set(os.listdir(rdir)))
    if not names:
        sys.exit('no matching filenames in left/ and right/')

    good, centres, fills = [], [], []
    n_lowcontrast = 0
    print(f'{len(names)} pairs, grid {grid[0]}x{grid[1]}\n')
    print(f'{"pair":<16}{"L":<5}{"R":<5}{"sharp":>8}  note')
    for n in names:
        L = cv2.imread(os.path.join(ldir, n), cv2.IMREAD_GRAYSCALE)
        R = cv2.imread(os.path.join(rdir, n), cv2.IMREAD_GRAYSCALE)
        if L is None or R is None:
            print(f'{n:<16}{"-":<5}{"-":<5}{"-":>8}  unreadable')
            continue
        small = cv2.resize(L, (768, 480))
        sharp = cv2.Laplacian(small, cv2.CV_64F).var()
        okL, cL = detect(L, grid)
        okR, _ = detect(R, grid)
        note = ''
        if okL and okR:
            good.append(n)
            pts = cL.reshape(-1, 2)
            centres.append((pts[:, 0].mean() / L.shape[1], pts[:, 1].mean() / L.shape[0]))
            fills.append((pts[:, 0].max() - pts[:, 0].min()) / L.shape[1])
            if sharp < a.sharp_min:
                note = f'soft (<{a.sharp_min:g})'
        else:
            note = 'DROPPED by MATLAB'
            # board present but under-contrasted -> fix exposure, not the pose
            if (detect_boosted(L, grid)[0] and detect_boosted(R, grid)[0]):
                note += ' -- low contrast (detects only after CLAHE)'
                n_lowcontrast += 1
        print(f'{n:<16}{"ok" if okL else "MISS":<5}{"ok" if okR else "MISS":<5}'
              f'{sharp:>8.0f}  {note}')

    n_ok = len(good)
    print(f'\nusable in BOTH eyes (plain detection, as MATLAB sees it): '
          f'{n_ok}/{len(names)}')
    if n_lowcontrast:
        print(f'of the rest, {n_lowcontrast} hold a detectable board but are too '
              f'low-contrast --\n  raise analogue_gain and keep bright windows '
              f'BEHIND the cameras; do not\n  paper over it with CLAHE, which '
              f'biases subpixel corner positions.')
    if lonely:
        print(f'unpaired filenames (ignored): {", ".join(lonely[:6])}')

    if n_ok:
        # 3x3 spatial coverage: corners of the frame are where distortion lives,
        # so a set clustered in the centre gives you a good RMS and bad undistort.
        cells = set()
        for cx, cy in centres:
            col = 'L' if cx < 0.36 else ('R' if cx > 0.64 else 'C')
            row = 'T' if cy < 0.36 else ('B' if cy > 0.64 else 'M')
            cells.add(row + col)
        print(f'spatial cells hit: {len(cells)}/9  {" ".join(sorted(cells))}')
        print(f'board fill: min {min(fills) * 100:.0f}%  max {max(fills) * 100:.0f}%  '
              f'(want some <25% far and some >45% near)')

        missing = sorted({r + c for r in 'TMB' for c in 'LCR'} - cells)
        if missing:
            print(f'MISSING cells: {" ".join(missing)}')

    print()
    if n_ok < 15:
        print('VERDICT: not enough. Recapture -- MATLAB wants 15-20+ usable pairs.')
    elif len(cells) < 7:
        print('VERDICT: enough pairs but thin coverage. Add poses in the missing cells.')
    elif max(fills) - min(fills) < 0.2:
        print('VERDICT: coverage ok but the board sits at one distance. '
              'Add near and far poses so focal length separates from depth.')
    else:
        print('VERDICT: good set. Load into MATLAB Stereo Camera Calibrator.')
        print('         Square size = the value you MEASURED with a ruler '
              '(25.6 mm for the\n         board in use, not the 25 mm nominal) '
              '-- the baseline scales with it.')


if __name__ == '__main__':
    main()
