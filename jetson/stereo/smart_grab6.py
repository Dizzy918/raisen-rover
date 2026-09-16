#!/usr/bin/env python3
"""Stereo-pair collector for MATLAB Stereo Camera Calibrator.

WHY THIS DIFFERS FROM smart_grab5.py
------------------------------------
smart_grab5 debayered each eye to 1920x1200 and verified detection there. That
verification almost never passed, so a full run kept nothing. Measured on the
19 pairs in captures/round2 (sharpness 419-1065, i.e. NOT motion blur):

    full res 1920x1200 demosaiced ...  6/19 detected
    exact 1/2 downscale  960x600  ... 18/19 detected
    0.75 downscale                ... 10/19
    full res + Gaussian sigma 1.0  ...  6/19

Demosaicing is the problem. A checkerboard is the worst case for a CFA: hard
achromatic edges alias against the GRBG mosaic and the interpolator emits
zipper/maze artifacts along exactly the edges the corner detector measures.
Blurring does not fix it (the artifact is structured, not white noise), and a
non-integer downscale does not either (it straddles CFA quads). An exact 2x box
average does fix it, because each 2x2 quad becomes one true luminance sample.

So: never demosaic. Bin each 2x2 GRBG quad to one grey pixel -> 960x600 per eye,
artifact-free, no interpolation. All four samples are averaged (not just the two
greens) because the 4-sample centroid sits exactly at the quad centre; a
green-only average is offset half a pixel diagonally, a fixed geometric bias.

Calibrating at 960x600 is fine. Intrinsics scale linearly to the 1920x1200
pipeline (fx, fy, cx, cy all x2); distortion coefficients are normalized and
carry over unchanged; the baseline and extrinsics are in mm and are
resolution-independent. See ../../stereo_calibration/README.md.

KEPT FROM smart_grab5
---------------------
  * ONE persistent V4L2 stream -- repeated open/close storms corr_err and
    wedges the Tegra VI channel
  * exposure set MID-stream (setting it before streaming is ignored)
  * inverted mount: rotate each half 180 deg, then SWAP (disparity sign verified)
  * no CLAHE on saved images -- CLAHE is for the detector only; baking it in
    adds noise that biases MATLAB's subpixel corner refinement
  * verify in BOTH eyes before saving, at the resolution MATLAB will see
  * pose-diversity gate + 3x3 coverage report
"""
import glob
import os
import subprocess
import time

import cv2
import numpy as np

BASE = os.path.expanduser('~/stereo/matlab_pairs')
LDIR, RDIR = BASE + '/left', BASE + '/right'
STOPFILE = '/tmp/smartgrab.stop'
W, H, HALF = 3840, 1200, 1920          # side-by-side RAW10 GRBG from the UC-512
OUT_W, OUT_H = HALF // 2, H // 2       # 960x600 after 2x2 binning
GOAL = 22
TIME_BUDGET = 480
CHECK_EVERY = 0.20
EXPOSURE = 8000                        # row-times; ~116ms, fine for a still board
GRID = (9, 6)                          # INNER corners
SHARP_MIN = 25.0                       # Laplacian var on the binned image
RAW_DUMPS = 2                          # first N accepted frames also saved as .npz


def sh(*a):
    subprocess.run(a, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def bin2x2(half16):
    """2x2 box average of a Bayer half-frame -> 960x600 uint8 grey.

    Averaged in float32: uint16 RAW10 sums would be fine, but the percentile
    stretch afterwards wants float anyway. The stretch is per-frame contrast
    normalization, not gamma -- it only maps [p1, p99.5] to [0, 255] so the
    detector sees consistent contrast regardless of exposure drift.
    """
    f = half16.astype(np.float32)
    quad = (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25
    lo, hi = np.percentile(quad, 1), np.percentile(quad, 99.5)
    return np.clip((quad - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)


def detect(gray, clahe):
    """Detect on the plain image first, then on a CLAHE copy as a fallback.
    Only the boolean matters here -- MATLAB redoes its own corner finding."""
    if cv2.findChessboardCornersSB(gray, GRID)[0]:
        return cv2.findChessboardCornersSB(gray, GRID)[1]
    ok, corners = cv2.findChessboardCornersSB(clahe.apply(gray), GRID)
    return corners if ok else None


def main():
    for d in (LDIR, RDIR):
        os.makedirs(d, exist_ok=True)
        for f in glob.glob(d + '/*'):
            os.remove(f)
    if os.path.exists(STOPFILE):
        os.remove(STOPFILE)

    # narsil-vision holds /dev/video0; only one consumer can stream at a time.
    sh('sudo', 'docker', 'stop', '-t', '20', 'narsil-vision')
    time.sleep(3)

    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        print('cannot open /dev/video0')
        sh('sudo', 'docker', 'start', 'narsil-vision')
        return
    for _ in range(5):
        cap.read()
    sh('v4l2-ctl', '-d', '/dev/video0', '-c', 'exposure=%d' % EXPOSURE)
    time.sleep(0.5)

    print('streaming %dx%d -> %dx%d/eye, exp=%d. goal %d pairs, %ds.'
          % (W, H, OUT_W, OUT_H, EXPOSURE, GOAL, TIME_BUDGET), flush=True)
    print('Fill the frame corners, tilt 20-40 deg, vary near/far. '
          'Hold ~1s per pose.', flush=True)

    clahe = cv2.createCLAHE(3.0, (8, 8))
    kept = blurry = onlyone = 0
    last = None
    cells = set()
    fills = []
    t0 = time.time()
    tlast = 0.0
    try:
        while kept < GOAL and (time.time() - t0) < TIME_BUDGET:
            if os.path.exists(STOPFILE):
                print('stopped.', flush=True)
                break
            ok, buf = cap.read()
            if not ok or buf is None:
                continue
            now = time.time()
            if now - tlast < CHECK_EVERY:
                continue
            tlast = now
            try:
                frame = buf.reshape(-1).view(np.uint16).reshape(H, W)
            except Exception:
                continue

            # inverted mount: rotate each half 180, then swap left/right
            L = cv2.rotate(bin2x2(frame[:, HALF:]), cv2.ROTATE_180)
            R = cv2.rotate(bin2x2(frame[:, :HALF]), cv2.ROTATE_180)

            if cv2.Laplacian(L, cv2.CV_64F).var() < SHARP_MIN:
                blurry += 1
                continue
            cL = detect(L, clahe)
            if cL is None:
                continue
            if detect(R, clahe) is None:
                onlyone += 1
                continue

            pts = cL.reshape(-1, 2)
            cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
            span = (pts[:, 0].max() - pts[:, 0].min()) / OUT_W
            # reject near-duplicates: the board must have actually moved
            if (last is not None and abs(cx - last[0]) < 25
                    and abs(cy - last[1]) < 25 and abs(span - last[2]) < 0.06):
                continue

            kept += 1
            last = (cx, cy, span)
            fills.append(span)
            cv2.imwrite('%s/img_%02d.png' % (LDIR, kept), L)
            cv2.imwrite('%s/img_%02d.png' % (RDIR, kept), R)
            if kept <= RAW_DUMPS:
                # raw Bayer for offline work (a better demosaic, or a
                # full-res two-stage corner refinement) without recapturing
                np.savez_compressed('%s/raw_%02d.npz' % (BASE, kept), bayer=frame)

            col = 'L' if cx < OUT_W * 0.36 else ('R' if cx > OUT_W * 0.64 else 'C')
            row = 'T' if cy < OUT_H * 0.36 else ('B' if cy > OUT_H * 0.64 else 'M')
            cells.add(row + col)
            print('KEPT %2d/%d cell=%s fill=%.0f%% (%ds) cells %d/9: %s'
                  % (kept, GOAL, row + col, span * 100, int(time.time() - t0),
                     len(cells), ' '.join(sorted(cells))), flush=True)

        miss = sorted({r + c for r in 'TMB' for c in 'LCR'} - cells)
        print('\nDONE: %d/%d in %ds | blur-rejected=%d one-eye-only=%d'
              % (kept, GOAL, int(time.time() - t0), blurry, onlyone), flush=True)
        print('cells %d/9 hit: %s' % (len(cells), ' '.join(sorted(cells))), flush=True)
        if miss:
            print('cells MISSED: %s' % ' '.join(miss), flush=True)
        if fills:
            print('fill %.0f%%-%.0f%%' % (min(fills) * 100, max(fills) * 100), flush=True)
    finally:
        sh('v4l2-ctl', '-d', '/dev/video0', '-c', 'exposure=700')
        cap.release()
        sh('sudo', 'docker', 'start', 'narsil-vision')
        print('vision restarted.', flush=True)


if __name__ == '__main__':
    main()
