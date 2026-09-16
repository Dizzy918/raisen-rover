#!/usr/bin/env python3
"""Stereo-pair collector for MATLAB Stereo Camera Calibrator.

WHAT WENT WRONG IN smart_grab6 (captures/round3)
------------------------------------------------
round3 kept 22/22 with zero blur rejections and 8/9 spatial cells. It was still
useless: **0 of 22 pairs detect with PLAIN corner finding**, which is the only
thing that matters, because MATLAB runs its own detector over the pixels you
hand it and applies no contrast enhancement. 21 of 22 detected only after a
CLAHE boost -- and smart_grab6's accept gate used exactly that boost, so it
happily saved pairs MATLAB will silently drop.

Root cause, from the round3 histograms: p5-p95 spanned 1-255 while the board sat
at mean 43-97. The office window blew to saturation, the foreground crushed to
black, and the board was squeezed into a narrow mid band. Two compounding
mistakes:

  1. `analogue_gain` was left at its MINIMUM (100 of 1200) -- 12x of signal
     headroom unused, so the board was underexposed even at exposure 8000.
  2. the contrast stretch used WHOLE-FRAME percentiles, so its limits were set
     by the blown window and the black floor, not by the board.

THE FIX
-------
  * raise `analogue_gain` (default 300). Gain, not exposure -- exposure is in
    sensor row-times (~14.57us here), so buying the same brightness with
    exposure means 230ms+ frames and motion blur. Gain costs noise instead, and
    the 2x2 bin averages 4 samples, halving it back.
  * set the stretch limits from the BOARD REGION of the 16-bit binned frame, not
    the whole frame. The map applied is still a single global affine ramp, so it
    is geometrically neutral -- unlike CLAHE, whose per-tile gains vary across a
    checker edge and bias subpixel corner positions.
  * gate on PLAIN detection of the final 8-bit image, in BOTH eyes. What is
    verified is exactly what gets written, so a saved pair cannot fail in MATLAB.

Measured on round3, restretching the already-quantized 8-bit PNGs from board
percentiles lifted plain detection 0/22 -> 11/22. Working from the 16-bit raw at
capture time has far more range than that re-stretch did.

Kept from earlier versions: one persistent V4L2 stream (repeated open/close
storms corr_err and wedges the Tegra VI channel); exposure set MID-stream
(before streaming it is ignored); rotate each half 180 then SWAP for the
inverted mount; 2x2 Bayer binning instead of demosaicing (a checkerboard aliases
against the GRBG mosaic -- see stereo_calibration/README.md).

SHOOTING NOTE: keep bright windows BEHIND the cameras. No gain setting rescues a
board held against a blown-out window.
"""
import argparse
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
GRID = (9, 6)                          # INNER corners
SHARP_MIN = 25.0
RAW_DUMPS = 2


def sh(*a):
    subprocess.run(a, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ctl(name, val):
    sh('v4l2-ctl', '-d', '/dev/video0', '-c', '%s=%d' % (name, val))


class V4l2Cap:
    """A cv2.VideoCapture stand-in that streams through v4l2-ctl.

    OpenCV could not open this device on jetson11 -- by path, by index and by
    CAP_ANY. That looked like a build limitation and was not: the cause was the
    missing Arducam-patched tegra-camera.ko, and once that was installed OpenCV
    opened it fine. Kept as a fallback anyway: OpenCV is tried first, so this
    costs nothing where it works, and every detection and gating decision below
    is untouched either way.

    read() hands back the same flat uint8 buffer the OpenCV path did, so the
    caller's buf.reshape(-1).view(np.uint16).reshape(H, W) is unchanged.
    """

    NBYTES = W * H * 2

    def __init__(self, frames=200000):
        self.p = subprocess.Popen(
            ['v4l2-ctl', '-d', '/dev/video0',
             '--set-fmt-video=width=%d,height=%d,pixelformat=BA10' % (W, H),
             '--stream-mmap', '--stream-count=%d' % frames, '--stream-to=-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)

    def isOpened(self):
        return self.p.poll() is None

    def read(self):
        # A raw pipe returns short reads, so loop until a whole frame is in.
        buf = b''
        while len(buf) < self.NBYTES:
            chunk = self.p.stdout.read(self.NBYTES - len(buf))
            if not chunk:
                return False, None
            buf += chunk
        return True, np.frombuffer(buf, dtype=np.uint8)

    def release(self):
        try:
            self.p.terminate()
            self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


def bin2x2(half16):
    """2x2 box average of a Bayer half-frame -> float32, still in sensor range.

    All four CFA samples are averaged, not just the two greens: the 4-sample
    centroid sits exactly at the quad centre, whereas a green-only average is
    offset half a pixel diagonally -- a fixed geometric bias on every corner.
    """
    f = half16.astype(np.float32)
    return (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25


def stretch(binned, lo, hi):
    return np.clip((binned - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)


def board_limits(binned, corners, plo=2.0, phi=98.0):
    """Stretch limits from the board's own pixel range, padded ~12% so the
    quiet white margin the detector needs is included."""
    p = corners.reshape(-1, 2)
    x0, y0 = p.min(0)
    x1, y1 = p.max(0)
    mx, my = (x1 - x0) * 0.12, (y1 - y0) * 0.12
    roi = binned[max(0, int(y0 - my)):int(y1 + my),
                 max(0, int(x0 - mx)):int(x1 + mx)]
    if roi.size < 100:
        return np.percentile(binned, 1), np.percentile(binned, 99.5)
    return np.percentile(roi, plo), np.percentile(roi, phi)


def locate(binned, clahe):
    """Find the board well enough to measure its pixel range. CLAHE is allowed
    HERE -- this only decides WHERE the board is, and its output is discarded.
    The accept decision is made later on the plain, globally-stretched image."""
    prov = stretch(binned, np.percentile(binned, 1), np.percentile(binned, 99.5))
    ok, c = cv2.findChessboardCornersSB(prov, GRID)
    if ok:
        return c
    ok, c = cv2.findChessboardCornersSB(clahe.apply(prov), GRID)
    return c if ok else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gain', type=int, default=300,
                    help='analogue_gain 100-1200 (100 = 1x). 300 suits indoor.')
    ap.add_argument('--exposure', type=int, default=8000, help='sensor row-times')
    ap.add_argument('--goal', type=int, default=22)
    ap.add_argument('--seconds', type=int, default=480)
    ap.add_argument('--interval', type=float, default=0.20)
    a = ap.parse_args()

    for d in (LDIR, RDIR):
        os.makedirs(d, exist_ok=True)
        for f in glob.glob(d + '/*'):
            os.remove(f)
    for f in glob.glob(BASE + '/raw_*.npz'):
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
        print('OpenCV will not open /dev/video0 on this machine; streaming '
              'through v4l2-ctl instead.', flush=True)
        cap = V4l2Cap()
    if not cap.isOpened():
        print('cannot open /dev/video0')
        sh('sudo', 'docker', 'start', 'narsil-vision')
        return
    for _ in range(5):
        cap.read()
    ctl('exposure', a.exposure)          # mid-stream, or it is ignored
    ctl('analogue_gain', a.gain)
    time.sleep(0.6)

    print('streaming %dx%d -> %dx%d/eye, exp=%d gain=%d. goal %d, %ds.'
          % (W, H, OUT_W, OUT_H, a.exposure, a.gain, a.goal, a.seconds), flush=True)
    print('Accept gate = PLAIN detection in BOTH eyes, exactly what MATLAB does.',
          flush=True)
    print('Keep bright windows BEHIND the cameras. Corners, tilt 20-40deg, '
          'vary near/far, hold ~1s.', flush=True)

    clahe = cv2.createCLAHE(3.0, (8, 8))
    kept = blurry = noboard = lowcontrast = 0
    last = None
    cells = set()
    fills = []
    t0 = time.time()
    tlast = 0.0
    try:
        while kept < a.goal and (time.time() - t0) < a.seconds:
            if os.path.exists(STOPFILE):
                print('stopped.', flush=True)
                break
            ok, buf = cap.read()
            if not ok or buf is None:
                continue
            now = time.time()
            if now - tlast < a.interval:
                continue
            tlast = now
            try:
                frame = buf.reshape(-1).view(np.uint16).reshape(H, W)
            except Exception:
                continue

            # inverted mount: rotate each half 180, then swap left/right
            Lb = cv2.rotate(bin2x2(frame[:, HALF:]), cv2.ROTATE_180)
            Rb = cv2.rotate(bin2x2(frame[:, :HALF]), cv2.ROTATE_180)

            cL = locate(Lb, clahe)
            if cL is None:
                noboard += 1
                continue
            cR = locate(Rb, clahe)
            if cR is None:
                noboard += 1
                continue

            # board-region limits -> single global affine ramp per eye
            L = stretch(Lb, *board_limits(Lb, cL))
            R = stretch(Rb, *board_limits(Rb, cR))

            if cv2.Laplacian(L, cv2.CV_64F).var() < SHARP_MIN:
                blurry += 1
                continue

            # THE gate: plain detection on the bytes about to be written
            okL, cLf = cv2.findChessboardCornersSB(L, GRID)
            if not okL or not cv2.findChessboardCornersSB(R, GRID)[0]:
                lowcontrast += 1
                if lowcontrast % 10 == 0:
                    print('  (%d rejected: board found but too low-contrast for '
                          'plain detection -- turn away from the window or raise '
                          '--gain)' % lowcontrast, flush=True)
                continue

            pts = cLf.reshape(-1, 2)
            cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
            span = (pts[:, 0].max() - pts[:, 0].min()) / OUT_W
            if (last is not None and abs(cx - last[0]) < 25
                    and abs(cy - last[1]) < 25 and abs(span - last[2]) < 0.06):
                continue

            kept += 1
            last = (cx, cy, span)
            fills.append(span)
            cv2.imwrite('%s/img_%02d.png' % (LDIR, kept), L)
            cv2.imwrite('%s/img_%02d.png' % (RDIR, kept), R)
            if kept <= RAW_DUMPS:
                np.savez_compressed('%s/raw_%02d.npz' % (BASE, kept), bayer=frame)

            col = 'L' if cx < OUT_W * 0.36 else ('R' if cx > OUT_W * 0.64 else 'C')
            row = 'T' if cy < OUT_H * 0.36 else ('B' if cy > OUT_H * 0.64 else 'M')
            cells.add(row + col)
            print('KEPT %2d/%d cell=%s fill=%.0f%% PLAIN-OK (%ds) cells %d/9: %s'
                  % (kept, a.goal, row + col, span * 100, int(time.time() - t0),
                     len(cells), ' '.join(sorted(cells))), flush=True)

        miss = sorted({r + c for r in 'TMB' for c in 'LCR'} - cells)
        print('\nDONE: %d/%d in %ds | no-board=%d low-contrast=%d blur=%d'
              % (kept, a.goal, int(time.time() - t0), noboard, lowcontrast, blurry),
              flush=True)
        print('cells %d/9: %s' % (len(cells), ' '.join(sorted(cells))), flush=True)
        if miss:
            print('cells MISSED: %s' % ' '.join(miss), flush=True)
        if fills:
            print('fill %.0f%%-%.0f%%' % (min(fills) * 100, max(fills) * 100),
                  flush=True)
        if lowcontrast > kept:
            print('NOTE: more frames were rejected for contrast than were kept. '
                  'Turn the rig away from the window, or retry with --gain 500.',
                  flush=True)
    finally:
        ctl('exposure', 700)
        ctl('analogue_gain', 100)
        cap.release()
        sh('sudo', 'docker', 'start', 'narsil-vision')
        print('vision restarted.', flush=True)


if __name__ == '__main__':
    main()
