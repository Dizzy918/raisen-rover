#!/usr/bin/env python3
"""Measure where the 2560x720 sensor mode sits inside the 3840x1200 one.

The calibration arrived at 1280x720 per eye, but the pipeline profile is
1920x1200 per eye. fx at 1280x720 matches jetson9's fx at 1920x1200 to 0.2%,
which says 2560x720 is a CROP at native pixel pitch rather than a downscale --
so fx,fy carry over untouched and only the principal point moves. This measures
by how much, instead of assuming the crop is centred.

Method: grab the same static scene in both modes back to back and locate the
smaller frame inside the larger by normalised cross-correlation at 1:1 scale. A
sharp peak confirms the crop hypothesis; the peak position IS the offset. Run on
the raw Bayer, so the answer also reveals whether the offset is even and the CFA
phase therefore preserved.
"""
import subprocess, sys, os
import numpy as np
import cv2

DEV = '/dev/video0'


def grab(w, h, path, frames=10):
    if os.path.exists(path):
        os.unlink(path)
    cmd = ['v4l2-ctl', '-d', DEV,
           '--set-fmt-video=width=%d,height=%d,pixelformat=BA10' % (w, h),
           '--stream-mmap', '--stream-count=%d' % frames, '--stream-to=' + path]
    r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=60)
    nb = w * h * 2
    got = os.path.getsize(path) // nb
    if got < 1:
        sys.exit('grab %dx%d failed: %s' % (w, h, r.stderr.decode()[:200]))
    with open(path, 'rb') as f:
        f.seek((got - 1) * nb)
        buf = f.read(nb)
    return np.frombuffer(buf, np.uint16).reshape(h, w)


def norm(a):
    a = a.astype(np.float32)
    return (a - a.mean()) / (a.std() + 1e-6)


big = grab(3840, 1200, '/tmp/big.raw')
small = grab(2560, 720, '/tmp/small.raw')
print('grabbed 3840x1200 mean=%.0f and 2560x720 mean=%.0f' % (big.mean(), small.mean()))

for eye, (b0, b1, s0, s1) in (('LEFT half', (0, 1920, 0, 1280)),
                              ('RIGHT half', (1920, 3840, 1280, 2560))):
    B = norm(big[:, b0:b1])
    S = norm(small[:, s0:s1])
    # trim the template so a search margin exists in both axes
    t = S[40:-40, 40:-40]
    res = cv2.matchTemplate(B, t, cv2.TM_CCOEFF_NORMED)
    mn, mx, mnl, mxl = cv2.minMaxLoc(res)
    ox, oy = mxl[0] - 40, mxl[1] - 40
    # peak sharpness: best score vs the 99.9th percentile of the rest
    others = np.partition(res.ravel(), -50)[:-50]
    print('%-11s peak corr %.4f at offset x=%+d y=%+d   (centred would be '
          'x=+320 y=+240);  peak/bg = %.2f'
          % (eye, mx, ox, oy, mx / (others.std() + 1e-6)))
    print('%-11s offset parity: x %s, y %s -> CFA phase %s'
          % ('', 'even' if ox % 2 == 0 else 'ODD',
             'even' if oy % 2 == 0 else 'ODD',
             'preserved' if (ox % 2 == 0 and oy % 2 == 0) else 'SHIFTED'))
