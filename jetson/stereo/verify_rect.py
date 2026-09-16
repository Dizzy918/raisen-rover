#!/usr/bin/env python3
"""Rectify a live pair with the converted calibration and measure row alignment.

This is the only check that actually tests the crop/invert/swap reasoning end to
end. If any of the three is wrong the RMS in the file still looks fine, but
rectified rows stop lining up -- a point on a horizontal line in the left image
no longer sits on that line in the right.

It also settles the one thing the YAML files cannot tell us: WHICH sensor half
the bench tool called "left". Both assignments are tried; the correct one gives
sub-pixel median |dy| and the wrong one does not come close.
"""
import os, subprocess, sys
import numpy as np, cv2, yaml

W, H, HALF = 3840, 1200, 1920
EW, EH = 1920, 1200
CAL = os.path.expanduser('~/stereo/cal_j11')


def m(d):
    return np.array(d['data'], float).reshape(d['rows'], d['cols'])


def load(side):
    c = yaml.safe_load(open('%s/acamst_%s.yaml' % (CAL, side)))
    return (m(c['camera_matrix']),
            np.array(c['distortion_coefficients']['data'], float).reshape(1, -1),
            m(c['rectification_matrix']), m(c['projection_matrix']))


def grab(frames=10):
    p = '/tmp/rect.raw'
    if os.path.exists(p):
        os.unlink(p)
    subprocess.run(['v4l2-ctl', '-d', '/dev/video0',
                    '--set-fmt-video=width=%d,height=%d,pixelformat=BA10' % (W, H),
                    '--stream-mmap', '--stream-count=%d' % frames,
                    '--stream-to=' + p],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90)
    nb = W * H * 2
    n = os.path.getsize(p) // nb
    if n < 1:
        sys.exit('grab failed')
    with open(p, 'rb') as f:
        f.seek((n - 1) * nb)
        return np.frombuffer(f.read(nb), np.uint16).reshape(H, W)


def grey(half16):
    a = half16.astype(np.float32)
    lo, hi = np.percentile(a, 1), np.percentile(a, 99.5)
    g8 = np.clip((a - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)
    return cv2.cvtColor(g8, cv2.COLOR_BayerGR2GRAY)


KL, DL, RL, PL = load('left')
KR, DR, RR, PR = load('right')
mlx, mly = cv2.initUndistortRectifyMap(KL, DL, RL, PL, (EW, EH), cv2.CV_32FC1)
mrx, mry = cv2.initUndistortRectifyMap(KR, DR, RR, PR, (EW, EH), cv2.CV_32FC1)

frame = grab()
rot = lambda x: cv2.rotate(x, cv2.ROTATE_180)
sl, sr = grey(frame[:, :HALF]), grey(frame[:, HALF:])

try:
    det = cv2.SIFT_create(nfeatures=4000)
    norm = cv2.NORM_L2
except Exception:
    det = cv2.ORB_create(nfeatures=4000)
    norm = cv2.NORM_HAMMING

def measure(Limg, Rimg):
    lr = cv2.remap(Limg, mlx, mly, cv2.INTER_LINEAR)
    rr = cv2.remap(Rimg, mrx, mry, cv2.INTER_LINEAR)
    k1, d1 = det.detectAndCompute(lr, None)
    k2, d2 = det.detectAndCompute(rr, None)
    if d1 is None or d2 is None or len(k1) < 20 or len(k2) < 20:
        return None
    bf = cv2.BFMatcher(norm, crossCheck=True)
    ms = bf.match(d1, d2)
    if len(ms) < 20:
        return None
    dy = np.array([k1[x.queryIdx].pt[1] - k2[x.trainIdx].pt[1] for x in ms])
    dx = np.array([k1[x.queryIdx].pt[0] - k2[x.trainIdx].pt[0] for x in ms])
    # keep plausible stereo matches: rows should agree, so trim the tail
    keep = np.abs(dy - np.median(dy)) < 12
    dy, dx = dy[keep], dx[keep]
    return len(dy), float(np.median(np.abs(dy))), float(np.percentile(np.abs(dy), 95)), \
        float(np.median(dx)), float((dx > 0).mean())

print('assignment                                    n   med|dy|  p95|dy|  med dx  dx>0')
for nm, Limg, Rimg in (
        ('(a) L=rot(sensor RIGHT), R=rot(sensor LEFT)', rot(sr), rot(sl)),
        ('(b) L=rot(sensor LEFT),  R=rot(sensor RIGHT)', rot(sl), rot(sr))):
    r = measure(Limg, Rimg)
    if r is None:
        print('%-44s  too few matches' % nm)
        continue
    print('%-44s %4d  %7.3f  %7.3f  %7.2f  %4.0f%%'
          % (nm, r[0], r[1], r[2], r[3], r[4] * 100))
print('\nCorrect assignment: sub-pixel med|dy| and dx>0 for nearly every match.')
print('jetson9 for reference: mean 0.108 px, p95 0.343 px.')
