#!/usr/bin/env python3
"""Validate the disparity search range against railing, with no ground truth.

WHY THIS EXISTS. A block matcher cannot report a disparity above numDisparities-1,
so anything nearer than fx*baseline/numDisparities gets CLAMPED to that range's
floor and comes back as a plausible-looking depth that is simply the edge of the
search window. That is exactly how an ArUco marker at 0.43 m reported 0.749 m with
numDisparities=128, and the same latent bug sat in mapping/depth_grid.py at 176
(floor 0.495 m) while its own Z_MIN was 0.40 m.

The fix raised depth_grid.py to 224. This checks the fix on one captured frame,
which needs no measured target: compute the SAME frame at several search ranges
and look for two signatures.

  RAILING     the share of valid pixels sitting at the top of the search window.
              A range wide enough for the scene has almost none; a range that is
              too narrow piles pixels against its ceiling.
  AGREEMENT   depth of the same pixels at range N vs a much wider range. If N is
              truncating the scene, its depths disagree with the wider one.

A pass here means the range covers THIS scene. If nothing in the scene is nearer
than a given range's floor, that range cannot be exercised and the run says so
rather than claiming a pass it did not earn.
"""
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

W, H, HALF = 3840, 1200, 1920
CAL = '/home/narsil/vslam-min/calib'
RANGES = [128, 176, 224, 256, 320]
WIDEST = 320


def load(p):
    d = yaml.safe_load(open(p))
    g = lambda k, r, c: np.array(d[k]['data'], float).reshape(r, c)
    return (g('camera_matrix', 3, 3), g('distortion_coefficients', 1, 5),
            g('rectification_matrix', 3, 3), g('projection_matrix', 3, 4))


def bin2x2(h16):
    f = h16.astype(np.float32)
    q = (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25
    lo, hi = np.percentile(q, (1, 99))
    return np.clip((q - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)


KL, DL, R1, P1 = load(CAL + '/acamst_left.yaml')
KR, DR, R2, P2 = load(CAL + '/acamst_right.yaml')
fxb = abs(P2[0, 3])           # fx_rect * baseline, metres * pixels

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
if not cap.isOpened():
    sys.exit('cannot open /dev/video0 -- narsil-vision still holding it?')
for _ in range(5):
    cap.read()
for k, v in (('exposure', 1500), ('analogue_gain', 800)):
    subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c', '%s=%d' % (k, v)],
                   stdout=-3, stderr=-3)
time.sleep(0.8)
# The camera hands back a few black frames after another process releases it;
# keep reading until one has real signal rather than measuring the transient.
raw = None
for _ in range(25):
    ok, buf = cap.read()
    if not ok:
        continue
    r = buf.reshape(-1).view(np.uint16).reshape(H, W)
    if float(r.mean()) > 50.0:
        raw = r
        break
cap.release()
if raw is None:
    sys.exit('only black frames -- camera did not wake up')
print('raw frame mean %.0f counts' % raw.mean())

L = cv2.rotate(bin2x2(raw[:, HALF:]), cv2.ROTATE_180)
Rg = cv2.rotate(bin2x2(raw[:, :HALF]), cv2.ROTATE_180)
sz = (L.shape[1], L.shape[0])
mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, sz, cv2.CV_32FC1)
mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, sz, cv2.CV_32FC1)
Lr = cv2.remap(L, *mL, cv2.INTER_LINEAR)
Rr = cv2.remap(Rg, *mR, cv2.INTER_LINEAR)


def disparity(nd):
    sg = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=nd, blockSize=5, P1=8 * 25, P2=32 * 25,
        disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=100,
        speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    return sg.compute(Lr, Rr).astype(np.float32) / 16.0


ref = disparity(WIDEST)
ref_ok = ref > 0.5
ref_z = np.where(ref_ok, fxb / np.maximum(ref, 1e-6), 0.0)
near = ref_z[ref_ok]
print('scene depth from the widest range (%d): 1st pct %.3f m, median %.3f m, '
      'nearest %.3f m' % (WIDEST, np.percentile(near, 1), np.median(near),
                          near.min()))
print('fx*baseline = %.3f m*px, so range floor = %.3f / numDisparities\n' % (fxb, fxb))

print('%5s %9s %9s %9s %9s  %s' % ('range', 'floor_m', 'valid%', 'railed%',
                                   'agree_mm', 'verdict'))
rows = []
for nd in RANGES:
    d = disparity(nd)
    ok = d > 0.5
    floor = fxb / nd
    railed = ok & (d >= nd - 1.5)
    both = ok & ref_ok
    z = fxb / np.maximum(d, 1e-6)
    dz = np.abs(z[both] - ref_z[both]) * 1000.0
    agree = float(np.median(dz)) if both.sum() else float('nan')
    rail_pct = 100.0 * railed.sum() / max(ok.sum(), 1)
    # Can this range even be exercised? Only if the scene has content nearer
    # than its floor, as judged by the widest range.
    exercised = float((near < floor).sum()) / max(len(near), 1) > 0.001
    if nd == WIDEST:
        verdict = 'reference'
    elif not exercised:
        verdict = 'not exercised (nothing nearer than its floor)'
    elif rail_pct < 0.5 and agree < 5.0:
        verdict = 'OK for this scene'
    else:
        verdict = 'TRUNCATING'
    print('%5d %9.3f %9.1f %9.2f %9.1f  %s'
          % (nd, floor, 100.0 * ok.sum() / ok.size, rail_pct, agree, verdict))
    rows.append((nd, floor, rail_pct, agree, exercised))

print('\nWHAT THIS RUN CAN AND CANNOT CONCLUDE')
ex = [r for r in rows if r[4] and r[0] != WIDEST]
if not ex:
    print('  INCONCLUSIVE. Nothing in this scene is near enough to reach any')
    print('  range floor, so no range could be caught truncating. Put something')
    print('  textured at 0.4-0.6 m and re-run.')
else:
    print('  Exercised: %s' % ', '.join(str(r[0]) for r in ex))
    bad = [r[0] for r in ex if r[2] >= 0.5 or r[3] >= 5.0]
    print('  Truncating: %s' % (', '.join(str(b) for b in bad) if bad else 'none'))
    g = [r for r in rows if r[0] == 224]
    if g and g[0][4]:
        print('  depth_grid.py at 224 (floor %.3f m, Z_MIN 0.40): %s'
              % (g[0][1], 'covered' if g[0][2] < 0.5 else 'STILL TRUNCATING'))
