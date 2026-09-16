#!/usr/bin/env python3
"""Verify base_link -> left_cam without moving the rover.

Whether "forward" is +X needs driving to confirm end to end, but the transform
itself can be checked statically, and this is the check that would catch a sign
error or a transposed matrix:

  TRANSFORM THE FLOOR INTO base_link AND SEE WHERE IT LANDS. If the extrinsics
  are right, floor points come out at z = 0 with a normal of [0, 0, 1]. A
  transposed rotation, a wrong sign, or a mixed-up axis all move the floor
  somewhere else, visibly.

This is not circular even though the transform was built from a floor fit: that
fit used one frame, this uses a fresh capture, and it checks the whole path
including the quaternion conversion, which the fit never touched. A transpose bug
in quat_from_matrix would sail through the original measurement and fail here.
"""
import argparse
import subprocess
import sys
import time

import camera_extrinsics as extrinsics
import cv2
import numpy as np
import yaml

W, H, HALF = 3840, 1200, 1920


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


def quat_to_matrix(q):
    """Independent inverse of quat_from_matrix, so a bug in one shows up here."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', default='/home/narsil/vslam-min/calib')
    ap.add_argument('--extrinsics', default=extrinsics.DEFAULT_PATH)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--near', type=float, default=0.35)
    ap.add_argument('--far', type=float, default=1.50)
    a = ap.parse_args()

    height, R, q, pitch, roll = extrinsics.load(a.extrinsics)
    print('extrinsics: height %.4f m, pitch %+.2f, roll %+.2f' % (height, pitch, roll))

    R_back = quat_to_matrix(q)
    err = float(np.abs(R_back - R).max())
    print('quaternion round trip: max elementwise error %.2e  %s'
          % (err, 'OK' if err < 1e-6 else 'FAIL -- quat conversion is wrong'))

    KL, DL, R1, P1 = load(a.calib + '/acamst_left.yaml')
    KR, DR, R2, P2 = load(a.calib + '/acamst_right.yaml')
    fx, cx, cy = P1[0, 0], P1[0, 2], P1[1, 2]
    base = abs(P2[0, 3]) / fx

    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        sys.exit('cannot open %s' % a.device)
    for _ in range(5):
        cap.read()
    for k, v in (('exposure', 1500), ('analogue_gain', 800)):
        subprocess.run(['v4l2-ctl', '-d', a.device, '-c', '%s=%d' % (k, v)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.8)
    ok = False
    for _ in range(10):
        ok, buf = cap.read()
    cap.release()
    if not ok:
        sys.exit('read failed')
    raw = buf.reshape(-1).view(np.uint16).reshape(H, W)

    L = cv2.rotate(bin2x2(raw[:, HALF:]), cv2.ROTATE_180)
    Rg = cv2.rotate(bin2x2(raw[:, :HALF]), cv2.ROTATE_180)
    sz = (L.shape[1], L.shape[0])
    mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, sz, cv2.CV_32FC1)
    mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, sz, cv2.CV_32FC1)
    Lr = cv2.remap(L, *mL, cv2.INTER_LINEAR)
    Rr = cv2.remap(Rg, *mR, cv2.INTER_LINEAR)

    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=256, blockSize=5,
                               P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
                               uniquenessRatio=10, speckleWindowSize=100,
                               speckleRange=2,
                               mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sg.compute(Lr, Rr).astype(np.float32) / 16.0
    Z = np.where(disp > 0.5, fx * base / np.maximum(disp, 1e-6), 0.0)
    ys, xs = np.nonzero((Z > a.near) & (Z < a.far))
    if len(ys) < 500:
        sys.exit('only %d near-field points' % len(ys))
    z = Z[ys, xs]
    opt = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fx, z], 1)

    # optical -> base_link
    pts = opt @ R.T + np.array([0.0, 0.0, height])

    # RANSAC, not a slab plus least squares. The first version of this check took
    # every point within 8 cm of z=0 and fitted them directly, which swept in desk
    # feet and cables lying on the floor -- 485 points of mixed origin, 7.8 mm rms,
    # and a bogus 4.13 deg residual tilt. The height was unaffected (-0.8 mm)
    # because offsets average out while orientation does not.
    slab = pts[np.abs(pts[:, 2]) < 0.10]
    floor = slab
    if len(slab) > 200:
        rng = np.random.default_rng(0)
        best = (None, None, -1)
        for _ in range(1500):
            p3 = slab[rng.integers(0, len(slab), 3)]
            nn = np.cross(p3[1] - p3[0], p3[2] - p3[0])
            ln = np.linalg.norm(nn)
            if ln < 1e-9:
                continue
            nn = nn / ln
            if nn[2] < 0:
                nn = -nn
            if nn[2] < np.cos(np.radians(20)):
                continue
            dd = -nn @ p3[0]
            cnt = int((np.abs(slab @ nn + dd) < 0.008).sum())
            if cnt > best[2]:
                best = (nn, dd, cnt)
        if best[0] is not None:
            floor = slab[np.abs(slab @ best[0] + best[1]) < 0.008]
    print('\n%d near-field points, %d in the floor slab, %d RANSAC inliers'
          % (len(pts), len(slab), len(floor)))
    if len(floor) < 200:
        print('FAIL: the floor did not land near z=0. Distribution of z:')
        for p in (1, 10, 25, 50, 75, 90, 99):
            print('   p%-2d  %+.3f m' % (p, np.percentile(pts[:, 2], p)))
        sys.exit(1)

    c = floor.mean(0)
    w, v = np.linalg.eigh(np.cov((floor - c).T))
    n = v[:, 0] / np.linalg.norm(v[:, 0])
    if n[2] < 0:
        n = -n
    tilt = np.degrees(np.arccos(min(1.0, abs(n[2]))))
    print('\nFLOOR IN base_link')
    print('  mean z          %+.4f m   (want 0)' % c[2])
    print('  median z        %+.4f m' % np.median(floor[:, 2]))
    print('  normal          [%+.4f %+.4f %+.4f]   (want [0 0 1])' % tuple(n))
    print('  residual tilt   %.2f deg   (want 0)' % tilt)
    print('  plane rms       %.1f mm' % (1000 * np.sqrt(max(w[0], 0.0))))
    print('  x range         %.2f to %.2f m   (forward)' % (pts[:, 0].min(), pts[:, 0].max()))
    print('  y range         %.2f to %.2f m   (left)' % (pts[:, 1].min(), pts[:, 1].max()))

    good = abs(c[2]) < 0.02 and tilt < 1.0
    print('\nVERDICT: %s' % ('transform is correct -- the floor lands flat at z=0'
                             if good else
                             'SUSPECT -- floor is offset or tilted in base_link'))
    print('\nNOTE: this confirms height and orientation. Whether driving forward '
          'increases X still needs one short drive to confirm.')


if __name__ == '__main__':
    main()
