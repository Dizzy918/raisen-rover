#!/usr/bin/env python3
"""Check stereo depth against a tape measurement, on a target at a known distance.

    python3 measure_target.py --distance 1.00

Run it at several distances. Depth error is predicted to grow with the square of
range, because depth = fx*baseline/disparity means a fixed disparity error maps to
z^2/(fx*baseline) of depth error. With fx*baseline = 87.12 and a quarter pixel of
matching noise that is 3 mm at 1 m, 11 mm at 2 m, 46 mm at 4 m. This measures
whether reality agrees.

WHY SEVERAL DISTANCES AND NOT ONE. A tape reaches the front of the lens; the
optical centre sits a few mm further back, and that datum offset is unknown. One
distance cannot separate it from a genuine scale error, but three can: fit
measured against true, and the offset is the INTERCEPT while a scale error is the
SLOPE. So approximate positions with accurately reported distances beat carefully
set positions with a guessed datum.

If a checkerboard is in view it also reports, from the same frame:

  SQUARE SIZE   solvePnP's translation scales linearly with the assumed square
                size, so a known true distance back-solves the real size. This is
                how an uncalibrated board becomes a calibrated one.

  PITCH         a board flush against a PLUMB wall has a horizontal normal. Its
                apparent tilt is therefore the camera's pitch: with the normal
                pointing away from the camera as (0, sin p, cos p) in the optical
                frame, p = atan2(n_y, n_z). This is independent of both the floor
                plane fit and the board-on-floor pose, so it is a real third
                opinion -- as long as the wall is actually plumb and the board
                actually flat against it.
"""
import argparse
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

W, H, HALF = 3840, 1200, 1920
GRIDS = [(7, 6), (9, 6), (8, 6), (7, 7), (6, 6)]


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, default=0.0,
                    help='tape distance, metres. Optional: when the board is '
                         'visible its solved pose is a BETTER reference than a '
                         'handheld tape, so this is only a cross-check.')
    ap.add_argument('--square-mm', type=float, default=25.65,
                    help='MEASURED square size of the board in view')
    ap.add_argument('--calib', default='/home/narsil/vslam-min/calib')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--exposure', type=int, default=1500)
    ap.add_argument('--gain', type=int, default=800)
    ap.add_argument('--out', default='/tmp/target')
    a = ap.parse_args()
    subprocess.run(['mkdir', '-p', a.out])

    KL, DL, R1, P1 = load(a.calib + '/acamst_left.yaml')
    KR, DR, R2, P2 = load(a.calib + '/acamst_right.yaml')
    fx, cx, cy = P1[0, 0], P1[0, 2], P1[1, 2]
    base = abs(P2[0, 3]) / fx
    K = P1[:3, :3]

    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        sys.exit('cannot open %s' % a.device)
    for _ in range(5):
        cap.read()
    for k, v in (('exposure', a.exposure), ('analogue_gain', a.gain)):
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
    R = cv2.rotate(bin2x2(raw[:, :HALF]), cv2.ROTATE_180)
    sz = (L.shape[1], L.shape[0])
    mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, sz, cv2.CV_32FC1)
    mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, sz, cv2.CV_32FC1)
    Lr = cv2.remap(L, *mL, cv2.INTER_LINEAR)
    Rr = cv2.remap(R, *mR, cv2.INTER_LINEAR)
    cv2.imwrite(a.out + '/left.png', Lr)

    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=256, blockSize=5,
                               P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
                               uniquenessRatio=10, speckleWindowSize=100,
                               speckleRange=2,
                               mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sg.compute(Lr, Rr).astype(np.float32) / 16.0

    if a.distance:
        print('tape distance     %.4f m' % a.distance)
    print('depth model       %.3f / disparity   (fx*baseline)' % (fx * base))
    print('board squares     %.2f mm (measured)' % a.square_mm)

    # Central patch, avoiding the floor in the lower third
    h, w = disp.shape
    patch = disp[int(h * 0.20):int(h * 0.60), int(w * 0.35):int(w * 0.65)]
    good = patch[patch > 0.5]
    print('\nSTEREO, central patch')
    if len(good) < 200:
        print('  only %d valid disparities -- target too textureless' % len(good))
    else:
        d_med = float(np.median(good))
        z = fx * base / d_med
        p16, p84 = np.percentile(good, (16, 84))
        print('  valid px        %d' % len(good))
        print('  disparity       median %.2f px  (16-84%%: %.2f to %.2f)'
              % (d_med, p16, p84))
        print('  DEPTH           %.4f m' % z)
        print('  error vs tape   %+.1f mm  (%+.2f%%)'
              % (1000 * (z - a.distance), 100 * (z - a.distance) / a.distance))
        print('  NB the 16-84%% disparity range spans %.2f to %.2f m -- that is '
              'the patch containing background as well as target, i.e. scene '
              'depth range, NOT measurement noise'
              % (fx * base / max(p84, 1e-6), fx * base / max(p16, 1e-6)))

    print('\nCHECKERBOARD')
    got = None
    for grid in GRIDS:
        ok2, c = cv2.findChessboardCornersSB(
            Lr, grid, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if not ok2:
            continue
        objp = np.zeros((grid[0] * grid[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:grid[0], 0:grid[1]].T.reshape(-1, 2)
        objp *= a.square_mm / 1000.0
        co = c.reshape(-1, 2).astype(np.float32)
        ok3, rvec, tvec = cv2.solvePnP(objp, co, K, np.zeros(5))
        if not ok3:
            continue
        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, np.zeros(5))
        err = float(np.sqrt(((proj.reshape(-1, 2) - co) ** 2).sum(1).mean()))
        got = (grid, co, rvec, tvec, err)
        print('  grid %s found, reprojection %.2f px' % (str(grid), err))
        break
    if got is None:
        print('  no board detected')
    else:
        grid, co, rvec, tvec, err = got
        Rm, _ = cv2.Rodrigues(rvec)
        n = Rm[:, 2]
        if n[2] < 0:
            n = -n                       # point the normal AWAY from the camera
        z_pnp = float(tvec.ravel()[2])
        print('  BOARD DISTANCE  %.4f m   <- absolute, from %.2f mm squares'
              % (z_pnp, a.square_mm))
        if a.distance:
            print('  vs tape         %+.1f mm  (tape datum and technique, not a '
                  'camera error)' % (1000 * (z_pnp - a.distance)))
        pitch = np.degrees(np.arctan2(n[1], n[2]))
        yaw = np.degrees(np.arctan2(n[0], n[2]))
        print('  board normal    [%+.4f %+.4f %+.4f]' % tuple(n))
        print('  CAMERA PITCH    %+.2f deg   <- valid only if the wall is plumb '
              'and the board flat on it' % pitch)
        print('  board yaw       %+.2f deg   (how squarely it faces the camera)'
              % yaw)
        vis = cv2.cvtColor(Lr, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(vis, grid, co.reshape(-1, 1, 2), True)
        cv2.imwrite(a.out + '/board.png', vis)

        # Stereo depth sampled at the detected corners: a like-for-like comparison
        # against PnP on the very same points.
        zs = []
        for x, y in co:
            d = disp[int(round(y)), int(round(x))]
            if d > 0.5:
                zs.append(fx * base / d)
        if zs:
            zc = float(np.median(zs))
            print('  stereo at the %d corners: median %.4f m' % (len(zs), zc))
            print('  STEREO vs BOARD  %+.1f mm  (%+.3f%%)  <- the real accuracy '
                  'number, two independent optical methods on the same points'
                  % (1000 * (zc - z_pnp), 100 * (zc - z_pnp) / z_pnp))
    print('\nimages in', a.out)


if __name__ == '__main__':
    main()
