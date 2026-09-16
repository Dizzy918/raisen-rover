#!/usr/bin/env python3
"""Measure the camera's height above the floor and its pitch and roll.

This is the camera-to-ROVER calibration. Stereo calibration gives camera-to-
camera; without this, anything projecting depth into a rover-frame map is using
a guess. The height was a tape measurement (15.2 cm) and the pitch was chosen by
eye ("a little bit upwards"), and at 4 m a 1.25 deg pitch error is 87 mm of
height error -- larger than the 60 mm band the occupancy grid classifies floor
against.

    python3 measure_floor.py                    # plane fit only
    python3 measure_floor.py --board            # also try the checkerboard

Two independent methods, because either alone can be confidently wrong:

  A  PLANE FIT on near-field stereo, 0.3 to 1.5 m. No board needed. An earlier
     attempt fit over the whole depth range and failed: at 4 m a quarter pixel of
     disparity noise is 46 mm, so distant points dominate the residual and drag
     the normal around. Near field is both where stereo is precise (3-10 mm) and
     the only part of the floor relevant to the camera's own height.

     THE FIT MUST BE CONSTRAINED TO NEARLY-VERTICAL NORMALS. Unconstrained, it
     happily returns the largest plane in view -- measured once as a box front
     45 cm away, reported as "floor" with a 1.4 mm rms and a pitch of -87 deg.
     A tight rms means the points really are coplanar; it says nothing about
     whether they are the floor.

  B  CHECKERBOARD lying on the floor. Its solved pose IS the floor plane, with no
     plane-fitting assumption. The catch is geometric: at 15 cm camera height a
     board 1 m away is seen at 8.6 deg grazing, which squashes a 25.6 mm square
     to about 2 px tall. Even at 45 cm it is only ~9 px per square vertically,
     and findChessboardCornersSB fails. Hence the upscaled retry, which is worth
     a try but not worth relying on.

     THE BOARD MUST LIE DIRECTLY ON THE FLOOR. Taped to a box, its plane is the
     box lid, offset from the floor by an unmeasured amount.

Camera frame: X right, Y down, Z forward. Upward floor normal is (0,-1,0) for a
level camera, so pitch = atan2(n_z, -n_y) and roll = atan2(n_x, -n_y).
"""
import argparse
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

W, H, HALF = 3840, 1200, 1920
GRID = (9, 6)
MAX_TILT_DEG = 25.0     # candidate planes beyond this from vertical are not floor


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


def angles(n):
    """Pitch and roll of the camera, from an upward-pointing floor normal."""
    return (np.degrees(np.arctan2(n[2], -n[1])),
            np.degrees(np.arctan2(n[0], -n[1])))


def grab(dev, exposure, gain):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        sys.exit('cannot open %s -- narsil-vision or the live feed still has it'
                 % dev)
    for _ in range(5):
        cap.read()
    for k, v in (('exposure', exposure), ('analogue_gain', gain)):
        subprocess.run(['v4l2-ctl', '-d', dev, '-c', '%s=%d' % (k, v)],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.8)
    ok = False
    for _ in range(10):
        ok, buf = cap.read()
    cap.release()
    if not ok:
        sys.exit('read failed')
    return buf.reshape(-1).view(np.uint16).reshape(H, W)


def plane_fit(pts, iters=3000, tol=0.005, seed=0,
               min_h=0.10, max_h=0.30, min_inliers=300, max_planes=4):
    """Find the floor among the several horizontal planes this floor produces.

    A SINGLE RANSAC IS NOT ENOUGH HERE, and both failure modes were measured:

      tol=10 mm  fits ONE plane through the floor AND whatever sits below it,
                 because a large minority of points land under the real surface.
                 Gave 12% inliers, 27.8 cm, -8.6 deg. (Why they land there is
                 unresolved -- see vslam/subfloor_check.py. It is not the floor
                 being polished; it is not polished.)
      tol=5 mm   separates them but then "largest plane" is not reliably the
                 floor -- it flips between surfaces as people move in the room.
                 Gave 41.0 cm, +21.9 deg on the very next frame.

    So enumerate planes and choose on physics rather than on support: GHOSTS ARE
    ALWAYS BELOW THE FLOOR, so among plausible candidates the floor is the
    HIGHEST, i.e. the smallest camera-to-plane distance. A height window rejects
    the rest -- a camera bolted to this rover cannot be 41 cm up.

    Enumerated on one frame, floor first by this rule:

        1965 inliers  15.7 cm  +2.63 deg   <- chosen: highest plausible plane
        1214 inliers  22.4 cm  -6.85 deg   <- ghost
        1112 inliers  16.2 cm  +4.33 deg
         952 inliers  29.1 cm  +14.74 deg  <- deeper ghost

    Returns (normal, d, (inliers, rms), rejected) for the chosen plane, and prints
    every candidate so the choice is auditable rather than implicit.
    """
    rng = np.random.default_rng(seed)
    cos_lim = np.cos(np.radians(MAX_TILT_DEG))
    rejected = 0
    rem = pts.copy()
    cands = []

    for _ in range(max_planes):
        if len(rem) < min_inliers:
            break
        best = (None, None, -1)
        for _ in range(iters):
            p = rem[rng.integers(0, len(rem), 3)]
            n = np.cross(p[1] - p[0], p[2] - p[0])
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n = n / nn
            if n[1] > 0:
                n = -n
            if -n[1] < cos_lim:
                rejected += 1
                continue
            d = -n @ p[0]
            cnt = int((np.abs(rem @ n + d) < tol).sum())
            if cnt > best[2]:
                best = (n, d, cnt)
        if best[0] is None or best[2] < min_inliers:
            break
        mask = np.abs(rem @ best[0] + best[1]) < tol
        inl = rem[mask]
        rem = rem[~mask]
        c = inl.mean(0)
        # eigh on the 3x3 covariance. np.linalg.svd on the full point matrix with
        # default full_matrices=True built a 23000x23000 array: 11.6 s and a
        # wrong normal.
        w, v = np.linalg.eigh(np.cov((inl - c).T))
        n = v[:, 0] / np.linalg.norm(v[:, 0])
        if n[1] > 0:
            n = -n
        cands.append((n, float(-n @ c), inl, float(np.sqrt(max(w[0], 0.0)))))

    if not cands:
        return None, None, None, rejected

    print('   candidate horizontal planes:')
    ok = []
    for n, d, inl, rms in cands:
        pitch, roll = angles(n)
        h = abs(d)
        good = min_h <= h <= max_h
        if good:
            ok.append((n, d, inl, rms))
        print('     %6d inliers  %5.1f cm  pitch %+6.2f  roll %+6.2f  rms %4.1f mm'
              '  %s' % (len(inl), 100 * h, pitch, roll, 1000 * rms,
                        '' if good else '<- outside %.0f-%.0f cm, rejected'
                        % (100 * min_h, 100 * max_h)))
    if not ok:
        print('     none inside the plausible height window')
        return None, None, None, rejected

    # Highest plausible plane: ghosts sit below the floor, never above it.
    n, d, inl, rms = min(ok, key=lambda c: abs(c[1]))
    print('   chosen: the highest plausible plane at %.1f cm' % (100 * abs(d)))
    return n, d, (inl, rms), rejected


def try_board(img, K, square_m, out):
    """findChessboardCornersSB, then retry on an upscaled lower crop.

    Upscaling cannot add information, but SB's corner model has a minimum
    workable spacing, and a foreshortened board sits under it. Resampling lifts
    the spacing above that floor; corners map straight back by dividing.
    """
    attempts = [('full frame', img, 1.0, (0, 0))]
    h, w = img.shape
    y0 = int(h * 0.55)
    crop = img[y0:, :]
    for s in (2, 3, 4):
        attempts.append(('lower crop x%d' % s,
                         cv2.resize(crop, None, fx=s, fy=s,
                                    interpolation=cv2.INTER_CUBIC),
                         float(s), (0, y0)))
    for label, im, scale, (ox, oy) in attempts:
        ok, c = cv2.findChessboardCornersSB(
            im, GRID, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        print('   %-16s %s' % (label, 'FOUND' if ok else 'not found'))
        if not ok:
            continue
        corners = c.reshape(-1, 2).astype(np.float64)
        corners[:, 0] = corners[:, 0] / scale + ox
        corners[:, 1] = corners[:, 1] / scale + oy
        objp = np.zeros((GRID[0] * GRID[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:GRID[0], 0:GRID[1]].T.reshape(-1, 2)
        objp *= square_m
        ok2, rvec, tvec = cv2.solvePnP(objp, corners.astype(np.float32), K,
                                       np.zeros(5))
        if not ok2:
            print('   solvePnP failed')
            return None
        Rm, _ = cv2.Rodrigues(rvec)
        n = Rm[:, 2]
        if n[1] > 0:
            n = -n
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(vis, GRID,
                                  corners.astype(np.float32).reshape(-1, 1, 2),
                                  True)
        cv2.imwrite(out + '/board_found.png', vis)
        return n, float(abs(n @ tvec.ravel()))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', default='/home/narsil/vslam-min/calib')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--exposure', type=int, default=1500)
    ap.add_argument('--gain', type=int, default=800)
    ap.add_argument('--square-mm', type=float, default=25.6)
    ap.add_argument('--near', type=float, default=0.30)
    ap.add_argument('--far', type=float, default=1.50)
    ap.add_argument('--board', action='store_true')
    ap.add_argument('--out', default='/tmp/floorcal')
    a = ap.parse_args()
    subprocess.run(['mkdir', '-p', a.out])

    KL, DL, R1, P1 = load(a.calib + '/acamst_left.yaml')
    KR, DR, R2, P2 = load(a.calib + '/acamst_right.yaml')
    fx, fy, cx, cy = P1[0, 0], P1[1, 1], P1[0, 2], P1[1, 2]
    base = abs(P2[0, 3]) / fx

    raw = grab(a.device, a.exposure, a.gain)
    L = cv2.rotate(bin2x2(raw[:, HALF:]), cv2.ROTATE_180)
    R = cv2.rotate(bin2x2(raw[:, :HALF]), cv2.ROTATE_180)
    sz = (L.shape[1], L.shape[0])
    mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, sz, cv2.CV_32FC1)
    mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, sz, cv2.CV_32FC1)
    Lr = cv2.remap(L, *mL, cv2.INTER_LINEAR)
    Rr = cv2.remap(R, *mR, cv2.INTER_LINEAR)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(Lr)
    cv2.imwrite(a.out + '/left.png', Lr)
    cv2.imwrite(a.out + '/left_clahe.png', clahe)

    print('rectified fx=%.2f cx=%.1f cy=%.1f baseline=%.4f m, depth = %.3f/disp'
          % (fx, cx, cy, base, fx * base))

    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=256, blockSize=5,
                               P1=8 * 25, P2=32 * 25, disp12MaxDiff=1,
                               uniquenessRatio=10, speckleWindowSize=100,
                               speckleRange=2,
                               mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disp = sg.compute(Lr, Rr).astype(np.float32) / 16.0
    Z = np.where(disp > 0.5, fx * base / np.maximum(disp, 1e-6), 0.0)
    ys, xs = np.nonzero((Z > a.near) & (Z < a.far))
    print('\nA  PLANE FIT, %.2f-%.2f m, normals within %.0f deg of vertical'
          % (a.near, a.far, MAX_TILT_DEG))
    if len(ys) < 500:
        print('   only %d points -- floor too dark or textureless' % len(ys))
    else:
        z = Z[ys, xs]
        pts = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1)
        n, d, extra, rejected = plane_fit(pts)
        if n is None:
            print('   no near-vertical plane found in %d points (%d candidates '
                  'rejected as walls or obstacles)' % (len(pts), rejected))
        else:
            inl, rms = extra
            pitch, roll = angles(n)
            print('   points %d, inliers %d (%.0f%%), %d candidates rejected as '
                  'non-floor' % (len(pts), len(inl), 100 * len(inl) / len(pts),
                                 rejected))
            print('   plane rms      %.1f mm' % (1000 * rms))
            print('   normal         [%+.4f %+.4f %+.4f]' % tuple(n))
            print('   CAMERA HEIGHT  %.4f m  (%.1f cm)' % (abs(d), 100 * abs(d)))
            print('   PITCH          %+.2f deg  (positive = nose up)' % pitch)
            print('   ROLL           %+.2f deg' % roll)
            frac = len(inl) / len(pts)
            if frac < 0.20:
                print('\n   LOW CONFIDENCE: only %.0f%% of near-field points are on '
                      'the chosen plane.\n   A trustworthy fit gets 85%%. Do not '
                      'overwrite recorded extrinsics with this.\n   Cause is almost '
                      'always the SCENE, not the fit: this needs a plain wall about '
                      '1.5 m\n   away, the checkerboard sheet flat on the floor for '
                      'texture, and nothing\n   moving in the near field. An open '
                      'office of people, chair legs and mirror\n   reflections '
                      'gives 5-30%% and a different answer every frame.' % (100 * frac))

    if a.board:
        print('\nB  CHECKERBOARD %dx%d inner corners, %.1f mm squares'
              % (GRID[0], GRID[1], a.square_mm))
        got = None
        for label, img in (('plain', Lr), ('clahe', clahe)):
            print('  on %s:' % label)
            got = try_board(img, P1[:3, :3], a.square_mm / 1000.0, a.out)
            if got:
                break
        if got:
            n, dist = got
            pitch, roll = angles(n)
            print('   CAMERA HEIGHT  %.4f m  (%.1f cm)  above the board plane'
                  % (dist, 100 * dist))
            print('   PITCH          %+.2f deg' % pitch)
            print('   ROLL           %+.2f deg' % roll)
            print('   NB this is height above the BOARD. If the board is not '
                  'directly on the floor, add nothing -- move the board.')
        else:
            print('   not detectable. Too grazing at this camera height; stand '
                  'the board upright facing the cameras instead.')
    print('\nimages in', a.out)


if __name__ == '__main__':
    main()
