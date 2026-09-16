#!/usr/bin/env python3
"""Stereo-calibrate the AR0234 pair and emit narsil acamst_*.yaml.

Deliberately mirrors how the existing calibration was produced -- its own header
says "K, D from cv2.stereoCalibrateExtended; R, P from cv2.stereoRectify
(CALIB_ZERO_DISPARITY, alpha=0)" -- so the output is drop-in compatible and there
is no MATLAB/OpenCV convention mismatch to reconcile (no 1-indexed principal
point, no transposed K or R).

Distortion is the 5-coefficient plumb_bob (k1 k2 p1 p2 k3) that ROS's
camera_info and the existing files use.

RESOLUTION. Calibration runs on the 960x600 2x2-binned images (see README: a
checkerboard aliases against the GRBG mosaic, so demosaiced full-res detection
fails). Two sets are written:

  out/960x600/    the native measurement, at the resolution actually calibrated
  out/1920x1200/  scaled for the ar0234_fullres pipeline profile

The scaling is NOT a plain doubling of the principal point. Binned pixel index i
covers full-res indices 2i and 2i+1, whose centres are 2i and 2i+1, so the quad
centroid sits at full-res coordinate 2i + 0.5. Hence

    f_full = 2 * f_binned            (d x_full / d x_binned = 2)
    c_full = 2 * c_binned + 0.5      (the half-pixel the naive x2 drops)

Distortion coefficients are normalized by the focal length and carry over
unchanged. Rectification R is a pure rotation and is resolution-independent.

Usage:
    python3 calibrate_stereo.py captures/final --square-mm 25.6
"""
import argparse
import os
import sys
from datetime import datetime, timezone

import cv2
import numpy as np

GRID = (9, 6)              # INNER corners


def find_corners(gray):
    """Plain findChessboardCornersSB. SB already refines to subpixel, so no
    cornerSubPix pass -- running one on top degrades SB's estimate."""
    ok, c = cv2.findChessboardCornersSB(
        gray, GRID, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return (c.reshape(-1, 2).astype(np.float32) if ok else None)


def yaml_mat(name, m, rows, cols, indent=0):
    pad = ' ' * indent
    out = [f'{pad}{name}:', f'{pad}  rows: {rows}', f'{pad}  cols: {cols}',
           f'{pad}  data:']
    out += [f'{pad}  - {v!r}' for v in np.asarray(m).reshape(-1).tolist()]
    return '\n'.join(out)


def write_cam(path, w, h, name, K, D, R, P, header):
    with open(path, 'w') as f:
        f.write(header)
        f.write(f'image_width: {w}\nimage_height: {h}\ncamera_name: {name}\n')
        f.write(yaml_mat('camera_matrix', K, 3, 3) + '\n')
        f.write('distortion_model: plumb_bob\n')
        f.write(yaml_mat('distortion_coefficients', D, 1, len(np.ravel(D))) + '\n')
        f.write(yaml_mat('rectification_matrix', R, 3, 3) + '\n')
        f.write(yaml_mat('projection_matrix', P, 3, 4) + '\n')


def scale_to_full(K, P):
    """960x600 -> 1920x1200. See module docstring for the +0.5."""
    K2 = K.copy()
    K2[0, 0] *= 2; K2[1, 1] *= 2
    K2[0, 2] = 2 * K[0, 2] + 0.5
    K2[1, 2] = 2 * K[1, 2] + 0.5
    P2 = P.copy()
    P2[0, 0] *= 2; P2[1, 1] *= 2
    P2[0, 2] = 2 * P[0, 2] + 0.5
    P2[1, 2] = 2 * P[1, 2] + 0.5
    P2[0, 3] *= 2          # = -f * baseline, so it scales with f
    P2[1, 3] *= 2
    return K2, P2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir', help='directory with left/ and right/')
    ap.add_argument('--square-mm', type=float, default=25.6,
                    help='MEASURED square size. The baseline scales with this.')
    ap.add_argument('--out', default=None)
    ap.add_argument('--exclude', default='',
                    help='comma-separated filenames to drop, e.g. img_16.png,'
                         'img_31.png. Prune only clear outliers: RMS always '
                         'falls as you remove data, so a low RMS on a thinned '
                         'set is worse than a slightly higher one on a full set.')
    a = ap.parse_args()

    drop = {s.strip() for s in a.exclude.split(',') if s.strip()}
    ldir, rdir = os.path.join(a.dir, 'left'), os.path.join(a.dir, 'right')
    names = sorted(n for n in (set(os.listdir(ldir)) & set(os.listdir(rdir)))
                   if n.endswith('.png') and n not in drop)
    if drop:
        print(f'excluding {len(drop)}: {", ".join(sorted(drop))}')
    if not names:
        sys.exit('no paired PNGs found')

    sq = a.square_mm / 1000.0
    objp = np.zeros((GRID[0] * GRID[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:GRID[0], 0:GRID[1]].T.reshape(-1, 2) * sq

    obj, ptsL, ptsR, used = [], [], [], []
    size = None
    for n in names:
        L = cv2.imread(os.path.join(ldir, n), cv2.IMREAD_GRAYSCALE)
        R = cv2.imread(os.path.join(rdir, n), cv2.IMREAD_GRAYSCALE)
        if L is None or R is None:
            continue
        size = (L.shape[1], L.shape[0])
        cL, cR = find_corners(L), find_corners(R)
        if cL is None or cR is None:
            print(f'  skip {n}: detect failed L={cL is not None} R={cR is not None}')
            continue
        obj.append(objp); ptsL.append(cL); ptsR.append(cR); used.append(n)

    w, h = size
    print(f'{len(used)}/{len(names)} pairs used, {w}x{h}, square {a.square_mm} mm\n')
    if len(used) < 10:
        sys.exit('too few usable pairs')

    flags_mono = 0
    rmsL, KL, DL, _, _ = cv2.calibrateCamera(obj, ptsL, (w, h), None, None,
                                             flags=flags_mono)
    rmsR, KR, DR, _, _ = cv2.calibrateCamera(obj, ptsR, (w, h), None, None,
                                             flags=flags_mono)
    print(f'mono RMS   left {rmsL:.4f} px   right {rmsR:.4f} px')

    # refine both intrinsics jointly with the extrinsics
    # OpenCV 5 returns rvecs/tvecs before perViewErrors; 4.x omits them.
    ret = cv2.stereoCalibrateExtended(
        obj, ptsL, ptsR, KL, DL, KR, DR, (w, h),
        R=np.eye(3), T=np.zeros(3),
        flags=cv2.CALIB_USE_INTRINSIC_GUESS,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-10))
    rms, KL, DL, KR, DR, Rot, T, E, F = ret[:9]
    perView = ret[-1]          # last element either way

    baseline_mm = float(np.linalg.norm(T) * 1000.0)
    rvec, _ = cv2.Rodrigues(Rot)
    rot_deg = np.degrees(rvec).reshape(-1)

    print(f'stereo RMS : {rms:.5f} px')
    print(f'baseline   : {baseline_mm:.4f} mm   (previous calibration 148.2304)')
    print(f'rotation   : {rot_deg[0]:+.4f} {rot_deg[1]:+.4f} {rot_deg[2]:+.4f} deg')
    print(f'T          : {T.reshape(-1)}')

    pv = np.asarray(perView).reshape(len(used), -1).mean(axis=1)
    worst = np.argsort(pv)[::-1][:5]
    print('\nworst pairs by reprojection error:')
    for i in worst:
        print(f'  {used[i]}  {pv[i]:.4f} px')

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        KL, DL, KR, DR, (w, h), Rot, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    stamp = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
    base = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')

    for tag, (ww, hh) in (('960x600', (w, h)), ('1920x1200', (w * 2, h * 2))):
        d = os.path.join(base, tag)
        os.makedirs(d, exist_ok=True)
        if tag == '960x600':
            kL, pL, kR, pR = KL, P1, KR, P2
        else:
            kL, pL = scale_to_full(KL, P1)
            kR, pR = scale_to_full(KR, P2)
        hdr = (f'# AR0234 stereo calibration — produced by '
               f'stereo_calibration/calibrate_stereo.py\n'
               f'# Source images: {a.dir} ({len(used)} pairs, {w}x{h} 2x2-binned)\n'
               f'# Generated: {stamp}\n'
               f'# K, D from cv2.stereoCalibrateExtended; R, P from '
               f'cv2.stereoRectify (CALIB_ZERO_DISPARITY, alpha=0).\n'
               f'# Checkerboard {GRID[0]}x{GRID[1]} inner corners, '
               f'{a.square_mm} mm squares. Stereo RMS {rms:.5f} px.\n')
        if tag != '960x600':
            hdr += ('# Scaled from the 960x600 calibration: f*2, c*2+0.5 (the '
                    'binned pixel\n#   centroid sits at full-res 2i+0.5). D and '
                    'R are resolution-independent.\n')
        for side, nm, K_, D_, R_, P_ in (
                ('left',  f'ar0234_left_{"3840x1200" if tag != "960x600" else "1920x600"}',
                 kL, DL, R1, pL),
                ('right', f'ar0234_right_{"3840x1200" if tag != "960x600" else "1920x600"}',
                 kR, DR, R2, pR)):
            write_cam(os.path.join(d, f'acamst_{side}.yaml'), ww, hh, nm,
                      K_, D_, R_, P_, hdr)

        with open(os.path.join(d, 'acamst_extrinsics.yaml'), 'w') as f:
            f.write('# AR0234 stereo extrinsics — relative pose of '
                    'ar0234_right_optical\n# in ar0234_left_optical frame. '
                    'R, T from cv2.stereoCalibrateExtended.\n'
                    '# Read by arducam_camera.launch.py to publish the static TF.\n')
            f.write(f'profile: {"3840x1200" if tag != "960x600" else "1920x600"}\n')
            f.write('parent_frame: ar0234_left_optical\n'
                    'child_frame: ar0234_right_optical\n')
            f.write(yaml_mat('rotation', Rot, 3, 3) + '\n')
            f.write(yaml_mat('translation', T, 3, 1) + '\n')
            f.write(f'baseline_mm: {baseline_mm:.4f}\n')
            f.write('rotation_xyz_deg:\n')
            for v in rot_deg:
                f.write(f'- {v:.4f}\n')
            f.write(f'rms_px: {rms:.5f}\nview_count: {len(used)}\n')
            f.write(f'checkerboard:\n  rows: {GRID[1]}\n  cols: {GRID[0]}\n'
                    f'  square_m: {sq}\n')
            f.write(f'source_images: {a.dir}\ngenerated: {stamp}\n')
        print(f'wrote {d}/acamst_{{left,right,extrinsics}}.yaml')

    # rectification check: with a good calibration, matched rows line up, so the
    # mean absolute y-disparity of the corners after rectification should be
    # well under a pixel. This catches a wrong T sign or a bad R directly.
    mapL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, (w, h), cv2.CV_32FC1)
    mapR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, (w, h), cv2.CV_32FC1)
    dy = []
    for pL_, pR_ in zip(ptsL, ptsR):
        uL = cv2.undistortPoints(pL_.reshape(-1, 1, 2), KL, DL, R=R1, P=P1)
        uR = cv2.undistortPoints(pR_.reshape(-1, 1, 2), KR, DR, R=R2, P=P2)
        dy.append(np.abs(uL.reshape(-1, 2)[:, 1] - uR.reshape(-1, 2)[:, 1]))
    dy = np.concatenate(dy)
    print(f'\nrectified row alignment: mean |dy| {dy.mean():.4f} px, '
          f'p95 {np.percentile(dy, 95):.4f} px, max {dy.max():.4f} px')
    print('  (well under 1 px = rectification is sane)')


if __name__ == '__main__':
    main()
