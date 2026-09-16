#!/usr/bin/env python3
"""Calibrate a stereo pair from captured checkerboard images.

Pipeline: per-eye intrinsics -> stereoCalibrate -> stereoRectify -> save maps.

Writes:
  stereo_calib.yml   K1 D1 K2 D2 R T E F R1 R2 P1 P2 Q image_size baseline
  rect_check_*.jpg   rectified pairs with horizontal rulings drawn -- the same
                     board feature must sit on the SAME line in both halves
  report printed to stdout with the numbers that actually tell you if it worked

Example
-------
  ./stereo_calibrate.py --dir ~/stereo/captures --pattern 9x6 --square-size 25.0
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def find_corners(gray, pattern):
    ok, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    )
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            gray, pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3),
            )
    if not ok:
        return None
    return corners.astype(np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=os.path.expanduser("~/stereo/captures"))
    ap.add_argument("--pattern", default="9x6", help="INNER corners, cols x rows")
    ap.add_argument("--square-size", type=float, required=True,
                    help="checkerboard square edge length in mm (sets the unit of T/baseline)")
    ap.add_argument("--out", default=None, help="output yml (default <dir>/stereo_calib.yml)")
    ap.add_argument("--refine-intrinsics", action="store_true",
                    help="let stereoCalibrate adjust intrinsics too (needs plenty of good views)")
    ap.add_argument("--rational", action="store_true",
                    help="8-coefficient distortion model, for wide-angle lenses")
    ap.add_argument("--alpha", type=float, default=0.0,
                    help="stereoRectify alpha: 0 = crop to valid pixels, 1 = keep all")
    ap.add_argument("--checks", type=int, default=3, help="how many rect_check images to write")
    args = ap.parse_args()

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    pattern = (cols, rows)
    out_path = args.out or os.path.join(args.dir, "stereo_calib.yml")

    lefts = sorted(glob.glob(os.path.join(args.dir, "left_*.png")))
    if not lefts:
        sys.exit(f"no left_*.png in {args.dir} -- run stereo_capture.py first")

    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= args.square_size

    obj_points, img_points_l, img_points_r, used = [], [], [], []
    image_size = None

    for lp in lefts:
        rp = lp.replace("left_", "right_")
        if not os.path.exists(rp):
            print(f"skip {os.path.basename(lp)}: no matching right image")
            continue
        imL, imR = cv2.imread(lp), cv2.imread(rp)
        if imL is None or imR is None:
            print(f"skip {os.path.basename(lp)}: unreadable")
            continue
        gL = cv2.cvtColor(imL, cv2.COLOR_BGR2GRAY)
        gR = cv2.cvtColor(imR, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gL.shape[1], gL.shape[0])
        elif (gL.shape[1], gL.shape[0]) != image_size:
            print(f"skip {os.path.basename(lp)}: size differs from {image_size}")
            continue

        cL, cR = find_corners(gL, pattern), find_corners(gR, pattern)
        if cL is None or cR is None:
            miss = "left" if cL is None else "right"
            print(f"skip {os.path.basename(lp)}: board not found in {miss}")
            continue

        obj_points.append(objp.copy())
        img_points_l.append(cL)
        img_points_r.append(cR)
        used.append((lp, rp))

    n = len(obj_points)
    print(f"\nusable pairs: {n} of {len(lefts)}  (image size {image_size[0]}x{image_size[1]})")
    if n < 8:
        sys.exit("need at least ~8 usable pairs; 15-25 is comfortable. Capture more.")

    mono_flags = cv2.CALIB_RATIONAL_MODEL if args.rational else 0
    rms_l, K1, D1, _, _ = cv2.calibrateCamera(
        obj_points, img_points_l, image_size, None, None, flags=mono_flags)
    rms_r, K2, D2, _, _ = cv2.calibrateCamera(
        obj_points, img_points_r, image_size, None, None, flags=mono_flags)
    print(f"mono reprojection RMS: left {rms_l:.4f} px   right {rms_r:.4f} px")

    stereo_flags = mono_flags | (
        cv2.CALIB_USE_INTRINSIC_GUESS if args.refine_intrinsics else cv2.CALIB_FIX_INTRINSIC)
    rms_s, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_points, img_points_l, img_points_r, K1, D1, K2, D2, image_size,
        flags=stereo_flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
    )
    baseline = float(np.linalg.norm(T))
    print(f"stereo reprojection RMS: {rms_s:.4f} px")
    print(f"baseline |T|: {baseline:.2f} mm")
    rvec, _ = cv2.Rodrigues(R)
    print("relative rotation (deg): " +
          np.array2string(np.degrees(rvec).ravel(), precision=3, suppress_small=True))

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=args.alpha)

    # The number that actually predicts depth quality: after rectification, a
    # point's row must match in both views. Mean |yL - yR| should be well under
    # ~0.5 px. A good mono RMS with a bad epipolar error means the extrinsics
    # are wrong, which is exactly the failure a single RMS number hides.
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_32FC1)

    y_err = []
    for cL, cR in zip(img_points_l, img_points_r):
        uL = cv2.undistortPoints(cL, K1, D1, R=R1, P=P1).reshape(-1, 2)
        uR = cv2.undistortPoints(cR, K2, D2, R=R2, P=P2).reshape(-1, 2)
        y_err.append(np.abs(uL[:, 1] - uR[:, 1]))
    y_err = np.concatenate(y_err)
    print(f"epipolar error after rectification: mean {y_err.mean():.4f} px, "
          f"p95 {np.percentile(y_err, 95):.4f} px, max {y_err.max():.4f} px")

    fs = cv2.FileStorage(out_path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("pattern_cols", cols)
    fs.write("pattern_rows", rows)
    fs.write("square_size_mm", args.square_size)
    fs.write("pairs_used", n)
    for name, val in (("K1", K1), ("D1", D1), ("K2", K2), ("D2", D2),
                      ("R", R), ("T", T), ("E", E), ("F", F),
                      ("R1", R1), ("R2", R2), ("P1", P1), ("P2", P2), ("Q", Q)):
        fs.write(name, val)
    fs.write("baseline_mm", baseline)
    fs.write("rms_left", rms_l)
    fs.write("rms_right", rms_r)
    fs.write("rms_stereo", rms_s)
    fs.write("epipolar_mean_px", float(y_err.mean()))
    fs.release()
    print(f"\nwrote {out_path}")

    for i, (lp, rp) in enumerate(used[: args.checks]):
        imL, imR = cv2.imread(lp), cv2.imread(rp)
        rL = cv2.remap(imL, map1x, map1y, cv2.INTER_LINEAR)
        rR = cv2.remap(imR, map2x, map2y, cv2.INTER_LINEAR)
        side = np.hstack([rL, rR])
        for y in range(0, side.shape[0], 40):
            cv2.line(side, (0, y), (side.shape[1], y), (0, 255, 0), 1)
        path = os.path.join(args.dir, f"rect_check_{i:03d}.jpg")
        cv2.imwrite(path, side)
        print(f"wrote {path}")

    print("\n--- how to read this ---")
    print("mono RMS      < 0.5 px  good, < 1.0 px acceptable")
    print("epipolar mean < 0.3 px  good, > 0.7 px means recapture with more varied poses")
    print("baseline should match the physical distance between the two lenses;")
    print("if it does not, --square-size is wrong and all depth will be scaled wrong.")
    print("Finally LOOK at rect_check_*.jpg: matching features must lie on the same green line.")


if __name__ == "__main__":
    main()
