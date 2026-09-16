#!/usr/bin/env python3
"""Capture synchronized stereo checkerboard pairs, headless.

Saves <out>/left_NNN.png + <out>/right_NNN.png and refreshes
<out>/preview_last.jpg (side-by-side, detected corners drawn) so you can copy
it off the board and eyeball framing without a display.

A pair is only kept when the board is found in BOTH views AND the pose differs
enough from the last accepted pair -- that is what stops you from collecting 30
near-identical frames, which is the most common way stereo calibration ends up
with a low reported error but bad real-world depth.

Two topologies are supported:

  --split   ONE video device carrying both sensors side by side in a single wide
            frame. This is what an Arducam Camarray HAT gives you: the HAT muxes
            both sensors over one CSI lane set and the two halves are
            HARDWARE-synchronised, which is strictly better for calibration than
            two independent cameras.

  default   TWO separate video devices, one per sensor, software-synchronised.

Examples
--------
  # Camarray HAT: grab one raw pair to confirm the feed works
  ./stereo_capture.py --split --raw-only

  # Camarray HAT: collect 25 checkerboard pairs, 9x6 inner corners
  ./stereo_capture.py --split --count 25 --pattern 9x6

  # two independent CSI cameras
  ./stereo_capture.py --count 25 --pattern 9x6

  # USB / v4l2 cameras instead of CSI
  ./stereo_capture.py --backend v4l2 --left 0 --right 1
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np


def argus_pipeline(sensor_id, cap_w, cap_h, fps, out_w, out_h):
    """GStreamer pipeline for a CSI sensor through the Jetson ISP."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={cap_w},height={cap_h},framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw,width={out_w},height={out_h},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=2"
    )


def open_camera(spec, args, label):
    if args.backend == "argus":
        pipeline = argus_pipeline(
            int(spec), args.cap_width, args.cap_height, args.fps, args.width, args.height
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    else:
        target = int(spec) if str(spec).isdigit() else spec
        cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
        if args.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)

    if not cap.isOpened():
        sys.exit(
            f"ERROR: could not open {label} camera ({args.backend}, spec={spec}).\n"
            "Run cam_diag.sh first -- if there is no /dev/video* and no I2C\n"
            "response on buses 9/10, the sensor is not connected yet."
        )
    return cap


def find_board(gray, pattern, scale):
    """Locate the checkerboard. Returns corners at FULL resolution, or None."""
    small = gray
    if scale != 1.0:
        small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    # findChessboardCornersSB is markedly more robust to blur and glare than the
    # classic detector, which matters a lot for handheld capture.
    ok, corners = cv2.findChessboardCornersSB(
        small, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    )
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            small,
            pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
    if not ok:
        return None

    if scale != 1.0:
        corners = corners / scale
    return corners.astype(np.float32)


def pose_signature(corners):
    """Cheap descriptor of where/how big/how skewed the board is in frame."""
    pts = corners.reshape(-1, 2)
    centre = pts.mean(axis=0)
    spread = pts.std(axis=0)
    return np.array([centre[0], centre[1], spread[0], spread[1]])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["argus", "v4l2"], default="argus",
                    help="argus = CSI via nvarguscamerasrc (default); v4l2 = /dev/videoN")
    ap.add_argument("--left", default="0",
                    help="sensor-id or /dev/videoN index for LEFT "
                         "(with --split: the single combined source)")
    ap.add_argument("--right", default="1", help="sensor-id or /dev/videoN index for RIGHT")
    ap.add_argument("--split", action="store_true",
                    help="one wide frame carrying both sensors side by side "
                         "(Arducam Camarray HAT); split it down the middle")
    ap.add_argument("--out", default=os.path.expanduser("~/stereo/captures"))
    ap.add_argument("--width", type=int, default=1280, help="output frame width")
    ap.add_argument("--height", type=int, default=720, help="output frame height")
    ap.add_argument("--cap-width", type=int, default=1920, help="sensor mode width (argus)")
    ap.add_argument("--cap-height", type=int, default=1080, help="sensor mode height (argus)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fourcc", default="MJPG", help="v4l2 only, e.g. MJPG or YUYV")
    ap.add_argument("--pattern", default="9x6",
                    help="INNER corners, cols x rows (a 10x7 square board is 9x6)")
    ap.add_argument("--count", type=int, default=25, help="pairs to collect")
    ap.add_argument("--interval", type=float, default=1.0, help="min seconds between saves")
    ap.add_argument("--min-move", type=float, default=40.0,
                    help="min pose-signature change (px) before a new pair is accepted")
    ap.add_argument("--detect-scale", type=float, default=0.5,
                    help="downscale factor for detection speed (corners still saved full-res)")
    ap.add_argument("--timeout", type=float, default=300.0, help="give up after N seconds")
    ap.add_argument("--raw-only", action="store_true",
                    help="grab a single pair with no board detection, then exit")
    args = ap.parse_args()

    cols, rows = (int(v) for v in args.pattern.lower().split("x"))
    pattern = (cols, rows)
    os.makedirs(args.out, exist_ok=True)

    if args.split:
        print(f"opening ONE combined source: backend={args.backend} spec={args.left}")
        print("--width/--cap-width are the COMBINED (double) width in --split mode")
        capL = open_camera(args.left, args, "COMBINED")
        capR = None
    else:
        print(f"opening cameras: backend={args.backend} left={args.left} right={args.right}")
        capL = open_camera(args.left, args, "LEFT")
        capR = open_camera(args.right, args, "RIGHT")

    # Let auto-exposure settle; the first few Argus frames are often black.
    for _ in range(10):
        capL.grab()
        if capR is not None:
            capR.grab()
    time.sleep(0.5)

    def grab_pair():
        if args.split:
            # One wide frame, both sensors already hardware-synchronised by the
            # HAT. Split down the middle; an odd width would mean the frame is
            # not actually a side-by-side pair, so refuse rather than guess.
            ok, frame = capL.read()
            if not ok or frame is None:
                return None, None
            w = frame.shape[1]
            if w % 2 != 0:
                sys.exit(f"ERROR: combined frame width {w} is odd -- this does "
                         "not look like a side-by-side stereo frame")
            half = w // 2
            # .copy() so the halves are contiguous; cv2.imwrite and the corner
            # refiner both dislike non-contiguous views.
            return frame[:, :half].copy(), frame[:, half:].copy()

        # Two independent cameras: grab() both before retrieve() to keep the
        # shutter skew as small as the software path allows. These are not
        # hardware-synced, so treat it as "close in time", not "simultaneous".
        okL = capL.grab()
        okR = capR.grab()
        if not (okL and okR):
            return None, None
        okL, frameL = capL.retrieve()
        okR, frameR = capR.retrieve()
        if not (okL and okR):
            return None, None
        return frameL, frameR

    if args.raw_only:
        frameL, frameR = grab_pair()
        if frameL is None:
            sys.exit("ERROR: grab failed on one or both cameras")
        cv2.imwrite(os.path.join(args.out, "raw_left.png"), frameL)
        cv2.imwrite(os.path.join(args.out, "raw_right.png"), frameR)
        cv2.imwrite(os.path.join(args.out, "preview_last.jpg"),
                    np.hstack([frameL, frameR]))
        print(f"wrote raw_left.png / raw_right.png / preview_last.jpg to {args.out}")
        print(f"per-eye frame size: {frameL.shape[1]}x{frameL.shape[0]}")
        if args.split:
            print(f"combined source was {frameL.shape[1] * 2}x{frameL.shape[0]}")
        for c in (capL, capR):
            if c is not None:
                c.release()
        return

    print(f"looking for a {cols}x{rows} inner-corner checkerboard in both views")
    print("move the board around: fill the frame, tilt it, near and far, corners "
          "of the image. Ctrl-C to stop early.\n")

    saved = 0
    last_sig = None
    last_save = 0.0
    start = time.time()
    last_report = 0.0

    try:
        while saved < args.count and (time.time() - start) < args.timeout:
            frameL, frameR = grab_pair()
            if frameL is None:
                print("grab failed, retrying...")
                time.sleep(0.2)
                continue

            grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
            grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)
            cornersL = find_board(grayL, pattern, args.detect_scale)
            cornersR = find_board(grayR, pattern, args.detect_scale)

            now = time.time()
            both = cornersL is not None and cornersR is not None

            if now - last_report > 1.0:
                state = "BOTH" if both else (
                    "left only" if cornersL is not None else
                    "right only" if cornersR is not None else "none")
                print(f"[{saved}/{args.count}] board: {state}")
                last_report = now

            if both:
                sig = np.concatenate([pose_signature(cornersL), pose_signature(cornersR)])
                moved = last_sig is None or np.linalg.norm(sig - last_sig) > args.min_move
                if moved and (now - last_save) >= args.interval:
                    name = f"{saved:03d}"
                    cv2.imwrite(os.path.join(args.out, f"left_{name}.png"), frameL)
                    cv2.imwrite(os.path.join(args.out, f"right_{name}.png"), frameR)

                    visL = cv2.drawChessboardCorners(frameL.copy(), pattern, cornersL, True)
                    visR = cv2.drawChessboardCorners(frameR.copy(), pattern, cornersR, True)
                    cv2.imwrite(os.path.join(args.out, "preview_last.jpg"),
                                np.hstack([visL, visR]))

                    saved += 1
                    last_sig = sig
                    last_save = now
                    print(f"  saved pair {name}  ({saved}/{args.count})")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for c in (capL, capR):
            if c is not None:
                c.release()

    print(f"\ncollected {saved} pairs in {args.out}")
    if saved < 10:
        print("WARNING: fewer than 10 pairs -- calibration will be unreliable.")
    else:
        print("next: ./stereo_calibrate.py --dir "
              f"{args.out} --pattern {args.pattern} --square-size <mm>")


if __name__ == "__main__":
    main()
