#!/usr/bin/env python3
"""Grab frames without cv2.VideoCapture, and work out how the two eyes sit.

WHY NOT OPENCV. When this was written OpenCV could not open the device on
jetson11 at all, which looked like a build limitation and was not -- the cause
was a missing Arducam-patched tegra-camera.ko, and OpenCV works there now. The
grab still goes through v4l2-ctl because that path is independent of the host's
OpenCV; only the image processing uses cv2, which was never the problem.

Controls are set MID-STREAM, in a second process, because this driver ignores
them before streaming starts -- the same reason smart_grab7.py sets them after
its first reads.

WHAT IT DECIDES. smart_grab7.py hardcodes jetson9's mount:

    L = rotate180(bin2x2(frame[:, HALF:]))    LEFT eye from the RIGHT half
    R = rotate180(bin2x2(frame[:, :HALF]))

Two separable questions, only one of which a machine can settle:

  swap      which sensor half is the left eye. Decidable: the wrong assignment
            leaves a positive-disparity matcher with almost nothing to match.
  rotation  whether the mount is inverted. NOT decidable here -- rotating both
            halves and swapping them is geometrically identical to doing
            neither, just upside down. A human has to say which way is up.
"""
import argparse
import os
import subprocess
import sys
import time

import cv2
import numpy as np

W, H, HALF = 3840, 1200, 1920
DEV = '/dev/video0'
FRAME_BYTES = W * H * 2


def have_ctrl(name):
    out = subprocess.run(['v4l2-ctl', '-d', DEV, '-l'],
                         capture_output=True, text=True).stdout
    return name in out


def grab(path, nframes, exposure, settle):
    """Stream nframes to path, setting exposure once streaming is under way."""
    if os.path.exists(path):
        os.unlink(path)
    cmd = ['v4l2-ctl', '-d', DEV,
           '--set-fmt-video=width=%d,height=%d,pixelformat=BA10' % (W, H),
           '--stream-mmap', '--stream-count=%d' % nframes,
           '--stream-to=' + path]
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    time.sleep(settle)
    if p.poll() is None:
        subprocess.run(['v4l2-ctl', '-d', DEV, '-c', 'exposure=%d' % exposure],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        _, err = p.communicate(timeout=40)
    except subprocess.TimeoutExpired:
        p.kill()
        sys.exit('TIMED OUT: no frames in 40s. The device enumerates but does '
                 'not stream -- this is the CSI-lane symptom, not a script bug.')
    if p.returncode != 0:
        sys.exit('v4l2-ctl failed: ' + (err or b'').decode()[:400])
    n = os.path.getsize(path) // FRAME_BYTES
    if n < 1:
        sys.exit('captured %d bytes, less than one frame' % os.path.getsize(path))
    return n


def last_frame(path, n):
    """Read only the final frame; earlier ones are pre-settle."""
    with open(path, 'rb') as f:
        f.seek((n - 1) * FRAME_BYTES)
        buf = f.read(FRAME_BYTES)
    return np.frombuffer(buf, dtype=np.uint16).reshape(H, W)


def bin2x2(half16):
    """Exactly smart_grab7's binning, so the geometry matches what gets
    calibrated: all four CFA samples, whose centroid is the quad centre."""
    f = half16.astype(np.float32)
    return (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25


def stretch(b):
    lo, hi = np.percentile(b, 1), np.percentile(b, 99.5)
    return np.clip((b - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)


def matched(L, R):
    sg = cv2.StereoSGBM_create(minDisparity=0, numDisparities=128, blockSize=7,
                               P1=8 * 49, P2=32 * 49, uniquenessRatio=10,
                               speckleWindowSize=100, speckleRange=2)
    d = sg.compute(L, R).astype(np.float32) / 16.0
    ok = d > 0.5
    return float(ok.sum()) / d.size, (float(np.median(d[ok])) if ok.any() else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=12)
    ap.add_argument('--exposure', type=int, default=3000)
    ap.add_argument('--settle', type=float, default=0.4)
    ap.add_argument('--out', default=os.path.expanduser('~/stereo/check'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if not have_ctrl('analogue_gain'):
        print('NOTE: no analogue_gain control on this device. smart_grab7 sets it '
              'and discards the error, so its round-4 contrast fix -- "gain, not '
              'exposure" -- has nothing to act on here. Exposure only.')

    raw = os.path.join(a.out, 'stream.raw')
    n = grab(raw, a.frames, a.exposure, a.settle)
    frame = last_frame(raw, n)
    print('captured %d frames, using the last. min=%d max=%d mean=%.1f'
          % (n, frame.min(), frame.max(), frame.mean()))
    np.savez_compressed(os.path.join(a.out, 'raw.npz'), frame=frame)

    LH, RH = bin2x2(frame[:, :HALF]), bin2x2(frame[:, HALF:])
    for nm, b in (('sensor LEFT half', LH), ('sensor RIGHT half', RH)):
        print('%-18s mean=%6.1f p5=%6.1f p95=%6.1f sharpness=%7.1f'
              % (nm, b.mean(), np.percentile(b, 5), np.percentile(b, 95),
                 cv2.Laplacian(stretch(b), cv2.CV_64F).var()))

    rot = lambda x: cv2.rotate(x, cv2.ROTATE_180)
    arr = [('A rot+swap  (smart_grab7/jetson9)', rot(RH), rot(LH)),
           ('B rot, no swap',                    rot(LH), rot(RH)),
           ('C no rot, swap',                    RH,      LH),
           ('D no rot, no swap',                 LH,      RH)]
    print('\narrangement                           matched   median disp')
    best, bv = None, -1.0
    for nm, Lb, Rb in arr:
        f, med = matched(stretch(Lb), stretch(Rb))
        print('%-36s %6.2f%%  %7.1f px' % (nm, f * 100, med))
        if f > bv:
            best, bv = nm, f
    print('\nbest: %s' % best)
    print('A and D are geometrically identical, so they should score alike; '
          'which is right is only a question of which way up the room is.')

    def lab(img, text):
        c = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(c, (0, 0), (c.shape[1] - 1, 24), (0, 0, 0), -1)
        cv2.putText(c, text, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return c

    sheet = np.vstack([
        np.hstack([lab(stretch(LH), 'sensor LEFT half, as read'),
                   lab(stretch(RH), 'sensor RIGHT half, as read')]),
        np.hstack([lab(stretch(rot(LH)), 'sensor LEFT half, rotated 180'),
                   lab(stretch(rot(RH)), 'sensor RIGHT half, rotated 180')])])
    cv2.imwrite(os.path.join(a.out, 'contact_sheet.png'), sheet)
    print('wrote %s/contact_sheet.png' % a.out)


main()
