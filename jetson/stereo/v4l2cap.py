"""A frame source that does not depend on which OpenCV the machine carries.

OpenCV could not open /dev/video0 on jetson11 -- by path, by index and by
CAP_ANY, failing in cap_v4l with "can't open camera by index". That looked like a
limitation of its generic cv2 4.8.0 build and it was NOT: the real cause was the
missing Arducam-patched tegra-camera.ko, and the device could not stream at all.
Once that module was installed, OpenCV opened it fine on jetson11 too.

The fallback is kept regardless. It costs nothing where OpenCV works, since that
is tried first, and it means a host whose OpenCV cannot handle a device offering
only 10-bit Bayer still captures.

open_stereo() tries OpenCV first and falls back, so a machine where OpenCV
already works behaves exactly as before. Either way read() returns the same flat
uint8 buffer, so a caller's

    buf.reshape(-1).view(np.uint16).reshape(H, W)

is unchanged.
"""
import subprocess
import time

import numpy as np


class V4l2Cap:
    """cv2.VideoCapture's shape, backed by a v4l2-ctl stream on stdout."""

    def __init__(self, device, width, height, frames=200000, settle=0.9):
        self.nbytes = width * height * 2
        self.p = subprocess.Popen(
            ['v4l2-ctl', '-d', device,
             '--set-fmt-video=width=%d,height=%d,pixelformat=BA10' % (width, height),
             '--stream-mmap', '--stream-count=%d' % frames, '--stream-to=-'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        # Give the child long enough to fail before anyone asks if it opened.
        # A device already held by another consumer -- narsil-vision on jetson9 --
        # makes v4l2-ctl exit almost at once with VIDIOC_REQBUFS busy, but polling
        # the instant after Popen still sees it alive and would report success,
        # handing the caller a source that never yields a frame.
        time.sleep(settle)

    def isOpened(self):
        return self.p.poll() is None

    def read(self):
        # A raw pipe returns short reads; loop until a whole frame is in.
        buf = b''
        while len(buf) < self.nbytes:
            chunk = self.p.stdout.read(self.nbytes - len(buf))
            if not chunk:
                return False, None
            buf += chunk
        return True, np.frombuffer(buf, dtype=np.uint8)

    def set(self, *_a):
        return True          # format is fixed on the command line

    def release(self):
        try:
            self.p.terminate()
            self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


def open_stereo(device, width, height, verbose=True):
    """OpenCV if it can, v4l2-ctl if it cannot. Returns None if neither works."""
    import cv2
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if cap.isOpened():
        return cap
    cap.release()
    if verbose:
        print('v4l2cap: this OpenCV will not open %s; streaming through '
              'v4l2-ctl instead.' % device, flush=True)
    fb = V4l2Cap(device, width, height)
    return fb if fb.isOpened() else None
