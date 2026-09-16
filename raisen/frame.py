"""Raw sensor frame -> two upright 8-bit eyes.

THE SENSOR GEOMETRY, which every consumer needs and none should restate:
the UC-512 HAT delivers ONE side-by-side 3840x1200 RAW10 GRBG frame at
/dev/video0, 1920x1200 per eye. The mount is inverted AND the eyes are swapped,
so each half is rotated 180 degrees and left comes from the RIGHT half.
"""
import cv2
import numpy as np

W, H = 3840, 1200          # one side-by-side frame as the sensor delivers it
HALF = W // 2              # 1920 columns per eye, before binning
EYE_W, EYE_H = HALF // 2, H // 2   # 960x600 after 2x2 binning


def bin2x2(half16):
    """2x2 Bayer bin one eye to uint8, stretched to its 1-99 percentile.

    BINNING, NOT DEMOSAICING. Averaging each 2x2 Bayer quad gives a real
    luminance sample per quad. Demosaicing would interpolate colour we do not
    need and cuVSLAM cannot take (mono8 only), at more cost and with invented
    detail in the process.

    The percentile is taken from a [::6, ::6] subsample: the full-resolution
    np.percentile cost about 31 ms per eye on the Jetson -- the single largest
    item in the frame budget -- and moved the result by under one grey level.
    """
    f = half16.astype(np.float32)
    q = (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25
    lo, hi = np.percentile(q[::6, ::6], (1, 99))
    return np.clip((q - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)


def split_pair(frame16):
    """Side-by-side raw -> (left, right), both 960x600 uint8 and upright.

    THE SWAP IS NOT A GUESS. The mount is inverted, so the physically-left camera
    lands in the RIGHT half of the frame. Getting this backwards produces
    negative disparities everywhere and a depth map that is uniformly invalid --
    which reads as "stereo is broken" rather than "the eyes are the wrong way
    round". stereo_calibration/check_rectification.py is the test that catches it:
    it asserts all disparities come out positive.
    """
    left = cv2.rotate(bin2x2(frame16[:, HALF:]), cv2.ROTATE_180)
    right = cv2.rotate(bin2x2(frame16[:, :HALF]), cv2.ROTATE_180)
    return left, right


def as_frame16(buf):
    """Reinterpret an OpenCV V4L2 buffer as the 16-bit side-by-side frame.

    Returns None rather than raising: a short or malformed buffer happens
    occasionally during startup and the right response is to drop that frame,
    not to kill the pipeline.
    """
    try:
        return buf.reshape(-1).view(np.uint16).reshape(H, W)
    except (ValueError, AttributeError):
        return None
