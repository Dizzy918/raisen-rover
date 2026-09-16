"""The single owner of /dev/video0.

Everything the standalone scripts each rediscovered, in one place:

  RAW10 SETUP. CONVERT_RGB off, FOURCC 'BA10', then the full 3840x1200. Miss any
  of those and OpenCV hands back a debayered 8-bit image of the wrong size.

  THE WAKE TRANSIENT IS REAL. For a short window after another process releases
  the device, reads succeed and return near-black frames. That is not a dead
  sensor and not a bad exposure -- a cloud test once reported 0 valid depth
  pixels this way while raw stats proved the sensor was fine. So opening waits
  for a frame with actual signal instead of trusting the first successful read.

  EXPOSURE IS A FRAME-RATE CEILING. It is in sensor row-times, and the fitted
  frame period is about exposure * 21.9 us. Measured: 8000 -> 5.8 fps, 4000 ->
  11.5, 2290 -> 20.1, 1500 -> 30.6. Longer exposure is ALSO worse for stereo
  because it blows out bright regions, so 1500 is both the fastest and the best
  starting point; buy brightness with gain instead.
"""
import time

from . import controls as _controls
from . import frame as _frame

BLACK_MEAN = 50.0          # raw counts below this is the wake transient, not a scene
WAKE_TRIES = 30


class StereoCamera:
    def __init__(self, device='/dev/video0', exposure=1500, gain=800,
                 open_now=True):
        self.device = device
        self.exposure = exposure
        self.gain = gain
        self.cap = None
        self.ctl = _controls.Controls(device)
        self.reads = 0
        self.drops = 0
        if open_now:
            self.open()

    def open(self):
        # Through v4l2cap: OpenCV is tried first and used when it works, with a
        # v4l2-ctl stream as the fallback. See raisen/v4l2cap.py for why -- the
        # short version is that an OpenCV that cannot open this device is a
        # symptom worth surviving, not a reason to fail the run.
        from . import v4l2cap as _v4l2cap
        cap = _v4l2cap.open_stereo(self.device, _frame.W, _frame.H)
        if cap is None:
            raise SystemExit('cannot open %s -- narsil-vision still holding it? '
                             'raisen.service.vision_stopped() releases it'
                             % self.device)
        self.cap = cap
        for _ in range(5):
            cap.read()
        if self.exposure is not None:
            self.set_exposure(self.exposure)
        if self.gain is not None:
            self.set_gain(self.gain)
        time.sleep(0.6)
        return self

    def set_exposure(self, value):
        self.exposure = int(value)
        self.ctl.set('exposure', self.exposure)

    def set_gain(self, value):
        self.gain = int(value)
        self.ctl.set('analogue_gain', self.gain)

    def read_raw(self):
        """One 16-bit side-by-side frame, or None if this read should be dropped."""
        ok, buf = self.cap.read()
        self.reads += 1
        if not ok or buf is None:
            self.drops += 1
            return None
        f = _frame.as_frame16(buf)
        if f is None:
            self.drops += 1
        return f

    def read_awake(self, tries=WAKE_TRIES, black_mean=BLACK_MEAN):
        """Read until a frame has real signal. Returns None if none ever does.

        Use this for one-shot measurements. A continuous pipeline should not use
        it -- it would silently swallow a genuinely dark scene.
        """
        for _ in range(tries):
            f = self.read_raw()
            if f is not None and float(f.mean()) > black_mean:
                return f
        return None

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
