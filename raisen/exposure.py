#!/usr/bin/env python3
"""AUTO-EXPOSURE -- package copy, byte-identical in behaviour to
jetson/stereo/auto_exposure.py, which stays in place as the proven standalone CLI.

Two copies exist ON PURPOSE and only for as long as the hardware is off: the
standalone version is what every AE measurement to date was made with, and
rewriting code that cannot be re-tested is how a working stack breaks. The tuned
constants below are pinned against that file by tests/test_offline.py, so the two
cannot drift apart silently while they wait for a hardware run to merge them.
"""

"""Auto-exposure for the AR0234, built to converge instead of hunt.

Import AutoExposure and call update(raw16) once per frame; it decides when to act
and writes v4l2 controls itself. Or run this file directly to watch it converge
from a deliberately wrong starting point.

    python3 auto_exposure.py --start 100     # far too dark
    python3 auto_exposure.py --start 20000   # far too bright


WHY NAIVE AE LIMIT-CYCLES HERE, which is the whole reason this file exists

  1. THE STRETCH HIDES THE SIGNAL. bin2x2() rescales every frame by its own 1-99
     percentiles, so the output mean is roughly constant no matter what the
     exposure is. An AE loop metering the POST-STRETCH image sees almost no
     response, concludes its correction did nothing, pushes harder, and slams
     into a rail. Metering must happen on the RAW data, which is why update()
     takes raw16 and not the processed image.

  2. CONTROLS TAKE EFFECT LATE. The sensor has frames in flight, so a write is
     not visible for two or three reads. Measuring immediately after writing
     means measuring the OLD exposure and correcting again on stale evidence --
     textbook oscillation. Hence SETTLE_FRAMES.

  3. FIXED STEPS CANNOT SETTLE. Brightness is roughly linear in exposure, so
     additive steps overshoot when far away and dither forever when close. This
     uses multiplicative correction with damping below 1, which converges
     geometrically, plus a deadband so it stops entirely once close enough.

  4. A BRIGHT WINDOW MUST NOT SET THE EXPOSURE. Metering the whole frame in this
     office drives exposure down until the room is black, protecting highlights
     nobody cares about. Metering is restricted to the lower-centre region --
     the floor and near obstacles, which is what a driving rover needs to see.
     A saturation guard still applies inside that region.


EXPOSURE IS A FRAME-RATE CEILING, so AE cannot treat it as free. Measured:

    exposure  1500 -> 30.6 fps    4000 -> 11.5 fps
              2290 -> 20.1 fps    8000 ->  5.8 fps

which fits frame_period ~ exposure * 21.9 us. So the exposure allowed by a
min_fps target is 1 / (min_fps * 21.9e-6), and brightness beyond that has to come
from GAIN instead. That ordering is also right on the merits for a moving rover:
gain adds noise, longer exposure adds motion blur, and blur destroys stereo
matching and feature tracking far more thoroughly than noise does.


MEASURED RESPONSE, 2026-08-17, ROI median in 10-bit counts

  gain at exposure 2283          exposure at gain 100
    100 ->  106   0.3% sat         300 ->  60   0.0% sat
    200 ->  198   0.7%             600 ->  77   0.2%
    400 ->  267  15.4%            1200 -> 110   0.3%
    600 ->  338  21.0%            2283 -> 170   0.6%
    800 ->  389  23.1%            4000 -> 261   8.7%
   1000 ->  462  27.7%
   1100 ->  504

Both monotonic over the usable range, so the linear model is sound enough for a
damped controller. The 1200 row is deliberately absent -- see GAIN_MAX below.

THE ROOM'S OWN LIGHT IS NOT STABLE. Exposure 2283 at gain 100 metered 106 in one
sweep and 170 in another minutes later -- a 60% swing at identical settings,
daylight through the windows. Two consequences: AE is not optional here, and any
"did it converge" test is being judged against a moving target, so trust the
control-write count and the settled spread rather than the absolute median.
"""
import argparse
import subprocess
import time

import numpy as np

EXP_MIN, EXP_MAX_HW = 2, 65535        # from v4l2-ctl -l
GAIN_MIN = 100

# v4l2-ctl ADVERTISES analogue_gain max=1200. It is not usable.
#
# Measured 2026-08-17, exposure fixed at 2283, each row reset to gain 100 first:
#
#     1000 -> applied     106 -> 456
#     1050 -> applied     170 -> 457
#     1100 -> applied     170 -> 504
#     1150 -> IGNORED     170 -> 170
#     1175 -> IGNORED     170 -> 171
#     1200 -> IGNORED     171 -> 171
#
# Above ~1100 the write is accepted, v4l2-ctl reads the new value back, and the
# SENSOR KEEPS ITS PREVIOUS GAIN. Nothing reports an error. That is the worst
# failure mode available: a silent no-op with a lying readback.
#
# It also invalidated a measurement of mine. A gain sweep showed 1000 -> 462 and
# 1200 -> 464, which I first read as the gain curve flattening out. It was not
# flattening; 1200 was a no-op holding 1000's value. Anything that looks like
# saturation at the top of this control should be suspected of being this bug.
GAIN_MAX = 1100
US_PER_EXP_UNIT = 21.9e-6             # fitted to the four measurements above

TARGET = 300.0        # desired ROI median, in 10-bit counts (0..1023)
DEADBAND = 0.18       # stop correcting inside +/-18%; wider than the noise
DAMP = 0.6            # exponent on the correction ratio; <1 to converge
SAT_LIMIT = 0.06      # ROI fraction at full scale that forces a decrease
SAT_CLEAR = 0.02      # ...and below which the ceiling may be relaxed again
SETTLE_FRAMES = 3     # reads to discard after writing a control
SLEW = 4.0            # largest brightness change one correction may request
BLACK_MED = 5.0       # ROI median at or below this is a fault, not a dark room

# BOTH OF THE ABOVE EXIST BECAUSE OF ONE OBSERVED FAILURE, worth recording since
# it is intermittent and will come back.
#
# Once, immediately after narsil-vision released the camera, the sensor delivered
# black frames for a full run. The controller did the worst possible thing with
# them: median 0 gives ratio = target/1 = 300, which demanded a 30x brightness
# increase, slammed exposure and gain to maximum in three writes, and then latched
# into "target unreachable, holding" -- reporting itself SETTLED on a black image.
#
# SLEW stops one bad frame from reaching the rails at all. BLACK_MED makes a black
# frame a loud fault instead of a converged state. Neither addresses the sensor
# transient itself, which I could not reproduce on demand in later runs -- these
# just ensure it can no longer be mistaken for success.

# THE FIRST VERSION OF THIS FILE LIMIT-CYCLED, and the reason is worth keeping.
#
# It treated "median below target" and "region too saturated" as two corrections
# of equal standing. In a backlit room they contradict each other: the floor
# cannot reach median 300 without the bright glare on the floor
# clipping past 6%. So it alternated -- up 1.29x, down 0.70x, up 1.29x -- forever,
# 38 control writes in 150 frames and a 31-count spread with no convergence.
#
# Damping cannot fix that. No gain below 1 converges a loop whose setpoint is
# unreachable; it just oscillates more slowly. The fix is to give the two demands
# different STATUS:
#
#   saturation is a HARD CEILING   exceeding it records an upper bound on
#                                  exposure*gain that later increases respect
#   the median is a SOFT GOAL      pursued only up to that ceiling
#
# When the ceiling binds and the median is still short, the scene simply exceeds
# the sensor's dynamic range. The controller says so and holds still, which is
# both stable and true. Recovering the shortfall is CLAHE's job downstream, not
# something more exposure can buy.
#
# The ceiling relaxes by RELAX per check once saturation falls below SAT_CLEAR,
# so driving from a bright room into a dark corridor still opens back up. That
# hysteresis gap -- 6% to trip, 2% to clear -- is what stops the ceiling itself
# from becoming a new oscillator.
RELAX = 1.15


class AutoExposure:
    def __init__(self, device='/dev/video0', min_fps=20.0, target=TARGET,
                 exposure=1500, gain=800, verbose=False):
        self._controls = None      # lazy: raisen.controls.Controls
        self.dev = device
        self.target = target
        self.verbose = verbose
        self.exp_max = int(min(1.0 / (min_fps * US_PER_EXP_UNIT), EXP_MAX_HW))
        self.exp = int(np.clip(exposure, EXP_MIN, self.exp_max))
        self.gain = int(np.clip(gain, GAIN_MIN, GAIN_MAX))
        self.skip = 0
        self.n = 0
        self.writes = 0
        self.settled_at = None
        self.sat_limited = False
        self.black = 0
        # Upper bound on exposure*gain/100, learned from saturation. Starts at
        # the hardware maximum, i.e. no constraint until saturation teaches one.
        self.ev_ceiling = self.exp_max * GAIN_MAX / 100.0
        self._apply()

    def _ev(self):
        return self.exp * self.gain / 100.0

    def _set_ev(self, ev):
        """Split a brightness target into exposure and gain.

        Exposure carries it up to the frame-rate ceiling, then gain takes over.
        That ordering matters for a moving rover: gain adds noise, exposure adds
        motion blur, and blur wrecks stereo matching and feature tracking far
        more thoroughly than noise does.
        """
        ev = max(ev, EXP_MIN * GAIN_MIN / 100.0)
        exp = min(self.exp_max, max(EXP_MIN, ev / (GAIN_MIN / 100.0)))
        gain = float(np.clip(ev / exp * 100.0, GAIN_MIN, GAIN_MAX))
        self.exp, self.gain = int(exp), int(gain)

    def _ctl(self, key, val):
        # Go through raisen.controls, not subprocess. This is the package copy and
        # it runs INSIDE vslam-min, which ships no v4l-utils -- shelling out to
        # v4l2-ctl there raises FileNotFoundError and kills the run at startup.
        # controls.Controls uses v4l2-ctl when it exists and the VIDIOC_S_CTRL
        # ioctl when it does not, so the same AE works on the host and in the
        # container. The standalone jetson/stereo/auto_exposure.py keeps its
        # subprocess call: it only ever runs on the host.
        if self._controls is None:
            from . import controls as _c
            self._controls = _c.Controls(self.dev)
            print('  AE control backend: %s' % self._controls.backend, flush=True)
        try:
            self._controls.set(key, val)
        except Exception as e:
            print('  AE could not write %s=%d: %s' % (key, val, e), flush=True)

    def _apply(self):
        self._ctl('exposure', self.exp)
        self._ctl('analogue_gain', self.gain)
        self.writes += 1
        self.skip = SETTLE_FRAMES

    @staticmethod
    def meter(raw16):
        """Median and saturated fraction of the lower-centre region, in 10-bit.

        The data is 10 bits left-aligned in a 16-bit container, so >>6 recovers
        0..1023. Strided by 4 because a full median over 3840x1200 costs more
        than the decision is worth.
        """
        h, w = raw16.shape
        roi = raw16[int(h * 0.50):int(h * 0.95), int(w * 0.25):int(w * 0.75)]
        ten = (roi[::4, ::4] >> 6).astype(np.float32)
        return float(np.median(ten)), float((ten >= 1015).mean())

    def update(self, raw16):
        """Call once per frame. Returns True if a control was written."""
        self.n += 1
        if self.skip > 0:
            self.skip -= 1
            return False

        med, sat = self.meter(raw16)
        ev = self._ev()

        # A black frame is a fault, not a dark room. Report it and change nothing:
        # there is no exposure that fixes a sensor delivering zeros, and reacting
        # to it only throws away a working setting.
        if med <= BLACK_MED:
            self.black += 1
            if self.black in (1, 30) or self.black % 300 == 0:
                print('  AE FAULT: ROI median %.1f at exp=%d gain=%d -- black '
                      'frames x%d. Holding settings; this is a sensor or driver '
                      'problem, not an exposure one.'
                      % (med, self.exp, self.gain, self.black), flush=True)
            return False
        self.black = 0

        # Saturation is the hard constraint: clipping teaches a ceiling that all
        # later increases must respect. Without this the loop oscillates, because
        # a clipped region reads as mid-grey and the median alone calls a
        # blown-out frame correctly exposed.
        if sat > SAT_LIMIT:
            self.ev_ceiling = min(self.ev_ceiling, ev * 0.85)
        elif sat < SAT_CLEAR and self.ev_ceiling < self.exp_max * GAIN_MAX / 100.0:
            # Headroom reappeared -- the scene got darker. Open up slowly.
            self.ev_ceiling *= RELAX
            self.sat_limited = False

        ratio = self.target / max(med, 1.0)
        # Slew limit before anything else: no single correction may ask for more
        # than SLEW times the current brightness in either direction.
        want = float(np.clip(ev * (ratio ** DAMP), ev / SLEW, ev * SLEW))
        capped = min(want, self.ev_ceiling)

        # Unreachable target: something binds and the median is still short. Say
        # so once and hold, rather than pushing against a wall every frame.
        #
        # The two reasons are NOT the same and an earlier version reported both
        # as saturation, which was simply false at 0.3% saturation:
        #
        #   saturation ceiling   the scene's dynamic range exceeds the sensor's.
        #                        More light would only clip more. CLAHE's problem.
        #   hardware ceiling     exposure is already at the min_fps cap and gain
        #                        at maximum. The room is genuinely too dark, and
        #                        the only fixes are accepting fewer fps or adding
        #                        light.
        at_hw_max = (self.exp >= self.exp_max and self.gain >= GAIN_MAX)
        if want > self.ev_ceiling and capped <= ev * 1.02:
            if not self.sat_limited:
                self.sat_limited = True
                self.settled_at = self.n
                why = ('AT HARDWARE CEILING: exposure capped at %d for the fps '
                       'floor and gain at max, room still under target'
                       % self.exp_max) if at_hw_max else (
                      'SATURATION-LIMITED: scene exceeds sensor dynamic range')
                if self.verbose:
                    print('  n=%4d med=%6.1f sat=%5.1f%%  %s, holding at '
                          'exp=%d gain=%d'
                          % (self.n, med, 100 * sat, why, self.exp, self.gain),
                          flush=True)
            return False

        # Deadband on the MEASURED ERROR, not on the damped step. Testing the
        # step instead lets DAMP shrink a real error below the threshold: a ratio
        # of 0.77 becomes a step of 0.85, which an 18% deadband swallows, and the
        # loop "settles" 30% off target. Measured that happening -- it held at
        # median 390 against a target of 300 and called itself converged.
        if abs(ratio - 1.0) < DEADBAND:
            if self.settled_at is None:
                self.settled_at = self.n
            return False

        self.settled_at = None
        self._set_ev(capped)
        self._apply()
        if self.verbose:
            print('  n=%4d med=%6.1f sat=%5.1f%% ratio=%.2f -> exp=%5d gain=%4d'
                  ' (ev %.0f, ceiling %.0f)'
                  % (self.n, med, 100 * sat, ratio, self.exp, self.gain,
                     self._ev(), self.ev_ceiling), flush=True)
        return True

    def state(self):
        return {'exposure': self.exp, 'gain': self.gain,
                'exp_max': self.exp_max, 'writes': self.writes,
                'fps_ceiling': 1.0 / (self.exp * US_PER_EXP_UNIT)}


def main():
    import cv2
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--start', type=int, default=100,
                    help='deliberately wrong starting exposure')
    ap.add_argument('--start-gain', type=int, default=100)
    ap.add_argument('--min-fps', type=float, default=20.0)
    ap.add_argument('--target', type=float, default=TARGET)
    ap.add_argument('--frames', type=int, default=200)
    a = ap.parse_args()

    W, H = 3840, 1200
    cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    if not cap.isOpened():
        raise SystemExit('cannot open %s -- narsil-vision still holding it?'
                         % a.device)
    for _ in range(5):
        cap.read()

    ae = AutoExposure(a.device, a.min_fps, a.target, a.start, a.start_gain,
                      verbose=True)
    print('start exp=%d gain=%d, target median %.0f, exposure capped at %d '
          'for >=%.0f fps' % (a.start, a.start_gain, a.target, ae.exp_max,
                              a.min_fps))
    time.sleep(0.5)

    hist = []
    t0 = time.time()
    for _ in range(a.frames):
        ok, buf = cap.read()
        if not ok:
            continue
        try:
            raw = buf.reshape(-1).view(np.uint16).reshape(H, W)
        except ValueError:
            continue
        ae.update(raw)
        med, sat = ae.meter(raw)
        hist.append((ae.n, med, ae.exp, ae.gain))
    cap.release()

    el = time.time() - t0
    print()
    print('frames %d in %.1f s (%.1f fps), %d control writes'
          % (len(hist), el, len(hist) / el, ae.writes))
    if ae.settled_at:
        print('settled at frame %d, then held without further writes'
              % ae.settled_at)
    else:
        print('DID NOT SETTLE -- still correcting at the end of the run')
    last = [h[1] for h in hist[-20:]]
    print('final   exp=%d gain=%d  ROI median %.1f (target %.0f)'
          % (ae.exp, ae.gain, np.mean(last), a.target))
    print('        last 20 frames median spread %.1f counts  <- hunting shows here'
          % (max(last) - min(last)))
    print('        frame-rate ceiling from this exposure: %.1f fps'
          % ae.state()['fps_ceiling'])


if __name__ == '__main__':
    main()
