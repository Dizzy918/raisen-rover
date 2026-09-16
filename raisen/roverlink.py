"""Talking to rover-01's ESP32: heading, rates, and where it is on the network.

THE IMU IS ON THE ESP32, NOT THE JETSON, so every sample is a WiFi round trip.
Measured over one persistent session, stationary: 19.2 Hz sustained, latency mean
52 ms with a 122 ms tail. The rate is fine; the LATENCY is the real constraint,
which is why the IMU supplies only the slow absolute heading and vSLAM supplies
the rotation rate -- timestamp slop washes out of a low-pass reference.

TWO THINGS THAT DECIDE HOW THIS IS USED:

  T:126's fused yaw is MAGNETOMETER-CORRECTED, so at rest it is the drift-free
  signal (0.183 deg/min against raw gyro integration's 22.4). Under powered
  rotation it is SUPPRESSED -- the motors sit centimetres from the magnetometer --
  measured -64.5% on a 90 deg pivot, and stuck inside an 82 deg band across a
  watched ~800 deg turn. Raw gz in the same packet is not corrected and is the
  right rotation source while driving: bias 0.37 deg/s.

  DELIBERATELY urllib, NOT rover.py. rover.py starts a 10 Hz heartbeat thread
  that can send motion commands. Nothing that only wants to READ the IMU should
  be able to move the rover.

MOTION HAZARD, unchanged: do not drive rover-01 from here while jetson11 also
targets it. Two 10 Hz heartbeat threads fighting over one base is a physical
hazard, not a software clash.
"""
import json
import threading
import time
import urllib.parse
import urllib.request

# DO NOT TRUST A HARDCODED GYRO BIAS. Measured stationary bias on gz:
#
#   2026-08-17   +0.37 deg/s
#   2026-08-18   +6.07 deg/s     same rover, same firmware, 16x larger
#
# Rock steady within a session -- four consecutive 10 s windows gave 6.079, 6.066,
# 6.063, 6.059 with a std of 0.23 and no drift across a 1 C temperature rise -- and
# completely different BETWEEN sessions. The likely cause is a boot-time bias
# estimate captured while the rover was being handled on the charger, which then
# stays baked in until it is power-cycled sitting still.
#
# Why this matters more than it looks: 6.07 deg/s integrates to +61 deg in ten
# seconds. Anything integrating raw gz for turn feedback would believe it had
# turned 90 degrees having barely moved, and any gate comparing raw |gz| against a
# 5 deg/s threshold sits permanently closed. Both fail SILENTLY, with
# plausible-looking numbers.
#
# So measure it, every session, with measure_gz_bias(). The value below is kept
# only as the historical reading it was, and is not used as a default anywhere.
GYRO_BIAS_HISTORICAL_DPS = 0.37
GATE_MARGIN_DPS = 5.0      # gate threshold, applied to BIAS-CORRECTED |gz|


def resolve_host(bin_dir='/home/narsil/bin'):
    """Ask rover.py where the rover is, without gaining the ability to drive it.

    Its address moved three times in four days (.196 -> .194 -> .188), so the IP
    on disk is not trustworthy; constructing Rover() re-resolves by MAC and
    persists the result. This sends no motion command -- the heartbeat thread only
    transmits once drive() has been called -- but it DOES import rover.py, so keep
    it in a short-lived helper rather than in a long-running node.
    """
    import sys
    if bin_dir not in sys.path:
        sys.path.insert(0, bin_dir)
    from rover import Rover
    with Rover() as r:
        if r.imu() is None:
            raise RuntimeError('rover did not answer T:126 -- powered on?')
        return r.cfg['host']


class ImuPoller:
    """Poll T:126 in a background thread and hold the latest sample."""

    def __init__(self, host, hz=15.0):
        self.url = 'http://%s/js?json=%s' % (host, urllib.parse.quote('{"T":126}'))
        self.period = 1.0 / hz
        self.yaw = None
        self.gz = 0.0            # raw, as reported
        self.gz_bias = 0.0       # subtracted by rate(); set by measure_gz_bias()
        self.gyro_yaw = 0.0      # integrated from BIAS-CORRECTED gz
        self.stamp = 0.0
        self.n = 0
        self.fails = 0
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._loop, daemon=True,
                         name='imu-poller').start()

    def _loop(self):
        while self.running:
            t = time.time()
            try:
                with urllib.request.urlopen(self.url, timeout=1.0) as r:
                    d = json.loads(r.read().decode())
                if 'y' in d:
                    now = time.time()
                    with self.lock:
                        gz = float(d.get('gz', 0.0))
                        if self.yaw is not None:
                            # Trapezoidal, over the actual sample interval, on the
                            # BIAS-CORRECTED rate. Integrating the raw value is how
                            # a stationary rover accumulates 61 deg in ten seconds.
                            self.gyro_yaw += (0.5 * ((gz - self.gz_bias)
                                                     + (self.gz - self.gz_bias))
                                              * (now - self.stamp))
                        self.gz = gz
                        self.yaw = float(d['y'])
                        self.stamp = now
                        self.n += 1
                else:
                    self.fails += 1
            except Exception:
                self.fails += 1
            time.sleep(max(0.0, self.period - (time.time() - t)))

    def read(self):
        """(yaw, age_seconds) or (None, None) if nothing has arrived yet."""
        with self.lock:
            if self.yaw is None:
                return None, None
            return self.yaw, time.time() - self.stamp

    def rate(self):
        """(gz_deg_s BIAS-CORRECTED, integrated_gyro_yaw_deg).

        Corrected, not raw, because every caller so far wanted a rotation rate and
        none wanted a sensor offset. Raw is still available as .gz for diagnostics.
        """
        with self.lock:
            return self.gz - self.gz_bias, self.gyro_yaw

    def measure_gz_bias(self, seconds=8.0, max_spread=1.5):
        """Estimate the gz offset while the rover is STATIONARY, and adopt it.

        Returns (bias, std, n). Refuses and returns a bias of 0.0 if the samples
        spread more than max_spread deg/s, because that means something was moving
        and calibrating against motion bakes the motion into every later reading.

        The caller is responsible for the rover actually being still. There is no
        way to tell a slow steady rotation from a bias inside this window -- the
        magnetometer-corrected yaw can, and does, which is how the 6.07 deg/s
        offset was identified as a bias rather than a turn.
        """
        import statistics
        vals, t0 = [], time.time()
        while time.time() - t0 < seconds:
            with self.lock:
                if self.yaw is not None:
                    vals.append(self.gz)
            time.sleep(self.period)
        if len(vals) < 5:
            return 0.0, float('nan'), len(vals)
        bias = statistics.fmean(vals)
        sd = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        if sd > max_spread:
            return 0.0, sd, len(vals)
        with self.lock:
            self.gz_bias = bias
            # Restart the integral. Everything accumulated up to here was integrated
            # against a bias of zero, i.e. against the offset itself -- an 8 s
            # calibration window at 6 deg/s leaves about 48 deg of pure fiction in
            # gyro_yaw. Integration starts when calibration ends.
            self.gyro_yaw = 0.0
        return bias, sd, len(vals)

    def wait_first(self, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            y, _ = self.read()
            if y is not None:
                return y
            time.sleep(0.1)
        return None

    def stop(self):
        self.running = False
