#!/usr/bin/env python3
"""Fuse cuVSLAM rotation rate with the ESP32's absolute magnetic heading.

Runs INSIDE vslam-min:3.2 (needs rclpy). The IMU lives on the rover's ESP32, not
on the Jetson, so every sample is an HTTP round trip over WiFi -- measured at
19.2 Hz sustained, 52 ms mean latency.

    python3 fuse_heading.py --imu-host 172.10.10.188 --csv /workspaces/run.csv


WHY THIS FILTER, AND WHY THIS WAY ROUND

The usual stereo-plus-IMU assumption is that the IMU drifts and vision corrects
it. Here it is the opposite, and getting this backwards would make the output
worse than either input alone:

  vSLAM yaw   smooth, precise frame to frame, drifts without bound
  IMU yaw     magnetometer-corrected so effectively drift-free (0.18 deg/min),
              but noisy (0.93 deg std) and vulnerable to magnetic distortion

So vSLAM supplies the RATE and the IMU anchors the ABSOLUTE heading. A
complementary filter is the right shape: integrate vSLAM's per-frame delta, then
pull slowly toward the IMU. A Kalman filter would need noise models we have no
principled way to set, for no gain over one time constant.

  fused += wrap(yaw_vslam - yaw_vslam_prev)      # trust vision short-term
  fused += alpha * wrap(yaw_imu - fused)         # let the IMU win long-term

TAU is that time constant in seconds. Short tau chases the IMU's noise; long tau
lets vSLAM drift accumulate before it gets corrected. 12 s sits clear of both:
vSLAM's measured drift floor is 0.33 deg per 40 s, so 12 s of uncorrected drift
is well under a tenth of a degree.

MEASURED at tau=12, 57 s stationary, 1789 frames at 31.3 Hz:

  IMU raw     std 0.441 deg, peak-to-peak 2.827
  fused       std 0.116 deg, peak-to-peak 0.484     3.8x quieter
  vSLAM only  std 0.029 deg, drift 0.028 deg
  fused vs IMU  mean +0.100 deg -- stays locked to absolute heading

3.8x, not the ~15x that sqrt(12 s * 19 Hz) would predict. The sqrt-N estimate
assumes independent samples, and magnetometer noise is not independent -- it
wanders on a timescale comparable to tau, so averaging cannot beat it down the
way white noise would. Raising tau would buy a little more smoothing and give up
responsiveness for it; 3.8x is already far below the 1 deg that matters here.

NOTE WHAT THIS TEST DOES NOT SHOW. Stationary, vSLAM drifts 0.028 deg, so there
is nothing for the IMU to correct. It proves the filter runs at full rate, cuts
IMU noise, and holds absolute heading. The actual payoff -- bounding vSLAM's
unbounded ROTATIONAL drift -- only appears once the rover turns, which needs the
physical tests.


THREE THINGS THAT WILL BITE

  ANGLE WRAPPING. Every single difference between two angles must be wrapped to
  [-180, 180]. Without it, a rover crossing the +/-180 boundary sees a 360 deg
  step, the filter yanks hard, and the fused heading is garbage from then on.
  This is the classic bug in this kind of code, hence wrap() on every subtraction.

  TRACKING RESETS. cuVSLAM re-localises after losing features and its yaw can
  jump discontinuously. An unguarded delta then injects that jump straight into
  the output. Anything above MAX_RATE deg/s is physically impossible for this
  rover, so it is treated as a reset: re-anchor rather than integrate.

  IMU STALENESS. The poll runs in its own thread because a 52 ms blocking HTTP
  call inside the odometry callback would throttle the fusion to 19 Hz and add
  jitter to every vSLAM frame. If the WiFi drops, the last sample goes stale --
  past IMU_TIMEOUT the correction is suspended rather than applied against an old
  heading, and the output degrades to pure vSLAM instead of being pulled wrong.
"""
import argparse
import csv
import json
import math
import os
import threading
import time
import urllib.parse
import urllib.request

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

TAU = 12.0          # complementary filter time constant, seconds

# THE IMU CORRECTION MUST BE SUPPRESSED WHILE THE BASE TURNS UNDER POWER.
#
# The premise this filter was built on -- IMU drift-free, vision drifting -- holds
# at rest and INVERTS during a powered pivot. Measured on a 90 deg turn:
#
#     integrated gyro   +91.60 deg    reference
#     vSLAM             +99.42 deg    +8.5%
#     fused yaw         +32.56 deg    -64.5%
#
# The motors run centimetres from the magnetometer and this AHRS leans on it for
# absolute heading, so during a pivot the fused yaw is pulled toward a
# current-dependent direction instead of accumulating. On a longer pivot the rover
# was watched turning about 800 deg while the fused yaw stayed inside an 82 deg
# band. Correcting a good vSLAM rate toward that is strictly harmful.
#
# So gate on the gyro, which is in the same packet and is NOT magnetometer
# corrected: above GYRO_GATE the base is turning and the correction is skipped.
# HOLDOFF keeps it suppressed briefly afterwards, because the magnetometer needs
# time to settle once the current stops rather than snapping back instantly.
# GYRO_GATE is compared against BIAS-CORRECTED |gz|, not the raw reading. On
# 2026-08-18 the stationary bias on this rover measured +6.07 deg/s -- above this
# threshold -- so a gate on the raw value would have sat permanently CLOSED and the
# IMU correction would never have been applied at all. The bias was +0.37 deg/s the
# day before, so it is not a constant and cannot be compiled in: measure_gz_bias()
# estimates it at startup while the rover is still.
GYRO_GATE = 5.0     # deg/s of corrected |gz| above which the IMU heading is not trusted
HOLDOFF_S = 2.0     # keep suppressing for this long after rotation stops
MAX_RATE = 400.0    # deg/s above which a vSLAM delta is a reset, not motion
IMU_TIMEOUT = 1.5   # seconds before the IMU sample is too stale to correct with

# One list, used for the header AND checked against every row written.
CSV_COLS = ['t', 'x', 'y', 'z', 'yaw_vslam', 'yaw_vslam_int', 'yaw_imu',
            'imu_age', 'yaw_fused', 'gz', 'yaw_gyro', 'imu_gated', 'marker']


def wrap(a):
    """Fold an angle difference into [-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class ImuPoller:
    """Poll T:126 on the ESP32 in a background thread.

    Deliberately urllib, not rover.py: rover.py pulls in requests and starts a
    heartbeat thread that can send motion commands. Nothing in this file should
    be able to move the rover.
    """

    def __init__(self, host, hz):
        self.url = 'http://%s/js?json=%s' % (host, urllib.parse.quote('{"T":126}'))
        self.period = 1.0 / hz
        self.yaw = None
        self.gz = 0.0            # raw, as the ESP32 reports it
        self.gz_bias = 0.0       # subtracted by rate(); see measure_gz_bias
        # Integrated gyro yaw, kept alongside the fused one. Today this number had
        # to be carried by hand from the drive script's stdout into the analysis;
        # logging it makes a run self-contained.
        self.gyro_yaw = 0.0
        self.stamp = 0.0
        self.n = 0
        self.fails = 0
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

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
                            # Trapezoidal, over the actual interval between samples,
                            # on the BIAS-CORRECTED rate. Integrating raw gz is how a
                            # stationary rover accumulates 61 deg in ten seconds.
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
        """Return (yaw, age_seconds) or (None, None) if nothing has arrived."""
        with self.lock:
            if self.yaw is None:
                return None, None
            return self.yaw, time.time() - self.stamp

    def rate(self):
        """(corrected gz, integrated gyro yaw). Raw stays available as .gz."""
        with self.lock:
            return self.gz - self.gz_bias, self.gyro_yaw

    def measure_gz_bias(self, seconds=8.0, max_spread=1.5):
        """Estimate the gz offset while the rover is STILL. (bias, std, n).

        Refuses (returns 0.0) if the spread exceeds max_spread, because that means
        something was moving and calibrating against motion bakes the motion into
        every later reading. Nothing here can tell a slow steady rotation from an
        offset -- the magnetometer-corrected yaw can, and that is how the 6.07 deg/s
        reading was identified as a bias rather than a turn.
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
            # Everything integrated so far was integrated against a zero bias, i.e.
            # against the offset itself. Integration starts when calibration ends.
            self.gyro_yaw = 0.0
        return bias, sd, len(vals)


class Fuser(Node):
    def __init__(self, a):
        super().__init__('fuse_heading')
        self.imu = ImuPoller(a.imu_host, a.imu_hz)
        self.tau = a.tau

        self.fused = None        # absolute heading, IMU frame, degrees
        self.vprev = None        # previous vSLAM yaw, for the delta
        self.vint = None         # vSLAM integrated alone, for drift comparison
        self.tprev = None
        self.pos = (0.0, 0.0, 0.0)
        self.n = 0
        self.resets = 0
        self.suspended = 0
        self.gated = 0
        self.last_spin = 0.0

        self.pub = self.create_publisher(Float64, 'fused/heading_deg', 10)
        self.create_subscription(Odometry, a.topic, self.on_odom, 20)

        self.csv = self.mark = None
        if a.csv:
            self.fh = open(a.csv, 'w', newline='')
            self.csv = csv.writer(self.fh)
            self.csv.writerow(CSV_COLS)
        self.markfile = a.markfile

        # Wait for a first IMU sample: the whole output frame is anchored to the
        # IMU's absolute heading, so starting without one would mean re-anchoring
        # mid-run and invalidating everything logged before it.
        self.get_logger().info('waiting for first IMU sample from %s' % a.imu_host)
        t0 = time.time()
        while time.time() - t0 < 10.0:
            y, _ = self.imu.read()
            if y is not None:
                self.get_logger().info('IMU up, heading %+.2f deg -- filter '
                                       'anchored here, tau=%.0f s' % (y, self.tau))
                # The rover MUST be stationary now. The gate compares corrected
                # |gz| against GYRO_GATE, and an uncorrected bias larger than the
                # threshold holds the gate shut for the whole run.
                bias, sd, n = self.imu.measure_gz_bias(a.bias_seconds)
                if n < 5:
                    self.get_logger().warn('gz bias not measured (only %d samples) '
                                           '-- gate runs on the raw rate' % n)
                elif bias == 0.0 and sd > 1.5:
                    self.get_logger().error(
                        'gz spread %.2f deg/s over the calibration window -- the '
                        'rover was MOVING. Bias left at zero; hold it still and '
                        'restart, or the gate and the gyro integral are both wrong.'
                        % sd)
                else:
                    self.get_logger().info(
                        'gz bias %+.3f deg/s (std %.3f, n=%d) -- subtracted from '
                        'here on. Raw %+.3f would integrate to %+.0f deg/min.'
                        % (bias, sd, n, bias, bias * 60.0))
                return
            time.sleep(0.1)
        raise SystemExit('no IMU response from %s in 10 s -- rover powered on? '
                         'address moved again? (it has, twice)' % a.imu_host)

    def _take_marker(self):
        """A marker file lets another shell annotate the log mid-run:
             echo "loop start" > /tmp/mark
        Simpler than reading stdin, which is not interactive under docker+ssh."""
        if not self.markfile or not os.path.exists(self.markfile):
            return ''
        try:
            with open(self.markfile) as fh:
                label = fh.read().strip()
            os.unlink(self.markfile)
            self.get_logger().info('MARKER: %s' % label)
            return label
        except OSError:
            return ''

    def on_odom(self, m):
        now = time.time()
        p = m.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        yv = yaw_of(m.pose.pose.orientation)
        yi, age = self.imu.read()

        if self.fused is None:
            self.fused = yi
            self.vint = yi
            self.vprev = yv
            self.tprev = now
            return

        dt = max(now - self.tprev, 1e-3)
        self.tprev = now
        d = wrap(yv - self.vprev)
        self.vprev = yv

        # cuVSLAM can jump on re-localisation; that is not rover motion
        if abs(d) / dt > MAX_RATE:
            self.resets += 1
            self.get_logger().warn('vSLAM yaw jumped %+.1f deg in %.0f ms -- '
                                   'treating as a tracking reset, not motion'
                                   % (d, dt * 1000))
            d = 0.0

        self.fused = wrap(self.fused + d)
        self.vint = wrap(self.vint + d)

        # Correct toward the IMU only while its sample is fresh AND the base is not
        # turning. Stale means the link dropped, and pulling toward a two-second-old
        # heading is worse than not correcting. Turning means the magnetometer is
        # being dragged by motor current, which is worse still -- measured at -64.5%
        # on a 90 deg pivot.
        gz, gyro_yaw = self.imu.rate()
        if abs(gz) > GYRO_GATE:
            self.last_spin = now
        turning = (now - self.last_spin) < HOLDOFF_S
        if yi is None or age is None or age >= IMU_TIMEOUT:
            self.suspended += 1
        elif turning:
            self.gated += 1
        else:
            alpha = dt / (self.tau + dt)
            self.fused = wrap(self.fused + alpha * wrap(yi - self.fused))

        msg = Float64()
        msg.data = float(self.fused)
        self.pub.publish(msg)

        self.n += 1
        marker = self._take_marker()
        if self.csv:
            row = ['%.3f' % now, '%.4f' % p.x, '%.4f' % p.y,
                   '%.4f' % p.z, '%.3f' % yv, '%.3f' % self.vint,
                   '' if yi is None else '%.3f' % yi,
                   '' if age is None else '%.3f' % age,
                   '%.3f' % self.fused,
                   # gz stays RAW here: the column has meant the raw reading in
                   # every run recorded so far, and the bias is in the startup
                   # log and the final report.
                   '%.3f' % self.imu.gz,
                   '%.3f' % gyro_yaw, 1 if turning else 0, marker]
            # A dropped value here shifts every later column and is invisible in
            # the file: an edit that removed yaw_fused produced a 12-value row
            # under a 13-name header, and the whole run read as though the gyro
            # had integrated -100 deg. Assert the width rather than trust the edit.
            if len(row) != len(CSV_COLS):
                raise RuntimeError('CSV row has %d values for %d columns: %r'
                                   % (len(row), len(CSV_COLS), row))
            self.csv.writerow(row)
            if marker or self.n % 60 == 0:
                self.fh.flush()

        if self.n % 60 == 0:
            self.get_logger().info(
                'n=%d  fused=%+7.2f  imu=%+7.2f  vslam_only=%+7.2f  '
                'vslam-imu=%+6.2f deg  imu_polls=%d fails=%d'
                % (self.n, self.fused, yi if yi is not None else float('nan'),
                   self.vint, wrap(self.vint - (yi if yi is not None else 0.0)),
                   self.imu.n, self.imu.fails))

    def report(self):
        yi, _ = self.imu.read()
        print('\n=== fusion summary ===')
        print('odometry frames      %d' % self.n)
        print('IMU polls            %d ok, %d failed' % (self.imu.n, self.imu.fails))
        print('tracking resets      %d' % self.resets)
        print('corrections skipped  %d (stale IMU)' % self.suspended)
        print('corrections gated    %d (%.0f%% of frames -- base was turning, so '
              'the\n                     magnetometer heading was not trusted)'
              % (self.gated, 100.0 * self.gated / max(self.n, 1)))
        _, gyro_yaw = self.imu.rate()
        print('integrated gyro      %+.2f deg over the run' % gyro_yaw)
        if self.n and yi is not None:
            print('final heading        fused %+.2f   IMU %+.2f   vSLAM-only %+.2f'
                  % (self.fused, yi, self.vint))
            print('fused vs IMU         %+.2f deg' % wrap(self.fused - yi))
            print('vSLAM-only vs IMU    %+.2f deg  <- the drift the IMU removed'
                  % wrap(self.vint - yi))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--imu-host', required=True)
    ap.add_argument('--imu-hz', type=float, default=15.0)
    ap.add_argument('--topic', default='/visual_slam/tracking/odometry')
    ap.add_argument('--tau', type=float, default=TAU)
    ap.add_argument('--bias-seconds', type=float, default=8.0,
                    help='stationary window used to estimate the gz bias')
    ap.add_argument('--seconds', type=float, default=0.0, help='0 = run forever')
    ap.add_argument('--csv', default='')
    ap.add_argument('--markfile', default='/tmp/mark')
    a = ap.parse_args()

    rclpy.init()
    node = Fuser(a)
    end = time.time() + a.seconds if a.seconds else None
    try:
        while rclpy.ok() and (end is None or time.time() < end):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.imu.running = False
        node.report()
        if node.csv:
            node.fh.close()


if __name__ == '__main__':
    main()
