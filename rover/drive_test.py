#!/usr/bin/env python3
"""Drive rover-01 on repeatable test moves, annotating the vSLAM/IMU log.

Runs on jetson9 (host, not in the container). Complements the esp-sonar branch's
jetson_control/: that code is a continuous obstacle-avoiding drive, this one does
bounded, measurable moves for calibrating and testing vSLAM.

    python3 drive_test.py preflight                        # no motion, always do this first
    python3 drive_test.py straight --speed 0.2 --seconds 6
    python3 drive_test.py turn --degrees 90
    python3 drive_test.py stop
    ... any command with --dry-run sends nothing and prints what it would send

Pair it with imu/run_fusion.sh in another shell. This writes /tmp/mark, which the
fusion node folds into its CSV, so each move is labelled in the log rather than
having to be matched up by timestamp afterwards.


WHY THIS EXISTS RATHER THAN REUSING obstacle_avoid_drive.py

That script cruises until something is close, which is the product behaviour. For
testing we need the opposite: a known command held for a known time, so vSLAM's
measurement can be compared against something. It also answers a question nobody
can currently answer -- what 0.35 means in m/s. The wheel command is a PWM
fraction, and the whole accel/cruise/slow design is specified in units that do not
convert to metres. Drive a fixed command, let vSLAM measure the distance, and the
scale factor falls out.


SAFETY, because this moves a wheeled base over a hard floor

  PREFLIGHT DETECTS A SECOND CONTROLLER. poc-isaac-ros warns not to drive rover-01
  from jetson9 while jetson11 also targets it, since two 10 Hz heartbeat threads
  fight over one base. That cannot be checked over the network, but it CAN be
  detected physically: command zero, then watch the gyro. If the base is moving
  while we command stop, somebody else is driving, and we abort.

  THE FIRMWARE DEADMAN IS THE LAST RESORT, NOT THE PLAN. The base stops about 3 s
  after commands cease, so a SIGKILL leaves it rolling for up to 3 s. Every exit
  path here commands zero repeatedly, and SIGINT and SIGTERM are handled.

  SONAR GATES EVERY CYCLE, and a failed read stops rather than being ignored. Note
  the reading can be ~0.5 s stale: the firmware loop is 4 x 60 ms of pulse
  timeouts plus a 200 ms delay. So STOP_CM is deliberately far larger than the
  10 cm trigger in the branch code -- at 0.2 the rover covers real ground in half
  a second, and a stale 30 cm may already be 15 cm.

  EVERY MOVE IS TIME BOUNDED. No loop can run longer than --max-seconds, whatever
  else fails.
"""
import argparse
import json
import math
import signal
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request

ROVER_IP = '172.10.10.188'      # e0:8c:fe:25:de:24, moves on DHCP -- see imu/README.txt
SONAR_IP = '172.10.10.192'      # 1c:69:20:30:16:a4
CONTROL_HZ = 10.0
MAX_ACCEL = 0.3                 # per second, matching the esp-sonar branch
STOP_CM = 40.0                  # abort forward motion inside this
SONAR_FAIL_LIMIT = 3

# MEASURED MOTOR DEADBAND, pivoting on this floor, 2026-08-17:
#
#     0.22 -> stalled (peak 15 deg/s, i.e. noise)
#     0.24 -> moves   (25 deg/s)
#     0.26 -> moves   (38 deg/s)
#     0.28 -> moves   (70 deg/s)
#     0.30 -> moves   (77 deg/s)
#     0.40 -> moves   (154 deg/s -- nearly a full circle in 1.5 s)
#
# Below ~0.23 the wheels do not turn at all. This wasted a 21 s run: a closed-loop
# turn commanded at 0.20 sat still while the controller patiently reported a 93 deg
# error it could never correct.
#
# IT ALSO BREAKS THE esp-sonar BRANCH. sonar_speed.py eases to min_speed = 0.08
# when approaching an obstacle, and 0.08 is a third of the deadband -- so that
# "slow down" is really a dead stop. Any speed floor there needs to be above 0.24.
#
# The floor also has to be applied in ABSOLUTE command units, not as a fraction of
# the requested speed, which is the bug this replaced.
# 0.24 IS NOT THE STRAIGHT-LINE DEADBAND. Measured properly on 2026-08-18 by
# running the Jetson's rover2_obstacle_control.py at command 0.10 and timing its
# sonar readout, the chassis moves at 0.079 m/s there, and three points fit
# speed = 1.238*command - 0.0469, putting the real floor at command 0.038.
# 0.24 was almost certainly measured in reverse, before it was known that positive
# L/R drives this rover backward. It is kept here only because the easing logic and
# TURN_MIN_CMD were tuned around it, and lowering it untested would change how every
# turn behaves. TURN_MIN_CMD is separately plausible: pivoting scrubs the wheels
# sideways and genuinely needs more torque than rolling. Neither has had a low-end
# sweep. See rover/README.txt.
DEADBAND_CMD = 0.24
TURN_MIN_CMD = 0.26     # deadband plus margin
MARKFILE = '/tmp/mark'
SIM_DEG_PER_S_PER_CMD = 240.0    # dry-run fiction only, see Rover.__init__


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def get(url, timeout=1.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read().decode()


class Rover:
    def __init__(self, ip, dry_run=False):
        self.base = 'http://%s/js?json=' % ip
        self.dry = dry_run
        self.left = 0.0
        self.right = 0.0
        self.sent = 0
        # Dry-run simulation state. SIM_DEG_PER_S_PER_CMD is a guess, not a
        # measurement -- its only job is to make the closed-loop turn actually
        # execute so the easing, sign handling and termination get exercised
        # without moving anything. Measuring the real value is the point of the
        # first live turn.
        self.sim_yaw = 0.0
        self.sim_t = time.time()

    def raw(self, payload):
        if self.dry:
            return '{}'
        return get(self.base + urllib.parse.quote(json.dumps(payload)))

    def speed(self, left, right):
        left = max(-0.5, min(0.5, left))
        right = max(-0.5, min(0.5, right))
        self.left, self.right = left, right
        self.sent += 1
        if self.dry:
            now = time.time()
            self.sim_yaw = wrap(self.sim_yaw
                                + (right - left) * 0.5 * SIM_DEG_PER_S_PER_CMD
                                * (now - self.sim_t))
            self.sim_t = now
            return
        try:
            self.raw({'T': 1, 'L': round(left, 3), 'R': round(right, 3)})
        except Exception as e:
            # A dropped command is not fatal on its own -- the next cycle is 100 ms
            # away and the deadman is 3 s -- but it must be visible.
            print('  WARN send failed: %s' % e, flush=True)

    def stop(self, times=4):
        """Repeat, because this is the last thing that runs and one packet can be lost."""
        for _ in range(times):
            self.speed(0.0, 0.0)
            time.sleep(0.05)

    def imu(self):
        if self.dry:
            return {'y': self.sim_yaw, 'gx': 0.0, 'gy': 0.0, 'gz': 0.0, 'az': 1000.0}
        try:
            d = json.loads(self.raw({'T': 126}))
            # An empty or malformed reply must read as "no IMU", not as yaw 0.
            return d if isinstance(d, dict) and 'y' in d else None
        except Exception:
            return None


# WHICH CHANNEL POINTS WHERE, confirmed on the hardware 2026-08-18.
#
# This was wrong here until now, and it was wrong in the dangerous direction:
# forward motion was gated on sonar1, which faces RIGHT. On the morning of the
# 18th sonar1 read 14 cm and blocked a straight run while the actual front was
# clear at 93 cm. Had the geometry been the other way round -- an obstacle ahead
# and a clear right side -- the same bug would have driven into it.
#
# PROJECT_CONTEXT.md on the esp-sonar branch says "1 (front)". It is wrong for
# this rover. rover2_obstacle_control.py reads sonar4 as the front, and that
# agrees with the hardware.
#
# THERE ARE THREE SONARS, NOT FOUR. sonar3 returns no echo on 40 of 40 samples and
# is not going to exist -- confirmed as final, not a fault to chase. The remaining
# three map exactly onto the {fwd, left, right} contract algorythm/rover.py expects,
# so nothing downstream wants a fourth. What it does mean is that there is NO SPARE:
# lose any one of these and the wall-follower loses a side or the collision guard.
SONAR_CH = {'front': 'sonar4', 'left': 'sonar2', 'right': 'sonar1'}


# THE SONAR EMITS OCCASIONAL WILD SINGLE SAMPLES AND THE GUARD MUST NOT BELIEVE
# THEM. Measured against a wall that a median of four reads put at 130.9 cm:
# one sample came back 70.8, and on the first attempted straight run one came back
# 5.5 cm -- one second after starting, with the command still at 0.03, i.e. below
# the deadband, so the rover had not moved at all. That single reading aborted the
# run. Truth cannot change 131 cm in one second; the sensor can.
#
# So: poll in the background at the firmware's own rate and brake on the MEDIAN of
# the last few, not on whatever arrived last. The median rejects one outlier in
# either direction, and both directions matter -- a spurious SHORT stops a good run
# (annoying, safe) while a spurious LONG would hide a real obstacle (dangerous).
#
# Polling in a thread also stops the 10 Hz control loop issuing 10 HTTP requests a
# second at a box whose full sensor cycle is about 440 ms. It could only ever have
# returned the same numbers, and hammering it may well be part of why the garbage
# samples appear at all.
# POSITIVE L/R DRIVES THIS ROVER BACKWARD. Measured 2026-08-18: commanding
# +0.30 for 2.5 s moved the FRONT sonar's reading from 131 cm to 169 cm, i.e. the
# rover retreated from the wall it was facing. esp-sonar's
# rover2_obstacle_control.py already sends {"L":-speed,"R":-speed} to go forward;
# PROJECT_CONTEXT.md's "positive = forward" is wrong for this chassis.
#
# WHY THIS HID FOR SO LONG. turn() is CLOSED LOOP on the gyro -- it drives until
# the measured rotation reaches the target, so it does not care how the wheels are
# wired and a commanded +45 deg came out +45.84 by vSLAM regardless. Only
# drive_straight is open loop, and open loop is where a sign error shows up. The
# first straight run this rover ever did is what exposed it.
FORWARD_SIGN = -1.0

SONAR_POLL_S = 0.45
SONAR_WINDOW = 5
SONAR_MEDIAN_OF = 3


class Sonar:
    def __init__(self, ip, dry_run=False):
        self.url = 'http://%s/sonar' % ip
        self.dry = dry_run
        self.fails = 0
        self.ok = 0
        self.last = {}
        self.hist = {k: [] for k in SONAR_CH}
        self.lock = threading.Lock()
        self.running = True
        if not dry_run:
            threading.Thread(target=self._loop, daemon=True,
                             name='sonar-poll').start()

    def _loop(self):
        while self.running:
            t0 = time.time()
            try:
                d = json.loads(get(self.url))
                out = {}
                for name, ch in SONAR_CH.items():
                    v = float(d.get(ch, -1))
                    # 999 stands for "nothing within range": -1 is the firmware's
                    # pulseIn timeout, and treating it as zero would brake for
                    # empty space.
                    out[name] = 999.0 if v < 0 else v
                with self.lock:
                    self.last = out
                    self.ok += 1
                    self.fails = 0
                    for name, v in out.items():
                        h = self.hist[name]
                        h.append(v)
                        del h[:-SONAR_WINDOW]
            except Exception:
                with self.lock:
                    self.fails += 1
            time.sleep(max(0.0, SONAR_POLL_S - (time.time() - t0)))

    def read_all(self):
        """Filtered {'front','left','right'} in cm, or None if nothing usable yet."""
        if self.dry:
            return {k: 999.0 for k in SONAR_CH}
        with self.lock:
            out = {}
            for name in SONAR_CH:
                h = self.hist[name][-SONAR_MEDIAN_OF:]
                if not h:
                    return None
                out[name] = sorted(h)[len(h) // 2]
            return out

    def raw_window(self, name='front'):
        """The unfiltered samples behind the median, for the log."""
        with self.lock:
            return list(self.hist[name][-SONAR_MEDIAN_OF:])

    def front_cm(self):
        got = self.read_all()
        return None if got is None else got['front']

    def wait_ready(self, timeout=4.0):
        """Block until the window has enough samples to form a median."""
        if self.dry:
            return True
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if len(self.hist['front']) >= SONAR_MEDIAN_OF:
                    return True
            time.sleep(0.1)
        return False


def step_toward(cur, target, max_step):
    d = target - cur
    if abs(d) <= max_step:
        return target
    return cur + max_step * (1.0 if d > 0 else -1.0)


def mark(label):
    try:
        with open(MARKFILE, 'w') as fh:
            fh.write(label)
    except OSError:
        pass


def preflight(rover, sonar, need_sonar=True):
    """Checks that must pass before anything moves.

    need_sonar is False for turns. Sonar is a FORWARD-collision sensor; a pivot in
    place does not approach anything with it, so requiring it there would block a
    safe move for no safety gain. Forward motion still hard-fails without it.
    """
    ok = True
    print('=== preflight ===')

    if not sonar.wait_ready():
        print('  WARN sonar window not filled in 4 s; readings may be unfiltered')

    d = rover.imu()
    if d is None or 'y' not in d:
        print('  FAIL rover %s not answering T:126'
              % rover.base.split('//')[1].split('/')[0])
        return False
    print('  rover reachable, yaw %+.2f deg, az %.0f milli-g' % (d['y'], d.get('az', 0)))

    if abs(d.get('az', 1000)) < 900:
        print('  WARN az %.0f is not ~1000 milli-g; is the rover level and upright?'
              % d.get('az', 0))

    c = sonar.front_cm()
    if c is None:
        if need_sonar:
            print('  FAIL sonar %s not answering -- required for forward motion'
                  % SONAR_IP)
            ok = False
        else:
            print('  WARN sonar %s not answering. Allowed for a pivot in place, '
                  'which does not\n       approach anything forward. Clear the '
                  'space anyway.' % SONAR_IP)
    else:
        allc = sonar.last or {}
        print('  sonar front %.1f cm%s   (left %.0f, right %.0f)'
              % (c, '  (nothing in range)' if c > 900 else '',
                 allc.get('left', -1), allc.get('right', -1)))
        if c < STOP_CM:
            print('  FAIL obstacle inside the %.0f cm stop threshold' % STOP_CM)
            ok = False

    # A second controller shows up as SUSTAINED motion while we command stop.
    #
    # Judging on the peak gave false alarms: two consecutive preflights reported
    # 16.4 and 16.2 deg/s peaks on a base that a separate 12 s read showed to be
    # still (mean 0.69, peak 1.66, nothing above 8). The difference was the poll
    # rate -- preflight polled at 10 Hz straight after sending stop commands,
    # which makes the ESP32 return occasional garbage gyro samples. So the spike
    # was an artefact of measuring, not motion.
    #
    # Hence: poll gently, and require the motion to PERSIST. A controller fighting
    # us drives the base continuously; it does not produce one sample and stop.
    #
    # AND JUDGE THE VARIATION, NOT THE MAGNITUDE. This check used to test the raw
    # gyro magnitude against a threshold, which broke on 2026-08-18 when the gz bias
    # measured +6.07 deg/s: a stationary rover reported a median of 6.11 and the
    # check declared "the base is moving CONTINUOUSLY". It failed SAFE, but it
    # blocked a legitimate move and the reason was a sensor offset, not a controller.
    #
    # The discriminator is steadiness. A DC offset is constant; a controller driving
    # the base produces motion that varies as it ramps, corrects and reverses. So
    # take the per-axis median as the offset and judge what is LEFT once it is
    # removed. An offset too large to be credible as a bias is refused outright,
    # because at that point the two cases genuinely cannot be told apart from here.
    print('  commanding stop, then watching the gyro for a second controller...')
    rover.stop()
    time.sleep(1.0)
    samples, yaws = [], []
    for _ in range(20):
        d = rover.imu()
        if d:
            samples.append((d.get('gx', 0.0), d.get('gy', 0.0), d.get('gz', 0.0)))
            if 'y' in d:
                yaws.append(d['y'])
        time.sleep(0.25)
    if len(samples) < 8:
        print('  FAIL only %d IMU samples during the stop check' % len(samples))
        ok = False
    else:
        def _median(xs):
            xs = sorted(xs)
            return xs[len(xs) // 2]

        off = tuple(_median([s[i] for s in samples]) for i in range(3))
        off_mag = math.sqrt(sum(v * v for v in off))
        resid = sorted(math.sqrt(sum((s[i] - off[i]) ** 2 for i in range(3)))
                       for s in samples)
        res_med = resid[len(resid) // 2]
        res_peak = resid[-1]
        hot = sum(1 for v in resid if v > 8.0)
        yaw_net = (wrap(yaws[-1] - yaws[0]) if len(yaws) >= 2 else float('nan'))
        print('    gyro offset  (%+.2f, %+.2f, %+.2f) = %.2f deg/s  <- steady part'
              % (off + (off_mag,)))
        print('    residual     median %.2f, peak %.2f deg/s, %d above 8  <- varying'
              % (res_med, res_peak, hot))
        print('    fused yaw    net %+.2f deg over %.0f s' % (yaw_net,
                                                              0.25 * len(samples)))
        if res_med > 3.0 or hot >= len(resid) // 3:
            print('  FAIL motion that VARIES while we command zero. That is a')
            print('       controller, not a sensor offset -- check jetson11. Two')
            print('       controllers on one base is a motion hazard.')
            ok = False
        elif off_mag > 15.0:
            print('  FAIL a steady %.1f deg/s is too large to accept as a gyro'
                  % off_mag)
            print('       bias, and a controller holding a constant rate looks')
            print('       identical from here. Power-cycle the rover sitting still')
            print('       and re-run; if it persists, something is driving it.')
            ok = False
        else:
            if off_mag > 1.0:
                print('    steady offset of %.2f deg/s treated as GYRO BIAS, not'
                      ' motion:' % off_mag)
                print('      it does not vary, and the magnetometer-corrected yaw'
                      ' is not turning.')
                print('      the turn controller measures and subtracts this'
                      ' separately.')
            if res_peak > 8.0:
                print('    isolated residual spike but not sustained -- sampling'
                      ' artefact or a bump')
            print('    no other controller detected')

    others = subprocess.run(['pgrep', '-af', 'obstacle_avoid_drive|rover.py teleop'],
                            capture_output=True, text=True).stdout.strip()
    if others:
        print('  FAIL another driver is running on this host:\n    %s' % others)
        ok = False

    print('=== preflight %s ===' % ('PASSED' if ok else 'FAILED'))
    return ok


def drive_straight(rover, sonar, a):
    """Hold a fixed command for a fixed time. Deliberately open loop: what vSLAM
    measures is the result, so closing the loop here would hide what we came for."""
    mark('straight %.2f start' % a.speed)
    print('driving %.2f for %.1f s (wire sign %+.0f, so the front sonar should '
          'COUNT DOWN)' % (a.speed, a.seconds, FORWARD_SIGN))
    start_front = sonar.front_cm()
    t0 = time.time()
    cur = 0.0
    period = 1.0 / CONTROL_HZ
    reason = 'completed'
    while True:
        now = time.time()
        el = now - t0
        if el >= a.seconds:
            break
        if el > a.max_seconds:
            reason = 'MAX SECONDS'
            break
        c = sonar.front_cm()
        if c is None:
            if sonar.fails >= SONAR_FAIL_LIMIT:
                reason = 'sonar unreadable %dx' % sonar.fails
                break
        elif c < STOP_CM:
            reason = 'obstacle at %.1f cm' % c
            break
        # Ramp down over the last stretch so the run ends smoothly rather than
        # lurching, which would corrupt the very measurement being taken.
        #
        # --hard-stop skips that and cuts the motors dead at `seconds`, which is
        # the ONLY way to measure braking distance. That number is the unmeasured
        # term in every collision-safety budget here: reaction distance is sonar
        # staleness plus command latency plus braking, and the first two are known.
        target = a.speed if (el < a.seconds - 0.6 or a.hard_stop) else 0.0
        cur = step_toward(cur, target, MAX_ACCEL / CONTROL_HZ)
        # cur stays positive so the log and the ramp read naturally; the chassis
        # sign is applied only at the wire.
        rover.speed(FORWARD_SIGN * cur, FORWARD_SIGN * cur)
        print('  t=%4.1f s  cmd=%+.3f  sonar=%s  raw=%s' % (el, cur,
              '%.0f cm' % c if c is not None else 'FAIL',
              ','.join('%.0f' % v for v in sonar.raw_window())), flush=True)
        time.sleep(period)
    mark('MOTORS CUT')       # written BEFORE the stop, so the cut is locatable
    rover.stop()
    mark('straight end (%s)' % reason)
    time.sleep(1.5)          # let the sonar window refill after the motors stop
    end_front = sonar.front_cm()
    print('stopped: %s after %.1f s' % (reason, time.time() - t0))
    if start_front is not None and end_front is not None:
        moved = start_front - end_front
        print('front sonar %.1f -> %.1f cm, so the rover advanced %+.1f cm'
              % (start_front, end_front, moved))
        print('  NOTE this is a lagged estimate: the box cycles at about 2.3 Hz and')
        print('  the guard medians three samples, so up to ~1.3 s of travel is not')
        print('  yet reflected. vSLAM is the measurement that settles the distance.')


def turn(rover, sonar, a):
    """Pivot until the IMU's absolute heading has moved by --degrees.

    Closed loop, unlike the branch's open-loop 1.2 s pivot, which produces an
    unknown angle. Two things bound the accuracy: the IMU's 0.93 deg of noise, and
    ~52 ms of HTTP latency per sample, which at 30 deg/s is 1.5 deg of lag. So it
    eases off near the target and REPORTS what it achieved rather than claiming the
    commanded angle.
    """
    d = rover.imu()
    if d is None:
        print('no IMU response -- refusing to turn blind')
        return
    start = d['y']

    # FEEDBACK COMES FROM INTEGRATED GYRO, NOT THE FUSED YAW.
    #
    # The ESP32's fused yaw is magnetometer-corrected, and a pivot runs both motors
    # hard centimetres from that magnetometer. Measured: the rover was watched
    # turning about 800 deg while the fused yaw stayed inside an 82 deg band and
    # never wrapped -- it was pulled to a motor-current-dependent direction instead
    # of accumulating. Every closed-loop turn timed out because of this: the
    # controller was obeying a sensor that had stopped reporting rotation.
    #
    # Raw gz is in the same T:126 packet and is not magnetometer-corrected, so it
    # keeps reporting rotation when the fused heading stops. The fused yaw is still
    # logged for comparison, just not trusted for control.
    #
    # BUT THE BIAS MUST BE MEASURED, NOT ASSUMED. It was +0.37 deg/s on 2026-08-17
    # and +6.07 deg/s on 2026-08-18 -- same rover, same firmware, 16x larger, rock
    # steady within each session. At 6.07 deg/s a stationary rover integrates +61 deg
    # in ten seconds, so this controller would declare a 90 deg turn "reached" having
    # barely moved, and it would look like a success in the log. The bias is therefore
    # sampled here, while the rover is definitely still, immediately before driving.

    # ACCUMULATE rotation; do not chase a wrapped absolute target.
    #
    # A wrapped target is degenerate for large angles: wrap(start + 360) == start,
    # so a commanded 360 has zero error at t=0. Measured -- "reached: commanded
    # +360, achieved +0.9 deg in 1.2 s", having not moved at all. Anything at or
    # beyond 180 is ambiguous the same way.
    #
    # Summing wrapped per-sample deltas handles any angle, including multiple
    # revolutions, as long as no single gap exceeds 180 deg of rotation. At 14.5 Hz
    # IMU updates and the ~29 deg/s this rover pivots at, one update is 2 deg, so
    # there is a wide margin.
    print('yaw start %+.2f, accumulating toward %+.0f deg' % (start, a.degrees))
    mark('turn %+.0f start' % a.degrees)

    t0 = time.time()
    cur = 0.0
    period = 1.0 / CONTROL_HZ
    reason = 'completed'
    last = start
    last_sign = None
    reversals = 0
    # Sample the bias with the motors off. Refuse to turn if the spread says the
    # rover is not actually still: calibrating against motion bakes that motion into
    # every subsequent reading, and the error compounds for the whole turn.
    bias_vals = []
    t_bias = time.time()
    while time.time() - t_bias < a.bias_seconds:
        db = rover.imu()
        if db is not None:
            bias_vals.append(float(db.get('gz', 0.0)))
        time.sleep(0.05)
    if len(bias_vals) < 5:
        # sys.exit, not return: turn() has no return contract and main() ignores
        # what it hands back, so a `return` here would exit 0 and read as success.
        # main's finally still commands the base to zero on the way out.
        sys.exit('gz bias NOT measured (%d samples) -- refusing to turn on an '
                 'uncalibrated gyro' % len(bias_vals))
    gz_bias = sum(bias_vals) / len(bias_vals)
    spread = (max(bias_vals) - min(bias_vals))
    sd = (sum((v - gz_bias) ** 2 for v in bias_vals) / len(bias_vals)) ** 0.5
    print('gz bias %+.3f deg/s (std %.3f, spread %.3f, n=%d) over %.0f s stationary'
          % (gz_bias, sd, spread, len(bias_vals), a.bias_seconds))
    if sd > 1.5:
        sys.exit('gz spread %.2f deg/s with the motors OFF -- the rover is moving '
                 'or being handled. Refusing to turn: the integral would be wrong '
                 'for the whole run.' % sd)
    print('  uncorrected, that bias alone would read %+.1f deg over a 15 s turn'
          % (gz_bias * 15.0))

    accum = 0.0
    d = rover.imu() or d
    gzprev = float(d.get('gz', 0.0)) - gz_bias
    tprev = time.time()
    while True:
        el = time.time() - t0
        if el > min(a.max_seconds, abs(a.degrees) / 5.0 + 8.0):
            reason = 'TIMEOUT'
            break
        d = rover.imu()
        now2 = time.time()
        if d is not None:
            # Trapezoidal integration of BIAS-CORRECTED gyro z over the actual
            # sample interval.
            gz = float(d.get('gz', 0.0)) - gz_bias
            dt_g = now2 - tprev
            accum += 0.5 * (gz + gzprev) * dt_g
            gzprev, tprev = gz, now2
            last = d['y']
        err = a.degrees - accum
        if abs(err) < a.tolerance:
            reason = 'reached'
            break
        # Ease inside 25 deg, but never below the deadband -- easing to a command
        # the motors ignore just stalls short of the target.
        mag = max(TURN_MIN_CMD, a.speed * min(1.0, abs(err) / 25.0))
        sign = 1.0 if err > 0 else -1.0
        # Anti-hunt: at the deadband the rover turns ~38 deg/s, and with 52 ms of
        # IMU latency plus a 100 ms loop it will overshoot a few degrees no matter
        # what. Correcting that overshoot overshoots the other way, so stop after
        # two reversals and report where it actually landed.
        if last_sign is not None and sign != last_sign:
            reversals += 1
            if reversals >= 2:
                reason = 'stopped after 2 reversals (deadband limits precision)'
                break
        last_sign = sign
        # Left wheel forward turns one way; the sign convention is confirmed by
        # watching yaw respond, which is why this reports err every cycle.
        cur = step_toward(cur, mag, MAX_ACCEL / CONTROL_HZ)
        rover.speed(-sign * cur, sign * cur)
        print('  t=%4.1f s  gyro-turned=%+8.2f  err=%+8.2f  cmd=%.3f  '
              '(fused yaw %+7.2f, not used)'
              % (el, accum, err, cur, last), flush=True)
        time.sleep(period)
    rover.stop()
    time.sleep(0.8)                       # let it settle before the final read
    achieved = accum
    mark('turn end %+.1f deg' % achieved)
    d2 = rover.imu()
    print('%s: commanded %+.0f, achieved %+.1f deg by integrated gyro '
          '(error %+.1f) in %.1f s'
          % (reason, a.degrees, achieved, achieved - a.degrees, time.time() - t0))
    if d2 is not None:
        print('  fused yaw went %+.2f -> %+.2f, a net %+.2f deg. If that is far '
              'short of the\n  achieved figure, the magnetometer was disturbed by '
              'the motors -- expected\n  during a pivot, and the reason gyro is '
              'used for control.'
              % (start, d2['y'], wrap(d2['y'] - start)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['preflight', 'straight', 'turn', 'stop'])
    ap.add_argument('--speed', type=float, default=0.30,
                    help='wheel command; anything under %.2f does not move'
                         % DEADBAND_CMD)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--degrees', type=float, default=90.0)
    ap.add_argument('--tolerance', type=float, default=6.0,
                    help='the rover cannot correct finer than its deadband allows, '
                         'so this is deliberately coarse; what matters is the '
                         'ACHIEVED angle, which is measured and reported')
    ap.add_argument('--max-seconds', type=float, default=20.0)
    ap.add_argument('--hard-stop', action='store_true',
                    help='cut the motors dead instead of ramping down, to measure '
                         'braking distance')
    ap.add_argument('--bias-seconds', type=float, default=6.0,
                    help='stationary window used to measure the gz bias before a '
                         'turn; it was 0.37 deg/s one day and 6.07 the next, so it '
                         'is measured every run and never assumed')
    ap.add_argument('--rover-ip', default=ROVER_IP)
    ap.add_argument('--sonar-ip', default=SONAR_IP)
    ap.add_argument('--dry-run', action='store_true',
                    help='run the whole loop, send nothing')
    ap.add_argument('--skip-preflight', action='store_true')
    a = ap.parse_args()

    rover = Rover(a.rover_ip, a.dry_run)
    sonar = Sonar(a.sonar_ip, a.dry_run)
    if a.dry_run:
        print('DRY RUN -- no commands will be sent\n')

    def bail(signum, frame):
        print('\nsignal %d -- stopping' % signum)
        rover.stop()
        sys.exit(1)
    signal.signal(signal.SIGINT, bail)
    signal.signal(signal.SIGTERM, bail)

    try:
        if a.mode == 'stop':
            rover.stop()
            print('stopped')
            return
        if a.mode == 'preflight':
            sys.exit(0 if preflight(rover, sonar, need_sonar=True) else 1)
        if not a.skip_preflight and not a.dry_run:
            if not preflight(rover, sonar, need_sonar=(a.mode == 'straight')):
                sys.exit('refusing to move')
            print()
        if a.mode == 'straight':
            drive_straight(rover, sonar, a)
        else:
            turn(rover, sonar, a)
    finally:
        # Whatever happened, leave the base commanded to zero.
        rover.stop()


if __name__ == '__main__':
    main()
