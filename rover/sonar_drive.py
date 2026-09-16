#!/usr/bin/env python3
"""Drive forward on sonar alone, and measure how fast it went.

    python3 rover/sonar_drive.py --speed 0.10 --seconds 8
    python3 rover/sonar_drive.py --sweep 0.05,0.07,0.10,0.15,0.20 --seconds 6
    python3 rover/sonar_drive.py --repeat 8 --speed 0.10 --seconds 4

WHY THIS EXISTS AND WHY IT NEEDS NOTHING ELSE. The IMU is currently reporting
1564 milli-g on a stationary rover and /dev/video0 is absent, so neither the gyro
nor vSLAM can say where the rover went. The front sonar can: it measures the
closing distance to whatever is ahead, which for a straight run at a flat surface
IS the distance travelled. That makes it the only working odometer on the vehicle,
and it is enough for speed and repeatability work.

Built on /home/narsil/rover2_obstacle_control.py, which is the script that already
works, with four things added that today's failures argued for:

  MEDIAN OF THREE, NOT THE LATEST READING. A single spurious sample aborted one run
  at "5.5 cm" while the rover had not moved, and a spurious 353 cm appeared mid-run
  in another. The median rejects one outlier in either direction, and both
  directions matter: a short reading stops a good run, a long one hides a wall.

  POLL AT THE FIRMWARE'S RATE. The four sensors fire 60 ms apart so they do not
  hear each other, plus a 200 ms tail: a full cycle is about 440 ms. The original
  script polls at 20 Hz, which returns the same numbers eight times over and may
  itself provoke the garbage samples.

  A STOP THRESHOLD DERIVED FROM THE SPEED, NOT TYPED IN. Reaction distance is
  sonar staleness plus the filter window plus command latency plus braking, all
  measured. At command 0.10 that is 8.9 cm and a 20 cm stop is right; at 0.30 it is
  36 cm and a 20 cm stop is 16 cm too late. The threshold follows the command.

  NO-ECHO IS NOT CLEAR. An HC-SR04 aimed obliquely at a wall reflects the pulse
  away: measured, the same stationary sensor gave 95, 95, 999, 999, 999, 95 cm.
  The rover hit a wall because a timeout maps to "clear". Here, a channel that has
  returned nothing for several consecutive cycles is treated as BLIND, and blind
  does not authorise motion.
"""
import argparse
import json
import statistics
import sys
import threading
import time
import urllib.parse
import urllib.request

SONAR_IP = '172.10.10.192'
ROVER_IP = '172.10.10.184'
FRONT = 'sonar4'
LEFT, RIGHT = 'sonar2', 'sonar1'

FORWARD_SIGN = -1.0        # positive L/R drives this rover BACKWARD
POLL_S = 0.45              # one firmware cycle
MEDIAN_OF = 3
CMD_HZ = 5.0               # the ESP32 stops itself after ~3 s without a command
BLIND_CYCLES = 6           # consecutive no-echo reads before a channel is "blind"

# Reaction-distance terms, all measured 2026-08-18. See rover/README.txt.
T_STALE = POLL_S * 2       # newest reading age + median window, worst case
T_LATENCY = 0.06
T_BRAKE = 0.19
MARGIN_CM = 8.0
# speed = SLOPE * command + ICEPT, fitted to 0.10/0.079, 0.30/0.320, 0.45/0.513
SLOPE, ICEPT = 1.238, -0.0469


def speed_of(cmd):
    return max(SLOPE * cmd + ICEPT, 0.0)


def stop_threshold_cm(cmd):
    v = speed_of(cmd)
    return 100.0 * v * (T_STALE + T_LATENCY + T_BRAKE) + MARGIN_CM


def get(url, timeout=2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode())


class Sonar:
    """Background poller at the firmware's own rate, with a median and a blind check."""

    def __init__(self, ip=SONAR_IP):
        self.url = 'http://%s/sonar' % ip
        self.hist = {FRONT: [], LEFT: [], RIGHT: []}
        self.noecho = {k: 0 for k in self.hist}
        self.samples = []          # (t, front_median) for the speed fit
        self.ok = self.fails = 0
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            t0 = time.time()
            try:
                d = get(self.url)
                with self.lock:
                    for k in self.hist:
                        raw = float(d.get(k, -1))
                        if raw < 0:
                            self.noecho[k] += 1
                            self.hist[k].append(999.0)
                        else:
                            self.noecho[k] = 0
                            self.hist[k].append(raw)
                        self.hist[k] = self.hist[k][-MEDIAN_OF:]
                    self.ok += 1
                    f = self._med(FRONT)
                    if f is not None:
                        self.samples.append((time.time(), f))
            except Exception:
                with self.lock:
                    self.fails += 1
            time.sleep(max(0.0, POLL_S - (time.time() - t0)))

    def _med(self, ch):
        h = self.hist[ch]
        return statistics.median(h) if h else None

    def read(self):
        with self.lock:
            return {k: self._med(k) for k in self.hist}, dict(self.noecho)

    def blind(self, ch=FRONT):
        with self.lock:
            return self.noecho[ch] >= BLIND_CYCLES

    def wait_ready(self, timeout=4.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if len(self.hist[FRONT]) >= MEDIAN_OF:
                    return True
            time.sleep(0.1)
        return False

    def stop(self):
        self.running = False


def send(cmd, ip=ROVER_IP):
    payload = {'T': 1, 'L': FORWARD_SIGN * cmd, 'R': FORWARD_SIGN * cmd}
    try:
        get('http://%s/js?json=%s' % (ip, urllib.parse.quote(json.dumps(payload))), 1.0)
        return True
    except Exception:
        return False


def halt(ip=ROVER_IP):
    for _ in range(3):
        if send(0.0, ip):
            return
        time.sleep(0.1)


def run_once(sonar, cmd, seconds, ip, label=''):
    """One bounded forward run. Returns a dict of what happened."""
    thresh = stop_threshold_cm(cmd)
    got, _ = sonar.read()
    start = got[FRONT]
    if sonar.blind():
        return dict(ok=False, why='front sonar blind: %d consecutive no-echo reads. '
                                  'Cannot tell open space from a wall at a grazing '
                                  'angle -- refusing to drive' % BLIND_CYCLES)
    if start is None:
        return dict(ok=False, why='no sonar reading yet')
    if start <= thresh:
        return dict(ok=False, why='already inside the %.0f cm threshold for command '
                                  '%.2f (front %.0f cm)' % (thresh, cmd, start))
    print('  %sstart %.1f cm, command %.2f, predicted %.3f m/s, stop at %.0f cm'
          % (label, start, cmd, speed_of(cmd), thresh), flush=True)

    t0 = time.time()
    period = 1.0 / CMD_HZ
    reason = 'completed'
    with sonar.lock:
        mark0 = len(sonar.samples)
    while True:
        el = time.time() - t0
        if el >= seconds:
            break
        got, _ = sonar.read()
        f = got[FRONT]
        if sonar.blind():
            reason = 'front went blind mid-run'
            break
        if f is not None and f <= thresh:
            reason = 'obstacle at %.0f cm' % f
            break
        if not send(cmd, ip):
            reason = 'command send failed'
            break
        time.sleep(period)
    halt(ip)
    time.sleep(POLL_S * MEDIAN_OF + 0.3)     # let the window refill past the stop

    with sonar.lock:
        seg = sonar.samples[mark0:]
    got, _ = sonar.read()
    end = got[FRONT]
    travelled = (start - end) if (start is not None and end is not None) else None
    # Fit the closing rate over the samples taken while driving.
    fit = None
    drive = [(t, d) for t, d in seg if t - t0 < (time.time() - t0) and d < 990]
    if len(drive) >= 4:
        ts = [t - drive[0][0] for t, _ in drive]
        ds = [d for _, d in drive]
        n = len(ts)
        mt = sum(ts) / n
        md = sum(ds) / n
        den = sum((t - mt) ** 2 for t in ts)
        if den > 0:
            slope = sum((t - mt) * (d - md) for t, d in zip(ts, ds)) / den
            fit = -slope / 100.0          # cm/s closing -> m/s advancing
    return dict(ok=True, reason=reason, start=start, end=end, travelled=travelled,
                seconds=time.time() - t0, fit=fit, n=len(drive))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.10)
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--sweep', help='comma-separated commands to try in turn')
    ap.add_argument('--repeat', type=int, default=0,
                    help='repeat the same command N times, for repeatability')
    ap.add_argument('--rover-ip', default=ROVER_IP)
    ap.add_argument('--sonar-ip', default=SONAR_IP)
    ap.add_argument('--settle', type=float, default=2.0)
    a = ap.parse_args()

    sonar = Sonar(a.sonar_ip)
    if not sonar.wait_ready():
        sys.exit('sonar box not answering')
    got, noecho = sonar.read()
    print('sonar: front %s  left %s  right %s   (no-echo streaks %s)'
          % (got[FRONT], got[LEFT], got[RIGHT], noecho))

    cmds = ([float(x) for x in a.sweep.split(',')] if a.sweep
            else [a.speed] * max(a.repeat, 1))
    results = []
    try:
        for i, c in enumerate(cmds):
            print('\nrun %d/%d, command %.3f' % (i + 1, len(cmds), c))
            r = run_once(sonar, c, a.seconds, a.rover_ip, '')
            if not r['ok']:
                print('  SKIPPED: %s' % r['why'])
                results.append((c, None, None))
                break
            print('  %s: %.1f -> %.1f cm = %.1f cm in %.1f s'
                  % (r['reason'], r['start'], r['end'], r['travelled'], r['seconds']))
            if r['fit'] is not None:
                print('  closing-rate fit %.3f m/s over %d samples' % (r['fit'], r['n']))
            results.append((c, r['travelled'], r['fit']))
            time.sleep(a.settle)
    finally:
        halt(a.rover_ip)
        sonar.stop()

    good = [(c, d, f) for c, d, f in results if d is not None]
    if len(good) >= 2:
        print('\n%-9s %-11s %-11s %s' % ('command', 'travel_cm', 'fit_m/s', 'predicted'))
        for c, d, f in good:
            print('%-9.3f %-11.1f %-11s %.3f'
                  % (c, d, ('%.3f' % f) if f else '-', speed_of(c)))
        if a.repeat and len(good) >= 3:
            ds = [d for _, d, _ in good]
            print('\nREPEATABILITY at command %.3f over %d runs' % (cmds[0], len(ds)))
            print('  travel mean %.1f cm, std %.2f, min %.1f, max %.1f, spread %.1f%%'
                  % (statistics.fmean(ds), statistics.pstdev(ds), min(ds), max(ds),
                     100 * (max(ds) - min(ds)) / max(statistics.fmean(ds), 1e-9)))


if __name__ == '__main__':
    main()
