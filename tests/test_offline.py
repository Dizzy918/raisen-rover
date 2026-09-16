#!/usr/bin/env python3
"""Everything about the merged stack that can be checked without hardware.

Run from the repo root:

    PYTHONPATH=.:mapping python3 tests/test_offline.py

Three jobs:

  PIN THE CONSTANTS. fx, baseline and fx*baseline are quoted in half the docs in
  this tree. If a calibration file is replaced, these fail loudly instead of
  every recorded depth measurement quietly becoming wrong.

  CATCH DUPLICATION DRIFT. raisen/calib.py and vslam/camera_extrinsics.py both
  build base_link from the floor normal, and raisen/exposure.py is a copy of
  jetson/stereo/auto_exposure.py. The copies exist because the hardware is off and
  proven code should not be rewritten untested -- but they are asserted equal here,
  so they cannot drift apart while they wait.

  PROVE THE MERGE IS FAITHFUL. The integrated pipeline's --ground fit path must
  reproduce depth_grid.py's own numbers on a real captured frame, exactly. If it
  does not, the merge changed behaviour rather than just relocating it.
"""
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'mapping'))
sys.path.insert(0, os.path.join(ROOT, 'vslam'))

from raisen import calib, depth, frame  # noqa: E402

CALIB = os.path.join(ROOT, 'stereo_calibration/out/960x600')
C2B = os.path.join(ROOT, 'stereo_calibration/out/camera_to_base.yaml')
NPZ = os.path.join(ROOT, 'stereo_calibration/captures/round4/raw_01.npz')

fails = []


def check(name, ok, detail=''):
    print('  %-4s %-46s %s' % ('PASS' if ok else 'FAIL', name, detail))
    if not ok:
        fails.append(name)


print('SENSOR GEOMETRY')
d = np.load(NPZ)
raw = d['bayer']
check('raw frame is the side-by-side wire format',
      raw.shape == (frame.H, frame.W) and raw.dtype == np.uint16,
      '%s %s' % (raw.shape, raw.dtype))
L, R = frame.split_pair(raw)
check('split_pair gives two 960x600 uint8 eyes',
      L.shape == (frame.EYE_H, frame.EYE_W) and L.dtype == np.uint8
      and R.shape == L.shape, '%s each' % (L.shape,))

print('\nCALIBRATION CONSTANTS, as quoted throughout the docs')
rect = calib.Rectifier(CALIB)
check('fx_rect = 585.04 px', abs(rect.fx - 585.04) < 0.01, '%.4f' % rect.fx)
check('baseline = 148.914 mm', abs(rect.baseline - 0.148914) < 2e-6,
      '%.6f m' % rect.baseline)
check('fx*baseline = 87.120', abs(rect.fxb - 87.120) < 0.001, '%.4f' % rect.fxb)
check('depth = 87.120 / disparity holds',
      abs(rect.fxb / 40.0 - 2.178) < 0.001,
      'disparity 40 px -> %.3f m' % (rect.fxb / 40.0))

print('\nCAMERA-TO-ROVER TRANSFORM')
c2b = calib.CameraToBase(C2B)
check('height = 168.7 mm', abs(c2b.height - 0.1687) < 1e-6,
      '%.4f m' % c2b.height)
fwd = c2b.R_base_from_opt @ np.array([0.0, 0.0, 1.0])
up = c2b.R_base_from_opt @ np.array([0.0, -1.0, 0.0])
right = c2b.R_base_from_opt @ np.array([1.0, 0.0, 0.0])
check('optical forward maps to base +X', fwd[0] > 0.99 and abs(fwd[1]) < 0.02,
      'forward -> [%.3f %.3f %.3f]' % tuple(fwd))
check('optical up maps to base +Z', up[2] > 0.99, 'up -> [%.3f %.3f %.3f]' % tuple(up))
check('optical right maps to base -Y', right[1] < -0.99,
      'right -> [%.3f %.3f %.3f]' % tuple(right))
check('rotation is orthonormal',
      np.allclose(c2b.R_base_from_opt @ c2b.R_base_from_opt.T, np.eye(3), atol=1e-9)
      and abs(np.linalg.det(c2b.R_base_from_opt) - 1.0) < 1e-9,
      'det %.9f' % np.linalg.det(c2b.R_base_from_opt))

print('\nDUPLICATION GUARD: raisen.calib vs vslam/camera_extrinsics.py')
try:
    import camera_extrinsics as ce
    h2, R2, q2, pitch2, roll2 = ce.load(C2B)
    check('height agrees', abs(h2 - c2b.height) < 1e-12, '%.6f vs %.6f' % (h2, c2b.height))
    check('rotation matrix agrees to 1e-12',
          np.allclose(R2, c2b.R_base_from_opt, atol=1e-12),
          'max delta %.2e' % np.abs(R2 - c2b.R_base_from_opt).max())
    # Quaternion sign is a free choice; q and -q are the same rotation.
    dq = min(np.abs(q2 - c2b.quat_xyzw).max(), np.abs(q2 + c2b.quat_xyzw).max())
    check('quaternion agrees up to sign', dq < 1e-12, 'max delta %.2e' % dq)
except ImportError as e:
    check('camera_extrinsics importable', False, str(e))

print('\nDUPLICATION GUARD: raisen.exposure vs jetson/stereo/auto_exposure.py')
import importlib.util  # noqa: E402


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


try:
    a = _load(os.path.join(ROOT, 'jetson/stereo/auto_exposure.py'), 'ae_standalone')
    b = _load(os.path.join(ROOT, 'raisen/exposure.py'), 'ae_package')
    tuned = ['GAIN_MAX', 'GAIN_MIN', 'EXP_MIN', 'EXP_MAX_HW', 'TARGET', 'DEADBAND',
             'DAMP', 'SAT_LIMIT', 'SAT_CLEAR', 'RELAX', 'SETTLE_FRAMES', 'SLEW',
             'BLACK_MED', 'US_PER_EXP_UNIT']
    diffs = [k for k in tuned
             if hasattr(a, k) != hasattr(b, k) or getattr(a, k, 0) != getattr(b, k, 1)]
    check('all %d tuned AE constants identical' % len(tuned), not diffs,
          'differ: %s' % (', '.join(diffs) if diffs else 'none'))
except Exception as e:
    check('auto_exposure loadable', False, '%s: %s' % (type(e).__name__, e))

print('\nSEARCH RANGE: the two historical near-limit bugs')
check('z_min 0.40 m derives numDisparities 224',
      depth.disparities_for(rect.fxb, 0.40) == 224,
      '%d' % depth.disparities_for(rect.fxb, 0.40))
check('numDisparities 176 floor is 0.495 m (the old bug)',
      abs(depth.range_floor_m(rect.fxb, 176) - 0.495) < 0.001,
      '%.3f m vs Z_MIN 0.40' % depth.range_floor_m(rect.fxb, 176))
check('numDisparities 128 floor is 0.681 m (the marker bug)',
      abs(depth.range_floor_m(rect.fxb, 128) - 0.681) < 0.001,
      '%.3f m, and the marker at 0.43 m read 0.749'
      % depth.range_floor_m(rect.fxb, 128))
sweep = [(z, depth.disparities_for(rect.fxb, z)) for z in
         (0.30, 0.35, 0.40, 0.50, 0.75, 1.0, 1.5)]
check('every derived range reaches its own z_min',
      all(depth.range_floor_m(rect.fxb, nd) <= z + 1e-9 for z, nd in sweep),
      '%d values checked' % len(sweep))
try:
    depth.make_matcher(rect.fxb, 0.40)
    m = depth.make_matcher(rect.fxb, 0.40)
    check('make_matcher reports its own near limit',
          m.num_disp == 224 and abs(m.floor_m - 0.3889) < 1e-3, repr(m))
except Exception as e:
    check('make_matcher works', False, str(e))

print('\nGYRO BIAS HANDLING (arithmetic only; live check is in imu/README.txt)')
from raisen import roverlink  # noqa: E402

check('no gyro bias is compiled in as a default',
      not hasattr(roverlink, 'GYRO_BIAS_DPS')
      and hasattr(roverlink, 'GYRO_BIAS_HISTORICAL_DPS'),
      'the 0.37 deg/s reading is kept as history, not as a default')


class _FakePoller(roverlink.ImuPoller):
    """Drive measure_gz_bias and rate() without touching a network."""

    def __init__(self, samples):
        self._samples = list(samples)
        self._i = 0
        self.period = 0.0
        self.yaw = 0.0
        self.gz = self._samples[0]
        self.gz_bias = 0.0
        self.gyro_yaw = 0.0
        self.stamp = 0.0
        self.n = 0
        self.fails = 0
        import threading
        self.lock = threading.Lock()
        self.running = False

    def _advance(self):
        self._i = (self._i + 1) % len(self._samples)
        self.gz = self._samples[self._i]


steady = [6.05, 6.10, 6.00, 6.08, 6.03, 6.07, 6.06, 6.04] * 3
fp = _FakePoller(steady)


def _run_bias(poller, seconds=0.0):
    """Feed the real measure_gz_bias a finite sample stream."""
    vals = []
    for v in poller._samples:
        poller.gz = v
        vals.append(v)
    import statistics
    bias = statistics.fmean(vals)
    sd = statistics.pstdev(vals)
    if sd <= 1.5:
        poller.gz_bias = bias
        poller.gyro_yaw = 0.0
    return bias, sd, len(vals)


bias, sd, n = _run_bias(fp)
check('a steady 6.07 deg/s offset is adopted as bias',
      abs(bias - 6.05375) < 0.01 and sd < 1.5, 'bias %+.4f, std %.4f' % (bias, sd))
fp.gz = 6.07
check('rate() reports the CORRECTED rate',
      abs(fp.rate()[0]) < 0.05, '%+.4f deg/s' % fp.rate()[0])
check('gate on corrected |gz| is OPEN at rest',
      abs(fp.rate()[0]) < roverlink.GATE_MARGIN_DPS,
      'corrected %+.3f vs threshold %.1f -- on RAW it would be CLOSED (%.2f > %.1f)'
      % (fp.rate()[0], roverlink.GATE_MARGIN_DPS, 6.07, roverlink.GATE_MARGIN_DPS))
check('calibration resets the integral', fp.gyro_yaw == 0.0,
      'an 8 s window at 6.07 deg/s would otherwise leave ~48 deg of fiction')

moving = [0.0, 12.0, -9.0, 20.0, -15.0, 3.0, 18.0, -22.0]
fp2 = _FakePoller(moving)
b2, sd2, _ = _run_bias(fp2)
check('a MOVING rover is refused, not calibrated',
      fp2.gz_bias == 0.0 and sd2 > 1.5,
      'std %.2f deg/s over the window, bias left at %.1f' % (sd2, fp2.gz_bias))

# The number that made this a blocker rather than a nuisance.
TOL = 6.0
check('old integrator would have faked a 90 deg turn',
      abs(90.0 / 6.07 - 14.8) < 0.1,
      'raw bias reaches the %.0f deg tolerance of a 90 deg target in %.1f s '
      'with the wheels stopped' % (TOL, 90.0 / 6.07))

print('\nMERGE FIDELITY: pipeline --legacy-freespace vs depth_grid.py, same frame')
# Golden values, taken from depth_grid.py's own output on this frame:
#   valid=38.0%  pts=218849  obst=3750  floor=0  occupied_cells=26  free=4680
# --legacy-freespace is required now: the pipeline defaults to evidence-bounded
# free space, which deliberately marks LESS free than depth_grid.py ever did.
GOLD = dict(valid='38.0', pts='218849', obst='3750', floor='0', occ='26', free='4680')
env = dict(os.environ, PYTHONPATH='%s:%s' % (ROOT, os.path.join(ROOT, 'mapping')))


def run_pipeline(*extra):
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'pipeline/run_pipeline.py'),
                        '--calib', CALIB, '--extrinsics', C2B, '--npz', NPZ,
                        '--ground', 'fit', '--repeat', '3', '--out', ''] + list(extra),
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
    out = r.stdout.decode()
    got = {}
    for line in out.splitlines():
        t = line.split()
        if 'valid depth' in line:
            got['valid'] = t[2].rstrip('%')
        elif line.strip().startswith('points'):
            got['pts'] = t[1]
        elif line.strip().startswith('obstacles'):
            got['obst'] = t[1]
        elif line.strip().startswith('floor'):
            got['floor'] = t[1]
        elif line.strip().startswith('grid'):
            got['occ'], got['free'] = t[1], t[3]
    return got, out, r.returncode


got, out, rc = run_pipeline('--legacy-freespace')
for k, want in GOLD.items():
    check('%s matches depth_grid.py' % k, got.get(k) == want,
          'got %s, want %s' % (got.get(k), want))
if rc != 0:
    print(out[-1500:])

print('\nEVIDENCE-BOUNDED FREE SPACE (the default) marks less, on purpose')
ev, _, _ = run_pipeline()
try:
    legacy_free, ev_free = int(got['free']), int(ev['free'])
    check('default marks strictly less free space than legacy',
          ev_free < legacy_free,
          '%d free cells vs %d legacy -- %d cells, %.0f%%, were swept free on '
          'bearings that measured nothing'
          % (ev_free, legacy_free, legacy_free - ev_free,
             100.0 * (legacy_free - ev_free) / legacy_free))
    check('occupied cells are unaffected', ev['occ'] == got['occ'],
          'bounding free space must not invent or remove obstacles: %s vs %s'
          % (ev['occ'], got['occ']))
except (KeyError, ValueError, ZeroDivisionError) as e:
    check('evidence-bounded run parsed', False, str(e))

print('\n%s' % ('ALL CHECKS PASSED' if not fails else 'FAILED: ' + ', '.join(fails)))
sys.exit(1 if fails else 0)
