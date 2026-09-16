#!/usr/bin/env python3
"""Judge the gate run against the scripted truth.

Also reconstructs the UNGATED filter from the very same logged inputs, so the
comparison is two filters over one set of samples rather than two runs that saw
different noise. That is the only way to attribute a difference to the gate.
"""
import csv
import sys

import gate_profile as gp

T0 = float(sys.argv[1])
TAU = 12.0
rows = []
with open(sys.argv[2]) as fh:
    for r in csv.DictReader(fh):
        if not r['yaw_imu']:
            continue
        rows.append({
            't': float(r['t']) - T0,
            'yv': float(r['yaw_vslam']),
            'yi': float(r['yaw_imu']),
            'age': float(r['imu_age']),
            'fused': float(r['yaw_fused']),
            'gz': float(r['gz']),
            'gated': int(r['imu_gated']),
        })

if not rows:
    sys.exit('no usable rows -- did the fake IMU answer?')

print('%d rows, %.1f s, %.1f Hz' % (len(rows), rows[-1]['t'] - rows[0]['t'],
      len(rows) / max(rows[-1]['t'] - rows[0]['t'], 1e-6)))

# ---- reconstruct the ungated filter over identical inputs -------------------
un = rows[0]['fused']
for i in range(1, len(rows)):
    a, b = rows[i - 1], rows[i]
    dt = max(b['t'] - a['t'], 1e-3)
    un = gp.wrap(un + gp.wrap(b['yv'] - a['yv']))
    alpha = dt / (TAU + dt)
    un = gp.wrap(un + alpha * gp.wrap(b['yi'] - un))
    b['ungated'] = un
rows[0]['ungated'] = rows[0]['fused']

fails = []


def window(s, e):
    return [r for r in rows if s <= r['t'] < e]


def check(name, ok, detail):
    print('  %-4s %-34s %s' % ('PASS' if ok else 'FAIL', name, detail))
    if not ok:
        fails.append(name)


print('\nGATE BEHAVIOUR')
q = window(3.5, 13.0)
check('quiet: gate stays open', all(r['gated'] == 0 for r in q),
      '%d/%d frames gated (want 0), |gz| max %.2f'
      % (sum(r['gated'] for r in q), len(q), max(abs(r['gz']) for r in q)))

sub = window(13.2, 15.8)
check('4 deg/s: no false positive', all(r['gated'] == 0 for r in sub),
      '%d/%d gated (want 0), |gz| max %.2f below GYRO_GATE=5.0'
      % (sum(r['gated'] for r in sub), len(sub), max(abs(r['gz']) for r in sub)))

piv = window(16.5, 31.0)
frac = sum(r['gated'] for r in piv) / max(len(piv), 1)
check('40 deg/s pivot: gate closes', frac >= 0.98,
      '%.1f%% of %d frames gated (want >=98%%)' % (100 * frac, len(piv)))

after = [r for r in rows if r['t'] >= 31.0 and r['gated'] == 0]
if after:
    reopen = after[0]['t'] - 31.0
    check('holdoff then reopen', 1.9 <= reopen <= 2.5,
          'reopened %.2f s after the pivot (HOLDOFF_S=2.0)' % reopen)
else:
    check('holdoff then reopen', False, 'never reopened')

print('\nHEADING ACCURACY vs SCRIPTED TRUTH')
for label, t in (('end of pivot', 30.8), ('after resettle', 44.0)):
    r = min(rows, key=lambda r: abs(r['t'] - t))
    tru = gp.wrap(gp.truth_yaw(r['t']))
    eg = gp.wrap(r['fused'] - tru)
    eu = gp.wrap(r['ungated'] - tru)
    print('  t=%5.1f s  truth %+8.2f   gated %+8.2f (err %+7.2f)   '
          'ungated %+8.2f (err %+7.2f)' % (r['t'], tru, r['fused'], eg,
                                           r['ungated'], eu))

r = min(rows, key=lambda r: abs(r['t'] - 30.8))
tru = gp.wrap(gp.truth_yaw(r['t']))
eg, eu = abs(gp.wrap(r['fused'] - tru)), abs(gp.wrap(r['ungated'] - tru))
check('gate beats no-gate through pivot', eg < eu / 3.0,
      'gated %.2f deg vs ungated %.2f deg error' % (eg, eu))
check('gated error under 2 deg', eg < 2.0, '%.2f deg' % eg)

print('\n%s' % ('ALL CHECKS PASSED' if not fails
                else 'FAILED: ' + ', '.join(fails)))
sys.exit(1 if fails else 0)
