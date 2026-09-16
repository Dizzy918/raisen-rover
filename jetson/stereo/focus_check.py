#!/usr/bin/env python3
"""Is one eye out of focus? Compare sharpness tile-by-tile on RECTIFIED pairs.

Raw half-frame sharpness is not comparable between eyes: they see slightly
different scenes and at different brightness, and Laplacian variance scales with
contrast. So rectify first (same scene geometry per tile) and normalise by local
variance (contrast-independent). A lens out of focus is soft EVERYWHERE; scene
content differences show up as scatter, not a consistent offset.

M12 manual-focus lenses, focus range about 2 m to infinity -- so this is worth
knowing before a calibration recapture, because a soft eye degrades corner
localisation and therefore every number that follows.
"""
import os, subprocess, sys
import numpy as np, cv2, yaml

W, H, HALF = 3840, 1200, 1920
EW, EH = 1920, 1200
CAL = os.path.expanduser('~/stereo/cal_j11')


def m(d):
    return np.array(d['data'], float).reshape(d['rows'], d['cols'])


def load(side):
    c = yaml.safe_load(open('%s/acamst_%s.yaml' % (CAL, side)))
    return (m(c['camera_matrix']),
            np.array(c['distortion_coefficients']['data'], float).reshape(1, -1),
            m(c['rectification_matrix']), m(c['projection_matrix']))


def grab(frames=10):
    p = '/tmp/foc.raw'
    subprocess.run(['v4l2-ctl', '-d', '/dev/video0',
                    '--set-fmt-video=width=%d,height=%d,pixelformat=BA10' % (W, H),
                    '--stream-mmap', '--stream-count=%d' % frames,
                    '--stream-to=' + p],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90)
    nb = W * H * 2
    n = os.path.getsize(p) // nb
    if n < 1:
        sys.exit('grab failed')
    with open(p, 'rb') as f:
        f.seek((n - 1) * nb)
        return np.frombuffer(f.read(nb), np.uint16).reshape(H, W)


def grey(half16):
    a = half16.astype(np.float32)
    lo, hi = np.percentile(a, 1), np.percentile(a, 99.5)
    g8 = np.clip((a - lo) * 255.0 / max(hi - lo, 1.0), 0, 255).astype(np.uint8)
    return cv2.cvtColor(g8, cv2.COLOR_BayerGR2GRAY)


KL, DL, RL, PL = load('left')
KR, DR, RR, PR = load('right')
mlx, mly = cv2.initUndistortRectifyMap(KL, DL, RL, PL, (EW, EH), cv2.CV_32FC1)
mrx, mry = cv2.initUndistortRectifyMap(KR, DR, RR, PR, (EW, EH), cv2.CV_32FC1)
rot = lambda x: cv2.rotate(x, cv2.ROTATE_180)

f = grab()
Lr = cv2.remap(rot(grey(f[:, HALF:])), mlx, mly, cv2.INTER_LINEAR).astype(np.float32)
Rr = cv2.remap(rot(grey(f[:, :HALF])), mrx, mry, cv2.INTER_LINEAR).astype(np.float32)


def sharp(t):
    """High-frequency energy normalised by local contrast, so brightness and
    contrast differences between the eyes cannot masquerade as focus."""
    v = t.var()
    if v < 25:            # featureless tile, ratio is meaningless
        return None
    return cv2.Laplacian(t, cv2.CV_32F).var() / v


TX, TY = 6, 4
ratios, pairs = [], []
for j in range(TY):
    for i in range(TX):
        y0, y1 = j * EH // TY, (j + 1) * EH // TY
        x0, x1 = i * EW // TX, (i + 1) * EW // TX
        sl, sr = sharp(Lr[y0:y1, x0:x1]), sharp(Rr[y0:y1, x0:x1])
        if sl is None or sr is None:
            continue
        ratios.append(sl / sr)
        pairs.append((i, j, sl, sr))

r = np.array(ratios)
print('usable tiles: %d of %d' % (len(r), TX * TY))
print('sharpness ratio LEFT/RIGHT   median %.3f   mean %.3f   spread %.3f-%.3f'
      % (np.median(r), r.mean(), r.min(), r.max()))
print('tiles where LEFT is softer:  %d of %d' % (int((r < 1).sum()), len(r)))
print()
print('tile  x,y   left    right   L/R')
for i, j, sl, sr in pairs[:24]:
    print(' %d,%d      %6.3f  %6.3f  %.3f' % (i, j, sl, sr, sl / sr))
print()
# A median alone is NOT evidence of a focus difference, and reporting it as one
# is how this script first produced a confident wrong answer: median 1.31 with a
# 0.34-2.80 spread and 6 of 17 tiles disagreeing, on a dim scene where 7 tiles
# were rejected outright. At a 148 mm baseline the same tile does not hold the
# same content in both eyes (parallax, occlusion), so scatter is expected. Only
# a CONSISTENT offset means anything.
med = np.median(r)
agree = float((r < 1).mean())
consistent = max(agree, 1 - agree)
spread = float(r.max() / max(r.min(), 1e-6))
print('consistency: %.0f%% of tiles agree on a direction; spread %.1fx'
      % (consistent * 100, spread))
if len(r) < 10 or consistent < 0.85 or spread > 3.0:
    print('VERDICT: INCONCLUSIVE. Too much scatter to separate focus from scene')
    print('  content. Point both eyes at a flat, well-lit, textured target that')
    print('  fills the frame -- the checkerboard does exactly this, so the real')
    print('  answer comes free with a calibration capture, where smart_grab7')
    print('  logs per-eye sharpness on a flat target at a known distance.')
elif med < 0.85:
    print('VERDICT: LEFT eye softer, consistently (median %.2f). Check its focus '
          'ring before a recapture.' % med)
elif med > 1.18:
    print('VERDICT: RIGHT eye softer, consistently (median %.2f). Check its focus '
          'ring before a recapture.' % med)
else:
    print('VERDICT: both eyes equally focused, within %.0f%%.' % (abs(1 - med) * 100))
