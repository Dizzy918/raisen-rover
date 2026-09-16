#!/usr/bin/env python3
"""What ARE the points that come out below the floor? Two candidates, one test.

    python3 vslam/subfloor_check.py

THE OBSERVATION, which is not in doubt. A single-frame cloud puts 16-20% of its
points more than 10 cm BELOW the floor plane, and measure_floor.py sees several
horizontal planes at once: at a 10 mm RANSAC tolerance it fits the floor and the
stuff underneath as one surface, and at 5 mm it separates them but "largest plane"
then flips between them as people move.

THE EXPLANATION WAS WRONG. Every document here said polished concrete acting as a
mirror. The floor is NOT polished -- there is bright light falling on it. So the
mechanism is open, and two candidates predict different data:

  GENUINE REFLECTED IMAGERY needs enough gloss to form an image. It would place a
  coherent inverted copy of the room below the floor, so sub-floor points cluster
  in z in a structured way and their depths are plausible-looking.

  FALSE MATCHES FROM GLARE need only a bright specular sheen. A highlight sits at
  a different place in each eye because it depends on viewing angle, so the matcher
  pairs the wrong pixels and the depth is simply WRONG -- scattered in z with no
  structure, and drawn disproportionately from BRIGHT pixels.

This prints the two discriminators: the z spread of the sub-floor points, and
whether they come from the bright end of the image. Structured and dim points to
reflection; spread and bright points to glare-driven mismatching.

Either way the grid already discards them, and the fix is not to widen the floor
band -- that would turn the artefact into data.
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, '/home/narsil/vslam-min')

from raisen import calib as rcal      # noqa: E402
from raisen import camera as rc       # noqa: E402
from raisen import depth as rd        # noqa: E402
from raisen import frame as rf        # noqa: E402
from raisen import service as rs      # noqa: E402

CALIB = '/home/narsil/vslam-min/calib'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exposure', type=int, default=1500,
                    help='raise it for a dark scene; this is a still measurement '
                         'so the frame-rate cost does not matter')
    ap.add_argument('--gain', type=int, default=800)
    a = ap.parse_args()
    rect = rcal.Rectifier(CALIB)
    c2b = rcal.CameraToBase(CALIB + '/camera_to_base.yaml')
    m = rd.make_matcher(rect.fxb, 0.40)
    with rs.vision_stopped():
        cam = rc.StereoCamera('/dev/video0', a.exposure, a.gain)
        raw = cam.read_awake()
        cam.close()
    if raw is None:
        sys.exit('only black frames -- camera did not wake')

    L, R = rf.split_pair(raw)
    Lr, Rr = rect.rectify(L, R)
    disp = m.compute(Lr, Rr)
    pts, ok = rd.points_optical(disp, rect, 0.40, 5.0)
    g = c2b.to_base(pts)
    bright = Lr[ok].astype(np.float32)
    z = g[:, 2]
    rng = np.hypot(g[:, 0], g[:, 1])
    below = z < -0.10

    # Report exposure from the RAW frame. bin2x2 stretches every frame to its own
    # 1-99 percentile, so the 8-bit output is nearly invariant to exposure: 1500,
    # 4000 and 8000 all gave a median of 6-7 here, which says the histogram is
    # skewed, NOT that the sensor was starved. Judging exposure on the stretched
    # image is measuring the stretch.
    raw_med = float(np.median(raw)) / 64.0        # 16-bit container -> 10-bit counts
    print('exposure %d gain %d -- RAW median %.0f of 1023 counts' % (a.exposure, a.gain, raw_med))
    print('  matched-pixel intensity median %.0f of 255, AFTER bin2x2 stretched the '
          'frame' % np.median(bright))
    if raw_med < 40:
        print('  WARNING: the sensor really is starved. Raise exposure or gain.')
    print('valid points %d (%.1f%% of the frame)' % (len(g), 100 * ok.mean()))
    if not below.any():
        print('no sub-floor points in this frame -- nothing to diagnose')
        return
    print('sub-floor (z < -10 cm): %d = %.1f%% of valid points'
          % (below.sum(), 100 * below.mean()))

    print('\nDISCRIMINATOR 1 -- z spread. Reflected imagery clusters; bad matches do not.')
    qs = np.percentile(z[below], [5, 25, 50, 75, 95])
    print('  p5 %+.2f  p25 %+.2f  median %+.2f  p75 %+.2f  p95 %+.2f m' % tuple(qs))
    print('  p5..p95 spread %.2f m' % (qs[4] - qs[0]))
    # A mirrored floor puts a point at -z for every real point at +z, so the
    # sub-floor cloud would roughly mirror the above-floor cloud.
    above = z > 0.10
    if above.any():
        print('  above-floor p5..p95 spread %.2f m, for comparison'
              % float(np.percentile(z[above], 95) - np.percentile(z[above], 5)))

    print('\nDISCRIMINATOR 2 -- brightness. Glare-driven mismatches favour bright pixels.')
    print('  all matched pixels  mean %5.1f  median %5.1f'
          % (bright.mean(), np.median(bright)))
    print('  sub-floor pixels    mean %5.1f  median %5.1f'
          % (bright[below].mean(), np.median(bright[below])))
    print('  the rest            mean %5.1f  median %5.1f'
          % (bright[~below].mean(), np.median(bright[~below])))
    hi = bright > np.percentile(bright, 90)
    print('  brightest 10%% of matched pixels: %.1f%% land sub-floor'
          % (100 * below[hi].mean()))
    print('  all the others:                  %.1f%% land sub-floor'
          % (100 * below[~hi].mean()))

    print('\nWHERE THEY ARE')
    for lo, hi_r in ((0.4, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 5.0)):
        sel = (rng >= lo) & (rng < hi_r)
        if sel.sum():
            print('  %.1f-%.1f m: %6d points, %5.1f%% sub-floor'
                  % (lo, hi_r, sel.sum(), 100 * below[sel].mean()))

    print('\nREADING IT')
    ratio = (below[hi].mean() / max(below[~hi].mean(), 1e-9))
    tight = (qs[4] - qs[0]) < 0.5
    if ratio > 1.5 and not tight:
        print('  Bright pixels are %.1fx more likely to land sub-floor and z is '
              'spread %.2f m.' % (ratio, qs[4] - qs[0]))
        print('  That is GLARE-DRIVEN FALSE MATCHING, not reflected imagery.')
    elif tight and ratio < 1.5:
        print('  z is tight (%.2f m) and brightness barely matters (%.1fx).'
              % (qs[4] - qs[0], ratio))
        print('  That looks like COHERENT REFLECTED IMAGERY after all.')
    else:
        print('  Mixed: brightness ratio %.1fx, z spread %.2f m. Inconclusive from '
              'one frame -- run it again on a different scene.' % (ratio, qs[4] - qs[0]))


if __name__ == '__main__':
    main()
