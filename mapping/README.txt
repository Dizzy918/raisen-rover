SGBM -> GROUND PROJECTION -> LOG-ODDS OCCUPANCY GRID
2026-08-13


WHAT IT DOES

depth_grid.py, one frame at a time:

  1. split the side-by-side AR0234 frame, 2x2 Bayer bin to 960x600 per eye
     (never demosaic - see stereo_calibration/README.md), rotate 180 and swap
     for the inverted mount
  2. rectify with stereo_calibration/out/960x600/
  3. SGBM -> disparity -> depth = 87.12 / disparity metres. That constant is the
     RECTIFIED focal (P[0,0] = 585.04) times the baseline (0.148914 m), NOT the
     unrectified fx
  4. fit the ground plane by RANSAC -> camera height and tilt
  5. transform to a ground frame (x forward, y left, z up), classify by height:
       |z| < 6 cm            floor, so that patch of ground is free
       6 cm .. 40 cm         obstacle
       > 40 cm               overhead, can be driven under
  6. reduce obstacles to a virtual laser scan - nearest obstacle per bearing
  7. Bayesian log-odds update, clamped at +/-8


WHY A VIRTUAL LASER SCAN

Free space is the part everyone gets wrong. Along each bearing, everything nearer
than the first obstacle was seen THROUGH, so it is free; the hit cell is
occupied; everything beyond stays UNKNOWN because it was never observed.

Marking beyond-the-hit as free is the classic bug - it erases walls the moment
anything passes in front of them. Reducing to a nearest-hit-per-bearing scan
makes the correct behaviour the easy one, and it is fast: 180 bearings instead of
200k points.


FRAME RATE - EXPOSURE IS THE CEILING, NOT RESOLUTION

The first live run managed 5.5 fps and it was NOT the pipeline's fault. Exposure
is measured in sensor row-times (~14.57 us), so it caps the frame rate outright.
Measured on jetson9:

  exposure   exposure time   camera fps
      8000        116.6 ms          5.8
      4000         58.3 ms         11.5
      2290         33.4 ms         20.1
      1500         21.9 ms         30.6

I had set exposure=8000, which pins the camera at 5.8 fps no matter how fast
anything downstream is. Keep it at 1500 and buy brightness back with gain.

Longer exposure is also WORSE for depth, because it blows out the bright regions:

  exposure/gain   camera fps   valid depth   plane fit
      1500/800          30.7         38.3%   h=0.179 tilt=2.2 support=15%
     1500/1200          30.7         38.6%   h=0.227 tilt=3.6 support=15%
     2290/1200          20.1         35.8%   h=0.205 tilt=2.0 support=16%
     4000/1200          11.5         33.1%   rejected
     8000/1200           5.8         30.0%   rejected

So 1500 is simultaneously the fastest and the best quality. Nothing to trade.

With exposure fixed, compute becomes the limit:

  960x600   157 ms    6.4 fps
  640x400    85 ms   11.7 fps   (plane loaded, not refitted)

A fixed cost worth knowing: split_pair runs on the FULL raw frame regardless of
--scale, so it does not shrink when you lower the resolution. np.percentile
inside it cost ~31 ms per eye until it was changed to work on a strided
subsample; the fitted plane moved by 2 mm, so the subsample costs nothing.


RESULTS

Measured on real office frames (stereo_calibration/captures/round3/raw_*.npz,
captured while the camera worked):

  valid depth              39-46 percent of pixels
  960x600, fitting plane   100 ms/frame   10.0 fps
  640x400, plane loaded     63 ms/frame   15.7 fps

Ground plane, fitted independently on two different frames:

  raw_01   height 0.906 m   tilt 2.5 deg   60950 inliers   10.8 mm rms
  raw_02   height 0.918 m   tilt 2.6 deg   59926 inliers   11.0 mm rms

Two separate RANSAC fits agreeing to 12 mm and 0.1 degrees is real
cross-validation, not a coincidence - that is the main evidence the geometry is
right.

Classification is visibly correct in classification_and_grid.png: green lands on
the actual floor, red on chair bases, legs and wheels, blue on desks, chair
backs, monitors and ceiling.


FIT THE PLANE ONCE, THEN SAVE IT

Camera height and tilt are fixed by the mount, so the plane is a CALIBRATION
step, not a per-frame one. Re-fitting every frame is both wasteful and fragile:
at 640x400 there are half as many floor points and the fit locked onto the wrong
plane (height 0.16 m, tilt 29.8 deg, floor points 0).

  # fit once at full resolution, where there is plenty of floor in view
  python3 depth_grid.py --npz ../stereo_calibration/captures/round3/raw_01.npz \
      --plane plane.yaml

  # then reuse it, at whatever resolution you want to run
  python3 depth_grid.py --live --scale 0.667 --plane plane.yaml

The fit refuses to return an implausible answer - height outside 0.05..2.5 m,
tilt over 35 degrees, or flatness worse than 50 mm rms all fail rather than
returning a plane. A wrong plane produces a map that looks plausible and is
wholly wrong, which is worse than no map.

plane.yaml here holds the round3 fit, which is the CAPTURE RIG's geometry.
plane_rover.yaml holds the MEASURED rover geometry -- use that one, and do not
let the RANSAC overwrite it.


FOUR BUGS FOUND WHILE BUILDING THIS

Recorded because two of them cost real time and both are easy to hit again:

  np.linalg.svd(inl - c) on the plane inliers took 11.6 SECONDS and returned a
  wrong normal. svd() defaults to full_matrices=True, so on a 23000x3 input it
  tries to build a 23000x23000 U - about 4 GB. The smallest-eigenvalue
  eigenvector of the 3x3 covariance is the same answer in microseconds.

  rng.choice(n, 3, replace=False) inside the RANSAC loop permutes the whole
  population on every call, so it is O(n) per iteration. rng.integers() instead.

  Building the Q matrix by re-running stereoRectify was fragile and crashed.
  Unprojecting straight from the rectified intrinsics is clearer, and avoids Q's
  sign conventions - a flipped sign there gives a mirrored map that still looks
  plausible.

  The board-region contrast stretch used for calibration capture deliberately
  blows out everything except the target. Correct for finding a checkerboard,
  wrong for mapping, so this uses a plain global stretch.


VERIFIED ON THE ROVER, 2026-08-14

Height MEASURED with a tape at 0.152 m (floor to lens centre). With height fixed,
tilt becomes a 1-D search instead of a 3-parameter RANSAC, and it has a clear peak:

  tilt   floor points/frame
  +0.50            15409
  +1.50            19246
  +2.00            19832   <- best
  +3.00            16989
  +3.50            15127

  tilt = 2.0 deg UP, confidence +/-1.25 deg

The direction is confirmed without relying on any sign convention: the horizon
falls at image row 316 of 600 with the centre at 296, i.e. BELOW centre, which
means the camera is pitched up. That independently matches the observation that
the cameras point "a little bit upwards, very little".

Result with the measured plane (see plane_rover.yaml):

  floor points    31939   (it was ZERO with the fitted plane)
  obstacle points 52964
  rate            11.4 fps at 640x400, 303 frames continuous

Classification verified in rover_verified.png: green on the floor including along
a cable, red on table legs, chair bases and skirting, blue on table tops and the
far wall.

POINTS BELOW THE FLOOR. About 5 percent of points come out 45 cm BELOW the floor
plane. They are already discarded, since they fall outside all three height bands
-- and do NOT "fix" this by widening the floor band, or the artefact becomes data.

The cause is open. This was written up as the floor being polished and acting as a
mirror; it is not polished, there is bright light on it, so that explanation is
withdrawn. vslam/subfloor_check.py tells the two candidates apart: reflected
imagery clusters in z and ignores pixel brightness, glare-driven false matching
scatters in z and comes from the bright end of the image.

ACCURACY LIMIT FROM TILT. The +/-1.25 deg tilt confidence is 87 mm of height error
at 4 m, which is larger than the 60 mm floor band. So trust the grid at close
range and treat the far edge as indicative. A longer wall-to-camera measurement
would tighten this.


WHAT IS NOT VERIFIED

  LIVE OPERATION IS NOW CONFIRMED. 178 frames at 960x600 and 207 at 640x400, on
  the rover, grid updating continuously. The earlier noise problem was the camera
  being unplugged, and is fixed.

  THE GROUND PLANE FIT DOES NOT WORK RELIABLY ON THIS FLOOR. The office floor is
  dark and gives stereo almost no texture to match, and bright light on it adds
  specular glare. (Not polished -- an earlier draft said so and was wrong.)
  Three consecutive fits gave heights of 0.179, 0.227 and 0.205 m -
  a 12 percent spread - with only 15-16 percent support, and one earlier attempt
  produced a 31.9 degree tilt that was plainly wrong (with it, floor points came
  out as ZERO and every point sat 0.6-3.8 m above the plane).

  So do NOT fit the plane from stereo on the rover. MEASURE the mount height and
  tilt with a tape measure, or read the tilt from the IMU, and write those into
  plane.yaml by hand. The stereo fit is still useful as a cross-check - it agrees
  on roughly 0.2 m and 2-3 degrees - but it is not precise or stable enough to be
  the source of truth.

  This is the same limitation that argues for the sonars: a featureless surface
  gives stereo nothing, and a dark floor under bright glare is close to the worst
  case.

  ACCURACY AGAINST GROUND TRUTH. Nothing here was checked against a tape measure.
  The plane fit cross-validates against itself, which is not the same thing.

  ACCUMULATION WHILE MOVING. The grid is ROBOT-CENTRIC, so accumulating across
  frames is only valid while the robot is stationary or nearly so. Moving
  requires shifting the grid by the pose from cuVSLAM - that is the next task,
  and it is where the vslam/ work plugs in.


FILES

  depth_grid.py                the pipeline; --npz for offline, --live on the Jetson
  plane.yaml                   fitted ground plane (capture rig, refit on the rover)
  grid.png                     the occupancy grid, occupied dark, free light,
                               unknown grey, robot at the bottom, 1 m rings
  disparity.png                disparity map
  classification_and_grid.png  two frames with floor/obstacle/overhead painted on,
                               and the resulting grid


NEXT

  1. Fix the camera power, then run --live and confirm the fps and the grid.
  2. Refit the plane on the rover and save it to config.
  3. Check the grid against a tape-measured room: a wall at a known distance
     should land in the right cell.
  4. Feed cuVSLAM pose in so the grid can accumulate while driving.
  5. Add the sonar readings, which cover the blank walls stereo cannot see.


THE GRID MARKED FREE SPACE THROUGH SOLID OBJECTS - FOUND AND FIXED 2026-08-18

Found while verifying the merged pipeline on hardware for the first time. It is a
flaw in the inverse sensor model, not a tuning problem, and it was present from
the beginning.

update_grid swept a bearing free all the way to Z_MAX whenever that bearing had
no obstacle:

    far = Z_MAX if not np.isfinite(rr) else rr

The docstring is careful about the other case -- "beyond it stays unknown because
it was never observed... Marking beyond-the-hit as free is the classic bug" -- and
then the no-hit branch commits the same error in a different form. Because for a
STEREO sensor, "no obstacle here" and "no measurement here" arrive identically.
Absence of disparity is not evidence of absence.

WHAT IT LOOKED LIKE ON REAL DATA. The camera was facing three black desk panels
1-2 m away, backlit by a window. 93% of that frame sat below 60 counts, the panels
returned no disparity at all, and the grid swept free space straight through them
to 5 m. Two large free wedges appeared to the left and right where the sensor had
no data whatsoever.

THE FIX. bearing_reach() returns the furthest VALID measurement per bearing, from
ALL depth points rather than only the obstacles -- a floor return at 2 m still
proves that ray was observed out to 2 m. update_grid takes it as seen_to and stops
free space at whichever comes first, the nearest obstacle or the end of the
evidence. Bearings with no returns stay wholly unknown.

    seen_to=None reproduces the old behaviour exactly, and
    tests/test_offline.py pins that, so the change is additive.

MEASURED, same scene, 25 s each, 116-117 integrated frames:

                              legacy      evidence-bounded
  bearings with no data        swept free  21% of 180 left unknown
  free cells (offline frame)   4680        2850
  occupied cells               26          26

1830 cells, 39% of everything the grid called free, had never been observed. The
occupied count is identical, which is the point: bounding free space must not
invent or remove obstacles, and a test asserts that.

WHY EXPOSURE IS NOT THE ANSWER HERE, though it looked like it at first. The scene
exceeds the sensor's dynamic range: auto-exposure metered its ROI at a median of
225 against a target of 300 with 30% of it already saturated, correctly reported
"scene exceeds sensor dynamic range", and held. It was right to. No single global
exposure renders both a window and a black panel.

CLAHE MAKES IT WORSE, measured on one frame:

  as captured     valid 29.8%    of dark pixels, 29.5% got depth
  CLAHE clip 2    valid 22.4%                    20.7%
  CLAHE clip 4    valid 18.0%                    14.7%

Applied per eye it gives each image a different local transfer function, which
breaks the photometric consistency the block matcher depends on. If it is ever
used for stereo it has to apply the SAME curve to both eyes.

WHAT THIS MEANS GENERALLY. Untextured surfaces are not an edge case: a plain
painted wall, a glass door, a dark panel and a blown-out window all return
nothing. Any occupancy grid built on stereo has to treat "no return" as unknown.
The fix above does that; the remaining exposure and dynamic-range work changes how
MUCH is unknown, not whether the distinction is made.

Sub-floor points remain large on this floor: about 28700 per frame, roughly 16% of
valid points, more than 10 cm below the floor plane, consistent with the 19.7%
measured from a single cloud. Cause unresolved -- run vslam/subfloor_check.py. It
is not the polished floor an earlier draft blamed; the floor is not polished.


SUB-FLOOR POINTS: BOTH MY CANDIDATES REFUTED - 2026-08-18

vslam/subfloor_check.py, run at exposures 1500, 4000 and 8000 with consistent
results. 28-29% of the frame gets valid depth; 21-23% of those points land more
than 10 cm below the floor plane.

CANDIDATE 1, GLARE-DRIVEN FALSE MATCHING: REFUTED. If a specular highlight sat in
a different place in each eye, the bad matches would come from bright pixels. They
come from dim ones, at every exposure:

  brightest 10% of matched pixels     16-17% land sub-floor
  all the others                      23-24% land sub-floor

A ratio of 0.7, i.e. bright pixels are LESS likely to be wrong, not more.

CANDIDATE 2, COHERENT REFLECTED IMAGERY: REFUTED. A mirror ghost would cluster
tightly in z at a depth-dependent offset. The sub-floor cloud spreads 0.62-0.96 m
between its 5th and 95th percentiles -- diffuse, not a surface.

WHAT THE DATA ACTUALLY SHOWS, and it is the same at every exposure:

  range        points    sub-floor
  0.4-1.0 m     1280       0.0%
  1.0-2.0 m    93000      17.2%
  2.0-3.0 m    22000      66.5%
  3.0-5.0 m    33000      17.8%

Nothing in the near field, two thirds of everything at 2-3 m. The effect is
RANGE-CONCENTRATED, which neither candidate predicts.

LEADING REMAINING EXPLANATION, untested: range-dependent depth error on shallow
floor rays. A point on the floor is seen along a downward ray, and over-estimating
its depth slides it along that ray to below the floor plane. Depth error grows as
range squared -- one disparity pixel is 42 mm at 2 m and 95 mm at 3 m -- while the
floor's ray angle gets shallower with range, so there is a band where the error
first exceeds the 10 cm threshold. That would put the peak at a particular range
and leave the near field clean, which is what is measured.

THE TEST THAT WOULD CONFIRM IT: take the sub-floor points, reduce each one's depth
by one or two disparity steps, and check whether they land on the floor plane. If
they do, these are mis-ranged floor, not reflections of anything, and the fix is a
range gate rather than a reflection filter. depth_grid.py's existing 0.4-5.0 m gate
is too generous if the damage concentrates at 2-3 m.

A NOTE ON MEASURING EXPOSURE. Raising exposure 1500 -> 4000 -> 8000 moved the
matched-pixel intensity median from 6 to 7 to 6. That is not the sensor ignoring
exposure; bin2x2 stretches every frame to its own 1-99 percentile, so the 8-bit
output is nearly invariant to it. Judge exposure on the RAW frame or you are
measuring the stretch.
