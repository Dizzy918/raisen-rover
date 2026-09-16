MERGING THE COMPONENTS
2026-08-18, written with jetson9 and rover-01 powered down


WHAT WAS WRONG

Every component was a standalone script that opened /dev/video0 itself. Counted
across the tree:

  18 files call cv2.VideoCapture on the camera
  15 files reimplement the rotate-180-and-swap for the inverted mount
  13 files reimplement bin2x2
  11 files reimplement calibration loading and rectification
   6 files build their own StereoSGBM
  20 files know how to stop and restart narsil-vision

The duplication is the small problem. The big one is that the camera admits
exactly ONE consumer, so those scripts are MUTUALLY EXCLUSIVE. Running the
occupancy grid meant not running vSLAM. Neither could run with the live preview.
Every component was proven alone and none had ever run beside another.


WHAT THE MERGE IS

raisen/ is now the single source of truth for the parts everything shares, and
pipeline/run_pipeline.py owns the camera once and fans frames out:

    StereoCamera --> FrameHub --+--> auto-exposure      (on the capture thread)
                                +--> vSLAM publisher    (mono8 + camera_info + TF)
                                +--> occupancy grid     (SGBM -> log-odds)
                                +--> preview snapshots  (JPEG)

  raisen/frame.py      sensor format, bin2x2, the split-and-swap
  raisen/calib.py      both calibrations, the Rectifier, camera-to-rover
  raisen/depth.py      SGBM sized FROM z_min, depth, railing check
  raisen/controls.py   v4l2 writes, with an ioctl fallback for the container
  raisen/camera.py     the single owner of /dev/video0
  raisen/hub.py        one capture thread, N latest-frame subscribers
  raisen/service.py    taking the camera from narsil-vision and giving it back
  raisen/roverlink.py  ESP32 IMU polling and address resolution
  raisen/exposure.py   the AE controller

fuse_heading.py stays a SEPARATE process on purpose. It only ever wanted the
odometry topic and the ESP32, never the camera, so it was never part of the
contention -- run_pipeline.sh starts it alongside with FUSE=1.


THREE REAL DISAGREEMENTS THE MERGE HAD TO SETTLE

1. TWO ANSWERS FOR THE GROUND PLANE. depth_grid.py fit its own plane every run
   while vSLAM used the measured camera_to_base.yaml. On a calibration frame the
   fit reports camera height 0.377 m and tilt 26.3 deg with 21% support, against
   a measured 0.1687 m and 2.72 deg -- it latched onto the checkerboard, the same
   failure that once returned a cardboard box front as "floor". depth_grid.py's own
   output says support above ~50% means a real floor dominates, and prints "a
   plausible-looking wrong plane makes a plausible-looking wrong map".

   So --ground measured is the default: it does not depend on what is in view.
   --ground fit reproduces the old behaviour exactly, and the offline test asserts
   that it does, number for number.

2. AUTO-EXPOSURE APPLIED TO NOTHING THAT MATTERED. It existed only inside the live
   preview; vSLAM and mapping both hardcoded exposure=1500 gain=800. It now runs on
   the capture thread, so every consumer gets the same metered frame -- and it
   meters the RAW frame, before bin2x2's percentile stretch hides what the sensor
   saw. This needed raisen/controls.py, because vslam-min ships no v4l-utils and
   AE has to write continuously from inside the container.

3. numDisparities WAS A CONSTANT IN SIX PLACES. It is now derived from z_min, the
   nearest depth the caller intends to trust, and make_matcher refuses to return a
   matcher that cannot reach it. The same bug appeared twice under two constants:
   128 made an ArUco marker at 0.43 m read 0.749 m, and 176 gave depth_grid.py a
   0.40-0.495 m band it trusted but structurally could not measure.


WHAT IS VERIFIED, AND HOW

  PYTHONPATH=.:mapping python3 tests/test_offline.py

24 checks, no hardware, against real captured frames:

  sensor format and the split, on stereo_calibration/captures/round4/raw_01.npz
  fx 585.04, baseline 148.914 mm, fx*baseline 87.120 -- pinned, because those
    numbers are quoted throughout the docs and a swapped calibration file would
    otherwise silently invalidate every recorded depth measurement
  the camera-to-rover rotation: forward -> +X, up -> +Z, right -> -Y, orthonormal
  raisen/calib.py agrees with vslam/camera_extrinsics.py to 1e-12
  all 14 tuned AE constants identical between raisen/exposure.py and the standalone
  the two historical near-limit bugs reproduce as arithmetic
  MERGE FIDELITY: pipeline --ground fit reproduces depth_grid.py's own numbers on
    the same frame exactly -- 38.0% valid, 218849 points, 3750 obstacles, 0 floor,
    26 occupied cells, 4680 free


WHAT IS NOT VERIFIED

Nothing below has run on hardware, because both machines were charging.

  the live camera path, the hub's fan-out, and the frame rate under three consumers
  the ROS publisher (rewritten around raisen, not the proven live_vslam.py)
  raisen/controls.py's ioctl fallback -- the whole reason AE can run in the
    container. Its ioctl numbers match the documented Linux values, which is not
    the same as having written a control.
  whether SGBM at 5 Hz plus publishing at 31 Hz fits in the Jetson's budget at all

live_vslam.py is UNCHANGED and still works. If the merged publisher misbehaves,
run_live_vslam.sh is the fallback that is known good.


ACCEPTANCE CHECKS, IN ORDER, ONCE THE MACHINES ARE BACK

  1. ./pipeline/deploy.sh                     md5-verified copy to jetson9
  2. HOST-side, camera only, no ROS:
       python3 pipeline/run_pipeline.py --calib ~/vslam-min/calib \
         --no-vslam --seconds 30 --stop-vision
     Expect the hub near 30 fps and the grid consumer near 5 Hz. This isolates
     capture and fan-out from everything ROS.
  3. Add AE: --auto-exposure. Check raisen.controls picked a backend and that
     exposure actually MOVED -- controls.verify() reads back, but gain above ~1100
     reads back correctly while doing nothing, so confirm against image brightness.
  4. ./pipeline/run_pipeline.sh 60            full stack, measured ground
  5. Compare the pose stream against run_live_vslam.sh over the same 60 s. The
     stationary drift floor was 1.0 cm and 0.33 deg over 40 s; the merged
     publisher must not be worse.
  6. FUSE=1 ./pipeline/run_pipeline.sh 60     add heading fusion, rover powered
  7. Only then delete nothing. Keep live_vslam.py and depth_grid.py --live until
     the merged path has a driving run behind it.


DELIBERATELY NOT DONE

The standalone diagnostics still carry their own copies of bin2x2, rectification
and calibration loading: measure_floor.py, measure_target.py,
check_rectification.py, range_sweep.py, cloud_check.py, verify_tf.py,
live_feed.py, auto_exposure.py. They are proven on hardware and the hardware is
off. Rewriting code that cannot be re-tested is how a working stack becomes a
broken one, so they wait -- and tests/test_offline.py pins the two copies that
could drift dangerously (the extrinsics maths and the AE constants) so they
cannot separate silently in the meantime.

Also not done, and still owed: the camera's fore/aft and lateral offset from the
rover's actual origin. base_link sits under the CAMERA, not at the wheel axis, so
a pure rotation about the rover's centre shows here as a small translation. Height,
pitch and roll are measured; a ruler from the wheel axis would finish it.

Mapping beyond the per-frame occupancy grid -- pose accumulation, map merging --
is deliberately out of scope. That is the team's own work and the handoff is the
grid plus the pose stream.


FIRST FULL-STACK RUNS ON HARDWARE - 2026-08-18

cuVSLAM plus the merged publisher, the occupancy grid, auto-exposure and preview,
all in one process on one camera. Three bugs and one architecture lesson.

BUG 1: THE IOCTL CONTROL PATH WORKED AND ITS NAMES DID NOT. raisen/controls.py's
VIDIOC_QUERYCTRL fallback enumerated the controls correctly on its first hardware
use -- which is what it was written for, since vslam-min ships no v4l-utils -- and
then every lookup failed. The ioctl reports human-readable labels, 'Exposure' and
'Analogue Gain'; v4l2-ctl -c takes lowercase snake_case ids. The error message
listed the control sitting right there. Both sides are normalised now.

BUG 2: AUTO-EXPOSURE STILL SHELLED OUT TO v4l2-ctl. The package copy inherited
subprocess.run(['v4l2-ctl', ...]) from the standalone, so inside the container it
died on FileNotFoundError at startup -- while raisen/controls.py sat unused, being
exactly the layer that exists to avoid this. AE now writes through it and prints
which backend it picked. The standalone keeps its subprocess call; it only ever
runs on the host.

BUG 3, THE ARCHITECTURE ONE, and it went the opposite way to expectation. Measured
on the same 60 s scene:

  split in each consumer    camera 30.7 fps   publisher 23.5 fps at 40.0 ms   20% dropped
  split on camera thread    camera 20.5       publisher 20.4 at 17.8 ms        0% dropped
  split in its own stage    camera 30.5       publisher 27.5 at 19.6 ms        2% dropped

The publisher, the grid and the preview were each running bin2x2 on the same frame,
so the obvious fix was to split once. Doing it on the capture thread halved the
whole hub: split_pair costs 31 ms, and serialising that behind the camera read
capped everything at 20.5 fps even though the publisher itself got CHEAPER than the
standalone's 21 ms.

The first arrangement was accidentally exploiting parallelism. numpy and cv2 drop
the GIL, so three concurrent bin2x2 calls achieve more aggregate throughput than one
serial call -- they were wasteful and fast at the same time. Removing the
duplication was still right, but only once it stopped blocking the camera. The split
now runs in its own stage, concurrent with the camera read.

WHAT THE MERGE COSTS, honestly: the publisher does 27.5 fps here against 31.7 fps
standalone, so about 13% of its rate buys a live occupancy grid and a preview on the
same camera. cuVSLAM still logs occasional 130-205 ms frame gaps, because the split
stage at 31 ms is now the limiter. Lowering --grid-hz would recover more.

STILL UNVERIFIED after these runs: nothing in the capture, fan-out, control or
publish path. What has NOT been checked is whether cuVSLAM's pose is as good through
the merged publisher as through live_vslam.py -- same rate is not the same accuracy,
and the frame gaps are a reason to check rather than assume.
