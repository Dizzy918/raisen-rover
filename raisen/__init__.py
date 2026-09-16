"""Shared core for the Raisen stereo/vSLAM/mapping stack.

WHY THIS PACKAGE EXISTS. Every component was built as a standalone script that
opens /dev/video0 itself. Counted across the tree at the time this was written:

  18 files call cv2.VideoCapture on the camera
  15 files reimplement the rotate-180-and-swap for the inverted mount
  13 files reimplement bin2x2
  11 files reimplement calibration loading and rectification
   6 files build their own StereoSGBM
  20 files know how to stop and restart narsil-vision

The camera admits exactly ONE consumer, so those scripts are not merely
duplicated, they are MUTUALLY EXCLUSIVE. Running the occupancy grid meant not
running vSLAM. That is the thing this package exists to fix: one owner of the
device, one definition of the geometry, and consumers that subscribe rather than
compete.

DEPLOYMENT. This directory is deployed to /home/narsil/vslam-min/raisen, which
the container already mounts at /workspaces -- so the container gets it for free
and host scripts add /home/narsil/vslam-min to sys.path.

WHAT IS DELIBERATELY NOT TOUCHED. The standalone diagnostics (measure_floor.py,
check_rectification.py, measure_target.py, range_sweep.py, cloud_check.py,
auto_exposure.py's CLI, live_feed.py) still carry their own copies. They are
proven on hardware and the hardware is currently off; rewriting code that cannot
be re-tested is how a working stack becomes a broken one. tests/test_offline.py
pins the package against the same real frames those scripts used, and asserts the
duplicated geometry agrees numerically with vslam/camera_extrinsics.py, so the
copies cannot drift silently while they wait their turn.
"""

__all__ = ['frame', 'calib', 'depth', 'controls', 'camera', 'hub', 'service',
           'roverlink', 'exposure']
