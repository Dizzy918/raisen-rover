VSLAM: DECISION AND VERIFICATION
jetson9, 2026-08-13


DECISION: Isaac ROS Visual SLAM (cuVSLAM), ros-humble-isaac-ros-visual-slam 3.2.6


THE COMPARISON

  Isaac ROS / cuVSLAM   CHOSEN. Built by NVIDIA for this exact SoC. Installs as
                        a single apt package into the Isaac ROS base image, which
                        was already on this Jetson. GPU-accelerated, actively
                        maintained. Raisen has already run it live on this same
                        AR0234 stereo HAT (three runs, 2026-06-11), so the camera
                        integration is a solved problem here.

  ORB-SLAM3             CPU-based, which would dodge the GPU contention below.
                        But no official ROS 2 wrapper, a painful build against
                        JetPack 6 / OpenCV 4.8, and effectively unmaintained.
                        Days of work to reach where cuVSLAM already is.

  Custom OpenCV         Visual ODOMETRY, not SLAM. No loop closure, so drift is
                        never corrected. Worth writing as a learning exercise,
                        not as the thing that ships.

GPU CONTENTION - MEASURED 2026-08-14, AND IT IS FINE

This was the one open risk in choosing cuVSLAM. Measured properly, 118 samples
at 500 ms each, cuVSLAM fed a static pair so it shares the GPU with the live
detector without fighting over /dev/video0:

                        mean   median   p90   max   >=95%
  narsil-vision only    17.2%    3.0%   58%   79%     0%
  vision + cuVSLAM      21.0%    3.0%   71%   99%     3%

  cuVSLAM adds +3.8 percentage points of mean GPU, and still holds 28 Hz while
  sharing. The ORB-SLAM3 CPU fallback is not needed.

CORRECTION: earlier notes in this project cite "GR3D 65-87 percent" as the
baseline. That came from a 2-sample glance that happened to catch bursts. The
detector is very bursty - median 3 percent, p90 58 percent - so a short window is
badly misleading. The real mean is 17 percent.


WHAT WAS NOT USED, AND WHY

poc-isaac-ros is a DRONE project. Its docker/Dockerfile pulls in mavros,
mavros-extras, geographiclib, orbbec-camera, PeopleNet, ESS, RAFT and
Depth-Anything, and the `_fcu` in its run names is Flight Control Unit. A ground
rover doing stereo pose needs none of it, and building that image is a 14-stage
job on a disk already at 79 percent.

Its cuVSLAM launch file is still worth reading though - vslam_ar0234.launch.py
documents hard-won lessons about this exact camera (see FOR THE LIVE PIPELINE).

Instead: isaac-ros-base:3.2 (already on disk) plus ONE apt package. See Dockerfile.


LIVE - VERIFIED 2026-08-14

  odometry            1244 messages in 40 s = 31.0 Hz
  publisher           31.9 fps, 20.8 ms/frame processing
  stationary drift    0.0100 m translation, -0.33 deg yaw over 40 s

THAT DRIFT IS THE FLOOR. Every later distance or angle measurement is only as
good as it. Note it is WORSE than the same test at 4.5 Hz (1.5 mm, 0.01 deg):
more frames means more integration steps and more accumulated noise. A higher
rate helps tracking while moving and costs a little at rest.

    ./run_live_vslam.sh 40

FOUR BLOCKERS FOUND GETTING HERE

1. narsil-vision RESTARTS ITSELF within a second of docker stop, and takes
   /dev/video0 back. Its policy is restart=on-failure with unlimited retries, and
   `docker stop` exits 137, which counts as a failure. Every "stop the camera
   consumer" step in this project was silently being undone. Disable the policy
   first, then stop, then restore:

     docker update --restart=no  narsil-vision && docker stop  narsil-vision
     ... work ...
     docker update --restart=on-failure narsil-vision && docker start narsil-vision

   run_live_vslam.sh does this in a trap. depth_grid.py --live does NOT yet.

2. msg.data = img.tobytes() COSTS 96 ms per image in rclpy. Assigning raw bytes
   to a uint8[] field takes a slow per-element path. Two images per frame was
   193 ms of a 210 ms budget and pinned the publisher at 4.5 fps. Use
   array.array('B', img.tobytes()) instead -- it matches the field's internal
   type and assigns in ~0 ms. Measured: assign bytes 96.48 ms, assign
   array.array 0.00 ms, publish() itself 0.17 ms. This one line was a 7x speedup.

3. The container needs --privileged, not just --device /dev/video0. With only
   --device, reads fail with "select() timeout": the Tegra VI path needs more
   than the video node. narsil-vision and poc-isaac-ros both run privileged for
   the same reason.

4. v4l-utils is not in this image, so v4l2-ctl is missing inside the container.
   The runner sets exposure and gain from the host instead.


VERIFICATION - STATIC PAIR, PASSED

Built vslam-min:3.2, published a static rectified stereo pair at 30 Hz, ran the
cuVSLAM node, and confirmed it tracks.

  node                 alive, no errors
  topics               14 visual_slam topics up, including image_N/nitros
  odometry rate        ~27 Hz on /visual_slam/tracking/odometry
  pose                 position 0,0,0  orientation identity

The zero pose is the CORRECT result: the same frame is republished, so there is
no motion to report. What is being verified is that cuVSLAM ingests our images
and our calibration, initialises, and produces odometry - not that it moves.

CAVEAT ON THIS TEST. The input turned out to be pure sensor NOISE - the camera
was unplugged at the time, which was only discovered later. So this verifies that
cuVSLAM loads, ingests images and camera_info, initialises and publishes odometry.
It does NOT verify that it tracks anything: the identity pose is equally
consistent with tracking nothing at all.

A corner-count check (goodFeaturesToTrack returned 500) was used to claim the
input had texture. That check is worthless - it happily returns 500 corners on
noise. Look at the image; do not trust a corner count.


TWO PROBLEMS HIT, AND THE FIXES

1. libgxf_serialization.so: cannot open shared object file

   The node dies immediately. The library IS present and
   ros-humble-isaac-ros-gxf IS installed - the package's environment hook simply
   does not add the gxf plugin directories to the loader path. Fix, in
   run_test.sh:

     GXF=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
     export LD_LIBRARY_PATH="$(find $GXF -maxdepth 1 -type d | tr '\n' ':')$LD_LIBRARY_PATH"

   Anything that runs cuVSLAM outside the poc image needs this.

2. "Delta between current and previous frame [66.67 ms] is above threshold"

   cuVSLAM sees occasional 66.67 ms gaps - exactly two frame intervals - so it is
   missing about one frame in ten, which matches 27 Hz output against 30 Hz
   published. Harmless for a smoke test. For a live run, either raise
   image_jitter_threshold_ms or find the drop (QoS depth, or the node not keeping
   up).


FILES

  Dockerfile      isaac-ros-base:3.2 + ros-humble-isaac-ros-visual-slam. That is all.
  grab_pair.py    grab one rectified mono8 stereo pair from the AR0234 (run on
                  the Jetson host, needs /dev/video0 free)
  pub_pair.py     republish that pair at a steady 30 Hz with the camera_info and
                  the static TFs cuVSLAM needs for the rig geometry
  run_test.sh     start the node, start the publisher, check odometry
  live_vslam.py   live camera -> rectified mono8 + camera_info + TF for cuVSLAM
  run_live_vslam.sh  starts cuVSLAM + the live publisher, logs odometry
  pose_log.py     logs odometry rate, pose and net drift

The static test pair is NOT committed: the images captured at the time were pure
sensor noise (the camera was unplugged), so they would be a misleading "test
pair". Regenerate with grab_pair.py when the camera is up.

REPRODUCE (on the Jetson)

  cd ~/vslam-min && docker build -t vslam-min:3.2 .
  docker stop narsil-vision            # frees /dev/video0
  python3 grab_pair.py                 # only if regrabbing the pair
  docker run --rm --runtime nvidia --network host --ipc host \
      -v /home/narsil/vslam-min:/workspaces vslam-min:3.2 bash /workspaces/run_test.sh
  docker start narsil-vision


FOR THE LIVE PIPELINE - LESSONS FROM vslam_ar0234.launch.py

Recorded in that file's header by whoever fought them:

  - cuVSLAM accepts mono8 ONLY. Feed it bgr8 and it throws "Received unknown
    image encoding: bgr8".
  - Do NOT feed it ESS-rectified images. The ROS-bridge conversion silently drops
    the rectified intrinsics, so cuVSLAM sees rectified pixels with distorted K.
    It warns "Falls back to raw camera model" and the poses are quietly wrong.
  - A missing input topic HANGS it - it blocks forever waiting on image_0 rather
    than erroring. There is a recorded case of the left chain publishing nothing
    while the right ran fine.
  - The debayer must be C++ inside the camera node; the Python republisher could
    not sustain the rate.

Also: that launch reads per-eye camera_info from config/camera/ar0234_<profile>_*.yaml.
The files currently there are the old 2560x720 ones. The new calibration in
stereo_calibration/out/ needs to land there for a live run.


UNRESOLVED

The AR0234 failed to probe at boot ("arducam_read: Reading register 0x103
failed") because the camera was unplugged. It came back once plugged in, no
module reload needed. Worth knowing that a cold boot without the camera attached
leaves the driver loaded but with no /dev/video0.


LIVE PIPELINE VERIFICATION - 2026-08-17

vslam/verify_live_pipeline.py, run via run_verify.sh. 120 s continuous, rover
stationary.

  CONTINUOUS POSE
    3571 odometry messages over 120.0 s, 29.77 Hz mean
    interval median 31.4 ms, p99 64.8 ms, max 156.9 ms
    zero gaps over 200 ms

  TIMESTAMP SYNCHRONISATION
    3812 left stamps, 3811 right, 3811 IDENTICAL pairs = 100% of the smaller side
    1 left-only, which is the final frame published after the count was taken
    Every pair shares one stamp exactly, which is what lets cuVSLAM treat the two
    images as simultaneous rather than interpolating between them.

  LATENCY, capture stamp to pose arrival
    mean 4.5 ms, median 4.4, p99 5.9, max 13.6
    first half 4.5 ms -> second half 4.4 ms, drift -0.0 ms
    Steady over two minutes, so nothing is queueing. Latency growth is the way
    this fails in practice: a filling queue looks perfect for ten seconds.

  ROTATION WELL FORMED
    quaternion norm 1.000000 across every message

  VERDICT: continuous, synchronised, real-time pose output sustained without
  lagging or crashing.


TWO HONEST CAVEATS

  THE ROVER WAS STATIONARY, so X, Y, Z and yaw hold near-constant non-zero values
  (-0.0043, -0.0013, +0.0005 m, yaw spread 0.0008 deg). That demonstrates the
  fields carry real values and the pose is stable at rest. It does NOT demonstrate
  that they respond correctly to motion -- that is a separate measurement and
  needs the rover to move.

  cuVSLAM DOES NOT PRODUCE A POSE FOR EVERY FRAME. The publisher sustained
  31.7 fps while odometry arrived at 29.77 Hz, so about 6% of frames yielded no
  pose. The node also logged six "Delta between current and previous frame above
  threshold" warnings, at 124-157 ms. Neither shows up as a gap in the output, and
  neither is a fault, but "31 fps in" is not "31 poses out" and anything counting
  frames should not assume it is.

  Measure it again after anything changes with:  ./run_verify.sh 120
