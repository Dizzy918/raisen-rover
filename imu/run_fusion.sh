#!/bin/bash
# cuVSLAM + ESP32 IMU heading fusion, on the live camera.
#
# Run on the Jetson:
#   ./run_fusion.sh [seconds] [csv_name]
#
# Annotate a run from a second shell while it is going:
#   ssh narsil@jetson9.local 'echo "straight start" > /tmp/mark'
#
# The CSV lands in /home/narsil/vslam-min/ and holds one row per vSLAM frame:
# pose, vSLAM yaw, vSLAM-only integrated yaw, IMU yaw, IMU age, fused yaw, marker.
# That is everything the physical tests need to be measured after the fact rather
# than read off a scrolling terminal.
set +u
SECS=${1:-60}
CSV=${2:-fusion_run.csv}

# narsil-vision holds /dev/video0. It has restart=on-failure and `docker stop`
# exits 137, which counts as a failure -- so without disabling the policy it
# comes back within a second and takes the camera again.
cleanup() {
  docker update --restart=on-failure narsil-vision >/dev/null 2>&1
  docker start narsil-vision >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

docker update --restart=no narsil-vision >/dev/null 2>&1
docker stop narsil-vision >/dev/null 2>&1
sleep 4

# rover-01's address has moved three times in four days (.196 -> .194 -> .188).
# Constructing Rover() on the host re-resolves by MAC and persists the result,
# so read the config AFTER that rather than trusting what is on disk.
# This sends no motion command: the heartbeat thread only transmits once
# drive() has been called.
IMU_HOST=$(python3 - <<'PY'
import json, os, sys
sys.path.insert(0, '/home/narsil/bin')
try:
    from rover import Rover
    with Rover() as r:
        if r.imu() is None:
            raise SystemExit('rover did not answer T:126')
        print(r.cfg['host'])
except SystemExit:
    raise
except Exception as e:
    sys.exit('rover discovery failed: %s' % e)
PY
) || { echo "cannot reach the IMU -- is rover-01 powered on?"; exit 1; }
echo "IMU host: $IMU_HOST"

# Exposure and gain from the host: v4l-utils is not in vslam-min. Exposure is in
# sensor row-times and caps the frame rate -- 1500 is ~30 fps, 8000 would be 5.8.
EXPOSURE=${EXPOSURE:-1500}
GAIN=${GAIN:-800}
v4l2-ctl -d /dev/video0 -c exposure=$EXPOSURE
v4l2-ctl -d /dev/video0 -c analogue_gain=$GAIN
echo "host set exposure=$EXPOSURE gain=$GAIN"

docker run --rm --runtime nvidia --network host --ipc host \
  --privileged -v /dev:/dev --group-add video \
  -v /home/narsil/vslam-min:/workspaces \
  -v /tmp:/tmp \
  -e SECS="$SECS" -e IMU_HOST="$IMU_HOST" -e CSV="$CSV" \
  -e LEGACY_TF="${LEGACY_TF:-}" \
  vslam-min:3.2 bash -c '
set +u
. /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77

# The isaac_ros_gxf env hook leaves the GXF plugin dirs off the loader path, so
# the node dies with "libgxf_serialization.so: cannot open shared object file".
GXF=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
export LD_LIBRARY_PATH="$(find $GXF -maxdepth 1 -type d | tr "\n" ":")$LD_LIBRARY_PATH"

echo "=== cuVSLAM node ==="
ros2 run isaac_ros_visual_slam isaac_ros_visual_slam --ros-args \
  -p num_cameras:=2 \
  -p camera_optical_frames:="[left_cam,right_cam]" \
  -p base_frame:=base_link \
  -p rectified_images:=true \
  -p enable_imu_fusion:=false \
  -p enable_image_denoising:=false \
  -p image_jitter_threshold_ms:=120.0 \
  > /tmp/vslam.log 2>&1 &
VS=$!
sleep 10
kill -0 $VS 2>/dev/null && echo "node alive" || { echo "NODE DIED"; tail -20 /tmp/vslam.log; exit 1; }

echo "=== live stereo publisher ==="
# LEGACY_TF=1 restores the identity base_link->left_cam transform, for A/B-ing
# whether the measured height and tilt broke cuVSLAM'"'"'s rotation estimate.
PUBARGS=""
if [ -n "$LEGACY_TF" ]; then PUBARGS="--legacy-tf"; echo "PUBLISHER: LEGACY TF"; fi
python3 /workspaces/live_vslam.py --calib /workspaces/calib $PUBARGS > /tmp/pub.log 2>&1 &
PB=$!
sleep 12

# FAIL LOUDLY IF THE PUBLISHER HAS NO CAMERA. Without this the run continues,
# fuse_heading logs a header and zero rows, and the summary says "odometry frames
# 0" twelve lines below where anyone is looking. A whole repeatability test was
# driven against an empty CSV that way: /dev/video0 had vanished after a reboot,
# the publisher said so in its own log, and nothing checked.
if ! kill -0 $PB 2>/dev/null || grep -qiE "cannot open|No such file|is narsil-vision" /tmp/pub.log; then
  echo "PUBLISHER FAILED -- refusing to run. Its log says:"
  tail -5 /tmp/pub.log
  echo
  echo "If it cannot open /dev/video0, check the device exists at all:"
  echo "    ls -l /dev/video0        # gone entirely after some reboots"
  echo "    lsmod | grep arducam     # module can load and still probe nothing"
  echo "See jetson/camera_device_tree.txt."
  kill $VS 2>/dev/null
  exit 1
fi
echo "publisher alive"


echo "=== fusion, $SECS s ==="
rm -f /tmp/mark
python3 /workspaces/fuse_heading.py \
  --imu-host "$IMU_HOST" --seconds "$SECS" \
  --csv "/workspaces/$CSV" --markfile /tmp/mark

echo "=== publisher ==="; tail -3 /tmp/pub.log
echo "=== cuVSLAM warnings ==="; grep -iE "warn|error" /tmp/vslam.log | tail -4
kill $PB $VS 2>/dev/null
wait 2>/dev/null
'
echo "csv: /home/narsil/vslam-min/$CSV"
