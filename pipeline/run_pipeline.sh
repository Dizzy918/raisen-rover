#!/bin/bash
# The merged pipeline: cuVSLAM + stereo publisher + occupancy grid on ONE camera.
#
# Run on the Jetson:
#   ./run_pipeline.sh [seconds]
#
# Env:
#   FUSE=1        also run the IMU heading fusion (needs rover-01 powered)
#   AE=1          auto-exposure instead of a fixed exposure=1500 gain=800
#   LEGACY_TF=1   identity base_link->left_cam, for reproducing pre-extrinsics runs
#   GROUND=fit    per-frame ground plane fit instead of the measured extrinsics
#
# WHAT REPLACED WHAT. This runs in place of run_live_vslam.sh AND depth_grid.py
# --live, which could not previously run at the same time because each opened
# /dev/video0 itself. fuse_heading.py is still its own process: it only ever
# wanted the odometry topic and the ESP32, so it was never part of the contention.
set +u
SECS=${1:-60}

# narsil-vision has restart=on-failure and `docker stop` exits 137, which counts
# as a failure -- so without disabling the policy first it is back within a second
# and has taken the camera again. Restored on the way out, including on a signal.
cleanup() {
  docker update --restart=on-failure narsil-vision >/dev/null 2>&1
  docker start narsil-vision >/dev/null 2>&1
}
trap cleanup EXIT INT TERM
docker update --restart=no narsil-vision >/dev/null 2>&1
docker stop narsil-vision >/dev/null 2>&1
sleep 4

IMU_HOST=""
if [ -n "$FUSE" ]; then
  # Constructing Rover() re-resolves the address by MAC and persists it; the IP on
  # disk has gone stale three times in four days. Sends no motion command.
  IMU_HOST=$(python3 - <<'PY'
import sys
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
) || { echo "FUSE=1 but rover-01 is unreachable -- powered on?"; exit 1; }
  echo "IMU host: $IMU_HOST"
fi

# Exposure is set here as a STARTING POINT only. With AE=1 the pipeline meters
# every frame itself from inside the container, via raisen.controls -- which falls
# back to the VIDIOC_S_CTRL ioctl because vslam-min ships no v4l-utils.
v4l2-ctl -d /dev/video0 -c exposure=${EXPOSURE:-1500}
v4l2-ctl -d /dev/video0 -c analogue_gain=${GAIN:-800}

docker run --rm --runtime nvidia --network host --ipc host \
  --privileged -v /dev:/dev --group-add video \
  -v /home/narsil/vslam-min:/workspaces \
  -v /tmp:/tmp \
  -e SECS="$SECS" -e IMU_HOST="$IMU_HOST" -e FUSE="$FUSE" -e AE="$AE" \
  -e LEGACY_TF="$LEGACY_TF" -e GROUND="${GROUND:-measured}" \
  vslam-min:3.2 bash -c '
set +u
. /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
export PYTHONPATH=/workspaces:/workspaces/mapping:$PYTHONPATH

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

ARGS="--calib /workspaces/calib --seconds $SECS --ground $GROUND"
ARGS="$ARGS --out /tmp/pipeline/grid.png --preview-dir /tmp/pipeline"
[ -n "$AE" ] && ARGS="$ARGS --auto-exposure"
[ -n "$LEGACY_TF" ] && ARGS="$ARGS --legacy-tf"

if [ -n "$FUSE" ]; then
  echo "=== heading fusion (separate process, no camera) ==="
  python3 /workspaces/fuse_heading.py --imu-host "$IMU_HOST" \
    --seconds $((SECS + 20)) --csv /workspaces/pipeline_run.csv \
    --markfile /tmp/mark > /tmp/fuse.log 2>&1 &
  FU=$!
fi

echo "=== merged pipeline, $SECS s ==="
python3 /workspaces/pipeline/run_pipeline.py $ARGS

echo "=== cuVSLAM warnings ==="; grep -iE "warn|error" /tmp/vslam.log | tail -5
[ -n "$FUSE" ] && { echo "=== fusion tail ==="; tail -12 /tmp/fuse.log; }
kill $FU $VS 2>/dev/null
wait 2>/dev/null
'
echo "grid:    /tmp/pipeline/grid.png"
echo "preview: /tmp/pipeline/left.jpg, /tmp/pipeline/pair.jpg"
[ -n "$FUSE" ] && echo "csv:     /home/narsil/vslam-min/pipeline_run.csv"
