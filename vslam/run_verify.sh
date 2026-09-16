#!/bin/bash
# cuVSLAM on the LIVE AR0234 stereo pair.
#
# Run on the Jetson. Stops narsil-vision first (it holds /dev/video0) and
# restarts it on exit.
#
#   ./run_live_vslam.sh [seconds]
set +u
SECS=${1:-60}

# narsil-vision has restart=on-failure with unlimited retries, and `docker stop`
# exits 137 which counts as a failure -- so it comes back within a second and
# grabs /dev/video0 again. Disable the policy, stop it, and restore on the way out.
cleanup() {
  docker update --restart=on-failure narsil-vision >/dev/null 2>&1
  docker start narsil-vision >/dev/null 2>&1
}
trap cleanup EXIT

docker update --restart=no narsil-vision >/dev/null 2>&1
docker stop narsil-vision >/dev/null 2>&1
sleep 4

# Set exposure and gain from the HOST: v4l-utils is not in vslam-min (only the
# drone image installs it). Exposure is a frame-rate ceiling -- 1500 row-times is
# ~22 ms, so ~30 fps; 8000 would cap the camera at 5.8 fps.
EXPOSURE=${EXPOSURE:-1500}
GAIN=${GAIN:-800}
v4l2-ctl -d /dev/video0 -c exposure=$EXPOSURE
v4l2-ctl -d /dev/video0 -c analogue_gain=$GAIN
echo "host set exposure=$EXPOSURE gain=$GAIN" 

docker run --rm --runtime nvidia --network host --ipc host \
  --privileged -v /dev:/dev --group-add video \
  -v /home/narsil/vslam-min:/workspaces \
  -e SECS="$SECS" \
  vslam-min:3.2 bash -c '
set +u
. /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77

# The isaac_ros_gxf env hook does not put the GXF plugin dirs on the loader path,
# so the node dies with "libgxf_serialization.so: cannot open shared object file".
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
python3 /workspaces/live_vslam.py --calib /workspaces/calib > /tmp/pub.log 2>&1 &
PB=$!
sleep 12

echo "=== odometry rate ==="
timeout 15 ros2 topic hz /visual_slam/tracking/odometry 2>&1 | head -4

echo "=== pipeline verification, $SECS s ==="
python3 /workspaces/verify_live_pipeline.py --seconds "$SECS"

echo "=== publisher log ==="; tail -4 /tmp/pub.log
echo "=== node log (warnings) ==="; grep -iE "warn|error" /tmp/vslam.log | tail -6
kill $PB $VS 2>/dev/null
wait 2>/dev/null
'
