#!/bin/bash
# cuVSLAM smoke test: static rectified stereo pair -> odometry
set +u
. /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77          # keep off narsil's domain

# The isaac_ros_gxf env hook does NOT add the gxf plugin dirs to the loader
# path, so the node dies with "libgxf_serialization.so: cannot open shared
# object file". Add every gxf lib subdir explicitly.
GXF=/opt/ros/humble/share/isaac_ros_gxf/gxf/lib
export LD_LIBRARY_PATH="$(find $GXF -maxdepth 1 -type d | tr '\n' ':')$LD_LIBRARY_PATH"

echo "=== starting cuVSLAM node ==="
ros2 run isaac_ros_visual_slam isaac_ros_visual_slam --ros-args \
  -p num_cameras:=2 \
  -p camera_optical_frames:="['left_cam','right_cam']" \
  -p base_frame:=base_link \
  -p rectified_images:=true \
  -p enable_imu_fusion:=false \
  -p enable_image_denoising:=false \
  -p image_jitter_threshold_ms:=40.0 \
  > /tmp/vslam_node.log 2>&1 &
VS=$!
sleep 10
kill -0 $VS 2>/dev/null && echo "node alive" || echo "NODE DIED"

echo "=== starting publisher ==="
python3 /workspaces/pub_pair.py > /tmp/pub.log 2>&1 &
PB=$!
sleep 18

echo "=== visual_slam topics ==="
ros2 topic list | grep -E "visual_slam" | head -14
echo
echo "=== odometry rate ==="
timeout 12 ros2 topic hz /visual_slam/tracking/odometry 2>&1 | head -5
echo
echo "=== one odometry message ==="
timeout 10 ros2 topic echo --once /visual_slam/tracking/odometry 2>&1 | head -20
echo
echo "=== node log (last 20) ==="
tail -20 /tmp/vslam_node.log

kill $PB $VS 2>/dev/null
wait 2>/dev/null
