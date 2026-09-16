#!/bin/bash
# Verify the fuse_heading.py gyro gate against a scripted scenario.
#
# No camera and no rover: a fake T:126 server and a fake odometry publisher feed
# the REAL fuse_heading.py, so the code under test is the committed code and not
# a reimplementation of it. ROS_DOMAIN_ID 88 keeps it off the live domain 77.
set +u
cd /workspaces/gatetest
. /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export PYTHONPATH=/workspaces/gatetest:$PYTHONPATH

T0=$(python3 -c 'import time; print(time.time() + 6.0)')
echo "T0 = $T0"

python3 fake_imu.py "$T0" 8099 &
IMU=$!
sleep 1
python3 fake_odom.py "$T0" > /tmp/fake_odom.log 2>&1 &
OD=$!
sleep 1

python3 /workspaces/fuse_heading.py \
  --imu-host 127.0.0.1:8099 --imu-hz 15 \
  --seconds 53 --tau 12 --csv /workspaces/gatetest/gate_run.csv \
  --markfile /tmp/nomark_gate
FR=$?

kill $IMU $OD 2>/dev/null
wait 2>/dev/null
echo "=== fuse_heading exit $FR ==="
echo
python3 gate_check.py "$T0" /workspaces/gatetest/gate_run.csv
