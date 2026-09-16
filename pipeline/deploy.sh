#!/bin/bash
# Copy the merged stack to the Jetson and PROVE it landed.
#
# This script exists because of a specific, repeated failure. Twice now, code was
# committed to git and never copied to jetson9, and both times the machine went on
# running the old file while the repo said otherwise:
#
#   live_vslam.py   patched locally, never uploaded, so the container kept dying
#                   on a host path that the local copy no longer contained.
#   fuse_heading.py the gyro gate was committed and never uploaded, so every run
#                   on that machine used the UNGATED filter. Found only by
#                   grepping the remote file for GYRO_GATE and getting zero.
#
# So it does not merely copy: it md5s both ends and refuses to claim success on a
# mismatch. "I committed it" is not the same as "it is running".
set -eu
HOST=${HOST:-narsil@jetson9.local}
DEST=${DEST:-/home/narsil/vslam-min}
HERE=$(cd "$(dirname "$0")/.." && pwd)

FILES=(
  raisen/__init__.py raisen/frame.py raisen/calib.py raisen/depth.py
  raisen/controls.py raisen/camera.py raisen/hub.py raisen/service.py
  raisen/roverlink.py raisen/exposure.py raisen/v4l2cap.py
  pipeline/run_pipeline.py
  mapping/depth_grid.py
  imu/fuse_heading.py imu/run_fusion.sh
  vslam/camera_extrinsics.py vslam/live_vslam.py
  rover/drive_test.py
)

echo "deploying to $HOST:$DEST"
ssh "$HOST" "mkdir -p $DEST/raisen $DEST/pipeline $DEST/mapping"
for f in "${FILES[@]}"; do
  case "$f" in
    raisen/*)   target="$DEST/$f" ;;
    pipeline/*) target="$DEST/$f" ;;
    mapping/*)  target="$DEST/mapping/$(basename "$f")" ;;
    # drive_test.py lives outside vslam-min: it needs ~/bin/rover.py on the HOST
    # and never runs inside the container.
    rover/*)    target="/home/narsil/rover/$(basename "$f")" ;;
    # run_fusion.sh sits beside the code it launches, not in a subdirectory.
    imu/*.sh)   target="$DEST/$(basename "$f")" ;;
    *)          target="$DEST/$(basename "$f")" ;;
  esac
  scp -q "$HERE/$f" "$HOST:$target"
  printf '%-34s ' "$f"
  local_md5=$(md5 -q "$HERE/$f" 2>/dev/null || md5sum "$HERE/$f" | cut -d' ' -f1)
  remote_md5=$(ssh "$HOST" "md5sum $target" | cut -d' ' -f1)
  if [ "$local_md5" = "$remote_md5" ]; then
    echo "ok  $local_md5"
  else
    echo "MISMATCH  local $local_md5  remote $remote_md5"
    exit 1
  fi
done
echo
echo "all files verified by md5 on both ends"
echo "note: pipeline/run_pipeline.py imports depth_grid, so the container needs"
echo "      PYTHONPATH=/workspaces:/workspaces/mapping -- run_pipeline.sh sets it."
