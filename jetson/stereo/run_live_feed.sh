#!/bin/bash
# Serve the live AR0234 view at http://jetson9.local:8080/
#
# Run on the Jetson. Frees /dev/video0 from narsil-vision and gives it back on
# exit, including on ctrl-c.
#
#   ./run_live_feed.sh            # runs until ctrl-c
#   EXPOSURE=4000 ./run_live_feed.sh
set +u

# narsil-vision has restart=on-failure and `docker stop` exits 137, which counts
# as a failure -- so it restarts within a second and takes the camera back.
# Disable the policy first, restore it on the way out.
cleanup() {
  echo
  echo "restoring narsil-vision..."
  docker update --restart=on-failure narsil-vision >/dev/null 2>&1
  docker start narsil-vision >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

docker update --restart=no narsil-vision >/dev/null 2>&1
docker stop narsil-vision >/dev/null 2>&1
sleep 4

# NOT exec: exec replaces this shell with python, which destroys the trap above,
# so narsil-vision is left stopped with restart=no and the camera never comes
# back. Learned by doing exactly that.
python3 "$(dirname "$0")/live_feed.py" \
  --exposure "${EXPOSURE:-1500}" --gain "${GAIN:-800}" --port "${PORT:-8080}"
