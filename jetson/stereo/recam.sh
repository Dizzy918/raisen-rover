#!/usr/bin/env bash
# Re-run the camera probe WITHOUT rebooting.
#
# Why you need this: the arducam_csi2 driver probes the HAT once, early in boot.
# If the HAT gets its 5V after that moment -- you plugged the USB-C in late, or
# it comes from a separate adapter switched on after the Jetson -- the probe has
# already failed and nothing will retry it. This forces a fresh probe.
#
# Safe to run repeatedly. Does NOT touch the CSI ribbon; still power the board
# down before reseating any cable.
set +e

echo "unloading arducam_csi2..."
sudo rmmod arducam_csi2 2>&1
sleep 1

echo "reloading arducam_csi2..."
sudo modprobe arducam_csi2 2>&1
sleep 2

echo
echo "--- fresh kernel messages ---"
sudo dmesg | grep -iE 'arducam|9-000c|10-000c' | tail -8

echo
echo "--- video devices ---"
ls -l /dev/video* 2>&1

echo
echo "--- I2C (expect 0c on exactly ONE bus: the port the HAT is in) ---"
for b in 9 10; do
  printf 'bus %s: ' "$b"
  if sudo i2cdetect -y -r "$b" 2>/dev/null | grep -qE '(^|[ :])0c([ ]|$)'; then
    echo "0x0c PRESENT"
  else
    echo "empty"
  fi
done

echo
if ls /dev/video* >/dev/null 2>&1; then
  echo "SUCCESS -> next: ~/stereo/stereo_capture.py --split --raw-only \\"
  echo "                   --cap-width 3840 --cap-height 1200 --width 3840 --height 1200"
else
  echo "still nothing -> run ~/stereo/cam_diag.sh for the full picture"
fi
