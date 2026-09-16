#!/usr/bin/env bash
# Stereo camera diagnostic for Jetson Orin Nano + Arducam.
# Run after any reseat/reboot:  ./cam_diag.sh
set +e

hr() { printf '\n=== %s ===\n' "$1"; }

hr "platform"
tr -d '\0' < /proc/device-tree/model; echo
head -1 /etc/nv_tegra_release
echo "kernel: $(uname -r)"
echo "booted Image: $(grep -E '^\s*LINUX' /boot/extlinux/extlinux.conf | head -1)"
echo "FDT lines: $(grep -cE '^\s*FDT' /boot/extlinux/extlinux.conf) (0 = DTB comes from flashed partition)"

hr "video devices"
ls -l /dev/video* 2>&1

hr "v4l2 devices"
v4l2-ctl --list-devices 2>&1

hr "formats per device"
for d in /dev/video*; do
  [ -e "$d" ] || continue
  echo "--- $d"
  v4l2-ctl -d "$d" --list-formats-ext 2>&1 | head -25
done

hr "camera modules in device tree"
for m in /proc/device-tree/tegra-camera-platform/modules/*/; do
  [ -e "$m/badge" ] && { printf '%s -> ' "$(basename "$m")"; tr -d '\0' < "$m/badge"; echo; }
done

# i2c-9 is "i2c-2-mux (chan_id 1)" and i2c-10 is "chan_id 0", so bus 10 is most
# likely CAM0 and bus 9 CAM1 -- the opposite of the obvious guess. Both are
# scanned, so it only matters when reading the result.
hr "I2C scan on CSI buses (10 = likely CAM0, 9 = likely CAM1)"
for b in 9 10; do
  echo "--- bus $b"
  sudo i2cdetect -y -r "$b" 2>&1
done

hr "driver state"
lsmod | grep -E 'arducam|tegra_camera'
echo "tegra-camera.ko in use:"
modinfo tegra_camera 2>&1 | grep -E '^filename'
echo "arducam patched copy: /boot/arducam/tegra-camera.ko"
md5sum /boot/arducam/tegra-camera.ko \
       /lib/modules/"$(uname -r)"/updates/drivers/media/platform/tegra/camera/tegra-camera.ko 2>&1

hr "kernel messages (camera)"
sudo dmesg | grep -iE 'arducam|imx|ov[0-9]{4}|nvcsi|vi-output|tegra-capture' | tail -25

hr "nvargus daemon"
systemctl is-active nvargus-daemon

hr "verdict"
# NOTE: with a Camarray HAT only ONE CSI port is populated, so exactly one of
# buses 9/10 should answer at 0x0c. A failed probe on the empty port is normal
# and cosmetic -- do not chase it.
# A device at 0x0c shows as "0c" when unclaimed, but as "UU" once a driver has
# bound to it -- which is the SUCCESS case. Match both, and read column 0x0c of
# the "00:" row specifically (field 5 = addresses 08..0f -> 0c is the 5th).
found_bus=""
scanned=0
for b in 9 10; do
  # Keep stderr: a sudo that cannot run is NOT an empty bus, and conflating the
  # two is how this script previously reported "the HAT is not answering" about
  # a scan it never performed.
  raw=$(sudo -n i2cdetect -y -r "$b" 2>&1)
  if printf '%s' "$raw" | grep -qiE 'password|not allowed|terminal is required'; then
    continue
  fi
  scanned=1
  cell=$(printf '%s' "$raw" | awk '/^00:/ {print $6}')
  case "$cell" in
    0c|UU) found_bus="$b" ;;
  esac
done

if [ "$scanned" -eq 0 ]; then
  echo "I2C SCAN DID NOT RUN -- sudo needs a password here, so the bus state is"
  echo "UNKNOWN. This is not a hardware finding. Re-run with a terminal:"
  echo "    ssh -t narsil@\$(hostname).local 'bash ~/stereo/cam_diag.sh'"
  echo "Do not reseat anything on the strength of this run."
elif ls /dev/video* >/dev/null 2>&1; then
  echo "Camera ENUMERATED on i2c bus ${found_bus:-?}."
  echo "Camarray HAT -> expect ONE device with a DOUBLE-WIDTH frame."
  echo "Check the width reported above, then:"
  echo "  ./stereo_capture.py --split --raw-only"
elif [ -n "$found_bus" ]; then
  echo "HAT ANSWERS on i2c bus $found_bus but no /dev/video* was created."
  echo "That is a driver / device-tree problem, not wiring. Likely the DT is"
  echo "configured for dual independent cameras rather than a single HAT, or"
  echo "the HAT firmware predates the JetPack 5.x+ requirement."
else
  echo "NO I2C response on either CSI bus -> the HAT is not answering at all."
  echo "  1. Is the HAT's USB-C power input connected to 5V?"
  echo "  2. Is the single CSI ribbon fully seated at BOTH ends, latch closed,"
  echo "     contacts facing the correct way?"
  echo "  3. Are both camera modules seated in the HAT?"
  echo "  (Power the board down before reseating anything -- CSI is not hot-pluggable.)"
fi
