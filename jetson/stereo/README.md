# Stereo camera access + calibration — Jetson Orin Nano (Arducam)

## Current blocker

Neither camera responds on I2C. `i2cdetect -y -r 9` and `-r 10` (the two CSI
connectors) are completely empty, so there is no `/dev/video*` and no feed yet.
The software side is already correct — the Arducam driver is loaded and the
device tree declares both camera modules — so this is wiring, power, or a
camera model mismatch, not configuration.

Run `./cam_diag.sh` after any change; it ends with a verdict line.

## Hardware

- 2 × Arducam **AR0234** colour global-shutter modules (1920×1200 each)
- 1 × **UC-512** Camarray stereo HAT
- single CSI ribbon into **CAM0**
- HAT USB-C = **5V power input** (not a data link to the Jetson)

Expected once working: **one** `/dev/video0` at **3840×1200** — the two 1920×1200
frames side by side, hardware-synchronised. The AR0234 modules only function
behind the Camarray HAT; they are not standalone CSI cameras.

## Topology: Camarray HAT

Both sensors sit on a Camarray HAT, which reaches the Jetson over a **single**
CSI ribbon. The HAT muxes both sensors and emits **one wide frame** with the two
images side by side, so:

- only **one** CSI port is populated — a failed probe on the empty port is
  normal and cosmetic
- there will be **one** `/dev/video*`, at **double width** (e.g. 2560x720 for a
  pair of 1280x720 sensors)
- the two halves are **hardware-synchronised**, which is strictly better for
  calibration than two independent cameras

Hence `--split` everywhere below.

## Once the camera enumerates

```bash
# 0. if the HAT got power after boot, force a fresh probe
./recam.sh

# 1. confirm the feed works at all — grabs one pair, no board needed
./stereo_capture.py --split --raw-only \
    --cap-width 3840 --cap-height 1200 --width 3840 --height 1200
# then look at ~/stereo/captures/preview_last.jpg

# 2. collect checkerboard pairs (9x6 = a 10x7-square board)
./stereo_capture.py --split --count 25 --pattern 9x6 \
    --cap-width 3840 --cap-height 1200 --width 3840 --height 1200

# 3. calibrate — square-size is your board's square edge in mm
./stereo_calibrate.py --pattern 9x6 --square-size 25.0
```

Confirm the real mode list with `v4l2-ctl -d /dev/video0 --list-formats-ext`
before trusting 3840×1200; the HAT may also offer a downscaled combined mode.

In `--split` mode, `--width` and `--cap-width` are the **combined** (double)
width. If the HAT exposes a plain `/dev/videoN` rather than working through
`nvarguscamerasrc`, add `--backend v4l2 --left 0`.

## Capture tips that decide whether calibration is any good

- 15–25 pairs. Fewer than 10 is not enough.
- Vary the pose hard: board near and far, tilted left/right/up/down, and pushed
  into all four image corners — not just centred and flat.
- The board must be fully visible in **both** views; the script enforces this.
- With the Camarray HAT the halves are hardware-synchronised, so you can keep the
  board moving gently; you are not fighting inter-camera shutter skew.
- Print the board on something rigid. A bent or curled sheet quietly corrupts
  the result while still reporting a plausible error.

## Reading the calibration report

| number | good | meaning |
|---|---|---|
| mono RMS | < 0.5 px | per-lens intrinsics fit |
| epipolar mean | < 0.3 px | the one that predicts depth quality |
| baseline | matches physical lens spacing | wrong ⇒ `--square-size` is wrong |

Then actually open `rect_check_*.jpg`: the same feature must sit on the same
green line in both halves.

## Files

- `cam_diag.sh` — full hardware/driver diagnostic, ends with a verdict
- `recam.sh` — force a fresh probe without rebooting (after late HAT power-up)
- `stereo_capture.py` — headless synchronized pair capture with pose-diversity gating
- `stereo_calibrate.py` — mono + stereo calibration, writes `stereo_calib.yml`

## Two pre-existing defects found on this system

1. `arducam-nvidia-l4t-kernel` is **half-configured** (`dpkg -l` shows `iF`).
   Its postinst hardcoded kernel `5.15.148-tegra` but this box runs
   `5.15.185-tegra`, so it aborted on `mv .../tegra-camera.ko`.
2. As a result the Arducam-patched `tegra-camera.ko` was never installed —
   `/boot/arducam/tegra-camera.ko` and the loaded
   `/lib/modules/5.15.185-tegra/updates/.../tegra-camera.ko` have different
   checksums. The stock module is the one in use.

Neither explains the I2C silence (that read fails inside `arducam_csi2` before
`tegra-camera` is involved), but #2 would likely break capture later.

### #2 DID break capture, and here is the fix (jetson11, 2026-08-19)

On jetson11 the camera enumerated perfectly -- `/dev/video0` at 3840x1200,
`arducam_csi2` refcount 1, `UU` at 0x0c on i2c bus 10, a clean probe in dmesg --
and streaming returned **zero frames**: `v4l2-ctl --stream-mmap --stream-count=1`
sat until killed and wrote 0 bytes. `arducam_csi2` does the sensor and I2C side,
which is why enumeration looked healthy; `tegra-camera.ko` is the VI capture
core, which is why nothing came out.

The postinst failure is visible live during install:

    mv: cannot move '/boot/arducam/tegra-camera.ko' to
        '/lib/modules/5.15.148-tegra/updates/.../tegra-camera.ko': No such file

There is no `set -e`, so dpkg still reports `ii` and the breakage is silent. The
shipped module IS built for the running kernel -- `modinfo /boot/arducam/tegra-camera.ko`
gives `vermagic: 5.15.185-tegra` -- so only the destination path is wrong. Do
what the postinst meant to do:

```bash
DST=/lib/modules/$(uname -r)/updates/drivers/media/platform/tegra/camera/tegra-camera.ko
sudo cp -a "$DST" "$DST.stock-backup"
sudo cp /boot/arducam/tegra-camera.ko "$DST"
sudo depmod -a
sudo reboot
```

After the reboot the same grab returned exactly 9216000 bytes. **Check this on
any new Jetson before blaming wiring, lane count or the HAT**: it presents as
"enumerates but never streams", which invites exactly the wrong diagnosis. A
4-lane/2-lane theory was chased first here and was wrong -- the installed
package was the 2-lane build the whole time.

A second symptom worth knowing, because it also points the wrong way: while the
module was broken, `cv2.VideoCapture` could not open `/dev/video0` at all -- not
by path, not by index, not with `CAP_ANY` -- and failed inside `cap_v4l` with
"can't open camera by index". That reads like a limitation of a generic cv2
build against a 10-bit-Bayer-only device, and it is not. With the patched module
installed, the same OpenCV 4.8.0 opens it and reads a full 9216000-byte frame.
**If OpenCV cannot open this camera, suspect this module before suspecting your
OpenCV.**

Two related notes from the same session:

- **`set_camera_dtb.py` is obsolete.** It aborts looking for
  `/boot/arducam/dts/dtb/tegra234-p3768-merged-camera-spi.dtb`, which current
  Arducam packages do not ship (10 DTBs, none "merged"). It is also unnecessary:
  the package's own extlinux entry already selects
  `Custom Header Config: <CSI Camera ARDUCAM Dual>` and the camera comes up.
- **`cam_diag.sh` used to lie when sudo was unavailable.** Its I2C scan
  discarded stderr, so a scan that could not run reported as "the HAT is not
  answering at all" and sent you to reseat a correctly-wired ribbon. It now
  distinguishes the two.
