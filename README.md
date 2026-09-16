# Raisen — jetson9 stereo camera + indoor mapping

Local mirror of everything used on the Jetson (`narsil@jetson9.local`), so that
work is not stranded inside a container that gets recreated.

```
stereo_calibration/     stereo calibration — START HERE (README.md)
  target/               checkerboard PDF/SVG + generator
  captures/             round1, round2, calib3840_old
  existing_calibration/ acamst_*.yaml pulled from the vision container
  verify_pairs.py       full-res detection + coverage check
jetson/
  stereo/               mirror of ~/stereo on jetson9 — capture scripts + logs
  container_files/      narsil-vision config: upstream/, patched/, .patch, apply script
test_camera.py          Mac-side AVFoundation probe (stale: assumes 2560x720)

raisen/                 SHARED CORE — the single source of truth for the parts
                        every component needs: sensor format, both calibrations,
                        depth, camera ownership, the frame hub, v4l2 controls
pipeline/               the merged run: one camera, many consumers
  run_pipeline.py       vSLAM publish + occupancy grid + AE + preview, one process
  run_pipeline.sh       full stack on the Jetson (FUSE=1 adds heading fusion)
  deploy.sh             md5-verified copy to jetson9 — see below for why
  README.txt            what the merge settled, and what is still unverified
tests/test_offline.py   24 checks, no hardware, against real captured frames
vslam/                  cuVSLAM publisher, extrinsics, verification, cloud check
imu/                    heading fusion + the gyro-gate simulation harness
mapping/                occupancy grid (depth_grid.py holds the log-odds update)
rover/                  bounded drive tests, gyro-feedback turns
aruco/                  marker generation + measurements
```

## Verify without hardware

`tests/test_offline.py` runs 24 checks against real captured frames -- no Jetson,
no camera. It is the fastest way to tell whether a change broke the geometry, the
exposure constants or the occupancy grid.

```bash
python3 -m venv .venv && .venv/bin/pip install numpy opencv-python pyyaml
PYTHONPATH=.:mapping .venv/bin/python tests/test_offline.py
```

It ends in `ALL CHECKS PASSED`, or names the first check that failed.

## The camera admits one consumer

`/dev/video0` can be held by exactly one process, and for a long time every
component opened it itself — so vSLAM, the occupancy grid and the live preview
were **mutually exclusive**. Each was proven alone; none had run beside another.
`pipeline/run_pipeline.py` owns the camera once and fans frames out to all of
them. See `pipeline/README.txt`.

## Deploy with the script, not by hand

`pipeline/deploy.sh` md5s both ends and refuses to report success on a mismatch.
It exists because "committed" and "running on the Jetson" came apart twice:
`live_vslam.py` was patched locally and never uploaded, and the `fuse_heading.py`
gyro gate was committed and never uploaded — so that machine ran the **ungated**
filter for days while the repo said otherwise. Grepping the remote file for
`GYRO_GATE` and getting zero is how it was found.

## Start here

- **[CAMERA.md](CAMERA.md)** -- the camera end to end: hardware, how to get a
  frame, exposure, calibration, extrinsics, depth, cuVSLAM, the five traps, and
  what to do when `/dev/video0` disappears.
- **[INTEGRATION.txt](INTEGRATION.txt)** -- how to move the car and read the
  sonars, the units seam with the `algorythm` branch, and what still conflicts.
- **[pipeline/README.txt](pipeline/README.txt)** -- what the merged one-camera
  pipeline settled, and what is still unverified.

## Camera facts

The UC-512 Camarray HAT delivers **one side-by-side 3840x1200 RAW10 GRBG frame**
at `/dev/video0` — 1920x1200 per eye. Driver is Arducam Jetvariety
`arducam_csi2`, **V4L2 only**; `nvarguscamerasrc` does not work. The mount is
inverted and the eyes are swapped: rotate each half 180° then swap L/R.

`narsil-vision` holds `/dev/video0`. Only one consumer can stream at a time, so
any capture script must stop it first and restart it after.

## Three fixes that are NOT in the narsil source repo

**STATUS 2026-08-12: all three are applied and verified live**, in both the
running container and the `~/narsil-dev` source tree on jetson9 (`.orig` backups
alongside each). They had reverted silently once already when the container was
recreated at 2026-08-11 17:24.

Verified after applying:
- `ar0234_right_debayer` running — "gpu debayer 2.10 ms/frame avg" in the vision
  log. `/right/image_raw` is published for the first time.
- "Streaming raw 29.9 fps" (was 13.7). Note the scene at the time held exposure
  at 250–730 row-times, well under both the old 5000 cap and the new 2290, so
  bright light does not actually exercise the cap — it matters in dim scenes.

`~/narsil-dev` is **not a git checkout**, so those edits are not a commit and
will not reach other units. It was byte-identical to the container's copy of
`_common.py` and differed from `camera_csi.yaml` by one line (a replay-profile
filename, preserved), so it is current rather than stale.

| file | change | why |
|---|---|---|
| `launch/_common.py` | second debayer node → `/right/image_raw` | upstream debayers the left eye only (the detector is monocular). The capture node already splits and publishes `ar0234/right/image_raw` as raw bayer16, but nothing converted it, so `/right/image_raw` did not exist and no stereo consumer could subscribe. |
| `config/ar0234.yaml` | `auto_exposure_max: 2290` | the AE upper clamp doubles as a frame-rate ceiling. Exposure is in sensor row-times (~14.57 µs at 3840x1200), so `max=5000` pins the longest frame at 72.9 ms → 13.7 fps and it never recovers. 2290 row-times = 33.4 ms, the longest exposure fitting a 30 fps period. **Measured 13.7 → 29.9 fps.** |
| `config/camera_csi.yaml` | `ar0234_fullres` profile, 3840x1200 | the stock `ar0234` profile claims 2560x720, a downscale. Calibration and depth must use native geometry or intrinsics are scaled against a resolution the sensor never emits. |

`jetson/container_files/` holds `upstream/` (pristine, as pulled), `patched/`,
unified `.patch` files, and:

```bash
jetson/container_files/apply_to_container.sh --check
```

`--check` reports presence and changes nothing; without it, the script backs up
each file to `.orig` in-container, pushes the patched copy, and restarts vision.
This is a stopgap — the durable fix is landing all three in the narsil source
repo, where a rebuild cannot lose them.

## Do NOT switch to ar0234_fullres yet

`CAMERA_PROFILE=ar0234` (1280x720/eye) is what the container currently runs.

**Correction, 2026-08-14:** earlier notes here said "GR3D 65–87%", which came from
a 2-sample glance that caught bursts. Measured over 118 samples the detector
averages **17.2%** with a **median of 3%** and p90 of 58% — very bursty, so short
windows mislead. Adding cuVSLAM takes the mean to 21.0%, i.e. **+3.8 points**.

So the GPU is *not* the reason to avoid `ar0234_fullres` — that argument rested on
the bad number. The real reason stands on its own: **the mapping pipeline does not
need full res.** Calibration was done at 960x600, which is a sensible SGBM
resolution anyway, and SGBM cost scales with pixel count.

**The mapping pipeline does not need the full-res profile.** Calibration was done
at 960x600, which is a sensible SGBM resolution anyway. Feed SGBM 2x-downscaled
rectified images with `stereo_calibration/out/960x600/`, and leave the detector
on its existing profile. The 960x600 intrinsics are valid for a 2x-downscaled
debayered image as well as for a 2x2 raw bin — both map full-res coordinate
`2i + 0.5` to binned index `i`, and SGBM does not need the corner-localization
precision that made binning matter for calibration.

## Two more stability fixes (already in the capture scripts)

- **One persistent `cv2.VideoCapture`.** Repeated `v4l2-ctl` open/close storms
  `corr_err` and wedges the Tegra VI channel.
- **Set exposure mid-stream.** Setting it before streaming starts is ignored.

If `arducam_read: 0x103 failed` appears, reseat the CSI ribbon and
`sudo rmmod arducam_csi2 && sudo modprobe arducam_csi2`.

## Capture scripts (`jetson/stereo/`)

`smart_grab7.py` is current. Two fixes matter, both measured:

- **2×2 Bayer binning instead of demosaicing** (from smart_grab6) — a
  checkerboard's hard achromatic edges alias against the GRBG mosaic and the
  interpolator lays zipper artifacts along exactly the edges the corner detector
  measures. Full res 6/19, exact ½ downscale 18/19.
- **gain + board-region contrast, and a plain-detection accept gate** (new) —
  smart_grab6 gated on CLAHE-assisted detection and so saved 22 pairs of which
  **0** detect the way MATLAB detects. `analogue_gain` was also sitting at its
  minimum, 100 of 1200.

See [stereo_calibration/README.md](stereo_calibration/README.md) for both
measurement tables. `smart_grab.py` … `smart_grab6.py` are kept because their
logs are the evidence trail.

## Indoor mapping, 2 rovers

Planned approach:

- **Pose** — stereo VO, or VI SLAM with ORB-SLAM3 **if** the IMU is wired to the
  Jetson at 100–200 Hz
- **Depth** — SGBM on the rectified pairs
- **Map** — occupancy grid, Bayesian log-odds updates
- **Exploration** — frontier-based, Voronoi partitioning to split the room
- **Comms** — ROS 2 / FastDDS, one domain ID per rover; each publishes its local
  map, a shared merger node fuses with Umeyama/Horn + RANSAC on loop closures;
  DBoW2 vocabulary for cross-rover place recognition

### Blocking question: how is the IMU wired?

The choice of algorithm depends entirely on this, so identify the chip
(MPU6050 / MPU9250 / BMI088 / ICM-20948 / QMI8658) and its bus:

- **Jetson-connected (I2C/USB)** → VI SLAM is viable, use ORB-SLAM3 VI
- **ESP32-mediated over WebSocket** (`{"T":1,"L":...,"R":...}` at 20–30 Hz, loose
  timestamps) → not usable for VI; the timestamps are too soft for tight
  IMU-camera coupling. Use it only for PID wheel feedback, and run stereo VO
  for pose

## Next

1. Print `stereo_calibration/target/checkerboard_9x6_25mm.pdf` at 100%, verify
   the 100 mm line with a ruler, mount it rigid and flat
2. Run `smart_grab7.py` **facing away from the window**, pull to
   `captures/round4/`, check with `verify_pairs.py`
3. MATLAB Stereo Camera Calibrator → new `acamst_*.yaml` → rivendell ACAMST twin
4. Land the three container fixes in the narsil source repo
5. Identify the IMU chip and wiring
6. Second rover: its own calibration run
