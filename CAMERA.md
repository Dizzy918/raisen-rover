# The stereo camera: everything, and how to use it

Everything measured about the AR0234 stereo pair on jetson9, in one place, with
the commands to reproduce it. Written 2026-08-18.

If you read one section, read **Getting a frame** and **Five traps**. Those are
where the time went.

---

## The hardware, and what it actually delivers

- **Arducam AR0234**, 2.3 MP, **global shutter**, colour sensor, M12 manual-focus
  lenses, focus range about 2 m to infinity.
- **UC-512 Camarray HAT** hardware-syncs the two sensors and combines them into
  **one** MIPI CSI stream. The Jetson sees a single camera.
- Single CSI ribbon into **CAM0**, which is **i2c bus 10**. Working looks like
  `0x0c` on bus 10 (`UU` once a driver has bound). This said "cam1" until
  2026-08-19 and that was wrong: `jetson/stereo/README.md` records the ribbon
  in CAM0, HANDOFF records bus 10 on a working jetson9, and `cam_diag.sh` maps
  bus 10 to CAM0. Repeated on jetson11, whose HAT answers on bus 10 too. The
  wrong note cost time moving a correctly-wired cable.
- Driver is Arducam Jetvariety **`arducam_csi2`**, and it is **V4L2 only**.
  `nvarguscamerasrc` does not work with it.

What arrives at `/dev/video0` is **one 3840x1200 RAW10 GRBG frame** with the two
eyes side by side, 1920x1200 each. Not two devices, not two streams.

**The mount is inverted AND the eyes are swapped.** Each half needs rotating 180
degrees, and the physically-left camera lands in the **right** half of the frame.
Get the swap backwards and every disparity comes out negative, which reads as
"stereo is broken" rather than "the eyes are the wrong way round".

We **bin 2x2** rather than demosaic, giving **960x600 per eye**. Averaging each
Bayer quad is a real luminance sample; demosaicing would interpolate colour we do
not need, cost more, and invent detail. cuVSLAM takes mono8 only anyway.

---

## Getting a frame

```python
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
frame16 = buf.reshape(-1).view(np.uint16).reshape(1200, 3840)
```

Miss any of those four settings and OpenCV hands back a debayered 8-bit image of
the wrong size. `raisen/camera.py` and `raisen/frame.py` do this once so nothing
else has to.

**Only one process can hold the camera.** On this machine `narsil-vision` holds it
by default, so anything else must stop that container first and put it back
afterwards. `raisen/service.py` has the context manager.

---

## Five traps, each of which cost real time

**1. `narsil-vision` comes straight back.** It runs with `restart=on-failure` and
unlimited retries, and `docker stop` exits 137, which counts as a failure. Stop it
without disabling the policy first and it has the camera again within a second.
Disable, stop, and restore on the way out.

**2. Never `exec` in a wrapper that has a trap.** `exec` replaces the shell and
destroys the `EXIT` trap, so `narsil-vision` gets left stopped with `restart=no`
and the camera never comes back. Learned by doing exactly that.

**3. The wake transient is real.** For a short window after another process
releases the device, reads succeed and return near-black frames. A cloud test once
reported **0 valid depth pixels** this way while raw stats proved the sensor was
fine. Wait for a frame with actual signal; `camera.read_awake()` does.

**4. Exposure is a frame-rate ceiling.** It is in sensor row-times, and the fitted
frame period is about `exposure * 21.9 us`:

| exposure | fps |
|---|---|
| 1500 | 30.6 |
| 2290 | 20.1 |
| 4000 | 11.5 |
| 8000 | 5.8 |

Longer exposure is also *worse* for stereo, because it blows out bright regions.
**1500 is both the fastest and the best.** Buy brightness with gain.

**5. `analogue_gain` lies above ~1100.** It advertises max 1200. Above roughly
1100 the write is accepted, `v4l2-ctl` reads the new value back, and the image
does not get any brighter. So the advertised maximum is not the usable maximum,
and a controller that trusts it sits at the top of its range believing it has
authority it does not have. `GAIN_USABLE_MAX = 1100`.

---

## Looking at it live

```bash
./jetson/stereo/run_live_feed.sh
```

Then open `http://jetson9.local:8080/` from the Mac. MJPEG, about 30.7 fps, one
reader thread with per-client JPEG encode. Modes as buttons or `?mode=`:

- `pair` both eyes, grey
- `left` left eye only
- `clahe` left eye with local contrast, best for judging texture
- `color` demosaiced with grey-world white balance, for eyeballing only

`--auto` runs auto-exposure while streaming.

---

## Auto-exposure

```bash
python3 jetson/stereo/auto_exposure.py --start 20 --start-gain 100 --frames 180
```

Meters the **raw** frame, before `bin2x2`'s percentile stretch hides what the
sensor saw. Targets a median of 300 counts out of 1023. Converges in 1-8 control
writes; an early version limit-cycled at 38 writes per 150 frames.

Three fixes that mattered, in case it is ever retuned:

- The limit cycle was not a damping problem, it was **two contradictory demands** --
  raise the median, and stop clipping highlights. Fixed with a hard saturation EV
  ceiling plus hysteresis, not more damping.
- The deadband must be on the **measured error**, not the damped step. On the step
  it settled 30% off target and called that converged.
- A black-frame transient made it report itself **settled on a black image**. Hence
  `SLEW` and `BLACK_MED`.

In the merged pipeline AE runs on the capture thread, so vSLAM and mapping get the
metered frame too. Both previously hardcoded `exposure=1500 gain=800`, because the
only AE implementation lived in the preview.

---

## Calibration

Stereo calibration lives in `stereo_calibration/`; its README has the full
process, including why 32 pairs and how the coverage was checked. Results:

| | |
|---|---|
| stereo RMS | **0.29667 px** |
| mono RMS | 0.2504 / 0.2900 px (left / right) |
| **baseline** | **148.9143 mm** |
| rectified row alignment | mean 0.108 px, p95 0.343 px |
| `fx, fy` at 960x600 (raw K) | 568.82, 569.09 |
| **`fx_rect`** (from P) | **585.04** |
| **`fx_rect * baseline`** | **87.120** |

The strongest validation available is that the baseline agrees to **0.5%** with an
earlier independent calibration (148.2304 mm). The mount is rigid, so the true
baseline cannot have changed -- and any error in the assumed square size, the 2x2
binning geometry, or the left/right swap would have moved it far more.

**Measure your checkerboard square with a ruler.** Every extrinsic distance,
baseline included, scales linearly with it. A printer that scales the page 2.4%
moves the baseline 3.5 mm.

Use `fx_rect` for depth, not the raw `fx`. They differ because rectification
changes the effective focal length, and mixing them up is a quiet few-percent
error.

### Checking rectification

```bash
python3 stereo_calibration/check_rectification.py
```

Known-good baseline to compare against:

| | median dy | MAD | within 2 px | disparity |
|---|---|---|---|---|
| unrectified | -6.000 | 0.000 | 1% | all positive |
| rectified | +0.000 | 1.000 | 93% | all positive |

**The unrectified row is the point.** It shows the sensors sit a systematic 6 px
apart vertically and rectification takes that to zero. Without that control, a
rectified median of 0.0 could just mean the test is insensitive. Failure looks
like a rectified median of several px, or **any negative disparity**.

---

## Where the camera sits on the rover

`stereo_calibration/out/camera_to_base.yaml`:

| | |
|---|---|
| height | **0.1687 m** |
| pitch (nose up) | **+2.72 deg** |
| roll | **+0.62 deg** |
| uncertainty | 0.6 deg |

Confirmed **three independent ways**, which is the only reason to trust it:

1. RANSAC plane fit on near-field stereo: 16.87 cm, +2.49 deg, 85% inliers, 2.0 mm rms
2. `solvePnP` on a checkerboard flat on the floor: +2.95 deg
3. The point cloud's floor landing at **+12.6 mm** of z=0 across 2.5 m

Tape said 16.5 cm.

**Build the transform from the floor normal, not from Euler angles.** The YAML
carries both. Composing a rotation from pitch and roll means choosing a sign
convention for each, and a sign error there produces a transform that looks
entirely plausible and is wrong. The normal is a measured direction with no
convention to get wrong. `raisen/calib.py: CameraToBase` does it.

Two traps found while measuring this:

- **An unconstrained plane fit returned a cardboard box front as "floor"** -- 1.4 mm
  rms, pitch -87 deg. A tight rms proves points are coplanar, not that they are the
  floor. Constrain candidate normals to within 25 deg of vertical.
- **Every sub-block of a checkerboard is a valid checkerboard.** A search "found" a
  (7,6) grid at 0.35 px on a board that is really (9,6) at 0.06 px. Low reprojection
  error does not confirm the grid. Orientation survived that; scale did not.

This floor produces **several horizontal planes at once**: at 5 mm tolerance
"largest plane" flipped frame to frame between the real floor and something below
it. The fix is to enumerate candidate planes and take the **highest** plausible
one, since the spurious ones are always below the floor.

An earlier version of this document blamed polished concrete acting as a mirror.
**The floor is not polished** -- there is bright light falling on it -- so that
mechanism is wrong and the cause is open. `vslam/subfloor_check.py` distinguishes
the two candidates: coherent reflected imagery would cluster in z and not care
about pixel brightness, whereas glare-driven false matching would scatter in z and
draw disproportionately from bright pixels.

**Still owed:** the camera's fore/aft and lateral offset from the rover's actual
origin. `base_link` sits under the *camera*, not the wheel axis, so a pure pivot
shows a small translation -- measured at **78 mm**. A ruler from the wheel axis
would finish it.

---

## Depth

```
depth = fx_rect * baseline / disparity  =  87.120 / disparity   metres
```

Validated against a checkerboard at measured distances:

| range | error |
|---|---|
| 1 m | **+0.3 mm** |
| 2 m | **+3.0 mm** |
| 4 m | degrades badly |

### The search-range trap, twice

A block matcher cannot report a disparity above `numDisparities - 1`, so anything
nearer than `87.120 / numDisparities` is **clamped** and comes back as a
plausible-looking depth that is really the edge of the search window.

| numDisparities | nearest depth it can express |
|---|---|
| 128 | 0.681 m |
| 176 | 0.495 m |
| 224 | 0.389 m |
| 256 | 0.340 m |

It bit twice. At 128, an ArUco marker at **0.43 m reported 0.749 m** -- not
obviously wrong, which is why it survived. At 176, `depth_grid.py` trusted a
0.40-0.495 m band it structurally could not measure, and that band is exactly the
one that matters for not hitting things.

Choosing a bigger constant is not a fix, it is the same bug with a different
number. `raisen/depth.make_matcher(fxb, z_min)` derives the range from the nearest
depth you intend to trust and **refuses** to return a matcher that cannot reach it.

Check it empirically on a real frame, no ground truth needed:

```bash
python3 stereo_calibration/range_sweep.py
```

It computes one frame at several ranges and looks for pixels **railed** against
the top of the window, plus depth agreement against a much wider range. If nothing
in the scene is near enough to reach a given range's floor, it says so rather than
claiming a pass it did not earn.

### Subpixel resolution is coarse

59% of disparities come back as exact integers, with only 8 distinct 1/16 steps
across a board. One whole disparity pixel is **42 mm at 2 m** and **168 mm at
4 m**. Also, periodic black-and-white patterns bias a block matcher about 0.9 px
**near** -- so a checkerboard is not a neutral depth target.

---

## Running cuVSLAM

Isaac ROS **cuVSLAM 3.2.6**, ROS 2 Humble, in `vslam-min:3.2`.

```bash
./vslam/run_live_vslam.sh 60          # camera -> cuVSLAM, logs pose and drift
./imu/run_fusion.sh 60 run.csv        # the above, plus ESP32 heading fusion
./pipeline/run_pipeline.sh 60         # merged: one camera, vSLAM + grid + AE
```

Measured:

| | |
|---|---|
| publish rate | 31.0-31.9 fps |
| publisher cost | 20.8-21.0 ms/frame |
| stationary drift, 40 s | 1.0 cm, -0.33 deg |
| stationary rotation, 72.5 s | **0.00 deg** |
| rotation accuracy | **+1.9%** on a 45 deg pivot |

At rest vSLAM is a *better* rotation reference than the gyro (0.00 deg vs 5.34 deg
over 72.5 s). Translation scale has **never** been measured -- see the open list.

### Four things that will bite

- **mono8 only.** Hand it bgr8 and it throws `Received unknown image encoding: bgr8`.
- **Rectified `camera_info`, not raw.** `K = P[:3,:3]`, `D = 0`, `R = I`, and
  `rectified_images:=true`. Passing rectified pixels with the unrectified K makes it
  warn *"Falls back to raw camera model"* and quietly produce wrong poses. A warning,
  not an error, so it is easy to miss.
- **A missing topic hangs it silently.** It blocks forever waiting on `image_0`
  rather than erroring. If nothing appears, suspect the publisher, not cuVSLAM.
- **`array.array('B', ...)`, never `img.tobytes()`.** Assigning raw bytes to a ROS
  `uint8[]` costs **96 ms** per 960x600 image in rclpy; two of those was 193 ms of a
  210 ms budget and pinned the publisher at 4.5 fps. `array.array` matches the
  field's internal type and assigns in ~0 ms. One line, 7x speedup.

Also: the `isaac_ros_gxf` env hook leaves the GXF plugin dirs off the loader path,
so the node dies with `libgxf_serialization.so: cannot open shared object file`.
Every runner prepends them to `LD_LIBRARY_PATH`.

cuVSLAM reads the **rig geometry from TF**, not from `camera_info`. Published as a
chain `base_link -> left_cam -> right_cam`, with the measured height and tilt on the
first link and the baseline as a trivial +X on the second. `--legacy-tf` restores the
old identity transform where `base_link` *was* the optical frame and forward was +Z --
kept only to reproduce runs recorded before the extrinsics were measured. With the
measured TF, odometry is a real rover frame: forward +X, up +Z, yaw about Z.

---

## Seeing the 3D structure

```bash
python3 vslam/cloud_check.py           # one frame -> PLY + three orthographic views
```

From a single viewpoint, stationary:

- 139319 valid depth points, **24.2%** of the 960x600 pixels
- floor within 6 cm of z=0 and nearer than 2.5 m: mean z **+12.6 mm**, plane rms
  15.5 mm, residual tilt 1.29 deg
- **19.7% of the cloud is physically impossible** -- more than 10 cm *below* the
  floor, plus far-field noise. Cause unresolved -- see `vslam/subfloor_check.py`;
  it is NOT the polished floor an earlier draft blamed, because the floor is not
  polished. Anything building a map on this must reject sub-floor points either
  way; `depth_grid.py` does.

Z spread per range band makes the quadratic depth error visible: **0.62 m** in the
nearest band against **5.71 m** beyond 4 m. Points smear along the viewing ray,
which is why a top-down render shows radial streaks rather than surfaces. Any
accumulation wants a range gate; the grid uses 0.4 to 5.0 m.

A single-viewpoint cloud is **not a map** -- it is one depth image in 3D, with
everything behind the first surface missing. Accumulating a room needs the rover to
move and needs pose to stitch the views.

---

## If the camera disappears

This happened once and took a while. Full account in
`jetson/camera_device_tree.txt`. Symptom: `/dev/video0` gone after a reboot.

Chain of causes: a broken Arducam package's postinst overwrote the boot config,
then `config-by-hardware.py` aborted on *"Multiple DTBs found"* in `/boot/dtb/pio/`.
Fixed, and verified across two reboots.

Three commands that diagnose it, in order:

```bash
ls -l /dev/video0
dmesg | grep -i arducam
python3 /opt/nvidia/jetson-io/config-by-hardware.py -l
```

Two wrong theories were chased first -- an `extlinux.conf` FDT change (disproved:
the live file was byte-identical to its backup) and a missing overlay `.dtbo`
(disproved: the listing had simply been truncated by `head`). Check that a file is
actually absent before concluding it is.

---

## Verifying without hardware

```bash
PYTHONPATH=.:mapping python3 tests/test_offline.py
```

31 checks against real captured frames. Pins fx/baseline/`fx*b`, the
camera-to-rover rotation, the two search-range bugs as arithmetic, and asserts the
merged pipeline reproduces `depth_grid.py` number for number. Run it after touching
anything in `raisen/`.

---

## File map

| | |
|---|---|
| `raisen/` | shared core: sensor format, calibration, depth, camera ownership, frame hub, v4l2 controls |
| `pipeline/run_pipeline.py` | merged run: one camera, vSLAM + grid + AE + preview |
| `pipeline/deploy.sh` | md5-verified copy to jetson9 |
| `jetson/stereo/live_feed.py` | MJPEG preview on :8080 |
| `jetson/stereo/auto_exposure.py` | the AE controller and its CLI |
| `stereo_calibration/` | calibration process, `check_rectification.py`, `range_sweep.py`, `measure_floor.py`, `measure_target.py` |
| `vslam/live_vslam.py` | camera -> rectified mono8 + camera_info + TF |
| `vslam/cloud_check.py` | single-frame 3D structure + PLY |
| `vslam/verify_tf.py` | transforms the floor into `base_link` and checks it |
| `vslam/verify_live_pipeline.py` | acceptance criteria for the live path |
| `mapping/depth_grid.py` | occupancy grid, log-odds ray cast |
| `imu/fuse_heading.py` | complementary filter, vSLAM rate + IMU heading |

---

## Open, and honest about it

- **vSLAM translation scale has never been measured.** Rotation is +1.9%;
  displacement has not been checked against a tape or anything else. Blocked on
  clear floor and knowing which sonar faces forward. This is the biggest gap.
- **About 1% absolute scale uncertainty** in the stereo calibration is accepted
  unresolved, by decision.
- **Whatever puts points below the floor makes plane fits scene-dependent.** The
  mechanism is unresolved; run `vslam/subfloor_check.py`. A clean extrinsics re-run
  wants a plain wall about 1.5 m away, a sheet on the floor for texture, and a
  static near field.
- **Far-field depth smears** beyond 4 m, so accumulation needs a range gate.
- The merged pipeline's live path, its ROS publisher, and `raisen/controls.py`'s
  ioctl fallback are **written but not yet run on hardware**. `live_vslam.py` is
  unchanged and remains the known-good fallback.
- `apt` on jetson9 is still broken by a half-configured Arducam package.
