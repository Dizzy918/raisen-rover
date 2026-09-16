# AR0234 stereo calibration — jetson9

## Rig

| | |
|---|---|
| Cameras | 2x Arducam AR0234, global shutter |
| HAT | Arducam UC-512 Camarray (**needs its own power supply** — an underpowered HAT was the original throttle) |
| Wire format | one side-by-side frame, **3840x1200 RAW10 GRBG** at `/dev/video0` |
| Per eye, native | 1920x1200 |
| Driver | Arducam Jetvariety `arducam_csi2` — **V4L2 only**, `nvarguscamerasrc` is unsupported |
| Mount | inverted, and the eyes are swapped: rotate each half 180° then swap L/R (verified by disparity sign) |
| Host | Jetson Orin Nano, JetPack 6.2 / L4T R36.5.0, kernel 5.15.185-tegra, `narsil@jetson9.local` |

## Why the existing calibration is invalid

`existing_calibration/acamst_*.yaml` reports RMS 0.157 px and baseline 148.23 mm,
but its header says `profile: 2560x720` and it predates the current mount
geometry. Both the resolution and the extrinsics are wrong for this rig. It is
kept here only as a reference for the file format the rivendell ACAMST twin
expects.

## The demosaic trap (read before recapturing)

A checkerboard is the worst possible subject for a colour-filter-array sensor.
Its hard achromatic edges alias against the GRBG mosaic, and the demosaic
interpolator emits zipper/maze artifacts along exactly the edges a corner
detector measures. Measured on the 19 pairs in `captures/round2` — sharpness
419–1065, so **not** motion blur:

| processing | detected (of 19) |
|---|---|
| full res 1920x1200, demosaiced | 6 |
| **exact 1/2 downscale, 960x600** | **18** |
| 0.75 downscale | 10 |
| full res + Gaussian σ=1.0 | 6 |
| full res + Gaussian σ=2.0 | 9 |

Blurring does not help — the artifact is structured, not white noise. A
non-integer downscale does not help either, because it straddles CFA quads.
An exact 2× box average does, because each 2×2 quad collapses to one true
luminance sample.

`smart_grab6.py` and `smart_grab7.py` therefore **never demosaic**. It bins each 2×2 GRBG quad to
one grey pixel → 960x600 per eye. All four samples are averaged, not just the
two greens: the 4-sample centroid sits exactly at the quad centre, whereas a
green-only average is offset half a pixel diagonally — a fixed geometric bias.

### Calibrating at 960x600 is not a compromise

- `fx, fy, cx, cy` scale linearly to 1920x1200 — multiply by 2
- distortion coefficients are normalized — they carry over unchanged
- baseline and extrinsics are in mm — resolution-independent

A clean detection at half res beats a failed or artifact-biased one at full res.
The capture script also dumps the raw Bayer of the first 2 accepted frames as
`.npz`, so a better demosaic or a two-stage full-res corner refinement can be
explored later without recapturing.

## The contrast trap (the round-3 failure)

`smart_grab6.py` kept 22/22 with zero blur rejections and 8/9 spatial cells, and
the set was still useless:

| | round3 |
|---|---|
| plain detection, both eyes — **what MATLAB does** | **0/22** |
| CLAHE-assisted, both eyes | 21/22 |

MATLAB runs its own corner finder over the pixels you give it and applies no
contrast enhancement, so **plain detection is the only criterion that counts**.
smart_grab6's accept gate used a CLAHE fallback, so it saved 22 pairs MATLAB
will silently drop. `verify_pairs.py` had the same bug and reported "21/22,
good set" — it now gates on plain detection and flags low-contrast pairs
separately.

Cause, from the round3 histograms: `p5–p95` spanned 1–255 while the board sat at
mean 43–97. The window blew to saturation, the foreground crushed to black, the
board was squeezed into a narrow mid band. Two compounding mistakes:

1. **`analogue_gain` was at its minimum** — 100 of 1200, i.e. 1× with 12× of
   headroom unused. The board was underexposed even at exposure 8000.
2. **The stretch used whole-frame percentiles**, so its limits were set by the
   blown window and the black floor rather than by the board.

`smart_grab7.py` fixes both:

- **gain, not exposure.** Exposure is in row-times (~14.57 µs), so matching the
  brightness with exposure means 230 ms+ frames and motion blur. Gain costs
  noise instead, and the 2×2 bin averages 4 samples, halving it back.
- **stretch limits from the board region** of the 16-bit binned frame. The map
  applied is still one global affine ramp, so it is geometrically neutral —
  unlike CLAHE, whose per-tile gains vary across a checker edge and bias
  subpixel corner positions. Restretching round3's already-quantized 8-bit PNGs
  this way lifted plain detection 0/22 → 11/22; at capture time it works from
  the 16-bit raw, with far more range.
- **the accept gate is plain detection of the final 8-bit image, both eyes.**
  What is verified is exactly what is written, so a saved pair cannot fail in
  MATLAB.

No gain setting rescues a board held against a blown-out window. **Keep bright
windows behind the cameras.**

## Result (round 4)

| | round3 (smart_grab6) | round4 (smart_grab7) |
|---|---|---|
| kept | 22 in 247 s | 22 in **86 s** |
| plain detection, both eyes | **0/22** | **22/22** |
| rejected for low contrast | (not gated) | 4 |
| rejected for blur | 0 | 0 |
| spatial cells | 8/9 | 7/9 (no BC, MR) |
| fill range | 12–49% | 13–38% |

A 10-pair top-up (`round4_topup`, 10/10 in 91 s) then filled the missing BC
cell. **`captures/final/` is the merged set to calibrate on: 32 pairs, 32/32
usable, 8/9 cells.**

| | final (round4 + topup) |
|---|---|
| pairs | 32 |
| plain detection, both eyes | **32/32** |
| spatial cells | 8/9 — only MR missing |
| fill range | 13–38% |

Two residual caveats, neither blocking:

- **MR never landed.** TR and BR already constrain the right of the frame, so
  the region is not unobserved, just less densely sampled.
- **Nothing nearer than 38% fill.** Near poses are what separate focal length
  from distance, so if MATLAB reports focal length and the baseline trading off
  against each other (large `fx` uncertainty, or a baseline that drifts when you
  drop a pose), that is the cause. A handful of poses at 45–55% fill would fix
  it, and a run now costs ~90 s.

## Capture

The board must be **rigid and flat** — tape it to foamboard or stiff card. A
sheet held in the hand bows, and bow is indistinguishable from lens distortion,
which quietly inflates the distortion coefficients.

```bash
ssh narsil@jetson9.local 'python3 ~/stereo/smart_grab7.py 2>&1 | tee ~/stereo/sg8_log.txt'
```

`--gain 500` if it still rejects for contrast; `--gain`/`--exposure`/`--goal`/
`--seconds` are all tunable. The run prints a `low-contrast=N` counter — if N
exceeds the number kept, the lighting is wrong, not the poses.

The script stops `narsil-vision` first (it holds `/dev/video0` — only one
consumer can stream at a time) and restarts it on exit, including on Ctrl-C.
To stop early: `ssh narsil@jetson9.local 'touch /tmp/smartgrab.stop'`

While it runs, work through the 3×3 grid it reports. It gates on both-eye
detection, sharpness, and pose novelty, so a pose that does not move is not
double-counted.

- **keep bright windows behind the cameras** — this was what killed round 3
- both eyes must see the **whole** board, all 4 outer edges plus white margin
- push the board into all four **corners** of the frame — that is where
  distortion lives; a centre-clustered set gives a flattering RMS and a bad
  undistort
- **tilt 20–40°** in pitch and yaw. Frontoparallel-only poses leave focal
  length and depth degenerate
- vary distance: some poses filling <25% of frame width, some >45%
- hold ~1 s per pose

Then pull and check:

```bash
rsync -a narsil@jetson9.local:stereo/matlab_pairs/ captures/round4/
.venv/bin/python stereo_calibration/verify_pairs.py captures/round4
```

`verify_pairs.py` re-detects with **plain** corner finding — the same criterion
MATLAB applies — and reports per-pair status plus spatial and scale coverage.
Do not skip it: MATLAB **silently drops** any pair it cannot detect in both eyes,
so it will happily calibrate on 6 of 20 pairs and still report a good-looking
RMS. Pairs that fail plain but pass CLAHE are flagged as low-contrast: that is
an exposure problem to fix at capture, not a pose to retake.

## Target

`target/checkerboard_9x6_25mm.pdf` — 9×6 **inner corners**, 25 mm squares,
250×175 mm on A4 **landscape**. Inner corners are the interior vertices where
four squares meet; the board is 10×7 squares. The counts are deliberately
asymmetric (odd × even) so board orientation is unambiguous.

Print at **100% / "actual size"**, fit-to-page **off**. Then measure the 100 mm
reference line with a ruler. If it is not exactly 100 mm, the printer scaled the
page and every extrinsic distance — the baseline included — is wrong by that
same ratio. Regenerate other sizes with `target/make_checkerboard.py --help`.

If you reuse the board already printed for rounds 1–2, **measure one square with
a ruler** and give MATLAB that number. The detector does not care about square
size, but every millimetre of the baseline scales with it.

**MEASURED 2026-08-12: the board in use has 25.6 mm squares**, not 25 — its
printer scaled to ~102.4%. Give MATLAB **25.6**. Using 25 would shrink every
extrinsic distance by 2.4%, i.e. ~3.5 mm on a ~148 mm baseline, and depth would
be biased by the same 2.4% at every range.

## Calibration result (OpenCV, 2026-08-12)

Run locally, no MATLAB needed — and this is the *native* route, not a
workaround: the existing `acamst_*.yaml` header states its own provenance as
"K, D from cv2.stereoCalibrateExtended; R, P from cv2.stereoRectify
(CALIB_ZERO_DISPARITY, alpha=0)". OpenCV is how these files have always been
produced, so there is no 1-indexed principal point and no transposed K or R to
reconcile.

```bash
.venv/bin/python stereo_calibration/calibrate_stereo.py \
    stereo_calibration/captures/final --square-mm 25.6 \
    --exclude img_16.png,img_31.png
```

| | value | check |
|---|---|---|
| pairs used | 29 of 32 | img_12 failed strict detection; img_16/img_31 pruned as outliers |
| stereo RMS | **0.29667 px** | under the 0.3 target |
| mono RMS | 0.2504 / 0.2900 px | left / right |
| **baseline** | **148.9143 mm** | previous calibration measured **148.2304** — agrees to **0.5%** |
| relative rotation | −0.235°, +0.154°, −0.047° | small, as a rigid factory HAT should be |
| rectified row alignment | mean 0.108 px, p95 0.343 px | well under 1 px |
| fx, fy @ 960x600 | 568.82, 569.09 | |
| cx, cy @ 960x600 | 480.70, 294.49 | ideal centre is 480.0, 300.0 |

**The baseline agreement is the strongest validation available here.** The mount
changed but the two sensors' spacing on the HAT did not, and an independent
calibration from new images recovered it to within 0.75 mm. That simultaneously
confirms the 25.6 mm square size, the 2x2 binning geometry, and the left/right
swap — any one of those being wrong would move the baseline by far more.

Dropping 2 pairs moved the baseline by only **0.06 mm**, so the fit is
well-conditioned in spite of the missing near poses. The earlier worry about
focal length trading against distance did not materialise.

### Output

```
out/960x600/     the native measurement, at the calibrated resolution
out/1920x1200/   scaled for the ar0234_fullres pipeline profile
```

The scale is **not** a plain doubling of the principal point. A binned pixel
index `i` covers full-res indices `2i` and `2i+1`, whose centres are `2i` and
`2i+1`, so the quad centroid sits at full-res coordinate `2i + 0.5`:

```
f_full = 2 * f_binned
c_full = 2 * c_binned + 0.5      <- the half pixel a naive x2 drops
```

D is normalized by focal length and carries over unchanged; rectification `R` is
a pure rotation and is resolution-independent.

### Remaining step

Upload `out/1920x1200/acamst_*.yaml` to the rivendell **ACAMST** twin — these
files are provisioned per-unit from there, not committed to the narsil repo.
`camera_name` is already set to `ar0234_left_3840x1200` /
`ar0234_right_3840x1200` to match the `ar0234_fullres` profile; the camera_info
manager rejects a name mismatch.

## MATLAB (optional cross-check)

1. **Stereo Camera Calibrator** (Image Processing and Computer Vision toolbox)
2. **Add Images** → left folder `captures/final/left`, right `captures/final/right`
   — filenames match pairwise, which is how MATLAB pairs them
3. Square size **25 mm** (or your measured value)
4. Radial distortion **2 coefficients**; add the 3rd only if the reprojection
   error stays structured. Skew and tangential off unless the residuals show it
5. **Calibrate**, then read the reprojection-error bar chart. Delete individual
   outlier pairs above ~1 px and recalibrate — one bad pose drags the whole fit
6. Sanity checks before exporting:
   - mean reprojection error **< 0.3 px**
   - baseline ≈ the physical eye spacing you can measure on the HAT
   - Show Rectified: rows must line up; a point on a horizontal line in the left
     image sits on that same line in the right
7. **Export Camera Parameters** → `stereoParams` → save as
   `matlab_out/stereoParams_final.mat`

## Converting to narsil YAML

`stereoParams` → three files matching `existing_calibration/` shape:

- `acamst_left.yaml`, `acamst_right.yaml` — `camera_matrix`, `distortion_coefficients`,
  `rectification_matrix`, `projection_matrix`, plus `camera_name` and `image_width/height`
- `acamst_extrinsics.yaml` — `rotation` (3×3) and `translation`, left→right

Two conversions are mandatory and easy to get wrong:

1. **MATLAB is 1-indexed, OpenCV/ROS is 0-indexed.** Subtract 1 from the
   principal point: `cx_ros = cx_matlab - 1`, likewise `cy`.
2. **MATLAB stores intrinsics and rotations transposed** relative to OpenCV.
   Transpose `K` and `R` on the way out.

Scale `fx, fy, cx, cy` by 2 if the pipeline consumes 1920x1200 while
calibration ran at 960x600. Set `camera_name` to match `camera_csi.yaml`
(`ar0234_left_3840x1200` / `ar0234_right_3840x1200`) — the camera_info manager
rejects a mismatched name.

Then upload to the rivendell **ACAMST** twin; the per-unit files are provisioned
from there, not committed to the narsil repo.

## Layout

```
target/          checkerboard PDF/SVG + generator
captures/
  round1/        19 pairs, 2026-08-11 — superseded
  round2/        19 pairs, 2026-08-11 — 5/19 usable, see demosaic trap
  round3/        22 pairs, 2026-08-12 — 0/22 usable, see contrast trap
                 (+ raw_01/02.npz, raw Bayer for offline experiments)
  round4/        22 pairs, 2026-08-12 — 22/22 usable. gain 300, board-region
                 stretch, plain-detection gate. 7/9 cells (no BC, MR).
  round4_topup/  10 pairs, 2026-08-12 — 10/10 usable, added the BC cell.
  final/         round4 + round4_topup merged and renumbered img_01..img_32.
                 32/32 usable, 8/9 cells. >>> CALIBRATE ON THIS ONE <<<
  calib3840_old/ 2 early poses + raw .npz
existing_calibration/  acamst_*.yaml from the vision container (2560x720, invalid)
verify_pairs.py  full-res detection + coverage check
```

## Open

- second rover needs its own full calibration run — intrinsics are per-unit
- mount-tilt (camera-to-ground) transform, needed to project depth into the map
- wheel radius + track width for differential-drive kinematics
