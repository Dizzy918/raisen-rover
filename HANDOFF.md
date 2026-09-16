# Handoff: stereo camera, vSLAM, rover motion

Everything a fresh session needs. Written 2026-08-19 at the end of a long
session. Read "Hardware state right now" first -- one thing is broken and it
blocks a lot.

---

## Where things live

| | |
|---|---|
| repo | `github.com/milar111/RaisenInternship` |
| branch | **`feature/jetson-stereo`** -- all of this work |
| local clone | `~/Programming/RaisenInternship` |
| working mirror | `~/Programming/Raisen` -- **not a git repo**; rsync it into the clone |
| other branches | `algorythm` (two-rover mission, simulated), `feature/esp-sonar` (sonar firmware + motion scripts), `udpSocket` (C++ transport), `main`/`development` (LICENSE only) |

Deploy to the mirror, then:

```bash
rsync -a --exclude '.venv/' --exclude '__pycache__/' --exclude '*.pyc' \
  --exclude '.DS_Store' --exclude 'RaisenInternship/' --exclude 'captures/' \
  ~/Programming/Raisen/ ~/Programming/RaisenInternship/
```

`chmod 755 stereo_calibration/calibrate_stereo.py` afterwards -- the mirror keeps
losing its exec bit and it shows up as a spurious mode change.

### Standing conventions

- **Sole author.** No co-author trailers, no Claude/Anthropic attribution anywhere.
- Plain commit messages. Not elaborate.
- **Push only to `feature/jetson-stereo`.** Never `main`. No PRs.
- **All text artifacts pure ASCII.** Check with a byte scan before committing.

---

## Machines

| host | address | notes |
|---|---|---|
| jetson9 | `jetson9.local` | camera, cuVSLAM, all the code. Reboots spontaneously; mDNS drops out. |
| rover-01 | `172.10.10.184` | MAC `e0:8c:fe:25:de:24`. **DHCP moved it 4 times in 5 days** -- always re-check. |
| sonar box | `172.10.10.192` | MAC `1c:69:20:30:16:a4`. Endpoint is `/sonar`, NOT `/js?json=`. |
| jetson11 | down | **Motion hazard:** never let it drive rover-01 while you are. Two 10 Hz heartbeats fighting one base. |

Deploy code with `pipeline/deploy.sh` -- it md5s both ends and refuses to claim
success on a mismatch. It exists because "committed" and "running on the Jetson"
came apart twice.

---

## Hardware state right now

**The IMU on rover-01 is faulty.** Stationary it reports:

- `|a|` = **1550 milli-g**, where a body at rest must read 1000
- `gz` = **+37.6 deg/s** while the magnetometer yaw holds still
- reported pitch contradicts its own raw accelerometer

Steady, not noisy, and **a power cycle did not clear it**. Consequence: there is no
attitude sensor during motion. vSLAM can supply heading, the magnetometer cannot
(see below). Check whether the impact knocked the ESP32 or IMU board loose --
a 1.55x accelerometer scale error surviving a power cycle is not software state.

**The camera works, and here is how to fix it when it vanishes:**

```bash
sudo modprobe -r arducam_csi2 && sudo modprobe arducam_csi2
```

No reboot needed. The driver probes once, 7.8 s after boot, and gives up. If the
camera is connected or power-cycled after boot you get `Reading register 0x103
failed / probe failed` and no `/dev/video0`, while `i2cdetect` still shows `0x0c`
on bus 10. Success looks like `firmware version: 0x10004, Sensor ID: 0x0A56`.

**Sonars: three, and that is final.** `sonar4` = FRONT, `sonar2` = left,
`sonar1` = right. `sonar3` does not exist and is not going to. No spare.

---

## The numbers

### Stereo and depth

| | |
|---|---|
| `fx_rect` | 585.04 px |
| baseline | 148.9143 mm |
| `fx*baseline` | **87.120** |
| depth | `87.120 / disparity` metres |
| accuracy | +0.3 mm at 1 m, +3.0 mm at 2 m, degrades badly at 4 m |
| `numDisparities` 224 | near limit **0.389 m** |

`raisen/depth.make_matcher(fxb, z_min)` derives the search range from the nearest
depth you intend to trust and refuses a matcher that cannot reach it. The old bug
bit twice: 128 made a marker at 0.43 m read 0.749 m.

### Camera on the rover

`stereo_calibration/out/camera_to_base.yaml`: height **0.1687 m**, pitch **+2.72
deg**, roll **+0.62 deg**. Confirmed three independent ways. Build the transform
from the **floor normal**, never from Euler angles -- `raisen/calib.CameraToBase`
does. `base_link` sits under the *camera*, not the wheel axis, so a pure pivot
shows ~78 mm of translation.

### Motion, all measured by sonar closing rate

| command | speed |
|---|---|
| 0.05 | 0.000 m/s -- does not move |
| 0.07 | 0.001 -- does not move |
| **0.08** | **0.041 -- breakaway is between 0.07 and 0.08, and it is sharp** |
| 0.09 | 0.053 |
| 0.10 | 0.076 |
| 0.14 | 0.138 |
| 0.30 | 0.320 |
| 0.45 | 0.513 |

Above breakaway: `speed = 1.257 * command - 0.0529` m/s, residuals under 0.015.

**`FORWARD_SIGN = -1`. Positive L/R drives this rover BACKWARD.** It hid for ages
because `turn()` closes the loop on the gyro and does not care how the wheels are
wired; only open-loop straight runs expose it.

**Reaction distance** = `speed * (0.88 + 0.06 + 0.19)` s = sonar staleness +
median window + latency + braking. Braking itself is negligible (0.4 cm).

| command | reaction | safe stop | 15 cm wall standoff |
|---|---|---|---|
| 0.10 | 8.6 cm | 20 cm | **ok, 6 cm spare** |
| 0.14 | 15.6 cm | 24 cm | 1 cm late |
| 0.30 | 36.2 cm | 44 cm | 21 cm late |

**0.10 is the fastest command that can follow a wall at 15 cm.** The Jetson's
`/home/narsil/rover2_obstacle_control.py` cruises at exactly 0.10 and stops at
20 cm -- correctly tuned, and correct *because* it is slow.

### vSLAM

| | |
|---|---|
| standalone `live_vslam.py` | 31.7 fps, 21 ms/frame |
| merged pipeline | 27.5 fps, 19.6 ms/frame, 2% dropped |
| stationary drift | 1.0 cm / 0.33 deg over 40 s; 0.00 deg over 72 s |
| rotation accuracy | **+1.9%** on a 45 deg pivot |
| translation | 79.0 cm vs the sonar's 78.0 -- correct to a few percent |

---

## Traps that cost real time

**`narsil-vision` holds `/dev/video0`.** It runs `restart=on-failure` and
`docker stop` exits 137, which counts as a failure -- so stop it without disabling
the policy and it is back in a second. Use `raisen.service.vision_stopped()`.

**Never `exec` in a wrapper with a trap.** It replaces the shell and destroys the
EXIT trap, leaving the container stopped with `restart=no`.

**`array.array('B', ...)`, never `img.tobytes()`** for ROS image data. 96 ms per
960x600 image in rclpy versus ~0. One line, 7x speedup.

**cuVSLAM takes mono8 only**, needs **rectified** `camera_info` (`K = P[:3,:3]`,
`D = 0`, `R = I`), reads rig geometry from **TF**, and **hangs silently** on a
missing topic rather than erroring. GXF libs must be on `LD_LIBRARY_PATH`.

**The camera returns black frames for a moment** after another process releases
it. Not a dead sensor. `camera.read_awake()` waits it out.

**`analogue_gain` above ~1100 is accepted, reads back, and does nothing.** The
advertised 1200 is not usable.

**`bin2x2` stretches every frame to its own 1-99 percentile**, so you cannot judge
exposure from its output -- 1500, 4000 and 8000 all gave a median of 6-7. Read the
raw frame.

**v4l2 control names differ by interface.** `VIDIOC_QUERYCTRL` reports `Exposure`
and `Analogue Gain`; `v4l2-ctl -c` takes `exposure` and `analogue_gain`.
`raisen.controls.normalise()` handles both.

**Sonar `-1` means no echo, which the guard maps to 999 cm = CLEAR.** An HC-SR04
aimed obliquely at a wall reflects the pulse away: the same stationary sensor gave
`95, 95, 999, 999, 999, 95`. **This is how the rover hit a wall** -- it veered,
approached at an angle, lost the echo *because* of the angle, and read clear all
the way in. Treat persistent no-echo as blind, and guard the sides too.

**Poll the sonar at 2.3 Hz, not faster.** The firmware fires the four sensors
60 ms apart so they do not hear each other, plus a 200 ms tail: a 440 ms cycle.

**The marker file races.** Two `mark()` calls within a frame interval and one is
lost. Do not write two in quick succession.

**The magnetometer is unusable for a shared frame.** Suppressed under powered
rotation (reported 20% of a 45 deg pivot) and it takes a **persistent 16 deg
positional offset over 79 cm of travel**, noise unchanged either side. Fine at
rest as a low-frequency anchor; useless for carrying a frame between two places.
With one rover this problem disappears entirely.

**The raw gyro bias is not a constant.** 0.37, then 6.07, then -0.4, then +37.6
deg/s across one day. Measure it every session while stationary; never compile it
in. And do not read a deadband off a linear fit's x-intercept -- extrapolating
through a stiction threshold cannot find it.

---

## What is verified, and what is not

**Verified on hardware:** camera capture and recovery; stereo calibration and
rectification (with an unrectified control); camera-to-rover extrinsics three
ways; depth to 2 m; cuVSLAM rate and rotation; the gyro gate in simulation *and*
on hardware (43% of frames gated during a pivot, 2% when not turning); the merged
pipeline's capture, fan-out, controls and publish; the command-to-speed curve;
evidence-bounded free space.

**Not verified:**

- **cuVSLAM pose accuracy through the merged publisher.** Equal frame rate is not
  equal accuracy, and 130-205 ms gaps remain. A/B it against `live_vslam.py` on
  the same route.
- Translation scale to better than a few percent -- the sonar reference is only
  good to a centimetre or two. A tape from a marked start would tighten it.
- Repeatability beyond 4 short runs: forward CV 10.7% over ~13 cm, which is close
  to the measurement floor. Yaw held under 0.74 deg, which is the reassuring part.
- Anything depending on the IMU.

---

## Outstanding, roughly in order

1. **Fix the IMU.** Blocks heading during motion, turn-rate measurement, and the
   fusion filter's whole purpose.
2. **`pipeline/deploy.sh` omits `imu/run_fusion.sh`**, so the publisher-alive
   guard is not on the Jetson. Add it to `FILES`. That guard exists because eight
   drive runs were once executed against a session whose publisher had already
   failed, producing an empty CSV.
3. **A/B the merged publisher against `live_vslam.py`** for pose accuracy.
4. **Build the single-rover driver.** `algorythm` has no single-rover path --
   `sim.py` hardwires two rovers and three of its five stages exist only because
   there are two. But `wallfollow.py` and `explore.py` have **zero** `World`
   references and `OccupancyGrid` already takes both stereo points and the three
   sonars, with `world.py`'s mount angles matching ours exactly. What is missing
   is a driver, not an algorithm. See `SINGLE_ROVER_PLAN.txt`.
5. **`viewer.html` crashes on a single-rover frame** -- `f.b[0].toFixed(1)` at
   line 780 is unguarded. Six small edits, or stub B off-screen.
6. **Sub-floor points: test the range-gate hypothesis.** 21% of a cloud lands
   below the floor, concentrated at 2-3 m (66%) and absent inside 1 m. Both
   earlier explanations were refuted by measurement -- it is not glare (bright
   pixels are *less* likely to be wrong) and not a mirror ghost (spread 0.62-0.96 m).
   Leading candidate is range-dependent depth error on shallow floor rays. Test:
   reduce those points' depth by a disparity step and see if they land on the floor.
7. Turn-rate per command; a longer repeatability run; `apt` is still broken on
   jetson9 by a half-configured Arducam package.

---

## Documents in the repo

| | |
|---|---|
| `CAMERA.md` | the camera end to end -- hardware, frames, exposure, calibration, extrinsics, depth, cuVSLAM, recovery |
| `INTEGRATION.txt` | how to move the car and read the sonars; the units seam with `algorythm` |
| `SINGLE_ROVER_PLAN.txt` | readiness for running the mission on one rover |
| `rover/README.txt` | addresses, protocol, every motion measurement, the collision |
| `imu/README.txt` | IMU characterisation, the fusion filter, the gyro gate, the magnetometer findings |
| `mapping/README.txt` | occupancy grid, the free-space-through-solids fix, sub-floor findings |
| `pipeline/README.txt` | what the merge settled, the three bugs the first full-stack run found |
| `jetson/camera_device_tree.txt` | both signatures of the camera outage and both fixes |
| `vslam/ROTATION_PROBLEM.txt` | a wrong conclusion and how it was found -- worth reading before trusting a sign convention |

Offline suite, no hardware needed, 31 checks:

```bash
PYTHONPATH=.:mapping python3 tests/test_offline.py
```
