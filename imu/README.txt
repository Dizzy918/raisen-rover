IMU CHARACTERISATION - rover-01
2026-08-14, jetson9


CONNECTION

rover-01 MAC e0:8c:fe:25:de:24. DHCP had moved it, so ~/.config/rover/config.json
pointed at a dead address:

    172.10.10.196 (stale)  ->  172.10.10.194

Found by ARP-sweeping the subnet for the MAC. Worth pinning by DHCP reservation,
because the config carries a fixed IP and silently stops working when it moves.

    python3 ~/bin/rover.py cmd '{"T":126}'   ->  T:1002

UPDATE 2026-08-17. The address moved AGAIN, twice: .196 -> .194 -> .188. Three
addresses in four days. auto_discover is the only reason anything still works.
Pin it by DHCP reservation.


POLL RATE - THE IMU IS ON THE ESP32, SO EVERY SAMPLE IS A WIFI ROUND TRIP

Measured 2026-08-17 over one persistent requests.Session, rover stationary:

  rate         19.2 Hz sustained, 230 samples, 0 failures
  latency      mean 52 ms, median 49, min 27, p90 80, max 122
  yaw noise    std 0.930 deg, peak-to-peak 4.681
  yaw net      +0.140 deg over 12 s

19.2 Hz, not the 0.61 Hz the characterisation below reports. That earlier figure
was an artefact of spawning a new python process per sample -- the cost was
process startup, not the network. A fusion node holding one session gets 19 Hz.

The real constraint is LATENCY, not rate: 52 ms mean with a 122 ms tail means
each sample's true timestamp is uncertain by tens of ms. That would matter if
the IMU were supplying rotation RATE against 31 Hz vSLAM. It does not matter
here, because the IMU only supplies the slow absolute heading -- the whole point
of the filter split below. Timestamp slop washes out of a low-pass reference.


THE ESP32 ALREADY RUNS AN AHRS

There is no gyro integration to write. T:1002 returns fused attitude directly:

  r, p, y     roll, pitch, yaw    degrees
  ax,ay,az    accelerometer       milli-g
  gx,gy,gz    gyro                deg/s
  mx,my,mz    magnetometer        raw
  temp

Sample, rover stationary and level:

  {"T":1002,"r":-0.484,"p":0.732,"y":-121.885,
   "ax":-12.81,"ay":-12.74,"az":984.91,
   "gx":0.316,"gy":0.561,"gz":-0.447,
   "mx":31,"my":-48,"mz":-5,"temp":68.9}


THE YAW IS ABSOLUTE, NOT INTEGRATED

Measured over 75 s, 46 samples, stationary:

  fused yaw drift              +0.183 deg/min   (11 deg/hour)
  raw gyro z bias              -0.373 deg/s  =  -22.4 deg/min if integrated
  yaw noise (residual std)      0.684 deg
  yaw peak-to-peak              3.264 deg

The fused yaw drifts 122x less than raw gyro integration would. That is only
possible if the magnetometer is correcting it, so the yaw is an ABSOLUTE heading.

CONSEQUENCE FOR FUSION. This inverts the usual assumption. The IMU is the
drift-free one and vSLAM is the drifting one:

  vSLAM yaw   smooth, precise short-term, drifts without bound
  IMU yaw     no drift, 0.7 deg noise, vulnerable to LOCATION-DEPENDENT magnetic
              distortion from steel structure and the rover's own motors

They are complementary: take rotation RATE from vSLAM, let the IMU bound the
long-term absolute heading. A complementary filter (high-pass vSLAM, low-pass
IMU) is the right shape, not a Kalman filter.

The risk indoors is distortion, not drift -- and distortion is a function of
WHERE the rover is, not how long it has been running, so it will not average out
by sampling longer. Test it by driving the same heading in several parts of the
room and comparing.


CROSS-CHECKS THAT PASSED

  az = 984.9 milli-g ~ 1 g            confirms the rover is level, and matches
                                      what rover-docs/PROVISIONING.md expects
  atan(12.81/984.9) = 0.745 deg       matches the reported pitch of 0.732 deg,
                                      so the fused attitude agrees with its own
                                      raw accelerometer
  IMU body pitch 0.73 deg up          vs the CAMERA measured at 2.0 deg up
                                      (mapping/README.txt), so the camera sits
                                      ~1.3 deg up relative to the rover body --
                                      inside the +/-1.25 deg tilt confidence


NOT DONE - NEEDS THE ROVER TO DRIVE

The physical tests in the task (straight-line distance, rotation angle, full-loop
drift) all require driving plus a tape measure plus a person. Two things gated
them; the first is now cleared:

  1. CLEARED 2026-08-16. cuVSLAM now runs on the live camera at 31 Hz via
     vslam/live_vslam.py, with a stationary drift floor of 1.0 cm and 0.33 deg
     over 40 s. See vslam/README.txt.
  2. STILL OPEN. Driving rover-01 from jetson9 may conflict with jetson11.
     poc-isaac-ros's
     own config warns: "Do not point this at rover-01 (E0:8C:FE:25:DE:24) while
     jetson11 also targets it -- two 10Hz heartbeat threads would fight over the
     same base." That is a motion hazard, not just a software clash. Confirm
     jetson11 is not driving it before any powered test.


FUSION NODE - BUILT AND VERIFIED STATIONARY 2026-08-17

imu/fuse_heading.py, run via imu/run_fusion.sh. Complementary filter, tau 12 s:
vSLAM supplies the per-frame rotation delta, the IMU anchors absolute heading.

  fused += wrap(yaw_vslam - yaw_vslam_prev)
  fused += alpha * wrap(yaw_imu - fused)      alpha = dt / (tau + dt)

60 s stationary, 1789 frames at 31.3 Hz, 900 IMU polls, 0 failures, 0 resets:

  signal            std      p2p    net
  IMU raw         0.441    2.827   -0.369 deg
  fused           0.116    0.484   -0.452 deg      3.8x quieter than the IMU
  vSLAM only      0.029    0.102   +0.028 deg

  fused vs IMU    mean +0.100 deg   -- locked to absolute heading
  IMU age         mean 35 ms, p90 62 ms, max 150 ms

WHAT THIS PROVES: the filter runs at the full 31 Hz vSLAM rate without throttling
on the 52 ms HTTP round trip, it cuts magnetometer noise 3.8x, and it holds the
absolute heading rather than wandering off with vSLAM.

WHAT IT DOES NOT PROVE: stationary, vSLAM drifts 0.028 deg, so the IMU has
nothing to correct. The point of the filter -- bounding vSLAM's unbounded
rotational drift -- can only be shown by turning the rover.

The 3.8x is worth noting against the sqrt(N) prediction of ~15x. Magnetometer
noise is correlated on a timescale near tau, so averaging cannot beat it down
like white noise.


LOGGING FOR THE PHYSICAL TESTS

run_fusion.sh writes one CSV row per vSLAM frame:

  t, x, y, z, yaw_vslam, yaw_vslam_int, yaw_imu, imu_age, yaw_fused, marker

yaw_vslam_int is vSLAM integrated ALONE with no IMU correction, kept solely so
the drift the IMU removed can be measured after the fact rather than argued about.

Annotate a run from a second shell without touching the running node:

  ssh narsil@jetson9.local 'echo "straight start" > /tmp/mark'

The label lands in the marker column on the next frame. That is what makes the
straight-line, rotation and loop tests measurable from the log instead of read
off a scrolling terminal.


MAGNETOMETER HEADING FAILS UNDER POWERED ROTATION - 2026-08-17

Measured against integrated gyro on a 90 deg pivot:

  integrated gyro    +91.60 deg
  fused yaw          +32.56 deg    -64.5% wrong
  vSLAM              +99.42 deg    +8.5%

And on a longer pivot the rover was WATCHED turning about 800 deg while the fused
yaw stayed inside an 82 deg band and never wrapped past +/-180.

Not noise -- suppression. The motors run centimetres from the magnetometer, and
this AHRS uses the magnetometer for absolute heading, so during a powered pivot the
fused yaw is pulled toward a current-dependent direction instead of accumulating.
Brief bursts get through: a 2 s pivot measured 59 deg correctly. Sustained rotation
does not.

CONSEQUENCE FOR THE FILTER. The premise recorded above -- that the IMU is the
drift-free one and vSLAM the drifter -- holds AT REST and inverts under powered
rotation. The stationary result (3.8x noise reduction, locked to absolute heading)
stands, because there was no rotation to get wrong. But the IMU correction term
must be suppressed or heavily de-weighted while the base is turning under power,
or the filter drags a good vSLAM rate toward a heading that has stopped moving.

Raw gz in the same T:126 packet is not magnetometer-corrected and is the right
source for rotation while driving: 0.37 deg/s bias, about 7 deg over a 20 s turn.


GYRO GATE - VERIFIED IN SIMULATION 2026-08-18, NOT YET ON HARDWARE

The gate added to fuse_heading.py suppresses the IMU correction while the base is
turning under power (GYRO_GATE = 5 deg/s on |gz|, HOLDOFF_S = 2 s afterwards).
It was committed unverified because the rover was off charging, and it is still
unverified against real motors. What follows is a logic test, not a hardware one.

FIRST, THE FILE ON THE JETSON WAS THE PRE-GATE VERSION. /home/narsil/vslam-min/
fuse_heading.py had zero occurrences of GYRO_GATE -- the gate was committed to
git and never uploaded, so every run on that machine since had used the ungated
filter. Second time this class of mistake has happened here. Check md5sum against
the repo copy before trusting a result from the Jetson:

    ssh narsil@jetson9.local 'md5sum /home/narsil/vslam-min/fuse_heading.py'

THE TEST. gatetest/ feeds the REAL fuse_heading.py from a fake T:126 HTTP server
and a fake odometry publisher, on ROS_DOMAIN_ID 88 so it cannot touch the live
domain 77. No camera and no rover. The scripted scenario reproduces the measured
hardware failure: 10 s stationary, 3 s at 4 deg/s (just under the gate), 15 s at
40 deg/s with the IMU yaw FROZEN as the magnetometer does under motor current,
then 14 s stopped. 1502 frames at 30.3 Hz, 799 IMU polls, 0 failures.

  quiet 10 s              0 of 288 frames gated          gate stays open
  4 deg/s, 3 s            0 of 79 frames gated           no false positive
  40 deg/s, 15 s          440 of 440 frames gated        gate closes
  after rotation stops    reopened 2.07 s later          holdoff works

ACCURACY, against the scripted truth. The ungated filter is reconstructed offline
from the SAME logged inputs, so the difference is attributable to the gate rather
than to two runs seeing different noise:

                      truth      gated             ungated
  end of pivot      -106.41   -106.55 (-0.13)   -166.02 (-59.61)
  after resettle     -98.00    -98.03 (-0.03)   -119.61 (-21.61)

459x better through the pivot. The -59.61 deg ungated error is the same character
of failure as the -64.5% measured on a real 90 deg pivot, which is the point: the
scenario is not arbitrary, it is the observed failure written down.

WHAT IS STILL UNVERIFIED. Real motor current, real magnetometer distortion, real
19 Hz WiFi jitter, and whether 5 deg/s is the right threshold on this chassis --
the rover's own slowest useful pivot rate has not been measured against it. A gate
set too low would suppress the correction during ordinary manoeuvring and leave
vSLAM to drift uncorrected; too high and the magnetometer drags the heading. That
number needs the rover powered.


THE RAW GYRO BIAS IS NOT A CONSTANT - 2026-08-18

Measured on rover-01, stationary, read straight from the Mac over HTTP:

  2026-08-17    gz bias  +0.37 deg/s
  2026-08-18    gz bias  +6.07 deg/s      same rover, same firmware, 16x larger

Four consecutive 10 s windows on the 18th: +6.079, +6.066, +6.063, +6.059, std
0.23, no drift across a 1 C rise in reported temperature. So it is rock steady
WITHIN a session and completely different BETWEEN sessions. Best guess at the
cause is a boot-time bias estimate captured while the rover was being handled on
the charger, which then stays baked in until it is power-cycled sitting still.

HOW IT WAS CONFIRMED AS A BIAS AND NOT A TURN. At 6.07 deg/s a real rotation would
be 61 deg in ten seconds, and the magnetometer-corrected fused yaw would show it.
Over four windows the fused yaw moved -0.80, -0.71, +1.04, -1.70 deg. Two
independent signals, one answer: gz carries a DC offset and the rover is still.

WHAT IT WOULD HAVE BROKEN, SILENTLY

  drive_test.py integrates raw gz for turn feedback. Its tolerance is 6.0 deg, so
  at a 6.07 deg/s bias a commanded 90 deg turn reads as "reached" after 14.8 s
  WITH THE WHEELS NEVER TURNING -- and prints a success line saying so.

  fuse_heading.py's gate compares |gz| against GYRO_GATE = 5.0. A 6.07 deg/s bias
  is above the threshold at rest, so the gate would have sat PERMANENTLY CLOSED and
  the IMU correction would never have been applied even standing still. The filter
  would silently degrade to bare vSLAM integration with no absolute anchor -- the
  exact opposite of what the gate was added to achieve.

Both failures produce plausible numbers. Neither would announce itself.

THE FIX: MEASURE IT, EVERY RUN

  raisen/roverlink.py   ImuPoller.measure_gz_bias(seconds) -- adopts the offset,
                        integrates the CORRECTED rate, and resets the integral,
                        because everything accumulated during the calibration
                        window was integrated against a bias of zero
  fuse_heading.py       calibrates at startup, right after the first IMU sample;
                        --bias-seconds, default 8. The gate now compares corrected
                        |gz|. The CSV's gz column stays RAW so recorded runs keep
                        their meaning; the bias is in the startup log and report.
  drive_test.py         samples with the motors off immediately before each turn;
                        --bias-seconds, default 6

All three REFUSE rather than guess if the spread over the window exceeds 1.5
deg/s, because that means the rover was moving and calibrating against motion
bakes that motion into every later reading.

VERIFIED LIVE against rover-01 on 2026-08-18, via raisen/roverlink.py:

  measured bias         +6.018 and +6.045 deg/s on two runs, std 0.22-0.24
  corrected gz at rest  -0.138 deg/s, so the gate reads OPEN as it should
  corrected integral    -0.06 deg at 10 s, -0.36 at 20 s, -0.63 at 30 s
  raw integral          +60.1, +105.1, +176.4 deg over the same windows

0.63 deg of drift over 30 s is usable for turn measurement. 176 deg is not.

STILL UNVERIFIED. fuse_heading.py and drive_test.py carry the same algorithm but
neither has been re-run: jetson9 was down, and the gate harness needs rclpy, which
only exists in the container on that machine. THE EARLIER GATE VERIFICATION NO
LONGER COVERS THE CURRENT FILE -- imu/gatetest must be re-run after deploying.

WORTH TRYING FIRST. Power-cycle the rover while it sits completely still. If the
firmware does estimate gyro bias at boot, that alone may restore the 0.37 deg/s
figure, which is better than compensating for a large offset in software.


vSLAM CROSS-CHECK OF A COMMANDED PIVOT - 2026-08-18

First run with all three rotation sources recording at once: cuVSLAM on the live
stereo pair, the ESP32 gyro integrated inside the fuser at 15 Hz, and the turn
controller's own integral at about 7 Hz. 3071 odometry frames, 31.7 fps published,
0 tracking resets, 0 IMU failures.

  commanded                     +45.00 deg
  drive_test integrated gyro    +44.30 deg    its own control loop, ~7 Hz
  fuser integrated gyro         +47.48 deg    same sensor, 15 Hz
  vSLAM                         +45.84 deg    INDEPENDENT of the IMU entirely

  vSLAM vs commanded            +0.84 deg, +1.9%
  vSLAM vs drive_test           +1.54 deg
  vSLAM vs fuser gyro           -1.64 deg

Three sources inside 3.2 degrees, and vSLAM sits BETWEEN the two gyro integrals
rather than outside them. The 3.2 deg spread between the two gyro figures is a
sampling-rate effect: at a 65 deg/s peak the 7 Hz loop loses area on the ramp that
the 15 Hz one catches, which is why the faster integral lands closer to vSLAM. That
is also the reason not to read the control loop's own number as ground truth.

vSLAM was previously +8.5% on a 90 deg pivot. +1.9% here is better, and this time
it is measured against a gyro that has had its bias removed rather than against one
carrying an unknown offset.

THE GATE FIRED ON HARDWARE, AT LAST. 131 of 303 frames inside the turn were gated,
43%, having been verified only in simulation until now. Bias calibration at startup
reported +5.886 deg/s and noted that raw would integrate to +353 deg/min.

STATIONARY, AFTER THE TURN, 72.5 s:

  vSLAM     +0.00 deg     the better rotation reference at rest, by a wide margin
  gyro      +5.34 deg     0.074 deg/s residual after bias removal

BASE_LINK TRANSLATED 0.078 m DURING A PURE PIVOT. Expected, and it quantifies a
known gap: base_link sits under the CAMERA, not at the wheel axis, so rotation
about the rover's real centre shows up here as a small translation. 78 mm is the
size of that lever arm. A ruler from the wheel axis would remove it.

AN UNCOMMANDED ~100 DEG ROTATION, 11 s BEFORE THE COMMANDED ONE

Between t=1.9 and 6.2 s of the run, both sensors independently recorded a large
rotation that nothing in the session commanded:

  vSLAM       -101.0 deg
  gyro         -93.2 deg
  peak raw gz  130 deg/s

Two independent sensors agreeing rules out a sensor artefact -- the rover really
turned about a quarter turn. The most likely explanation is that it was
repositioned by hand while the stack was still starting up. Worth confirming,
because the alternative -- something else driving the base -- is the motion hazard
the preflight check exists to catch, and preflight had already passed clean minutes
earlier. Incidentally it is also a free demonstration that vSLAM tracks a
hand-turn as faithfully as a driven one.

A LOGGING BUG WAS INTRODUCED AND FIXED IN THIS RUN

The edit that kept the gz column raw dropped '%.3f' % self.fused from the same
writerow, so the file carried 12 values under a 13-name header. Every column after
imu_age shifted left, and read through the header the run appeared to show the gyro
integrating -100 deg while stationary. The data was fine; the labels were not.

fuse_heading.py now builds the row, compares its length against a single shared
CSV_COLS list, and raises rather than writing a short row. xcheck_turn.csv itself
is still 12-column and must be read positionally.


THE MAGNETOMETER TAKES A POSITIONAL OFFSET: 16 DEG OVER 79 CM - 2026-08-18

Measured during the first straight-line run. The rover drove 79 cm forward and
stopped; nothing else changed.

  rover's ACTUAL heading change over the 95 s run
    gyro, bias-corrected        -1.34 deg
    vSLAM                       -0.23 deg

  the ESP32's magnetometer-corrected heading
    yaw_imu                   -110.77 -> -125.02 deg   =  -14.25 deg

  first 15 s, before the move   mean -110.56, std 0.53, p2p 2.95
  last 30 s, after the move     mean -126.80, std 0.69, p2p 3.63

A -16.2 deg shift in the mean, with the noise UNCHANGED either side. That is the
signature of a new positional offset, not a disturbance: had the motors merely
upset it, the reading would have been noisy during the move and returned
afterwards. It did not return, and it is just as quiet in its new place.

So the rover's true heading changed by at most a degree or so while its magnetic
heading changed by sixteen. Translating 79 cm indoors moved magnetic north.

WHY THIS MATTERS MORE THAN THE PIVOT SUPPRESSION. The suppression under powered
rotation is at least easy to detect and avoid -- read the heading stopped. This
is worse, because it is quiet, persistent and indistinguishable from a real
turn if the heading is all you look at.

CONSEQUENCE FOR THE MAPPING DESIGN. PROJECT_CONTEXT.md on the esp-sonar branch
builds the two-rover map merge on this premise:

    "IMU magnetometer/compass heading is used as an absolute heading reference
     for both rovers independently -- this means the rotational alignment between
     the two rovers' coordinate frames is already known from the start (both
     reference magnetic north), and only the translational offset needs to be
     established via the initial ArUco calibration."

If 79 cm of travel moves that reference 16 deg, the two frames are not
rotationally aligned by magnetic north in any useful sense once either rover has
driven anywhere. The document already anticipated the mechanism -- "distortion is
a function of WHERE the rover is, not how long it has been running, so it will
not average out by sampling longer" -- this is the measurement of it, and it is
much larger than the 0.7 deg noise the design was budgeting around.

WHAT STILL WORKS. Stationary at one spot the heading is excellent: std 0.53 deg,
and it drifts 0.18 deg/min. It is a fine reference for holding a heading in place
and for the complementary filter's low-frequency anchor while the rover sits.
What it cannot do is carry a common frame between two places.

WHAT TO USE INSTEAD, in rough order of cost: the ArUco rendezvous the design
already has for loop closure, extended to correct rotation as well as
translation; vSLAM's own yaw, which drifted 0.23 deg over these 95 s and 0.00 deg
over 72 s stationary; or the gyro, provided the bias is measured every session.
Do not spend more effort calibrating the magnetometer -- the distortion is a
property of the building, not of the sensor.
