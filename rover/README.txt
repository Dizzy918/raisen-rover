OPERATING ROVER-01 FROM JETSON9
2026-08-17


ADDRESSES - found by subnet probe, previously undocumented anywhere

  rover ESP32   172.10.10.188   e0:8c:fe:25:de:24   motors + IMU
  sonar ESP32   172.10.10.192   1c:69:20:30:16:a4   4x HC-SR04

TWO SEPARATE ESP32s. The IMU lives on the rover box, the sonars on their own box
with its own address. Nothing in the repo recorded the sonar's address; it was
found by sweeping the subnet and asking each live host for /sonar.

BOTH MOVE ON DHCP. The rover has been at .196, .194 and .188 within four days.
Pin them by reservation, or resolve by MAC before use -- rover.py's Rover()
re-resolves and persists the result, which is why anything still works.

At the last check sonar1 read a real distance and sonar2..4 all returned -1.
That is the firmware's pulseIn timeout, so either nothing was in range or three
of the four are not wired. Worth establishing before relying on side coverage.


PROTOCOL

  motors   GET http://<rover>/js?json={"T":1,"L":<l>,"R":<r>}
           l, r in -0.5..0.5, where 0.5 is 100% PWM
  IMU      GET http://<rover>/js?json={"T":126}   -> T:1002 with fused r/p/y
  sonar    GET http://<sonar>/sonar               -> {"sonar1".."sonar4"} in cm,
                                                     -1 means out of range

WHAT 0.5 MEANS IN METRES PER SECOND IS UNKNOWN. The command is a PWM fraction.
The whole accelerate/cruise/slow design on the esp-sonar branch is specified in
units that do not convert to distance, which is measurable now that vSLAM runs:
hold a fixed command, let vSLAM measure the displacement, divide.


THE HAZARD

poc-isaac-ros's config warns: do not drive rover-01 from jetson9 while jetson11
also targets it, because two 10 Hz heartbeat threads fight over one base. That is
a motion hazard, not just a software clash.

It cannot be checked over the network -- jetson11 does not even resolve from the
Mac -- but it CAN be detected physically, and drive_test.py's preflight does:
command zero, then watch the gyro for a second. A base that moves while being
told to stop is being driven by someone else. Measured idle noise is about
2 deg/s, so the abort threshold is 8.

THE FIRMWARE DEADMAN IS ~3 SECONDS. It is the last resort, not the plan: a
SIGKILL leaves the rover rolling until it expires. drive_test.py commands zero on
every exit path and handles SIGINT and SIGTERM. Do not rely on the deadman.

NEVER RUN TWO CONTROLLERS AT ONCE, including two of ours. rover.py's Rover()
starts a heartbeat thread of its own the moment drive() is called, so it must not
overlap with drive_test.py or obstacle_avoid_drive.py. Reading the IMU with
Rover() is safe -- the heartbeat only transmits once drive() has been called.


DRIVE_TEST.PY

  python3 drive_test.py preflight                          # no motion
  python3 drive_test.py straight --speed 0.2 --seconds 6
  python3 drive_test.py turn --degrees 90
  python3 drive_test.py stop
  --dry-run   runs the full loop and sends nothing; the turn mode simulates yaw
              so the closed loop, easing and termination all get exercised

Sonar gates every forward cycle and stops inside STOP_CM, set to 40 cm rather
than the branch's 10 cm trigger. The reason is staleness, not caution: the sonar
firmware loop is 4 x 60 ms of pulse timeouts plus a 200 ms delay, so a reading can
be half a second old, and half a second of travel can turn 30 cm into 15 cm.

Turns are CLOSED LOOP on the IMU's absolute yaw, unlike the branch's open-loop
1.2 s pivot which produces an unknown angle. Accuracy is bounded by the IMU's
0.93 deg noise and ~52 ms of HTTP latency per sample, which at 30 deg/s is 1.5 deg
of lag, so it eases off approaching the target and reports what it ACHIEVED rather
than assuming it hit the commanded angle.

Every move writes /tmp/mark, which imu/run_fusion.sh's node folds into its CSV, so
moves are labelled in the log instead of being matched up by timestamp afterwards.


RUNNING A TEST SESSION

Two shells. First starts the logger, second drives:

  ssh narsil@jetson9.local '/home/narsil/vslam-min/run_fusion.sh 240 drive1.csv'
  ssh narsil@jetson9.local 'cd /home/narsil/rover && python3 drive_test.py straight --speed 0.2 --seconds 6'

Note run_fusion.sh stops narsil-vision to take the camera and restores it on exit.


SLOW PIVOT, FIRST VALID CLOSED-LOOP TURN ON A CORRECTED GYRO - 2026-08-18

  drive_test.py turn --degrees 45 --speed 0.28 --rover-ip 172.10.10.184

  gz bias measured       +5.983 deg/s (std 0.251, n=73) over 6 s stationary
  commanded              +45 deg
  achieved               +43.9 deg by bias-corrected integrated gyro
  error                  -1.1 deg, 2.4%
  duration               10.2 s total, of which 6 s was bias calibration
  command                ramped to 0.28, eased to 0.26 inside 25 deg of target

WHY THE BIAS LINE IS THE IMPORTANT ONE. The run printed "uncorrected, that bias
alone would read +89.7 deg over a 15 s turn". The tolerance is 6 deg, so on the
old raw integrator this turn would have declared itself "reached" within about
seven seconds at roughly twice the commanded angle -- and printed a success line.
Every turn measurement taken on this rover before the correction is suspect to the
extent that the bias was non-zero at the time.

PREFLIGHT'S SECOND-CONTROLLER CHECK HAD TO CHANGE FOR THE SAME REASON. It tested
raw gyro MAGNITUDE, and a 6 deg/s offset reads as "the base is moving
CONTINUOUSLY". It now separates the steady part from the varying part:

  gyro offset   (+0.17, +0.18, +5.94) = 5.95 deg/s     steady  -> bias
  residual      median 0.40, peak 16.00 deg/s          varying -> motion
  fused yaw     net +2.30 deg over 5 s                 not turning

A DC offset is constant; a controller fighting for the base produces motion that
varies as it ramps and corrects. An offset above 15 deg/s is refused outright,
because at that point the two cases cannot be told apart from here.

The peak residual near 16 deg/s recurs on every preflight (15.45, 16.00) and is
the known sampling artefact: polling the ESP32 right after stop commands returns
occasional garbage gyro samples. It is isolated, never sustained.

THE MAGNETOMETER SUPPRESSION REPRODUCED, THIRD TIME. Fused yaw went -145.92 to
-136.92, a net +9.00 deg, against +43.9 actually turned -- it reported 20% of the
rotation. Consistent with the -64.5% measured on a 90 deg pivot and with the
watched ~800 deg turn that stayed inside an 82 deg band. Gyro for control, always.

WHAT THIS RUN DOES NOT ESTABLISH. The gyro is the only rotation sensor in the loop,
so "achieved +43.9" is the corrected gyro's own account of itself. Its credibility
comes from the stationary drift measurement -- 0.63 deg over 30 s corrected, against
176 deg raw -- not from anything independent in this run. The fused yaw cannot
confirm it because the magnetometer is suppressed exactly when the rover turns. An
independent check needs either vSLAM running during the pivot, or a protractor.


SONAR BROUGHT UP - 2026-08-18

  GET http://172.10.10.192/sonar   ->  {"sonar1".."sonar4"} in cm, -1 = no echo

Note the endpoint: /sonar, NOT the rover's /js?json= interface. Probing the sonar
box with the rover's protocol returns an HTTP error, which reads like "wrong
address" and is really "right box, wrong path".

40 samples at 4 Hz, rover stationary:

  sensor      n      min   median      max     std    no-echo
  sonar1     40      0.1     14.3     14.9     2.2      0
  sonar2     40     59.8     65.3     75.8     2.7      0
  sonar3     40        -        -        -       -     40
  sonar4     40     84.0     92.7    111.6     5.6      0

THE SENSORS ARE STEADY, which contradicts a first impression. Consecutive early
reads gave sonar1 = 233.4 then 230.9, and minutes later a steady 14.3 -- two
internally consistent regimes, not noise. Nothing commanded the rover between
them, so something in front of it changed physically. Judge this sensor on a
sample, never on two readings.

  sonar1  the one drive_test uses to gate forward motion. 39 of 40 readings
          between 14 and 15 cm, one 0.1 cm dropout. So there really is something
          about 14 cm ahead, well inside STOP_CM = 40, and preflight refuses --
          correctly. The rover had pivoted 90 deg in total during the turn tests,
          so it is no longer pointing where it started.
  sonar3  40 of 40 no-echo, and CONFIRMED AS FINAL -- there will not be a fourth
          sonar. So the fleet is three: front, left, right, which is exactly the
          {fwd, left, right} contract algorythm/rover.py already expects. The
          consequence is that there is NO SPARE. Lose one and the wall-follower
          loses a side, or forward motion loses its guard entirely, and nothing in
          the current code degrades gracefully when a channel stops answering.
  sonar4  std 5.6 cm, the widest spread. Worth knowing before it gates anything.

STILL NOT MEASURED: the straight-line run, and with it the vSLAM TRANSLATION
SCALE. The rotation scale is now confirmed to +1.9% against vSLAM, but no
displacement has ever been checked against a tape measure or against anything
else. Two independent references are available for it now and cost nothing extra:

  sonar1 delta across the run, if it faces a flat perpendicular surface
  a tape measure, if the start position is marked BEFORE the run

The rover needs about 1.5 m of clear space ahead of it for that. It does not have
it right now.


THE FIRST STRAIGHT-LINE RUN, AND THREE THINGS IT SETTLED - 2026-08-18

Everything below needed the sonar box working and someone to tell us which
channel faces where. Both arrived, and the run took about ten minutes.


1. WHICH SONAR FACES FORWARD: sonar4. THIS CODE HAD IT WRONG.

  sonar4 = FRONT      sonar2 = LEFT      sonar1 = RIGHT      sonar3 = does not exist

drive_test.py gated forward motion on sonar1, which faces RIGHT. On the morning
of the 18th sonar1 read 14 cm and refused a straight run while the actual front
was clear at 93 cm, which is how the error surfaced. It was wrong in the
dangerous direction: with an obstacle ahead and a clear right side, the same bug
would have driven into it.

PROJECT_CONTEXT.md on the esp-sonar branch says "1 (front)". It is wrong for this
rover. rover2_obstacle_control.py reads sonar4, and that matches the hardware.

The three channels map exactly onto the {fwd, left, right} contract
algorythm/rover.py expects, so there being no fourth costs nothing downstream --
but it also means no spare, and nothing in the code degrades gracefully if one
stops answering.


2. POSITIVE L/R DRIVES THIS ROVER BACKWARD.

Commanding +0.30 for 2.5 s moved the front sonar's reading from 131 cm to 169 cm
-- the rover retreated from the wall it was facing. FORWARD_SIGN = -1.0 now
applies the chassis sign at the wire, so cur stays positive in the log and the
ramp reads naturally.

WHY THIS HID FOR SO LONG, and it is worth remembering: turn() is CLOSED LOOP on
the gyro. It drives until the measured rotation reaches the target, so it does
not care how the wheels are wired, and a commanded +45 deg came out +45.84 by
vSLAM regardless. drive_straight is the only open-loop mover, and open loop is
where a sign error has to show up. The first straight run this rover ever did is
what exposed it.


3. THE SONAR EMITS WILD SINGLE SAMPLES, IN BOTH DIRECTIONS.

Against a wall that four reads put at 130.9 cm, one sample returned 70.8. On the
first attempted straight run one returned 5.5 cm -- one second in, with the
command still at 0.03, below the deadband, so the rover had not moved at all.
That single reading aborted the run.

And the dangerous direction exists too: mid-run a raw window read 131,132,353.
A spurious LONG reading is the one that hides a real obstacle.

Sonar now polls in a background thread at the firmware's own rate and the guard
brakes on the MEDIAN of the last three. The median rejected both the 5.5 and the
353. Polling in a thread also stops the 10 Hz control loop firing 10 HTTP
requests a second at a box whose full sensor cycle is 440 ms -- it could only
ever have returned the same numbers, and the hammering may itself be part of why
the garbage appears.


THE RUN, AND THE NUMBER THAT HAD NEVER BEEN MEASURED

  drive_test.py straight --speed 0.30 --seconds 3.0, with run_fusion.sh recording

  front sonar        184.1 -> 106.1 cm      advanced 78.0 cm
  vSLAM              dx +0.7899 m           advanced 79.0 cm
  difference         +1.0 cm, +1.3%

THAT IS THE vSLAM TRANSLATION SCALE, bounded for the first time. Two independent
sensors agree to about a centimetre over 79 cm.

Be careful how much this claims. The sonar reference is itself lagged, quantized,
and demonstrably capable of single-sample nonsense; HC-SR04 accuracy is about
+/-1 cm at best. So the honest statement is that the scale is correct to within a
few percent -- the first bound of any kind on it -- not that it is accurate to
1.3%. A tape measure from a marked start would tighten it, and costs nothing.

The path was straight and level while it did it: dy -8 mm and dz +5 mm over
790 mm of travel.


SPEED PER COMMAND, which the algorythm branch needs and did not have

Fitted to vSLAM's x over the steady-command portion:

  command 0.30        0.354 m/s      rms residual 4.1 mm over 1.81 s
  peak                0.496 m/s      during the ramp
  extrapolated to full scale 0.5     about 0.59 m/s

algorythm/rover.py assumes V_MAX = 0.45 m/s, so the real chassis is somewhat
FASTER than the simulation expects. The deadband is the bigger disagreement:

  measured command floor    0.24 of 0.5   = 48% of full scale = about 0.28 m/s
  simulated V_DEADBAND      0.035 / 0.45  = 7.8% of full scale

Eight times larger in m/s. The wall-follower's plan to creep at 15 cm from a wall
assumes a speed this chassis cannot produce, and rover2_obstacle_control.py's
CRUISE_SPEED = 0.15 and MIN_MOVING_SPEED = 0.08 are both below the floor -- that
script would command the motors and the rover would not move. Also remember the
deadband tracks battery voltage, so none of these numbers survive a flat pack.


COMMAND-TO-SPEED, AND A SWEEP THAT WAS INVALID - 2026-08-18

A four-point sweep at commands 0.26, 0.30, 0.38, 0.45 with --seconds 2.0 appeared
to show the chassis SATURATING: 0.38 and 0.45 returned identical steady speeds of
0.296 m/s and distances within 2 mm of each other.

IT WAS AN ARTEFACT OF MY OWN TEST. MAX_ACCEL ramps the command at 0.3 per second
and drive_straight begins ramping DOWN at (seconds - 0.6), so with --seconds 2.0
the command peaks at 1.4 s and can never exceed about 0.42. Commands of 0.38 and
0.45 were therefore never applied; both runs peaked near 0.40. Any sweep has to
allow ramp-up + hold + ramp-down, which for command 0.45 means --seconds 3.1 at
the very least.

Repeated at --seconds 3.4, where the log confirms cmd=+0.450 was actually held:

  command   steady speed   fit rms   hold     distance   lateral   dz
    0.30      0.320 m/s     4.6 mm   1.62 s     80.3 cm    32 mm   +2.76 cm
    0.45      0.513 m/s     7.4 mm   1.38 s    119.2 cm   120 mm   +2.65 cm

1.50x the command gave 1.61x the speed. NOT saturated -- roughly linear, slightly
super-linear over this range. Extrapolating to full scale 0.50 gives about
0.58 m/s, which agrees with the 0.59 estimated from the single 3.0 s run earlier.

Do NOT read a deadband out of these two points. The relationship is linear only
well above stiction; a two-point fit through 0.30 and 0.45 back-extrapolates to a
deadband of 0.05, against the 0.24 actually measured. Resolving the low end needs
points just above 0.24, each with enough time to reach steady state.


THE ROVER DOES NOT DRIVE STRAIGHT, AND IT GETS WORSE WITH SPEED

  command 0.30    32 mm lateral over 803 mm     0.4%
  command 0.45   120 mm lateral over 1192 mm    1.0%

Skid steer with unmatched motors. For open-loop dead reckoning this matters more
than the speed calibration does: a metre of travel at 0.45 lands 12 cm off course,
and nothing in the current straight-line code corrects it. Either close the loop on
heading, or drive slower, or accept it and let the mapping layer absorb it.


THE VERTICAL OFFSET IS A STEP, NOT DRIFT AND NOT PITCH

Both runs reported dz of about +2.7 cm despite one travelling 80 cm and the other
119 cm at a very different speed. That rules out two explanations at once: drift
would scale with distance, and acceleration-induced pitch would scale with
command. A constant offset regardless of either looks like a one-time step at
motion onset -- the camera sitting at a slightly different attitude while driving
than at rest, on the independent suspension.

It is 2-3% of travel, so it is not urgent, but it is worth knowing that the
measured extrinsics describe the camera AT REST and something moves when the
wheels turn. A slow-ramp run would separate onset from acceleration properly.


BRAKING DISTANCE, AND TWO RUNS THAT DID NOT MAKE SENSE - 2026-08-18

--hard-stop cuts the motors dead instead of ramping down, which is the only way to
measure braking. It is the term every collision-safety budget here was missing.

  command 0.26   braking 0.0 cm
  command 0.40   braking 0.4 cm over 0.19 s

Braking is effectively instantaneous. N20 gearmotors have enough internal friction
that cutting drive stops the chassis inside half a centimetre.

THE SAFETY BUDGET IS THEREFORE COMPLETE, at the slowest speed this chassis can do
(0.28 m/s, the deadband floor -- it cannot go slower):

  sonar staleness, one firmware cycle    0.44 s   12.3 cm
  median-of-three filter, worst case     0.44 s   12.3 cm
  command latency, measured mean         0.06 s    1.7 cm
  braking                                0.19 s    0.4 cm
  ----------------------------------------------------------
  reaction distance                               26.7 cm

  vs the 40 cm front stop threshold      13 cm of margin.        OK.
  vs the follower's 15 cm wall standoff  exceeds it by 12 cm.    NOT OK.

So the existing forward guard is sound, and wall-following at a 15 cm standoff is
not -- the rover would be touching the wall before it could react. The limit is
not the code. It is that the minimum speed is 0.28 m/s and the sonar refreshes at
2.3 Hz.

BUT TWO OF TODAY'S TEN STRAIGHT RUNS WERE ANOMALOUS, and I cannot explain them.

  command 0.26, 2.60 s    5.7 cm travelled.   Earlier, same command, 2.0 s: 28.6 cm.
  command 0.40, 2.68 s    25 cm BACKWARD along its own start heading, with 18 deg
                          of rotation confirmed by the gyro independently.

Ruled out by measurement afterwards: battery (12.14 V, healthy), obstruction (front
104 cm, left 76, right 45), levelness (roll -0.46, pitch +0.55, az 979), and a
stuck gyro (gz -0.17 deg/s at rest). The other eight runs today were consistent and
straight, so this is not a systematic sign or calibration error.

The leading candidate is WHEEL SLIP. It would produce both symptoms at once -- poor
translation for a given command, and yaw when the two sides slip unequally -- and
this floor is smooth with bright light on it. That is a guess, not a finding.

WHY THIS MATTERS MORE THAN THE ARITHMETIC ABOVE. Wall-following assumes that a
commanded velocity produces roughly the expected motion. Twice in ten runs it did
not, once by a factor of five and once in the wrong direction. Until that is
understood or shown to be rare, an autonomous run around a room is driving on an
assumption that has already failed twice today.

WHAT WOULD SETTLE IT: ten identical runs at one command from one position, logging
vSLAM displacement and gyro yaw each time. If they cluster, the anomalies were
positional or transient and the follower can proceed with a margin. If they
scatter, open-loop motion on this floor is not trustworthy and the follower needs
closed-loop heading correction before it goes anywhere near a wall.


THE IMU IS FAULTY, AND A WALL WAS HIT - 2026-08-18 LATE

THE IMU IS REPORTING PHYSICALLY IMPOSSIBLE VALUES. Ten samples, rover stationary,
after a power cycle:

  |a|          1564 milli-g, std 35      a body at rest reads 1000. Full stop.
  ax, az       ~750, ~1370               az alone exceeds 1 g
  pitch        -24.9 deg, std 0.78       but a real -24.9 deg tilt gives az = 907,
                                         not 1370, so the attitude and the raw
                                         accelerometer contradict each other
  gz           +37.6 deg/s               while the magnetometer yaw holds still
                                         to +/-0.8 deg over 6 s

Three readings, mutually inconsistent, all steady rather than noisy. This is not
vibration and not a bias that averaging fixes: the accelerometer magnitude is 1.56x
what gravity can be. A POWER CYCLE DID NOT CLEAR IT -- gz was +37.65 before and
+37.64 after, where an earlier power cycle had taken it from +6.07 to -0.42.

CONSEQUENCE: there is currently NO usable attitude sensor on this rover. The
magnetometer is suppressed under power and shifts with position, vSLAM needs a
camera that is currently absent, and the gyro is this. Nothing should be driven
until one of them is back.

WHEN IT WENT WRONG, from the bias measured at the start of each fusion run:

  braking test        -0.159 deg/s     healthy
  repeatability test  +37.651 deg/s    faulty

So it degraded between those two, which is AFTER the two anomalous braking runs and
BEFORE the eight repeatability runs. That ordering matters twice over. It means the
anomalous rotation in braking run 2 was measured by a healthy gyro and was
therefore real, not an artefact -- and it means the fault was already present before
the collision, so the collision did not cause it. A plausible reading is the
reverse: braking run 2's unexplained 25 cm and 18 deg was itself an impact, and
that impact disturbed the IMU. Unproven, but it fits the order of events.


WHY THE FRONT GUARD DID NOT PREVENT THE COLLISION

Two failures compounding, and both are structural rather than bad luck.

1. GRAZING INCIDENCE RETURNS NO ECHO. Six consecutive reads of the left sonar,
   rover stationary, nothing moving:

     left: 95, 95, 999, 999, 999, 95 cm

   The same sensor at the same distance from the same wall, alternating between a
   real reading and "nothing within range". An HC-SR04 pointed obliquely at a flat
   surface reflects the pulse away instead of back. The firmware's own
   documentation says a timeout "off a wall at grazing incidence is extremely
   common" -- and the guard maps a timeout to 999 cm, i.e. CLEAR. So the closer the
   approach angle gets to a wall, the more likely the guard is to see open space.

2. drive_straight GUARDS ONLY FORWARD. There is no side check at all. A rover that
   veers -- measured at 0.4% of distance at command 0.30 and 1.0% at 0.45 -- meets a
   wall corner-first while the front beam still points down open floor.

Combine them and the failure is predictable: veer toward a wall, approach it at an
angle, the front sonar loses its echo exactly because of that angle, and the guard
reads 999 the whole way in.

WHAT WOULD ACTUALLY FIX IT, in order of how much they buy:

  treat persistent no-echo as suspect, not clear. A channel reading 999 on every
  sample for several seconds while its neighbours vary is either aimed at open
  space or blind, and near a wall the second is likelier. At minimum, do not let a
  no-echo channel authorise motion it could not see an obstacle through.

  guard the sides too. left and right exist and are unused by drive_straight. A
  side reading collapsing while the front stays clear is the signature of exactly
  this collision.

  cap open-loop segments. 1.4 s at 0.32 m/s is 45 cm, over which 1% veer is 4.5 mm
  -- fine. Eight of them back to back is 3.6 m of accumulated heading error with
  nothing correcting it. Either re-square on the walls between segments, or close
  the loop on heading, which needs a working gyro.


THE esp-sonar SLOWDOWN THRESHOLDS, CHECKED AGAINST MEASUREMENT

feature/esp-sonar's rover2_obstacle_control.py is the existing answer to "when to
slow down and when to stop":

  CRUISE_SPEED        0.15      SLOWDOWN_START_CM   45
  MIN_MOVING_SPEED    0.08      NEARLY_STOPPED_CM   19
  RAMP_STEP           0.01      CONTROL_INTERVAL    0.1 s
  quadratic taper between the two distances, reading sonar4 as the front

The SHAPE is right -- taper quadratically, floor the speed above zero, stop hard
inside a threshold. Three of the numbers do not survive contact with this chassis.

  CRUISE_SPEED = 0.15        below the measured 0.24 deadband. The rover does not
  MIN_MOVING_SPEED = 0.08    move at either value. The script would command the
                             motors, print falling distances, and sit still.

  NEARLY_STOPPED_CM = 19     unreachable. At the slowest speed that moves this
                             chassis, 0.243 m/s, reaction distance is 27.4 cm:
                             0.44 s of sonar staleness, another 0.44 for the
                             median-of-three, 0.06 of command latency, 0.19 of
                             braking. The stop point sits 8.4 cm INSIDE the
                             distance already committed when the reading arrives.

  SLOWDOWN_START_CM = 45     the taper has nothing to taper. Between the deadband
                             and full scale there is one usable continuous speed
                             band, and its bottom is 0.243 m/s.

WHAT IS ACHIEVABLE WITH CONTINUOUS DRIVE

  command 0.24, 0.243 m/s    stop at 35 cm, begin slowing at 89 cm
  command 0.30, 0.320 m/s    stop at 44 cm, begin slowing at 110 cm

drive_test.py's STOP_CM = 40 sits correctly between those two. It was picked by
judgement and the arithmetic agrees with it.

PULSING WOULD MAKE THEIR 19 cm REACHABLE, AND THE OBVIOUS SUM IS WRONG

Duty-cycling the motors gets underneath the deadband, because stiction only has to
be broken during a burst. But a pulsed rover is not a slow rover: it is stopped,
then travelling at 0.32 m/s. An obstacle appearing mid-burst is met at full speed,
so the worst case is set by how far ONE burst carries it plus braking from full
speed -- not by the duty-cycle average.

  period  duty   burst    average predicts   worst case   stop threshold
   1.0 s   25%   0.25 s        9.0 cm          14.1 cm       22 cm
   0.5 s   25%   0.125 s       9.0 cm          14.1 cm       22 cm
   0.2 s   25%   0.05 s        9.0 cm          14.1 cm       22 cm
   1.0 s   50%   0.50 s       18.1 cm          22.1 cm       30 cm

The average-speed figure understates the worst case by 1.5-2x. Even so, 25% duty
lands a stop threshold near 22 cm, which is close to their 19 -- so their number
was not wrong, it was written for a rover that can creep, and pulsing is what makes
this one creep.

THE MEASUREMENT THAT HAS TO COME FIRST: the shortest burst that reliably breaks
stiction. Every ramp in drive_straight took about 0.8 s to clear the deadband, so
whether a 50 ms burst moves the chassis at all is unknown, and it decides which row
of that table is real. It needs a working rover, which currently means a working
IMU.


THE DEADBAND IS 0.038, NOT 0.24. I HAD IT WRONG BY 6x - 2026-08-18

The Jetson carries a working obstacle-avoidance script,
/home/narsil/rover2_obstacle_control.py, which drives at CRUISE_SPEED = 0.10 and
hard-stops at 20 cm. That is less than half the 0.24 command floor recorded here,
so by that figure it could not move at all. It moves perfectly well.

Measured by running it and watching its own front-sonar readout: 132.0 cm down to
64.9 cm, 67.1 cm of travel, per-cycle steps of 4.33 cm with a std of 0.26 over 17
cycles. Steady, not accelerating.

  command 0.10  ->  0.079 m/s

With the two points already measured, the curve is convincingly linear:

  0.10 -> 0.079      fit: speed = 1.238 * command - 0.0469 m/s
  0.30 -> 0.320      residuals +0.002, -0.005, +0.003 m/s
  0.45 -> 0.513      deadband (speed = 0) at command 0.038
                     full scale 0.50 -> 0.57 m/s

WHAT THIS RETRACTS. Every conclusion here that rested on a 0.24 floor:

  "the deadband makes wall-following as designed impossible"   WITHDRAWN
  "this chassis has two speeds: stopped, and 0.28 m/s"         WITHDRAWN
  "V_DEADBAND is wrong by 8x in m/s"                           WITHDRAWN -- the
      simulation's 0.035 was closer to right than my measurement was
  "the follower cannot creep, so pulsing is needed"            WITHDRAWN -- the
      chassis creeps fine, and the pulse analysis is now a curiosity, not a plan

WHERE 0.24 PROBABLY CAME FROM. It predates the discovery that positive L/R drives
this rover BACKWARD, so it was almost certainly measured in reverse, and it has been
propagated into DEADBAND_CMD and the easing logic ever since. Note that TURN_MIN_CMD
= 0.26 is a separate number for pivoting, which needs more torque than rolling
because the wheels scrub sideways -- that one has NOT been refuted and no low-end
turn sweep has been done.

THE THRESHOLDS, CORRECTED

  command  speed      reaction distance   their 20 cm stop
   0.10    0.079 m/s        8.9 cm         leaves 11.1 cm of margin
   0.30    0.320 m/s       36.2 cm         would be 16 cm too late
   0.45    0.513 m/s       58.0 cm         would be 38 cm too late

So rover2_obstacle_control.py is correctly tuned, and it is correct BECAUSE it is
slow. Its 20 cm stop and 0.10 cruise belong together; neither survives being raised
without the other. drive_test's STOP_CM = 40 suits its own 0.30 default.

AND WALL-FOLLOWING IS VIABLE AFTER ALL. At 0.079 m/s the reaction distance is
8.9 cm against the follower's 15 cm standoff -- 6 cm of margin where I previously
computed a 12 cm deficit. The single-rover run is a much better prospect than the
last assessment said, provided it runs at that speed.


THE FULL COMMAND-TO-SPEED CURVE, AND WHERE THE DEADBAND REALLY IS - 2026-08-18

Measured with rover/sonar_drive.py, which drives on the front sonar alone and fits
the closing rate. It needs no IMU and no camera, which is why it could run at all:
the IMU was reading 1564 milli-g stationary and /dev/video0 was absent.

  command     speed        note
    0.05      0.000 m/s    does not move
    0.07      0.001        does not move
    0.08      0.041        moves
    0.09      0.053
    0.10      0.076
    0.14      0.138
    0.30      0.320
    0.45      0.513

BREAKAWAY IS BETWEEN 0.07 AND 0.08, and it is sharp. 0.07 travelled 0.7 cm in 7 s,
which is noise; 0.08 travelled 22.7 cm. Nothing gradual about it.

Above breakaway the curve is close to linear: speed = 1.257 * command - 0.0529 m/s,
residuals under 0.015 across six points.

AND THE THIRD DEADBAND FIGURE IS THE RIGHT ONE. This number has now been wrong
twice, in opposite directions, for two different reasons worth remembering:

  0.24    measured in reverse, before it was known that positive L/R drives this
          rover backward. Too high by 3x.
  0.038   my linear fit's x-intercept, taken as the deadband. Too low: the fit
          predicts 0.035 m/s at command 0.07 and the truth is 0.001. EXTRAPOLATING
          A STRAIGHT LINE THROUGH A STICTION THRESHOLD CANNOT FIND THE THRESHOLD --
          the line describes the region above it and says nothing about where the
          wheels break free. The only way to find breakaway is to command values
          near it and see whether the rover moves.
  0.07-0.08   measured directly, both sides.

WHY rover2_obstacle_control.py's CRUISE_SPEED = 0.10 IS A GOOD CHOICE. It sits one
step above breakaway, so stiction variation cannot stall it, and it is the slowest
speed with margin. 0.08 would work and would be marginal.

REACTION DISTANCE, and which thresholds each speed can honour:

  command   m/s     reaction   vs a 20 cm stop   vs a 15 cm wall standoff
    0.08    0.041     4.6 cm      15 cm spare       10 cm spare
    0.09    0.053     6.0 cm      14 cm spare        9 cm spare
    0.10    0.076     8.6 cm      11 cm spare        6 cm spare
    0.14    0.138    15.6 cm       4 cm spare        1 cm LATE
    0.30    0.320    36.2 cm      16 cm LATE        21 cm LATE
    0.45    0.513    58.0 cm      38 cm LATE        43 cm LATE

So 0.10 is the fastest command that leaves real margin at a 15 cm standoff, and
0.14 is already too fast for it. Anything at 0.30 or above needs a 45 cm threshold
and cannot follow a wall closely at all. That is the whole speed budget for the
mapping run, in one table.

THE GUARD WORKS. In the 0.09 run it fired at a median of 14 cm and the rover settled
at 9.9 cm -- 4.1 cm of overshoot against 6.0 cm predicted. Conservative, and it
stopped without contact.
