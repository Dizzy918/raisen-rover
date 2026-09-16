#!/usr/bin/env python3
"""The scripted scenario that both fake sensors and the checker read.

One source of truth for the timeline, so the synthetic IMU, the synthetic
odometry and the analysis cannot disagree about what was supposed to happen.

Reproduces the measured hardware failure: during a powered pivot the ESP32's
magnetometer-corrected yaw stops accumulating (measured -64.5% on a 90 deg
pivot, and stuck inside an 82 deg band across a watched ~800 deg turn), while
the gyro still reports the rotation honestly.
"""

# (start_s, end_s, gz_deg_s, imu_reports_truth)
PHASES = [
    (0.0,  3.0,  0.2,  True),   # settle: fuser anchors to the IMU here
    (3.0,  13.0, 0.2,  True),   # quiet, stationary -- gate MUST stay open
    (13.0, 16.0, 4.0,  True),   # 4 deg/s: below GYRO_GATE=5, must NOT gate
    (16.0, 31.0, 40.0, False),  # powered pivot: IMU yaw freezes, gate MUST close
    (31.0, 45.0, 0.2,  True),   # stopped: holdoff, then gate reopens
]
DURATION = 45.0


def gz_at(t):
    for s, e, gz, _ in PHASES:
        if s <= t < e:
            return gz
    return 0.2


def truth_yaw(t):
    """Unwrapped true heading, degrees. Starts at 10, integrates gz."""
    y = 10.0
    for s, e, gz, _ in PHASES:
        if t <= s:
            break
        span = min(t, e) - s
        if gz > 1.0:                 # only the commanded rotations move it
            y += gz * span
    return y


def imu_yaw_at(t):
    """What the ESP32 would report: truth, except frozen during the pivot."""
    for s, e, gz, honest in PHASES:
        if s <= t < e and not honest:
            return truth_yaw(s)      # stuck at the heading where the pivot began
    return truth_yaw(t)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0
