#!/usr/bin/env python3
"""Verify the live stereo -> cuVSLAM pipeline against its acceptance criteria.

    python3 verify_live_pipeline.py --seconds 120

Checks, one per requirement:

  CONTINUOUS POSE          odometry arrives without gaps, and X, Y, Z and rotation
                           are all populated rather than silently zero
  SYNCHRONIZED TIMESTAMPS  left and right carry IDENTICAL stamps, and the pose
                           refers back to a real capture time
  REAL TIME, NOT LAGGING   end-to-end latency from capture stamp to pose arrival,
                           and whether it grows over the run (a queue filling up
                           looks fine for ten seconds and terrible for two minutes)
  NOT CRASHING             sustained operation with the node alive throughout

WHY camera_info AND NOT THE IMAGES. Stamp synchronisation is a property of the
headers, and camera_info carries the same stamp this publisher puts on its images.
Subscribing to the images instead would make rclpy deserialise 576 KB per frame
twice over at 31 Hz purely to read a header, which would itself perturb the timing
being measured.

WHAT A GAP MEANS. cuVSLAM stops publishing when it loses tracking rather than
publishing a bad pose, so a gap is not necessarily a crash -- it can be the
tracker giving up on a featureless wall. Both matter, and they look identical in a
rate figure, so gaps are reported individually with their timestamps.
"""
import argparse
import math
import statistics as st
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Verify(Node):
    def __init__(self, a):
        super().__init__('verify_live_pipeline')
        self.odom = []          # (recv, stamp, x, y, z, qw, qx, qy, qz)
        self.ci0 = {}           # stamp -> recv
        self.ci1 = {}
        self.create_subscription(Odometry, a.topic, self.on_odom, 50)
        self.create_subscription(CameraInfo, 'visual_slam/camera_info_0',
                                 lambda m: self.ci0.setdefault(stamp_s(m.header),
                                                               time.time()), 50)
        self.create_subscription(CameraInfo, 'visual_slam/camera_info_1',
                                 lambda m: self.ci1.setdefault(stamp_s(m.header),
                                                               time.time()), 50)

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.odom.append((time.time(), stamp_s(m.header),
                          p.x, p.y, p.z, q.w, q.x, q.y, q.z))


def yaw_of(qw, qx, qy, qz):
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                   1 - 2 * (qy * qy + qz * qz)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--topic', default='/visual_slam/tracking/odometry')
    ap.add_argument('--gap-ms', type=float, default=200.0,
                    help='inter-message interval above this counts as a gap')
    a = ap.parse_args()

    rclpy.init()
    n = Verify(a)
    end = time.time() + a.seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)

    o = n.odom
    print('=' * 68)
    print('LIVE STEREO -> cuVSLAM PIPELINE VERIFICATION, %.0f s' % a.seconds)
    print('=' * 68)
    if len(o) < 20:
        print('FAIL only %d odometry messages. Check /tmp/pub.log first: cuVSLAM'
              ' blocks silently on a missing input rather than erroring.' % len(o))
        return

    recv = [r[0] for r in o]
    span = recv[-1] - recv[0]
    gaps = [(recv[i + 1] - recv[i]) * 1000 for i in range(len(recv) - 1)]

    print('\n1. CONTINUOUS POSE OUTPUT')
    print('   messages          %d over %.1f s' % (len(o), span))
    print('   rate              %.2f Hz mean' % (len(o) / span))
    print('   interval          median %.1f ms, p99 %.1f ms, max %.1f ms'
          % (st.median(gaps), sorted(gaps)[int(len(gaps) * 0.99)], max(gaps)))
    big = [(i, g) for i, g in enumerate(gaps) if g > a.gap_ms]
    if big:
        print('   GAPS over %.0f ms: %d' % (a.gap_ms, len(big)))
        for i, g in big[:6]:
            print('     at t=%+.1f s  %.0f ms' % (recv[i] - recv[0], g))
        print('     (a gap can be lost tracking rather than a crash -- cuVSLAM'
              ' stops\n      publishing instead of emitting a bad pose)')
    else:
        print('   GAPS over %.0f ms: none' % a.gap_ms)

    xs = [r[2] for r in o]
    ys = [r[3] for r in o]
    zs = [r[4] for r in o]
    yaws = [yaw_of(r[5], r[6], r[7], r[8]) for r in o]
    print('\n2. POSE FIELDS POPULATED (not silently zero)')
    for name, v in (('X', xs), ('Y', ys), ('Z', zs), ('yaw', yaws)):
        print('   %-4s range %+8.4f to %+8.4f   spread %8.4f'
              % (name, min(v), max(v), max(v) - min(v)))
    quats = [math.sqrt(r[5] ** 2 + r[6] ** 2 + r[7] ** 2 + r[8] ** 2) for r in o]
    print('   quaternion norm   %.6f to %.6f  (must be 1.0, else rotation is'
          ' malformed)' % (min(quats), max(quats)))

    print('\n3. TIMESTAMP SYNCHRONISATION')
    common = set(n.ci0) & set(n.ci1)
    only0, only1 = set(n.ci0) - common, set(n.ci1) - common
    print('   left stamps       %d' % len(n.ci0))
    print('   right stamps      %d' % len(n.ci1))
    print('   IDENTICAL pairs   %d  (%.1f%% of the smaller side)'
          % (len(common),
             100.0 * len(common) / max(1, min(len(n.ci0), len(n.ci1)))))
    print('   unmatched         %d left-only, %d right-only' % (len(only0), len(only1)))
    if len(common) == min(len(n.ci0), len(n.ci1)) and not (only0 and only1):
        print('   -> every pair shares one stamp exactly, which is what cuVSLAM'
              ' needs to\n      treat the two images as simultaneous')

    lat = [(r[0] - r[1]) * 1000 for r in o]
    print('\n4. LATENCY, capture stamp to pose arrival')
    print('   mean %.1f ms, median %.1f ms, p99 %.1f ms, max %.1f ms'
          % (st.mean(lat), st.median(lat), sorted(lat)[int(len(lat) * 0.99)],
             max(lat)))
    h = len(lat) // 2
    drift = st.mean(lat[h:]) - st.mean(lat[:h])
    print('   first half %.1f ms -> second half %.1f ms  (drift %+.1f ms)'
          % (st.mean(lat[:h]), st.mean(lat[h:]), drift))
    if drift > 50:
        print('   LAGGING: latency is growing, so something is queueing rather'
              ' than keeping up')
    else:
        print('   -> steady, no queue growth: real time is being sustained')

    print('\n5. POSE CONTINUITY')
    jump = [math.dist(o[i][2:5], o[i + 1][2:5]) for i in range(len(o) - 1)]
    dy = [abs((yaws[i + 1] - yaws[i] + 180) % 360 - 180) for i in range(len(yaws) - 1)]
    print('   largest single-frame translation %.4f m (%.2f m/s implied)'
          % (max(jump), max(jump) / (st.median(gaps) / 1000)))
    print('   largest single-frame rotation    %.2f deg (%.0f deg/s implied)'
          % (max(dy), max(dy) / (st.median(gaps) / 1000)))
    print('   -> implausible implied speeds indicate a tracking reset, not motion')

    ok = (len(o) / span > 20 and not big and drift < 50
          and len(common) > 0.95 * min(len(n.ci0), len(n.ci1)))
    print('\n' + '=' * 68)
    print('VERDICT: %s' % ('PASS -- continuous, synchronised, real-time pose output'
                           if ok else 'see the individual checks above'))
    print('=' * 68)


if __name__ == '__main__':
    main()
