#!/usr/bin/env python3
"""Log cuVSLAM odometry: rate, pose, and net drift over a window.

Kept as its own file rather than a heredoc inside the runner -- nested quoting
around f-strings in a bash-in-docker-in-ssh sandwich is how you get
"f-string expression part cannot include a backslash".
"""
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=40.0)
    ap.add_argument('--topic', default='/visual_slam/tracking/odometry')
    a = ap.parse_args()

    rclpy.init()
    node = Node('pose_log')
    st = {'n': 0, 'first': None, 'last': None, 't0': time.time()}

    def cb(m):
        p = m.pose.pose.position
        cur = (p.x, p.y, p.z, yaw_of(m.pose.pose.orientation))
        st['last'] = cur
        st['n'] += 1
        if st['first'] is None:
            st['first'] = cur
        if st['n'] % 30 == 0:
            moved = math.dist(cur[:3], st['first'][:3])
            print('  n=%5d  x=%+7.3f y=%+7.3f z=%+7.3f  yaw=%+7.2f  |moved|=%.3f m'
                  % (st['n'], cur[0], cur[1], cur[2], cur[3], moved), flush=True)

    node.create_subscription(Odometry, a.topic, cb, 10)
    end = time.time() + a.seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    if st['n']:
        f, l = st['first'], st['last']
        el = time.time() - st['t0']
        print('\n%d odometry messages in %.0f s (%.1f Hz)' % (st['n'], el, st['n'] / el))
        print('  net translation %.4f m   net yaw %+.2f deg'
              % (math.dist(l[:3], f[:3]), l[3] - f[3]))
        print('  stationary rover: both should be near zero. Whatever they are '
              'IS the drift floor,\n  and every later measurement is only as good '
              'as this.')
    else:
        print('NO ODOMETRY on %s' % a.topic)
        print('  cuVSLAM blocks silently on a missing input rather than erroring,')
        print('  so check the publisher first: /tmp/pub.log')


if __name__ == '__main__':
    main()
