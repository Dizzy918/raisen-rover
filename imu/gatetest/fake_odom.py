#!/usr/bin/env python3
"""Synthetic cuVSLAM odometry following the scripted truth.

vSLAM is the honest rotation source in this scenario -- it was measured at
+8.5% on a real pivot -- so it tracks truth with only small noise. Published at
31 Hz to match the real node's measured rate.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

import gate_profile as gp

T0 = float(sys.argv[1])


def main():
    rclpy.init()
    node = Node('fake_odom')
    pub = node.create_publisher(Odometry, '/visual_slam/tracking/odometry', 20)
    while rclpy.ok() and time.time() - T0 < gp.DURATION + 1.0:
        t = time.time() - T0
        yaw = math.radians(gp.truth_yaw(t) + 0.03 * math.sin(t * 5.0))
        m = Odometry()
        m.header.frame_id = 'odom'
        m.header.stamp = node.get_clock().now().to_msg()
        m.child_frame_id = 'base_link'
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pub.publish(m)
        time.sleep(1.0 / 31.0)
    node.destroy_node()


main()
