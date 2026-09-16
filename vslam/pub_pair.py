#!/usr/bin/env python3
"""Publish one static rectified stereo pair to cuVSLAM at a steady 30 Hz.

A static pair means zero motion, which is a legitimate state (a stationary
robot) -- cuVSLAM should initialise and keep publishing odometry. Timestamps
advance at an exact 30 Hz so image_jitter_threshold_ms is never tripped.
"""
import sys
import numpy as np, cv2, yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

D = '/workspaces/testpair'
BASELINE = 0.1489144      # from our calibration, metres

def ci_from_yaml(p, frame):
    d = yaml.safe_load(open(p))
    m = CameraInfo()
    m.width = int(d['image_width']); m.height = int(d['image_height'])
    m.distortion_model = d['distortion_model']
    m.k = [float(x) for x in d['camera_matrix']['data']]
    m.d = [float(x) for x in d['distortion_coefficients']['data']]
    m.r = [float(x) for x in d['rectification_matrix']['data']]
    m.p = [float(x) for x in d['projection_matrix']['data']]
    m.header.frame_id = frame
    return m

def img_msg(arr, frame):
    m = Image()
    m.height, m.width = arr.shape
    m.encoding = 'mono8'; m.is_bigendian = 0; m.step = arr.shape[1]
    m.data = arr.tobytes(); m.header.frame_id = frame
    return m

class Pub(Node):
    def __init__(self):
        super().__init__('static_pair_pub')
        L = cv2.imread(D + '/left_rect.png', cv2.IMREAD_GRAYSCALE)
        R = cv2.imread(D + '/right_rect.png', cv2.IMREAD_GRAYSCALE)
        if L is None or R is None:
            sys.exit('test pair not found in ' + D)
        self.get_logger().info(f'pair {L.shape} loaded')
        self.iL = img_msg(L, 'left_cam');  self.iR = img_msg(R, 'right_cam')
        self.cL = ci_from_yaml(D + '/left_rect_info.yaml', 'left_cam')
        self.cR = ci_from_yaml(D + '/right_rect_info.yaml', 'right_cam')

        self.pi0 = self.create_publisher(Image, 'visual_slam/image_0', 10)
        self.pc0 = self.create_publisher(CameraInfo, 'visual_slam/camera_info_0', 10)
        self.pi1 = self.create_publisher(Image, 'visual_slam/image_1', 10)
        self.pc1 = self.create_publisher(CameraInfo, 'visual_slam/camera_info_1', 10)

        # cuVSLAM reads the rig geometry from TF: base_frame -> each optical frame
        self.tf = StaticTransformBroadcaster(self)
        ts = []
        for child, x in (('left_cam', 0.0), ('right_cam', BASELINE)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'; t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            ts.append(t)
        self.tf.sendTransform(ts)

        self.n = 0
        self.t0 = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        # exact 30 Hz stamps, so there is no jitter for cuVSLAM to reject
        ns = self.t0 + self.n * 33_333_333
        st = rclpy.time.Time(nanoseconds=ns).to_msg()
        for m in (self.iL, self.iR, self.cL, self.cR):
            m.header.stamp = st
        self.pi0.publish(self.iL); self.pc0.publish(self.cL)
        self.pi1.publish(self.iR); self.pc1.publish(self.cR)
        self.n += 1
        if self.n % 60 == 0:
            self.get_logger().info(f'published {self.n} pairs')

rclpy.init()
rclpy.spin(Pub())
