#!/usr/bin/env python3
"""Feed the live AR0234 stereo pair into cuVSLAM.

Runs INSIDE vslam-min:3.2 (needs rclpy + /dev/video0). Per frame:
  split the side-by-side raw -> 2x2 Bayer bin to 960x600/eye (never demosaic)
  -> rotate 180 and swap for the inverted mount -> rectify -> publish mono8.

Publishes exactly what cuVSLAM subscribes to:
  visual_slam/image_0        left,  mono8, RECTIFIED
  visual_slam/camera_info_0  K = P[:3,:3], D = 0, R = I  (rectified convention)
  visual_slam/image_1        right
  visual_slam/camera_info_1
  static TF base_link -> left_cam, right_cam   (cuVSLAM reads rig geometry here)

FOUR THINGS THAT WILL BITE, all learned the hard way:

  mono8 ONLY. Hand cuVSLAM bgr8 and it throws "Received unknown image encoding:
  bgr8". Documented in poc-isaac-ros's vslam_ar0234.launch.py header.

  EXPOSURE IS A FRAME-RATE CEILING. It is in sensor row-times (~14.57 us), so
  exposure=8000 is 117 ms/frame and caps the CAMERA at 5.8 fps regardless of
  anything downstream. Measured: 8000 -> 5.8 fps, 4000 -> 11.5, 2290 -> 20.1,
  1500 -> 30.6. Longer exposure is also worse for stereo, because it blows out
  bright regions. 1500 is both fastest and best; buy brightness with gain.

  RECTIFIED camera_info, not raw. Passing rectified pixels with the unrectified
  K makes cuVSLAM warn "Falls back to raw camera model" and quietly produce
  wrong poses. So D = 0, R = I, K = P[:3,:3], and rectified_images:=true.

  A MISSING TOPIC HANGS IT. cuVSLAM blocks forever waiting on image_0 rather
  than erroring, so if nothing appears, suspect the publisher, not cuVSLAM.

narsil-vision holds /dev/video0 -- stop it before running this.
"""
import argparse
import array
import time

import camera_extrinsics as extrinsics
import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

W, H, HALF = 3840, 1200, 1920
EYE_W, EYE_H = HALF // 2, H // 2


def load_calib(d):
    def one(p):
        y = yaml.safe_load(open(p))
        g = lambda k, r, c: np.array(y[k]['data'], float).reshape(r, c)
        return (g('camera_matrix', 3, 3), g('distortion_coefficients', 1, 5),
                g('rectification_matrix', 3, 3), g('projection_matrix', 3, 4))
    return one(d + '/acamst_left.yaml'), one(d + '/acamst_right.yaml')


def bin2x2(half16):
    """2x2 Bayer bin -> uint8. Contrast limits from a strided subsample: the full
    np.percentile cost ~31 ms per eye on the Jetson and changed the result by
    under a grey level."""
    f = half16.astype(np.float32)
    q = (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25
    lo, hi = np.percentile(q[::6, ::6], (1, 99))
    return np.clip((q - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)


class LiveStereo(Node):
    def __init__(self, a):
        super().__init__('ar0234_live_stereo')
        (KL, DL, R1, P1), (KR, DR, R2, P2) = load_calib(a.calib)
        sz = (EYE_W, EYE_H)
        self.mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, sz, cv2.CV_32FC1)
        self.mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, sz, cv2.CV_32FC1)
        self.baseline = abs(P2[0, 3]) / P1[0, 0]
        self.get_logger().info(
            f'rectified fx={P1[0,0]:.2f} baseline={self.baseline*1000:.2f} mm')

        self.ci = {}
        for side, P, frame in (('0', P1, 'left_cam'), ('1', P2, 'right_cam')):
            m = CameraInfo()
            m.width, m.height = EYE_W, EYE_H
            m.distortion_model = 'plumb_bob'
            m.k = [float(x) for x in P[:3, :3].reshape(-1)]
            m.d = [0.0] * 5
            m.r = [float(x) for x in np.eye(3).reshape(-1)]
            m.p = [float(x) for x in P.reshape(-1)]
            m.header.frame_id = frame
            self.ci[side] = m

        self.pub = {
            'i0': self.create_publisher(Image, 'visual_slam/image_0', 10),
            'c0': self.create_publisher(CameraInfo, 'visual_slam/camera_info_0', 10),
            'i1': self.create_publisher(Image, 'visual_slam/image_1', 10),
            'c1': self.create_publisher(CameraInfo, 'visual_slam/camera_info_1', 10),
        }

        # cuVSLAM takes the stereo rig geometry from TF, not from camera_info.
        #
        # Published as a CHAIN: base_link -> left_cam -> right_cam. The rig offset
        # is trivial in the optical frame (the baseline is just +X), so expressing
        # it there keeps the measured floor geometry in exactly one place.
        self.tfb = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        ts = []

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'left_cam'
        if a.legacy_tf:
            # Identity: base_link IS the optical frame, so forward is +Z and the
            # camera's height and tilt are absent. Kept only to reproduce runs
            # recorded before the extrinsics were measured.
            t.transform.rotation.w = 1.0
            self.get_logger().warn('LEGACY TF: base_link == optical frame, '
                                   'forward is +Z, height and tilt ignored')
        else:
            height, _, q, pitch, roll = extrinsics.load(a.extrinsics)
            t.transform.translation.z = float(height)
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.get_logger().info(
                'TF base_link->left_cam from measured floor geometry: '
                f'height {height*1000:.1f} mm, pitch {pitch:+.2f} deg, '
                f'roll {roll:+.2f} deg. Odometry is now a ROVER frame: '
                'forward is +X, up is +Z, turning is yaw about Z.')
        ts.append(t)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'left_cam'
        t.child_frame_id = 'right_cam'
        t.transform.translation.x = float(self.baseline)
        t.transform.rotation.w = 1.0
        ts.append(t)

        self.tfb.sendTransform(ts)

        self.cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        if not self.cap.isOpened():
            raise SystemExit(f'cannot open {a.device} -- is narsil-vision still running?')
        for _ in range(5):
            self.cap.read()
        # v4l-utils is NOT in the minimal image (the drone Dockerfile installed it;
        # this one deliberately does not), so tolerate its absence -- the runner
        # sets exposure and gain from the host instead.
        import subprocess
        try:
            for k, v in (('exposure', a.exposure), ('analogue_gain', a.gain)):
                subprocess.run(['v4l2-ctl', '-d', a.device, '-c', f'{k}={v}'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f'camera open, set exposure={a.exposure} '
                                   f'gain={a.gain}')
        except FileNotFoundError:
            self.get_logger().warn('v4l2-ctl not in this image; relying on the '
                                   'host having set exposure/gain already')
        time.sleep(0.6)

        self.n = 0
        self.t0 = time.time()
        self.tspent = 0.0
        self.create_timer(0.001, self.tick)

    def tick(self):
        ok, buf = self.cap.read()
        if not ok or buf is None:
            return
        t = time.time()
        try:
            frame = buf.reshape(-1).view(np.uint16).reshape(H, W)
        except Exception:
            return
        L = cv2.rotate(bin2x2(frame[:, HALF:]), cv2.ROTATE_180)
        R = cv2.rotate(bin2x2(frame[:, :HALF]), cv2.ROTATE_180)
        Lr = cv2.remap(L, *self.mL, cv2.INTER_LINEAR)
        Rr = cv2.remap(R, *self.mR, cv2.INTER_LINEAR)

        stamp = self.get_clock().now().to_msg()
        for key, ci, img in (('0', self.ci['0'], Lr), ('1', self.ci['1'], Rr)):
            m = Image()
            m.height, m.width = img.shape
            m.encoding = 'mono8'
            m.is_bigendian = 0
            m.step = img.shape[1]
            # array.array('B'), NOT img.tobytes(). Assigning raw bytes to a ROS
            # uint8[] field takes 96 ms for a 960x600 image in rclpy -- two of
            # them per frame was 193 ms of the 210 ms budget and pinned the
            # publisher at 4.5 fps. array.array matches the field's internal type
            # and assigns in ~0 ms. publish() itself is 0.17 ms.
            m.data = array.array('B', img.tobytes())
            m.header.stamp = stamp
            m.header.frame_id = ci.header.frame_id
            ci.header.stamp = stamp
            self.pub['i' + key].publish(m)
            self.pub['c' + key].publish(ci)

        self.tspent += time.time() - t
        self.n += 1
        if self.n % 60 == 0:
            el = time.time() - self.t0
            self.get_logger().info(
                f'{self.n} pairs, {self.n/el:.1f} fps published, '
                f'{self.tspent/self.n*1000:.1f} ms/frame processing')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', default='/workspaces/calib')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--exposure', type=int, default=1500,
                    help='sensor row-times; THIS CAPS THE FRAME RATE')
    ap.add_argument('--gain', type=int, default=800)
    ap.add_argument('--extrinsics', default='',
                    help='camera_to_base.yaml; defaults to <calib>/camera_to_base.yaml')
    ap.add_argument('--legacy-tf', action='store_true',
                    help='publish the old identity transform, where base_link is '
                         'the optical frame and forward is +Z')
    a = ap.parse_args()
    # Derive from --calib rather than using the module default. That default is a
    # HOST path, and this file runs inside the container where /home/narsil/vslam-min
    # is mounted at /workspaces -- so the host path does not exist and the node died
    # on a FileNotFoundError after cuVSLAM had already started, which reads as "no
    # camera" rather than "wrong path".
    if not a.extrinsics:
        a.extrinsics = a.calib.rstrip('/') + '/camera_to_base.yaml'
    rclpy.init()
    node = LiveStereo(a)
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()


if __name__ == '__main__':
    main()
