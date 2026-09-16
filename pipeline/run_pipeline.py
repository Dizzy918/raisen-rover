#!/usr/bin/env python3
"""One process, one camera, several consumers.

WHAT THIS REPLACES. live_vslam.py published stereo for cuVSLAM and depth_grid.py
built an occupancy grid, and each opened /dev/video0 itself -- so they could not
both run. Running the grid meant not running vSLAM. This owns the camera once and
fans the frames out:

    StereoCamera --> FrameHub --+--> auto-exposure      (on the capture thread)
                                +--> vSLAM publisher    (mono8 + camera_info + TF)
                                +--> occupancy grid     (SGBM -> log-odds)
                                +--> preview snapshots  (JPEG for eyeballing)

fuse_heading.py stays a SEPARATE process on purpose: it only ever wanted the
odometry topic and the ESP32, never the camera, so it was never part of the
contention. run_pipeline.sh starts it alongside.

THREE THINGS THIS FIXES BEYOND THE CONTENTION

  AUTO-EXPOSURE NOW APPLIES TO vSLAM AND MAPPING. Both previously hardcoded
  exposure=1500 gain=800 because the only AE implementation lived in the live
  preview. AE runs on the capture thread here, so every consumer gets the same
  metered frame -- and it meters the RAW frame, before bin2x2's percentile stretch
  hides what the sensor actually saw.

  ONE SOURCE OF GROUND GEOMETRY. depth_grid.py fit its own ground plane per run
  while vSLAM used the measured camera_to_base.yaml -- two answers to one
  question. On this floor the fit is the weaker of the two: something puts a lot of
  points below the floor, and at a 5 mm tolerance "largest plane" flipped between
  the real floor and that lower surface from frame to frame. The mechanism is
  unresolved -- see vslam/subfloor_check.py -- but the instability is measured. --ground measured is
  the default; --ground fit reproduces the old behaviour.

  THE SEARCH RANGE FOLLOWS Z_MIN. numDisparities is derived from the nearest depth
  to be trusted rather than typed in, so the 0.40 m / 0.495 m mismatch cannot be
  reintroduced by editing one number and not the other.

WHAT IS NOT VERIFIED. Written while jetson9 and rover-01 were powered down. The
offline path (--npz) is tested against real captured frames; the live path, the
ROS publisher and the container-side auto-exposure writes are NOT yet run on
hardware. See pipeline/README.txt for the acceptance checks to run first.
"""
import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.dirname(_HERE), os.path.join(os.path.dirname(_HERE), 'mapping'),
           '/workspaces', '/workspaces/mapping', '/home/narsil/vslam-min'):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from raisen import calib as rcalib          # noqa: E402
from raisen import camera as rcamera        # noqa: E402
from raisen import depth as rdepth          # noqa: E402
from raisen import exposure as rexposure    # noqa: E402
from raisen import frame as rframe          # noqa: E402
from raisen import hub as rhub              # noqa: E402
from raisen import service as rservice      # noqa: E402


def _load_grid_module():
    """depth_grid.py holds the tested log-odds update; reuse it, do not restate it.

    Its ray-cast update has one subtlety worth not reimplementing by accident:
    cells BEYOND the nearest hit stay unknown rather than free, because marking
    them free deletes walls the moment something passes in front of them.
    """
    try:
        import depth_grid
        return depth_grid
    except ImportError:
        return None


class GridConsumer:
    """Depth -> obstacles in the rover frame -> occupancy grid.

    Runs at its own rate. The grid does not need 31 Hz -- it integrates, so a few
    frames a second is plenty -- and decimating here keeps SGBM off the capture
    thread's budget. Frames it skips are counted by the hub as missed rather than
    quietly dropped.
    """

    def __init__(self, rect, c2b, dg, z_min=0.40, z_max=5.0, ground='measured',
                 block=rdepth.BLOCK, legacy_freespace=False):
        self.rect = rect
        self.c2b = c2b
        self.dg = dg
        self.z_min, self.z_max = z_min, z_max
        self.ground = ground
        self.legacy_freespace = legacy_freespace
        self.matcher = rdepth.make_matcher(rect.fxb, z_min, block=block)
        self.num_disp = self.matcher.num_disp
        self.logodds = np.zeros((dg.GRID_H, dg.GRID_W), np.float32)
        self.n = 0
        self.n_obst = 0
        self.railed = 0.0
        self.valid = 0.0
        self.fit = None          # only used when ground == 'fit'
        self.last_left = None
        self.unseen = 0.0
        self.below_total = 0

    def _to_ground(self, pts_opt):
        if self.ground == 'measured':
            return self.c2b.to_base(pts_opt)
        if self.fit is None:
            got = self.dg.fit_ground_plane(pts_opt)
            if got is None:
                return None
            n, height = got[0], got[1]
            self.fit = self.dg.ground_frame(n, height)
        Rg, tg = self.fit
        return pts_opt @ Rg.T + tg

    def step(self, frame16, eyes=None):
        L, R = eyes if eyes is not None else rframe.split_pair(frame16)
        Lr, Rr = self.rect.rectify(L, R)
        self.last_left = Lr
        disp = self.matcher.compute(Lr, Rr)
        pts, ok = rdepth.points_optical(disp, self.rect, self.z_min, self.z_max)
        self.valid = float(ok.mean())
        self.railed = rdepth.railed_fraction(disp, self.num_disp)
        if len(pts) < 500:
            return dict(ok=False, why='too few valid depth points',
                        valid=self.valid)
        g = self._to_ground(pts)
        if g is None:
            return dict(ok=False, why='no ground plane found', valid=self.valid)
        obst = g[(g[:, 2] > self.dg.FLOOR_TOL) & (g[:, 2] < self.dg.ROVER_TOP)]
        floor = g[np.abs(g[:, 2]) <= self.dg.FLOOR_TOL]
        # Evidence, not just obstacles: every return proves its own ray was seen,
        # so the floor counts. Without this, bearings that measured nothing get
        # swept free to Z_MAX and the grid reports free space through whatever
        # was too dark or too blank to match.
        seen_to = (None if self.legacy_freespace
                   else self.dg.bearing_reach(g[:, :2]))
        # Sub-floor points are physically impossible and this floor makes a lot of
        # them: a fifth of a single-frame cloud sat more than 10 cm BELOW the floor.
        # Cause unresolved (vslam/subfloor_check.py). Counted so a run that is mostly
        # artefact is visible rather than merely wrong.
        below = int((g[:, 2] < -0.10).sum())
        hits = self.dg.update_grid(self.logodds, obst[:, :2], seen_to=seen_to)
        self.n += 1
        self.n_obst += len(obst)
        self.below_total += below
        self.unseen = (0.0 if seen_to is None
                       else float((np.asarray(seen_to) <= 0.0).mean()))
        return dict(ok=True, valid=self.valid, railed=self.railed,
                    n_pts=len(pts), n_obst=len(obst), n_floor=len(floor),
                    n_below=below, hits=hits, unseen=self.unseen,
                    left=Lr, disp=disp)

    def run(self, sub, stop, hz, out_png, save_every):
        period = 1.0 / max(hz, 0.1)
        nxt = time.time()
        while not stop.is_set():
            item = sub.get(timeout=0.5)
            if item is None:
                continue
            now = time.time()
            if now < nxt:
                continue
            nxt = now + period
            r = self.step(item[1], item[2])
            if r['ok'] and save_every and self.n % save_every == 0:
                self.dg.render(self.logodds, out_png)


class PreviewConsumer:
    """Periodic JPEG snapshots, so a headless run can still be eyeballed.

    Deliberately not an MJPEG server: jetson/stereo/live_feed.py already is one,
    and it is the tool to reach for when someone is watching. This is for the
    unattended case, where the question afterwards is "what was it looking at".
    """

    def __init__(self, out_dir, quality=80):
        self.out_dir = out_dir
        self.quality = quality
        self.n = 0
        os.makedirs(out_dir, exist_ok=True)

    def run(self, sub, stop, hz):
        period = 1.0 / max(hz, 0.01)
        nxt = time.time()
        while not stop.is_set():
            item = sub.get(timeout=0.5)
            if item is None:
                continue
            if time.time() < nxt:
                continue
            nxt = time.time() + period
            L, R = item[2]
            cv2.imwrite(os.path.join(self.out_dir, 'left.jpg'), L,
                        [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            cv2.imwrite(os.path.join(self.out_dir, 'pair.jpg'),
                        np.hstack([L, R]),
                        [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            self.n += 1


def build_publisher(rect, c2b, legacy_tf):
    """The cuVSLAM-facing node, or None if rclpy is not importable.

    Kept behind a function so the grid and preview still run on a host without
    ROS -- which is also what makes the offline tests possible.
    """
    try:
        import array

        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import StaticTransformBroadcaster
    except ImportError:
        return None

    class StereoPublisher(Node):
        def __init__(self):
            super().__init__('raisen_stereo_publisher')
            self.rect = rect
            self.sub = None
            self.n = 0
            self.tspent = 0.0
            self.t0 = time.time()

            self.ci = {}
            K = rect.camera_info_k()
            for side, P, frame_id in (('0', rect.P1, 'left_cam'),
                                      ('1', rect.P2, 'right_cam')):
                m = CameraInfo()
                m.width, m.height = rect.size
                m.distortion_model = 'plumb_bob'
                # RECTIFIED convention: K = P[:3,:3], D = 0, R = I. Passing the
                # unrectified K with rectified pixels makes cuVSLAM warn "Falls
                # back to raw camera model" and quietly produce wrong poses.
                m.k = [float(x) for x in (K if side == '0'
                                          else P[:3, :3]).reshape(-1)]
                m.d = [0.0] * 5
                m.r = [float(x) for x in np.eye(3).reshape(-1)]
                m.p = [float(x) for x in P.reshape(-1)]
                m.header.frame_id = frame_id
                self.ci[side] = m

            self.pub = {
                'i0': self.create_publisher(Image, 'visual_slam/image_0', 10),
                'c0': self.create_publisher(CameraInfo, 'visual_slam/camera_info_0', 10),
                'i1': self.create_publisher(Image, 'visual_slam/image_1', 10),
                'c1': self.create_publisher(CameraInfo, 'visual_slam/camera_info_1', 10),
            }

            # cuVSLAM reads the rig geometry from TF, not from camera_info.
            # Published as a chain base_link -> left_cam -> right_cam: the rig
            # offset is trivial in the optical frame (baseline is just +X), so
            # expressing it there keeps the measured floor geometry in one place.
            self.tfb = StaticTransformBroadcaster(self)
            stamp = self.get_clock().now().to_msg()
            ts = []
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'left_cam'
            if legacy_tf:
                t.transform.rotation.w = 1.0
                self.get_logger().warn('LEGACY TF: base_link == optical frame, '
                                       'forward is +Z, height and tilt ignored')
            else:
                t.transform.translation.z = float(c2b.height)
                q = c2b.quat_xyzw
                t.transform.rotation.x = float(q[0])
                t.transform.rotation.y = float(q[1])
                t.transform.rotation.z = float(q[2])
                t.transform.rotation.w = float(q[3])
                self.get_logger().info(
                    'TF from measured floor geometry: height %.1f mm, pitch '
                    '%+.2f deg, roll %+.2f deg. Odometry is a ROVER frame: '
                    'forward +X, up +Z, yaw about Z.'
                    % (c2b.height * 1000, c2b.pitch_deg, c2b.roll_deg))
            ts.append(t)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'left_cam'
            t.child_frame_id = 'right_cam'
            t.transform.translation.x = float(rect.baseline)
            t.transform.rotation.w = 1.0
            ts.append(t)
            self.tfb.sendTransform(ts)

            self.create_timer(0.001, self.tick)

        def attach(self, sub):
            self.sub = sub

        def tick(self):
            if self.sub is None:
                return
            item = self.sub.get(timeout=0.0)
            if item is None:
                return
            t = time.time()
            L, R = item[2]
            Lr, Rr = self.rect.rectify(L, R)
            stamp = self.get_clock().now().to_msg()
            for key, img in (('0', Lr), ('1', Rr)):
                ci = self.ci[key]
                m = Image()
                m.height, m.width = img.shape
                m.encoding = 'mono8'        # mono8 ONLY; bgr8 is rejected outright
                m.is_bigendian = 0
                m.step = img.shape[1]
                # array.array('B'), NOT img.tobytes(). Assigning raw bytes to a ROS
                # uint8[] takes 96 ms per 960x600 image in rclpy -- two of those was
                # 193 ms of a 210 ms budget and pinned the publisher at 4.5 fps.
                m.data = array.array('B', img.tobytes())
                m.header.stamp = stamp
                m.header.frame_id = ci.header.frame_id
                ci.header.stamp = stamp
                self.pub['i' + key].publish(m)
                self.pub['c' + key].publish(ci)
            self.tspent += time.time() - t
            self.n += 1
            if self.n % 90 == 0:
                el = time.time() - self.t0
                self.get_logger().info('%d pairs, %.1f fps published, %.1f '
                                       'ms/frame' % (self.n, self.n / el,
                                                     self.tspent / self.n * 1000))

    import rclpy
    rclpy.init()
    return StereoPublisher()


def offline(a, rect, c2b, dg):
    """Run the grid path on a saved frame. No camera, no ROS, no rover."""
    d = np.load(a.npz)
    key = 'bayer' if 'bayer' in d.files else d.files[0]
    frame16 = d[key]
    gc = GridConsumer(rect, c2b, dg, a.z_min, a.z_max, a.ground,
                      legacy_freespace=a.legacy_freespace)
    print('%s  %s  numDisparities=%d (floor %.3f m)'
          % (a.npz, frame16.shape, gc.num_disp, rect.fxb / gc.num_disp))
    for i in range(a.repeat):
        r = gc.step(frame16)
        if not r['ok']:
            print('  frame %d NOT OK: %s (valid %.1f%%)'
                  % (i, r['why'], 100 * r['valid']))
            return 1
    print('  valid depth      %.1f%% of pixels' % (100 * r['valid']))
    print('  railed at range  %.2f%%' % (100 * r['railed']))
    print('  points           %d' % r['n_pts'])
    print('  obstacles        %d in the %.2f-%.2f m height band'
          % (r['n_obst'], dg.FLOOR_TOL, dg.ROVER_TOP))
    print('  floor            %d within %.0f mm of z=0'
          % (r['n_floor'], dg.FLOOR_TOL * 1000))
    print('  sub-floor        %d more than 100 mm below the floor' % r['n_below'])
    # Thresholds match depth_grid.render's, deliberately: occupied is logodds
    # > 1.0, not > 0. Counting at > 0 calls a single glancing hit "occupied" and
    # inflates the number against every figure recorded so far.
    occ = int((gc.logodds > 1.0).sum())
    free = int((gc.logodds < 0).sum())
    print('  grid             %d occupied, %d free, %d unknown of %d cells'
          % (occ, free, gc.logodds.size - occ - free, gc.logodds.size))
    if a.out:
        dg.render(gc.logodds, a.out)
        cv2.imwrite(a.out.replace('.png', '_left.png'), r['left'])
        print('  wrote            %s' % a.out)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', default='/workspaces/calib')
    ap.add_argument('--extrinsics', default='',
                    help='camera_to_base.yaml; defaults to <calib>/camera_to_base.yaml')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--seconds', type=float, default=0.0, help='0 = run forever')
    ap.add_argument('--z-min', type=float, default=0.40,
                    help='nearest depth to trust; SETS numDisparities')
    ap.add_argument('--z-max', type=float, default=5.0)
    ap.add_argument('--ground', choices=['measured', 'fit'], default='measured')
    ap.add_argument('--legacy-freespace', action='store_true',
                    help='sweep free to Z_MAX on bearings that measured nothing, '
                         'as the grid did before evidence bounding')
    ap.add_argument('--grid-hz', type=float, default=5.0)
    ap.add_argument('--preview-hz', type=float, default=1.0)
    ap.add_argument('--out', default='/tmp/pipeline/grid.png')
    ap.add_argument('--preview-dir', default='/tmp/pipeline')
    ap.add_argument('--save-every', type=int, default=10)
    ap.add_argument('--no-vslam', action='store_true')
    ap.add_argument('--no-grid', action='store_true')
    ap.add_argument('--no-preview', action='store_true')
    ap.add_argument('--legacy-tf', action='store_true')
    ap.add_argument('--exposure', type=int, default=1500)
    ap.add_argument('--gain', type=int, default=800)
    ap.add_argument('--auto-exposure', action='store_true',
                    help='meter every frame instead of holding exposure fixed')
    ap.add_argument('--stop-vision', action='store_true',
                    help='stop narsil-vision for the run and restore it after')
    ap.add_argument('--npz', help='offline: run the grid path on a saved frame')
    ap.add_argument('--repeat', type=int, default=3,
                    help='offline: integrate the same frame this many times')
    a = ap.parse_args()
    if not a.extrinsics:
        a.extrinsics = a.calib.rstrip('/') + '/camera_to_base.yaml'

    dg = _load_grid_module()
    if dg is None:
        sys.exit('cannot import depth_grid.py -- it holds the log-odds update. '
                 'Put mapping/ on PYTHONPATH.')
    rect = rcalib.Rectifier(a.calib)
    c2b = rcalib.CameraToBase(a.extrinsics)
    print(rect)
    print(c2b)

    if a.npz:
        sys.exit(offline(a, rect, c2b, dg))

    ctx = rservice.vision_stopped() if a.stop_vision else _nullctx()
    with ctx:
        cam = rcamera.StereoCamera(a.device, a.exposure, a.gain)
        ae = None
        if a.auto_exposure:
            ae = rexposure.AutoExposure(a.device, min_fps=20.0,
                                        exposure=a.exposure, gain=a.gain)
        hub = rhub.FrameHub(cam, on_frame=(ae.update if ae else None))

        stop = threading.Event()
        threads = []
        gc = pub = pv = None

        if not a.no_grid:
            gc = GridConsumer(rect, c2b, dg, a.z_min, a.z_max, a.ground,
                              legacy_freespace=a.legacy_freespace)
            print('grid: numDisparities=%d, near limit %.3f m, ground=%s'
                  % (gc.num_disp, rect.fxb / gc.num_disp, a.ground))
            s = hub.subscribe('grid')
            threads.append(threading.Thread(
                target=gc.run, args=(s, stop, a.grid_hz, a.out, a.save_every),
                daemon=True, name='grid'))

        if not a.no_preview:
            pv = PreviewConsumer(a.preview_dir)
            s = hub.subscribe('preview')
            threads.append(threading.Thread(
                target=pv.run, args=(s, stop, a.preview_hz), daemon=True,
                name='preview'))

        if not a.no_vslam:
            pub = build_publisher(rect, c2b, a.legacy_tf)
            if pub is None:
                print('rclpy not importable -- running WITHOUT the vSLAM '
                      'publisher. Inside vslam-min this means something is '
                      'wrong; on the host it is expected.')
            else:
                pub.attach(hub.subscribe('vslam'))

        hub.start()
        for t in threads:
            t.start()
        t_end = time.time() + a.seconds if a.seconds else None
        try:
            if pub is not None:
                import rclpy
                while rclpy.ok() and (t_end is None or time.time() < t_end):
                    rclpy.spin_once(pub, timeout_sec=0.05)
            else:
                while t_end is None or time.time() < t_end:
                    time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            hub.stop()
            for t in threads:
                t.join(timeout=2.0)
            print()
            print(hub.report())
            if ae is not None:
                print('auto-exposure: %s' % (ae.state(),))
            if gc is not None and gc.n:
                print('grid: %d frames, %d obstacle points, last valid %.1f%%, '
                      'railed %.2f%%' % (gc.n, gc.n_obst, 100 * gc.valid,
                                         100 * gc.railed))
                print('  bearings with NO measurement: %.0f%% of 180 -- left '
                      'unknown%s' % (100 * gc.unseen,
                                     ' (LEGACY: swept free instead)'
                                     if a.legacy_freespace else ''))
                print('  sub-floor points (below the floor, cause open): %d over '
                      '%d frames'
                      % (gc.below_total, gc.n))
                dg.render(gc.logodds, a.out)
                print('grid image: %s' % a.out)
            if pv is not None:
                print('preview: %d snapshots in %s' % (pv.n, pv.preview_dir
                                                       if hasattr(pv, 'preview_dir')
                                                       else a.preview_dir))
            cam.close()


class _nullctx:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


if __name__ == '__main__':
    main()
