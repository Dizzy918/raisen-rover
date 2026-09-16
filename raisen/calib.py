"""Calibration and the camera-to-rover transform: one loader, one convention.

Two separate calibrations feed everything downstream and they answer different
questions:

  acamst_left.yaml / acamst_right.yaml   camera-to-CAMERA. Intrinsics, distortion,
                                         and the rectifying R and P per eye.
  camera_to_base.yaml                    camera-to-ROVER. Where the camera sits
                                         above the floor and how it is tilted.

Without the second one, anything projecting depth into a rover frame is guessing;
before it was measured, the height was a tape reading and the pitch was "a little
bit upwards". See stereo_calibration/out/camera_to_base.yaml for the two
independent methods that agree on it.
"""
import numpy as np
import yaml

from . import frame as _frame


def _load_one(path):
    y = yaml.safe_load(open(path))

    def g(key, rows, cols):
        return np.array(y[key]['data'], float).reshape(rows, cols)

    return (g('camera_matrix', 3, 3), g('distortion_coefficients', 1, 5),
            g('rectification_matrix', 3, 3), g('projection_matrix', 3, 4))


def quat_from_matrix(R):
    """Rotation matrix -> (x, y, z, w).

    Branching on the largest diagonal term rather than using the single-branch
    formula, which divides by near-zero at 180 degree rotations.
    """
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w, x, y, z = (0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


class CameraToBase:
    """The measured camera pose, built FROM THE FLOOR NORMAL, not Euler angles.

    camera_to_base.yaml carries pitch/roll AND the raw upward floor normal in
    optical coordinates. Composing a rotation from pitch and roll means choosing
    a sign convention for each -- which way is positive pitch, does roll turn the
    same way in an optical frame as in a body frame -- and a sign error there
    produces a transform that looks entirely plausible and is wrong. The normal
    is a measured direction with no convention to get wrong, so use it:

        z_base = the floor normal                                  (up)
        x_base = optical forward projected into the horizontal      (forward)
        y_base = z cross x                                         (left)

    That is REP-103, and base_link's origin sits on the floor directly beneath
    the optical centre, so the camera is at (0, 0, height).

    STILL MISSING: the camera's fore/aft and lateral offset from the rover's real
    origin. base_link is under the CAMERA, not at the chassis centre or the wheel
    axis, so a pure rotation of the rover about its own centre shows up here as a
    small translation. Height, pitch and roll are right; a ruler from the wheel
    axis would finish it.
    """

    def __init__(self, path):
        y = yaml.safe_load(open(path))
        self.path = path
        self.height = float(y['height_m'])
        self.pitch_deg = float(y['pitch_deg'])
        self.roll_deg = float(y['roll_deg'])
        self.uncertainty_deg = float(y.get('uncertainty_deg', float('nan')))

        n = np.array(y['floor_normal'], float)
        n = n / np.linalg.norm(n)
        self.normal_opt = n

        fwd = np.array([0.0, 0.0, 1.0])          # optical +Z is forward
        x_b = fwd - (fwd @ n) * n
        nx = np.linalg.norm(x_b)
        if nx < 1e-6:
            raise ValueError('camera points straight up or down; forward is '
                             'undefined in %s' % path)
        x_b /= nx
        y_b = np.cross(n, x_b)
        y_b /= np.linalg.norm(y_b)

        # Columns are base_link's axes in optical coords, so this maps base->optical.
        self.R_base_from_opt = np.column_stack([x_b, y_b, n]).T
        self.quat_xyzw = quat_from_matrix(self.R_base_from_opt)

    def to_base(self, pts_opt):
        """(N,3) points in the left optical frame -> base_link, metres."""
        return pts_opt @ self.R_base_from_opt.T + np.array([0.0, 0.0, self.height])

    def __repr__(self):
        return ('CameraToBase(height=%.4f m, pitch=%+.2f deg, roll=%+.2f deg)'
                % (self.height, self.pitch_deg, self.roll_deg))


class Rectifier:
    """Rectification maps plus the numbers depth is computed from.

    THE ONE EQUATION EVERYTHING DEPENDS ON:  depth = fx_rect * baseline / disparity
    where fx_rect * baseline is exactly abs(P2[0, 3]), so it is read off the
    calibration rather than multiplied together from two separate places.
    """

    def __init__(self, calib_dir, size=None):
        (KL, DL, R1, P1) = _load_one(calib_dir.rstrip('/') + '/acamst_left.yaml')
        (KR, DR, R2, P2) = _load_one(calib_dir.rstrip('/') + '/acamst_right.yaml')
        self.calib_dir = calib_dir
        self.size = size or (_frame.EYE_W, _frame.EYE_H)
        self.P1, self.P2 = P1, P2
        self.fx = float(P1[0, 0])
        self.fy = float(P1[1, 1])
        self.cx = float(P1[0, 2])
        self.cy = float(P1[1, 2])
        self.fxb = float(abs(P2[0, 3]))          # fx_rect * baseline, px*m
        self.baseline = self.fxb / self.fx

        import cv2
        self.mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, self.size, cv2.CV_32FC1)
        self.mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, self.size, cv2.CV_32FC1)

        w, h = self.size
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float32),
                             np.arange(h, dtype=np.float32))
        self.ux = (uu - self.cx) / self.fx
        self.uy = (vv - self.cy) / self.fy

    def rectify(self, left, right):
        import cv2
        return (cv2.remap(left, *self.mL, cv2.INTER_LINEAR),
                cv2.remap(right, *self.mR, cv2.INTER_LINEAR))

    def camera_info_k(self):
        """K for a RECTIFIED image: P[:3,:3], with D = 0 and R = I.

        Handing cuVSLAM rectified pixels with the UNRECTIFIED K makes it warn
        "Falls back to raw camera model" and quietly produce wrong poses. It is a
        warning, not an error, so it is easy to miss.
        """
        return self.P1[:3, :3].copy()

    def __repr__(self):
        return ('Rectifier(%dx%d, fx=%.2f, baseline=%.2f mm, fx*b=%.3f)'
                % (self.size[0], self.size[1], self.fx, self.baseline * 1000,
                   self.fxb))
