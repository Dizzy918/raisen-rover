#!/usr/bin/env python3
"""Build the base_link -> left_cam transform from the measured floor geometry.

Until now live_vslam.py published this as identity rotation with the baseline as
a translation, i.e. it treated base_link AS the left camera's optical frame. That
is self-consistent, so cuVSLAM worked -- but it means poses came out in an
optical-ish frame where FORWARD IS +Z, and the 16.87 cm height and 2.72 deg tilt
were simply absent.

WHY THIS IS BUILT FROM THE NORMAL, NOT FROM EULER ANGLES

camera_to_base.yaml carries both pitch/roll and the raw upward floor normal in
the optical frame. Composing a rotation from pitch and roll means picking sign
conventions for each -- which way is positive pitch, does roll turn the same way
in an optical frame as in a body frame -- and a sign error there produces a
transform that looks plausible and is wrong. The normal has no convention to get
wrong: it is a measured direction, so use it.

    z_base = the floor normal (up, by measurement)
    x_base = optical forward, projected into the horizontal plane (forward)
    y_base = z cross x                                            (left)

That is REP-103: x forward, y left, z up. base_link's origin sits on the floor
directly beneath the optical centre, so the camera is at (0, 0, height).

WHAT THIS CHANGES DOWNSTREAM. cuVSLAM reports the pose of base_frame, so with a
correct transform its odometry becomes a real rover frame: driving forward
increases X, and turning shows up as yaw about Z. Under the old identity
transform, forward was +Z. Anyone measuring a straight-line test needs to know
which of those they are reading.

STILL MISSING: the camera's fore/aft and lateral offset from the rover's actual
origin. This puts base_link under the CAMERA, not at the chassis centre or the
wheel axis. Height, pitch and roll are right; a ruler from the wheel axis would
finish it. Until then, a pure rotation of the rover about its own centre will
show a small translation here, because base_link is offset from the true centre
of rotation.
"""
import numpy as np
import yaml

DEFAULT_PATH = '/home/narsil/vslam-min/calib/camera_to_base.yaml'


def quat_from_matrix(R):
    """Rotation matrix -> (x, y, z, w).

    Branching on the largest diagonal term avoids the near-zero divisions the
    naive single-branch formula hits at 180 degree rotations.
    """
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def load(path=DEFAULT_PATH):
    """Return (height, R_base_from_opt, quat_xyzw, pitch_deg, roll_deg).

    R_base_from_opt maps a vector expressed in the left camera's OPTICAL frame
    into base_link. It is also, read as columns, the orientation of left_cam
    relative to base_link -- which is exactly what a TransformStamped wants.
    """
    y = yaml.safe_load(open(path))
    height = float(y['height_m'])
    n = np.array(y['floor_normal'], float)
    n = n / np.linalg.norm(n)          # up, in optical coords

    fwd = np.array([0.0, 0.0, 1.0])    # optical +Z is forward
    x_b = fwd - (fwd @ n) * n          # ...projected horizontal
    nx = np.linalg.norm(x_b)
    if nx < 1e-6:
        raise ValueError('camera is pointing straight up or down; '
                         'forward is undefined')
    x_b /= nx
    y_b = np.cross(n, x_b)             # left = up cross forward
    y_b /= np.linalg.norm(y_b)

    # Columns are base_link's axes in optical coords, so this maps base -> optical.
    R_opt_from_base = np.column_stack([x_b, y_b, n])
    R_base_from_opt = R_opt_from_base.T
    return (height, R_base_from_opt, quat_from_matrix(R_base_from_opt),
            float(y['pitch_deg']), float(y['roll_deg']))


if __name__ == '__main__':
    import sys
    h, R, q, pitch, roll = load(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH)
    np.set_printoptions(precision=5, suppress=True)
    print('height %.4f m, pitch %+.2f deg, roll %+.2f deg' % (h, pitch, roll))
    print('R_base_from_opt =\n%s' % R)
    print('quat xyzw = %s' % q)
    print()
    # Sanity: where do the optical axes land in base_link?
    for name, v in (('optical +Z (forward)', [0, 0, 1]),
                    ('optical -Y (up)', [0, -1, 0]),
                    ('optical +X (right)', [1, 0, 0])):
        print('%-22s -> base %s' % (name, R @ np.array(v, float)))
    print('\nexpected: forward ~[1,0,0], up ~[0,0,1], right ~[0,-1,0]')
