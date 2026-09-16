"""Disparity and depth, with the search-range trap made structural.

THE TRAP. A block matcher cannot report a disparity above numDisparities-1, so
anything nearer than

    fx * baseline / numDisparities

is CLAMPED to that limit and comes back as a plausible-looking depth that is
really just the edge of the search window. It has bitten this project twice:

  numDisparities=128  ->  floor 0.681 m.  An ArUco marker at 0.43 m reported
                          0.749 m. Not obviously wrong, which is why it survived.
  numDisparities=176  ->  floor 0.495 m, while depth_grid.py's own Z_MIN was
                          0.40 m -- so the grid trusted a 0.40-0.495 m band it
                          structurally could not measure, and that band is
                          exactly the one that matters for not hitting things.

Fixing it by choosing a bigger number is not a fix, it is the same bug with a
different constant. So make_matcher takes the NEAREST DEPTH YOU INTEND TO TRUST
and derives the search range from it, then refuses to return a matcher that
cannot reach it. stereo_calibration/range_sweep.py is the empirical companion:
it looks for pixels railed against the top of the window on a real frame.
"""
import numpy as np

BLOCK = 5


def range_floor_m(fxb, num_disp):
    """Nearest depth a given search range can express, metres."""
    return fxb / float(num_disp)


def disparities_for(fxb, z_min):
    """Smallest multiple of 16 whose range floor reaches z_min."""
    need = fxb / float(z_min)
    return int(np.ceil(need / 16.0)) * 16


class Matcher:
    """SGBM plus the two numbers a caller must not have to look up separately.

    A thin wrapper because cv2.StereoSGBM permits no attribute assignment, but it
    earns its keep: num_disp and floor_m travel WITH the matcher, so a consumer
    can report the near limit it is actually working under instead of assuming it.
    """

    def __init__(self, sgbm, num_disp, floor_m, z_min):
        self.sgbm = sgbm
        self.num_disp = num_disp
        self.floor_m = floor_m
        self.z_min = z_min

    def compute(self, left_rect, right_rect):
        """Disparity in pixels, float32. SGBM returns fixed-point 1/16ths."""
        return self.sgbm.compute(left_rect, right_rect).astype(np.float32) / 16.0

    def __repr__(self):
        return ('Matcher(numDisparities=%d, near limit %.3f m, z_min %.3f m)'
                % (self.num_disp, self.floor_m, self.z_min))


def make_matcher(fxb, z_min, block=BLOCK, mode=None):
    """StereoSGBM sized from the nearest depth to be trusted.

    Raises rather than silently truncating: a matcher that cannot see as close as
    the caller intends to trust is not a degraded matcher, it is one that reports
    confident wrong numbers in the range the caller cares most about.
    """
    import cv2
    if mode is None:
        mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
    nd = disparities_for(fxb, z_min)
    floor = range_floor_m(fxb, nd)
    if floor > z_min + 1e-9:
        raise ValueError('numDisparities=%d only reaches %.3f m, but z_min is '
                         '%.3f m' % (nd, floor, z_min))
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=nd, blockSize=block,
        P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=2, mode=mode)
    return Matcher(sgbm, nd, floor, z_min)


def depth_from_disparity(disp, fxb, z_min, z_max):
    """(depth, valid_mask). Depth is 0 where invalid rather than inf or nan."""
    with np.errstate(divide='ignore', invalid='ignore'):
        z = fxb / disp
    ok = (disp > 0.5) & np.isfinite(z) & (z > z_min) & (z < z_max)
    return np.where(ok, z, 0.0), ok


def railed_fraction(disp, num_disp):
    """Share of valid disparities sitting at the top of the search window.

    The empirical signature of truncation. A range wide enough for the scene has
    almost none; a range that is too narrow piles pixels against its ceiling.
    """
    ok = disp > 0.5
    if not ok.any():
        return 0.0
    return float((ok & (disp >= num_disp - 1.5)).sum()) / float(ok.sum())


def points_optical(disp, rect, z_min, z_max):
    """Rectified disparity -> (N,3) points in the LEFT OPTICAL frame.

    Unprojected straight from the rectified intrinsics rather than through a Q
    matrix. Q's sign conventions are the easiest thing here to get subtly wrong,
    and a flipped sign yields a mirrored cloud that still looks plausible.
    Optical frame is x right, y down, z forward.
    """
    z, ok = depth_from_disparity(disp, rect.fxb, z_min, z_max)
    zz = z[ok]
    return np.stack([rect.ux[ok] * zz, rect.uy[ok] * zz, zz], axis=1), ok
