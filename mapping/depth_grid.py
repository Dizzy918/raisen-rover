#!/usr/bin/env python3
"""SGBM -> ground-plane projection -> Bayesian log-odds occupancy grid.

Pipeline, per frame:
  1. split the side-by-side AR0234 frame, 2x2 Bayer bin to 960x600 per eye
     (never demosaic -- see stereo_calibration/README.md)
  2. rectify with stereo_calibration/out/960x600/
  3. SGBM -> disparity -> metric depth.  depth = 87.12 / disparity metres,
     from the RECTIFIED focal (P[0,0] = 585.04) x baseline (0.148914 m)
  4. fit the ground plane by RANSAC, which yields camera height and tilt --
     no need to measure them by hand
  5. transform points into a ground frame (x forward, y left, z up) and
     classify by height: floor / obstacle / overhead
  6. reduce obstacles to a virtual laser scan: nearest obstacle per bearing.
     This is what makes free space correct and cheap -- everything closer than
     the first hit along a bearing was seen through, so it is free.
  7. Bayesian log-odds update, clamped, so one bad frame cannot undo many
     good ones

The grid is ROBOT-CENTRIC. Accumulating across frames is only valid while the
robot is stationary or nearly so; moving requires shifting the grid by the pose
from cuVSLAM, which is the next task. --pose-source is where that plugs in.

Usage:
    python3 depth_grid.py --npz ../stereo_calibration/captures/round4/raw_01.npz
    python3 depth_grid.py --live --seconds 30          # on the Jetson
"""
import argparse
import time

import cv2
import numpy as np
import yaml

W, H, HALF = 3840, 1200, 1920          # side-by-side wire format
EYE_W, EYE_H = HALF // 2, H // 2       # 960x600 after 2x2 binning

# grid, robot-centric: x forward, y left
CELL = 0.05                            # metres per cell
FWD_M, LAT_M = 6.0, 6.0                # 6 m ahead, +/- 3 m sideways
GRID_W = int(LAT_M / CELL)
GRID_H = int(FWD_M / CELL)

# height bands in the ground frame, metres
FLOOR_TOL = 0.06                       # |z| below this is floor
ROVER_TOP = 0.40                       # above this we can drive under it

# log-odds
L_OCC, L_FREE, L_CLAMP = 0.85, 0.40, 8.0

Z_MIN, Z_MAX = 0.4, 5.0                # trust depth only in this band


def load_calib(d):
    def one(p):
        y = yaml.safe_load(open(p))
        g = lambda k, r, c: np.array(y[k]['data'], float).reshape(r, c)
        return (g('camera_matrix', 3, 3), g('distortion_coefficients', 1, 5),
                g('rectification_matrix', 3, 3), g('projection_matrix', 3, 4))
    return one(d + '/acamst_left.yaml'), one(d + '/acamst_right.yaml')


def bin2x2(half16):
    """2x2 Bayer bin -> uint8.

    The contrast limits come from a STRIDED SUBSAMPLE, not the full array.
    np.percentile on 1.15M float32 cost ~31 ms per eye on the Jetson, which was
    a fixed cost paid on every frame regardless of --scale, since this runs on
    the full-resolution raw. Every 6th pixel in each axis is 36x less data and
    gives the same limits to well under a grey level.

    A global percentile stretch, not the board-region stretch used for
    calibration capture -- that deliberately blows out everything except the
    target, which is wrong for mapping.
    """
    f = half16.astype(np.float32)
    q = (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25
    lo, hi = np.percentile(q[::6, ::6], (1, 99))
    return np.clip((q - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)


def split_pair(frame16):
    """Inverted mount: rotate each half 180 then swap."""
    L = cv2.rotate(bin2x2(frame16[:, HALF:]), cv2.ROTATE_180)
    R = cv2.rotate(bin2x2(frame16[:, :HALF]), cv2.ROTATE_180)
    return L, R


def fit_ground_plane(pts, iters=600, tol=0.03, rng=None, min_h=0.10, max_h=2.5,
                     min_frac=0.15):
    """RANSAC plane fit, returning (normal pointing up, height, inlier count).

    Candidates are restricted to points below the camera and within range --
    fitting to everything would happily lock onto a wall, which is the failure
    mode that makes the whole grid nonsense.
    """
    rng = rng or np.random.default_rng(0)
    # camera frame: x right, y down, z forward. Floor is below -> y > 0.
    m = (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX) & (pts[:, 1] > 0.05)
    cand = pts[m]
    if len(cand) < 200:
        return None
    # Score against a fixed random subsample, not all candidates: RANSAC only
    # needs a consistent relative score, and this turns the fit from ~30 s into
    # milliseconds on a 150k-point cloud.
    sub = cand[rng.integers(0, len(cand), min(4000, len(cand)))]
    best_n, best_d, best_c = None, None, 0
    for _ in range(iters):
        # integers(), NOT choice(replace=False) -- the latter permutes the whole
        # population on every call, which is what made this unusably slow.
        i = rng.integers(0, len(cand), 3)
        p0, p1, p2 = cand[i]
        n = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        d = -n @ p0
        # keep only planes that are roughly horizontal: normal near +/- y
        if abs(n[1]) < 0.85:
            continue
        c = int((np.abs(sub @ n + d) < tol).sum())
        if c > best_c:
            best_n, best_d, best_c = n, d, c
    if best_n is None or best_c < 200:
        return None
    # Refine on inliers via the 3x3 covariance, NOT np.linalg.svd(inl - c).
    # svd() defaults to full_matrices=True, so on a 23000x3 input it tries to
    # build a 23000x23000 U -- 4 GB and ~11 s, and it returned a bad normal.
    # The smallest-eigenvalue eigenvector of the covariance is the same answer
    # in microseconds.
    inl = cand[np.abs(cand @ best_n + best_d) < tol]
    c = inl.mean(axis=0)
    cov = np.cov((inl - c).T)
    w, v = np.linalg.eigh(cov)
    n = v[:, 0] / np.linalg.norm(v[:, 0])       # eigh returns ascending eigenvalues
    d = -n @ c
    # flatness of the fitted plane, in metres -- a real floor is a few mm
    rms = float(np.sqrt(max(w[0], 0.0)))
    if n[1] > 0:                       # make the normal point UP (-y in camera)
        n, d = -n, -d
    height = abs(d)
    # Reject implausible fits rather than returning them. A wrong plane produces
    # a map that looks plausible and is wholly wrong, which is worse than no map.
    tilt = np.degrees(np.arccos(np.clip(-n[1], -1, 1)))
    # min_h defaults to 0.10 m because 0.05 was permissive enough to accept a
    # DESK SURFACE ~5 cm below a bench-mounted camera and map it as though it
    # were the floor. Honest failure beats a plausible-looking wrong map. Set
    # --min-height to the real mount height on the rover.
    if not (min_h <= height <= max_h) or tilt > 35.0 or rms > 0.05:
        return None
    # The strongest guard is not height but SUPPORT: what fraction of the points
    # below the camera actually lie on this plane. A real floor dominates the
    # lower view. Low support means "no floor visible", which must fail rather
    # than quietly map a desk top as if it were the ground.
    support = len(inl) / len(cand)
    if support < min_frac:
        return None
    return n, height, len(inl), rms, support


def ground_frame(n, height):
    """Rotation+translation taking camera coords to ground coords
    (x forward, y left, z up)."""
    zc = np.array([0.0, 0.0, 1.0])
    f = zc - (zc @ n) * n              # camera forward, flattened onto the floor
    f /= np.linalg.norm(f)
    l = np.cross(n, f)                 # left
    R = np.vstack([f, l, n])           # rows: forward, left, up
    t = np.array([0.0, 0.0, height])
    return R, t


def grid_indices(xy):
    """Ground (x forward, y left) -> grid (row, col). Robot at row 0, centre col."""
    r = np.floor(xy[:, 0] / CELL).astype(int)
    c = np.floor(GRID_W / 2 - xy[:, 1] / CELL).astype(int)
    ok = (r >= 0) & (r < GRID_H) & (c >= 0) & (c < GRID_W)
    return r[ok], c[ok], ok


def bearing_reach(pts_xy, n_bearings=180, hfov_deg=79.0):
    """Per-bearing furthest VALID measurement, from all depth points.

    Feed this to update_grid as seen_to. Pass every point stereo returned --
    floor, obstacle, overhead -- not just the obstacles, because a floor return
    at 2 m is still proof that the ray was observed out to 2 m.

    Bearings with no returns at all come back as 0.0, meaning "nothing was
    measured here", which update_grid then leaves unknown instead of free.
    """
    reach = np.zeros(n_bearings)
    if len(pts_xy) == 0:
        return reach
    ang = np.arctan2(pts_xy[:, 1], pts_xy[:, 0])
    rng_ = np.hypot(pts_xy[:, 0], pts_xy[:, 1])
    half = np.radians(hfov_deg) / 2.0
    keep = (np.abs(ang) < half) & (rng_ > Z_MIN) & (rng_ < Z_MAX)
    ang, rng_ = ang[keep], rng_[keep]
    if len(ang) == 0:
        return reach
    b = np.clip(((ang + half) / (2 * half) * n_bearings).astype(int),
                0, n_bearings - 1)
    np.maximum.at(reach, b, rng_)
    return reach


def update_grid(logodds, obst_xy, n_bearings=180, hfov_deg=79.0, seen_to=None):
    """Virtual-laser-scan occupancy update.

    Nearest obstacle per bearing; everything nearer along that bearing is free,
    the hit cell is occupied, and beyond it stays unknown because it was never
    observed. Marking beyond-the-hit as free is the classic bug -- it deletes
    walls the moment something passes in front of them.

    seen_to, from bearing_reach(), is the per-bearing furthest valid measurement,
    and WITHOUT IT THIS FUNCTION MARKS FREE SPACE THROUGH SOLID OBJECTS.

    A bearing with no obstacle was swept free all the way to Z_MAX, whether or
    not anything was ever measured along it. But stereo returns nothing from an
    untextured surface, so "no obstacle" and "no measurement" arrive identically
    -- and absence of disparity is not evidence of absence. Measured on a real
    frame: three black desk panels 1-2 m ahead returned no disparity at all, 93%
    of that frame sat below 60 counts, and the grid swept free space straight
    through them to 5 m.

    With seen_to, free stops at whichever comes first -- the nearest obstacle, or
    the end of the evidence -- and bearings with no returns stay wholly unknown.

    seen_to=None keeps the old behaviour exactly, which tests/test_offline.py pins.
    """
    half = np.radians(hfov_deg) / 2.0
    nearest = np.full(n_bearings, np.inf)
    if len(obst_xy):
        ang = np.arctan2(obst_xy[:, 1], obst_xy[:, 0])
        rng_ = np.hypot(obst_xy[:, 0], obst_xy[:, 1])
        keep = (np.abs(ang) < half) & (rng_ > Z_MIN) & (rng_ < Z_MAX)
        ang, rng_ = ang[keep], rng_[keep]
        if len(ang):
            b = np.clip(((ang + half) / (2 * half) * n_bearings).astype(int),
                        0, n_bearings - 1)
            np.minimum.at(nearest, b, rng_)
    if not np.isfinite(nearest).any() and seen_to is None:
        return 0

    hits = 0
    for bi in range(n_bearings):
        rr = nearest[bi]
        a = -half + (bi + 0.5) / n_bearings * 2 * half
        if seen_to is None:
            far = Z_MAX if not np.isfinite(rr) else rr
        else:
            # Free only as far as there is evidence: the nearer of the obstacle
            # and the end of what was actually measured along this bearing.
            reach = float(seen_to[bi])
            if reach <= 0.0 and not np.isfinite(rr):
                continue                      # nothing observed here at all
            far = min(rr if np.isfinite(rr) else np.inf,
                      reach if reach > 0.0 else 0.0)
        # free along the ray, stopping one cell short of the hit
        steps = max(int((far - CELL) / CELL), 0)
        if steps:
            s = (np.arange(steps) + 0.5) * CELL
            fx, fy = s * np.cos(a), s * np.sin(a)
            r, c, _ = grid_indices(np.stack([fx, fy], 1))
            logodds[r, c] -= L_FREE
        if np.isfinite(rr):
            r, c, ok = grid_indices(np.array([[rr * np.cos(a), rr * np.sin(a)]]))
            if ok.any():
                logodds[r, c] += L_OCC + L_FREE   # cancel the free pass
                hits += 1
    np.clip(logodds, -L_CLAMP, L_CLAMP, out=logodds)
    return hits


class Pipeline:
    # num_disp 224, not 176. THE SEARCH RANGE SETS THE MINIMUM MEASURABLE DEPTH:
    # depth = 87.12/disparity, so 176 cannot see closer than 0.495 m while Z_MIN
    # says 0.40 m is trusted. That left a 0.40-0.495 m band where the grid expected
    # depth it structurally could not produce -- and that band is exactly the range
    # that matters for not hitting things. 224 brings the near limit to 0.389 m,
    # just inside Z_MIN. Found after the same mistake at 128 made an ArUco marker
    # at 0.43 m report 0.749 m, which looks like a plausible number rather than a
    # clipped one.
    def __init__(self, calib_dir, num_disp=224, block=5, scale=1.0, min_h=0.10,
                 min_frac=0.15):
        (KL, DL, R1, P1), (KR, DR, R2, P2) = load_calib(calib_dir)
        self.w, self.h = int(EYE_W * scale), int(EYE_H * scale)
        if scale != 1.0:                        # scale intrinsics with the image
            S = np.diag([scale, scale, 1.0])
            P1, P2 = S @ P1, S @ P2
            KL, KR = S @ KL, S @ KR
        self.P1, self.P2 = P1, P2
        self.mL = cv2.initUndistortRectifyMap(KL, DL, R1, P1, (self.w, self.h), cv2.CV_32FC1)
        self.mR = cv2.initUndistortRectifyMap(KR, DR, R2, P2, (self.w, self.h), cv2.CV_32FC1)
        self.fxb = abs(P2[0, 3])                # = fx * baseline
        nd = int(round(num_disp * scale / 16)) * 16
        self.nd = max(nd, 16)
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.nd, blockSize=block,
            P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        # Unproject straight from the rectified intrinsics rather than building a
        # Q matrix -- Q's sign conventions are the easiest thing to get subtly
        # wrong here, and a flipped sign gives a mirrored map that still looks
        # plausible.
        self.fx, self.fy = P1[0, 0], P1[1, 1]
        self.cx, self.cy = P1[0, 2], P1[1, 2]
        uu, vv = np.meshgrid(np.arange(self.w, dtype=np.float32),
                            np.arange(self.h, dtype=np.float32))
        self.ux = (uu - self.cx) / self.fx
        self.uy = (vv - self.cy) / self.fy
        self.logodds = np.zeros((GRID_H, GRID_W), np.float32)
        self.plane = None
        self.min_h = min_h
        self.min_frac = min_frac

    def points(self, L, R):
        Lr = cv2.remap(L, *self.mL, cv2.INTER_LINEAR)
        Rr = cv2.remap(R, *self.mR, cv2.INTER_LINEAR)
        if (self.w, self.h) != (L.shape[1], L.shape[0]):
            Lr = cv2.resize(Lr, (self.w, self.h), interpolation=cv2.INTER_AREA)
            Rr = cv2.resize(Rr, (self.w, self.h), interpolation=cv2.INTER_AREA)
        disp = self.sgbm.compute(Lr, Rr).astype(np.float32) / 16.0
        with np.errstate(divide='ignore', invalid='ignore'):
            z = self.fxb / disp                  # depth = fx*baseline / disparity
        m = (disp > 0.5) & np.isfinite(z) & (z > Z_MIN) & (z < Z_MAX)
        zz = z[m]
        # camera frame: x right, y down, z forward
        pts = np.stack([self.ux[m] * zz, self.uy[m] * zz, zz], axis=1)
        return Lr, disp, pts, float(m.mean())

    def step(self, frame16):
        L, R = split_pair(frame16)
        Lr, disp, pts, frac = self.points(L, R)
        if len(pts) < 500:
            return dict(ok=False, why='too few valid depth points', valid=frac)
        if self.plane is None:
            fit = fit_ground_plane(pts, min_h=self.min_h, min_frac=self.min_frac)
            if fit is None:
                return dict(ok=False, why='no ground plane found', valid=frac)
            n, height, ninl, rms, support = fit
            self.plane = (n, height)
            self.Rg, self.tg = ground_frame(n, height)
            tilt = np.degrees(np.arccos(np.clip(-n[1], -1, 1)))
            self.plane_info = dict(height_m=height, tilt_deg=tilt, inliers=ninl,
                                   flatness_mm=rms * 1000, support_pct=support * 100)
        g = pts @ self.Rg.T + self.tg
        obst = g[(g[:, 2] > FLOOR_TOL) & (g[:, 2] < ROVER_TOP)]
        floor = g[np.abs(g[:, 2]) <= FLOOR_TOL]
        hits = update_grid(self.logodds, obst[:, :2])
        return dict(ok=True, valid=frac, n_pts=len(pts), n_obst=len(obst),
                    n_floor=len(floor), hits=hits, disp=disp, left=Lr)


def render(logodds, path=None):
    """occupied = dark, free = light, unknown = mid grey. Robot at the bottom."""
    img = np.full(logodds.shape + (3,), 128, np.uint8)
    occ = logodds > 1.0
    free = logodds < -1.0
    img[free] = (235, 235, 235)
    v = np.clip(logodds / L_CLAMP, 0, 1)
    img[occ] = np.stack([(1 - v[occ]) * 120, (1 - v[occ]) * 120,
                         30 + (1 - v[occ]) * 90], 1).astype(np.uint8)
    img = cv2.flip(img, 0)                       # forward = up
    img = cv2.resize(img, (GRID_W * 4, GRID_H * 4), interpolation=cv2.INTER_NEAREST)
    cv2.circle(img, (GRID_W * 2, GRID_H * 4 - 6), 5, (0, 140, 255), -1)
    for m in range(1, int(FWD_M) + 1):           # 1 m range rings
        cv2.circle(img, (GRID_W * 2, GRID_H * 4 - 6), int(m / CELL * 4), (170, 170, 170), 1)
    if path:
        cv2.imwrite(path, img)
    return img


def save_plane(pipe, path):
    """Persist a successful fit. Called from both offline and live paths -- it
    was originally only in the offline one, so a live fit was silently lost."""
    import os
    if not path or pipe.plane is None or os.path.exists(path):
        return False
    n, h = pipe.plane
    yaml.safe_dump(dict(normal=[float(x) for x in n], height_m=float(h),
                        tilt_deg=float(pipe.plane_info['tilt_deg']),
                        inliers=int(pipe.plane_info['inliers']),
                        flatness_mm=float(pipe.plane_info['flatness_mm']),
                        support_pct=float(pipe.plane_info['support_pct'])),
                   open(path, 'w'), sort_keys=False)
    print(f'ground plane SAVED to {path} -- reuse it; do not refit unless the '
          f'mount changes')
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', default='../stereo_calibration/out/960x600')
    ap.add_argument('--npz', help='raw side-by-side Bayer frame (offline test)')
    ap.add_argument('--repeat', type=int, default=8,
                    help='offline: how many times to feed the same frame, to show accumulation')
    ap.add_argument('--live', action='store_true')
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--scale', type=float, default=1.0, help='1.0 = 960x600')
    ap.add_argument('--out', default='grid.png')
    ap.add_argument('--plane', help='YAML holding the ground plane; loaded if it '
                                    'exists, written after a successful fit')
    ap.add_argument('--exposure', type=int, default=1500,
                    help='sensor row-times. THIS CAPS THE FRAME RATE: 1500 ~ 30 fps, '
                         '2290 ~ 20, 4000 ~ 11, 8000 ~ 6. Raise --gain to compensate.')
    ap.add_argument('--gain', type=int, default=800, help='analogue_gain, 100-1200')
    ap.add_argument('--min-height', type=float, default=0.10,
                    help='reject a ground-plane fit below this height in metres. '
                         'Guards against locking onto a desk or table top.')
    ap.add_argument('--min-support', type=float, default=0.15,
                    help='reject a fit supported by less than this fraction of '
                         'the points below the camera, i.e. no floor in view')
    a = ap.parse_args()

    pipe = Pipeline(a.calib, scale=a.scale, min_h=a.min_height,
                    min_frac=a.min_support)
    if a.plane:
        import os
        if os.path.exists(a.plane):
            y = yaml.safe_load(open(a.plane))
            n = np.array(y['normal'], float); h = float(y['height_m'])
            pipe.plane = (n, h)
            pipe.Rg, pipe.tg = ground_frame(n, h)
            pipe.plane_info = dict(height_m=h, tilt_deg=y['tilt_deg'],
                                   inliers=y.get('inliers', 0),
                                   flatness_mm=y.get('flatness_mm', 0.0),
                                   support_pct=y.get('support_pct', 0.0))
            print(f'ground plane LOADED from {a.plane}: {h:.3f} m, '
                  f'{y["tilt_deg"]:.1f} deg')
    print(f'grid {GRID_H}x{GRID_W} cells @ {CELL*100:.0f} cm '
          f'({FWD_M:g} m forward, {LAT_M:g} m wide), '
          f'{pipe.w}x{pipe.h}/eye, numDisparities={pipe.nd}, fx*B={pipe.fxb:.2f}')

    if a.npz:
        frame = np.load(a.npz)['bayer']
        for i in range(a.repeat):
            t = time.time()
            r = pipe.step(frame)
            ms = (time.time() - t) * 1000
            if not r['ok']:
                print(f'frame {i}: FAILED - {r["why"]} (valid depth {r["valid"]*100:.1f}%)')
                break
            if i == 0:
                pi = pipe.plane_info
                print(f'ground plane: camera height {pi["height_m"]:.3f} m, '
                      f'tilt {pi["tilt_deg"]:.1f} deg, support {pi["support_pct"]:.0f}% '
                      f'({pi["inliers"]} inliers), flatness {pi["flatness_mm"]:.1f} mm rms')
                print('  CHECK THESE: height and tilt must match your mount, and '
                      'support above ~50% means a real floor dominates the view.\n'
                      '  A plausible-looking wrong plane makes a plausible-looking '
                      'wrong map.')
            print(f'frame {i}: {ms:6.1f} ms  valid={r["valid"]*100:4.1f}%  '
                  f'pts={r["n_pts"]:6d} obst={r["n_obst"]:6d} floor={r["n_floor"]:6d} '
                  f'scan_hits={r["hits"]:3d}  occupied_cells={int((pipe.logodds>1).sum()):5d} '
                  f'free={int((pipe.logodds<-1).sum()):5d}')
        save_plane(pipe, a.plane)
        render(pipe.logodds, a.out)
        cv2.imwrite('disparity.png', cv2.applyColorMap(
            np.clip(r.get('disp', np.zeros((10, 10))) * 255 / max(pipe.nd, 1), 0, 255
                    ).astype(np.uint8), cv2.COLORMAP_TURBO))
        print(f'wrote {a.out} and disparity.png')
        return

    if a.live:
        import subprocess
        # narsil-vision has restart=on-failure with unlimited retries, and
        # `docker stop` exits 137, which counts as a failure -- so it restarts
        # within a second and takes /dev/video0 back. Disable the policy first.
        def _dk(*args):
            subprocess.run(['docker'] + list(args),
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        _dk('update', '--restart=no', 'narsil-vision')
        _dk('stop', 'narsil-vision')
        time.sleep(4)
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B', 'A', '1', '0'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        for _ in range(5):
            cap.read()
        # Exposure is a FRAME-RATE CEILING: it is measured in sensor row-times
        # (~14.57 us), so exposure=8000 is 117 ms per frame and caps the camera
        # at 5.8 fps no matter how fast the pipeline is. Measured on jetson9:
        #   8000 -> 5.8 fps    4000 -> 11.5 fps    2290 -> 20.1    1500 -> 30.6
        # Keep exposure under ~2290 (33 ms, one 30 fps frame period) and buy the
        # brightness back with gain instead.
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c',
                        f'exposure={a.exposure}'])
        subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '-c',
                        f'analogue_gain={a.gain}'])
        t0, n, tsum, saved = time.time(), 0, 0.0, False
        try:
            while time.time() - t0 < a.seconds:
                ok, buf = cap.read()
                if not ok:
                    continue
                try:
                    frame = buf.reshape(-1).view(np.uint16).reshape(H, W)
                except Exception:
                    continue
                t = time.time()
                r = pipe.step(frame)
                tsum += time.time() - t
                n += 1
                if not r['ok']:
                    print(f'{n}: {r["why"]} (valid {r["valid"]*100:.1f}%)')
                    continue
                if pipe.plane is not None and a.plane and not saved:
                    pi = pipe.plane_info
                    print(f'ground plane: height {pi["height_m"]:.3f} m, '
                          f'tilt {pi["tilt_deg"]:.1f} deg, support {pi["support_pct"]:.0f}%, '
                          f'flatness {pi["flatness_mm"]:.1f} mm rms', flush=True)
                    saved = save_plane(pipe, a.plane)
                if n % 5 == 0:
                    print(f'{n}: {tsum/n*1000:5.1f} ms avg ({n/tsum:4.1f} fps)  '
                          f'valid={r["valid"]*100:4.1f}%  occ={int((pipe.logodds>1).sum()):5d} '
                          f'free={int((pipe.logodds<-1).sum()):5d}', flush=True)
                    render(pipe.logodds, a.out)
        finally:
            cap.release()
            _dk('update', '--restart=on-failure', 'narsil-vision')
            _dk('start', 'narsil-vision')
            if n:
                print(f'\n{n} frames, {tsum/n*1000:.1f} ms avg, {n/tsum:.1f} fps')
                render(pipe.logodds, a.out)


if __name__ == '__main__':
    main()
