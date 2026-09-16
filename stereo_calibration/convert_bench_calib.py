#!/usr/bin/env python3
"""Convert the bench calibration into a pipeline-ready one.

INPUT   1280x720 per eye, images INVERTED (cameras mounted upside down).
OUTPUT  1920x1200 per eye, upright, left/right in the conventional order.

Three composed transformations, in this order:

1. UN-CROP.  1280x720 is the sensor's 2560x720 mode, which is a CROP of
   3840x1200 at native pixel pitch, not a downscale: fx here matches jetson9's
   fx at 1920x1200 to 0.2%, where a downscale would put it near 758. Measured on
   the hardware by locating the small frame inside the large one -- peak
   correlation 0.98/0.99 at x=+320 y=+240, i.e. exactly centred, both even so
   the CFA phase is preserved. So fx,fy are untouched and only the principal
   point moves.  A centred crop commutes with the 180 deg rotation, so the order
   of steps 1 and 2 does not matter.

2. UN-INVERT.  A 180 deg image rotation is the camera rotated 180 deg about its
   optical axis: fx,fy unchanged, cx -> (W-1)-cx, cy -> (H-1)-cy. Radial k1,k2,k3
   are even in r and carry over; tangential p1,p2 are odd under (x,y)->(-x,-y)
   and change sign.

3. SWAP.  Rotating BOTH eyes maps T -> Rz*T with Rz = diag(-1,-1,1), which flips
   T_x from negative to positive. A positive T_x means the eye the bench called
   "left" is the RIGHT eye once the picture is upright. So un-inverting forces a
   left/right swap; doing one without the other yields a calibration whose RMS
   still looks fine and whose geometry is mirrored.

Rectification and projection are RECOMPUTED with cv2.stereoRectify from the
corrected intrinsics and extrinsics -- they cannot be patched in place, since
they depend on the whole geometry. Same call the bench tool used, so the output
stays drop-in: CALIB_ZERO_DISPARITY, alpha=0.
"""
import io, os
import numpy as np, cv2, yaml

import argparse

_ap = argparse.ArgumentParser(description=__doc__)
_ap.add_argument("src", help="directory holding the bench acamst_*.yaml")
_ap.add_argument("--out", default=None,
                 help="output dir for the 1920x1200 profile "
                      "(default: <repo>/stereo_calibration/out_jetson11/1920x1200)")
_ap.add_argument("--crop", default="320,240",
                 help="crop offset of the calibrated mode inside the full frame, "
                      "x,y. MEASURE it with jetson/stereo/crop_offset.py rather "
                      "than assuming the crop is centred.")
_a = _ap.parse_args()

SRC = os.path.join(_a.src, "")
OUT = _a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "out_jetson11", "1920x1200")
CROP_DX, CROP_DY = (int(v) for v in _a.crop.split(","))
FULL_W, FULL_H = 1920, 1200

def m(d):
    return np.array(d["data"], float).reshape(d["rows"], d["cols"])

L = yaml.safe_load(io.open(SRC + "acamst_left.yaml"))
R = yaml.safe_load(io.open(SRC + "acamst_right.yaml"))
E = yaml.safe_load(io.open(SRC + "acamst_extrinsics.yaml"))
Rot, T = m(E["rotation"]), m(E["translation"]).ravel()

def fix_eye(C):
    K = m(C["camera_matrix"]).copy()
    d = np.array(C["distortion_coefficients"]["data"], float).copy()
    K[0, 2] += CROP_DX                       # 1: un-crop
    K[1, 2] += CROP_DY
    K[0, 2] = (FULL_W - 1) - K[0, 2]         # 2: un-invert
    K[1, 2] = (FULL_H - 1) - K[1, 2]
    d[2] = -d[2]                             # p1
    d[3] = -d[3]                             # p2
    return K, d

K_bench_left, D_bench_left = fix_eye(L)
K_bench_right, D_bench_right = fix_eye(R)

Rz = np.diag([-1.0, -1.0, 1.0])
Rr, Tr = Rz @ Rot @ Rz.T, Rz @ T              # both eyes rotated
# 3: swap, so the pair is (left, right) again. Invert the cam1->cam2 transform.
R_new = Rr.T
T_new = -Rr.T @ Tr
K1, D1 = K_bench_right, D_bench_right         # new LEFT  = bench right
K2, D2 = K_bench_left,  D_bench_left          # new RIGHT = bench left

print("corrected, upright, 1920x1200 per eye")
print("  new LEFT  (was bench right)  fx %.4f fy %.4f cx %.4f cy %.4f"
      % (K1[0,0], K1[1,1], K1[0,2], K1[1,2]))
print("  new RIGHT (was bench left)   fx %.4f fy %.4f cx %.4f cy %.4f"
      % (K2[0,0], K2[1,1], K2[0,2], K2[1,2]))
print("  T_new = [%.6f %.6f %.6f]   |T| = %.4f mm   T_x %s"
      % (*T_new, np.linalg.norm(T_new)*1000,
         "NEGATIVE (correct)" if T_new[0] < 0 else "POSITIVE (WRONG)"))
print("  baseline preserved: %.4f mm vs bench %.4f mm"
      % (np.linalg.norm(T_new)*1000, E["baseline_mm"]))

R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K1, D1.reshape(1, -1), K2, D2.reshape(1, -1), (FULL_W, FULL_H),
    R_new, T_new.reshape(3, 1),
    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
B = -P2[0, 3] / P2[0, 0]
print("  rectified fx %.4f   P2[0,3] %.4f  -> baseline %.4f mm  (%s)"
      % (P1[0,0], P2[0,3], B*1000,
         "sign correct" if P2[0,3] < 0 else "SIGN WRONG"))
print("  depth constant fx*B = %.4f  (jetson9's was 87.120 at 960x600)"
      % (P1[0,0]*B))

def yaml_mat(name, M, r, c, pad=0):
    p = " " * pad
    out = [f"{p}{name}:", f"{p}  rows: {r}", f"{p}  cols: {c}", f"{p}  data:"]
    out += [f"{p}  - {v!r}" for v in np.asarray(M).reshape(-1).tolist()]
    return "\n".join(out)

HDR = ("""# Stereo calibration for the jetson11 AR0234 pair, derived from the bench
# calibration of 2026-08-19 13:44:58 UTC (17 views, RMS 0.24627 px).
#
# NOT a straight copy of the bench files. Three corrections were composed:
#   1. un-crop  1280x720 -> 1920x1200. The bench ran in the sensor's 2560x720
#      mode, which is a CENTRED CROP at native pixel pitch, measured on the
#      hardware at offset x=+320 y=+240 (peak correlation 0.98/0.99). fx,fy
#      therefore carry over unchanged; only the principal point moves.
#   2. un-invert  the cameras are mounted upside down, so the bench images were
#      inverted. cx -> (W-1)-cx, cy -> (H-1)-cy; k1,k2,k3 unchanged (even in r);
#      p1,p2 negated (odd under x,y -> -x,-y).
#   3. swap  rotating both eyes flips T_x positive, which means the eye the
#      bench called "left" is the RIGHT eye once upright. Left and right are
#      therefore exchanged here. Doing 2 without 3 gives a plausible RMS and
#      mirrored geometry.
# R and P are recomputed with cv2.stereoRectify (CALIB_ZERO_DISPARITY, alpha=0)
# from the corrected intrinsics; they cannot be patched in place.
#
# Consumers must feed images rotated 180 and left/right swapped, which is what
# CAMERA.md documents and what smart_grab7.py already does.
""")

os.makedirs(OUT, exist_ok=True)
for side, K, D, Rr_, Pp in (("left", K1, D1, R1, P1), ("right", K2, D2, R2, P2)):
    with io.open(os.path.join(OUT, "acamst_%s.yaml" % side), "w") as f:
        f.write(HDR)
        f.write("image_width: %d\nimage_height: %d\n" % (FULL_W, FULL_H))
        f.write("camera_name: ar0234_%s_3840x1200\n" % side)
        f.write(yaml_mat("camera_matrix", K, 3, 3) + "\n")
        f.write("distortion_model: plumb_bob\n")
        f.write(yaml_mat("distortion_coefficients", D, 1, len(D)) + "\n")
        f.write(yaml_mat("rectification_matrix", Rr_, 3, 3) + "\n")
        f.write(yaml_mat("projection_matrix", Pp, 3, 4) + "\n")

with io.open(os.path.join(OUT, "acamst_extrinsics.yaml"), "w") as f:
    f.write(HDR)
    f.write("parent_frame: stereo_left_optical\nchild_frame: stereo_right_optical\n")
    f.write(yaml_mat("rotation", R_new, 3, 3) + "\n")
    f.write(yaml_mat("translation", T_new.reshape(3,1), 3, 1) + "\n")
    f.write("baseline_mm: %.4f\n" % (np.linalg.norm(T_new)*1000))
    f.write("rms_px: %s\n" % E["rms_px"])
    f.write("view_count: %s\n" % E["view_count"])
    f.write("checkerboard:\n  cols: 9\n  rows: 6\n  square_m: 0.0256\n")
    f.write("derived_from: bench 1280x720 inverted, %s\n" % E["measured"])
# ---- also emit the 960x600 binned profile the mapping code consumes --------
# Inverse of the documented scale rule (README: f_full = 2*f_binned,
# c_full = 2*c_binned + 0.5), so the half-pixel the naive x2 drops is restored.
OUT2 = OUT.replace("1920x1200", "960x600")
os.makedirs(OUT2, exist_ok=True)
def halve_K(K):
    K = K.copy()
    K[0,0] /= 2.0; K[1,1] /= 2.0
    K[0,2] = (K[0,2] - 0.5) / 2.0
    K[1,2] = (K[1,2] - 0.5) / 2.0
    return K
def halve_P(P):
    P = P.copy()
    P[0,0] /= 2.0; P[1,1] /= 2.0
    P[0,2] = (P[0,2] - 0.5) / 2.0
    P[1,2] = (P[1,2] - 0.5) / 2.0
    P[0,3] /= 2.0                      # -fx*B scales with fx
    return P
for side, K, D, Rr_, Pp in (("left", K1, D1, R1, P1), ("right", K2, D2, R2, P2)):
    Kb, Pb = halve_K(K), halve_P(Pp)
    with io.open(os.path.join(OUT2, "acamst_%s.yaml" % side), "w") as f:
        f.write(HDR)
        f.write("# Scaled from the 1920x1200 profile: f/2, c = (c-0.5)/2, D and R\n"
                "# unchanged (D is normalised, R is a pure rotation).\n")
        f.write("image_width: 960\nimage_height: 600\n")
        f.write("camera_name: ar0234_%s_960x600\n" % side)
        f.write(yaml_mat("camera_matrix", Kb, 3, 3) + "\n")
        f.write("distortion_model: plumb_bob\n")
        f.write(yaml_mat("distortion_coefficients", D, 1, len(D)) + "\n")
        f.write(yaml_mat("rectification_matrix", Rr_, 3, 3) + "\n")
        f.write(yaml_mat("projection_matrix", Pb, 3, 4) + "\n")
Pb = halve_P(P2)
print("960x600: rectified fx %.4f  P2[0,3] %.4f -> baseline %.4f mm  depth const %.4f"
      % (Pb[0,0], Pb[0,3], -Pb[0,3]/Pb[0,0]*1000, -Pb[0,3]))
print("\nwrote %s" % OUT)
print("wrote %s" % OUT2)
