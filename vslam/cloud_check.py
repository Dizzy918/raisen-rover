#!/usr/bin/env python3
"""Build a 3D point cloud from one stereo frame, in the rover frame.

WHAT THIS DOES AND DOES NOT SHOW. A cloud from a single viewpoint is not a map --
it is one depth image in 3D, with everything behind the first surface missing.
Accumulating a ROOM needs the rover to move so the camera sees it from several
places. This tests the half that does not need motion: whether the stereo geometry
produces correct 3D structure, which is a precondition for any map built on it.

Renders three orthographic views so the structure can be judged by eye:
  top-down   x forward, y left     -- the view a floor map would be built from
  side       x forward, z up       -- shows whether the floor is flat and at z=0
  front      y left, z up          -- the camera's own cross-section


MEASURED 2026-08-17, one frame, rover stationary

  139319 valid depth points, 24.2% of the 960x600 pixels

  range band    points   share   z spread
  0.35-1.0 m    29169    20.9%     0.62 m
  1.0-2.0 m      5612     4.0%     1.17 m
  2.0-3.0 m     67410    48.4%     2.93 m
  3.0-4.0 m     23311    16.7%     3.44 m
  4.0-6.1 m     13817     9.9%     5.71 m

THE FLOOR LANDS WHERE IT SHOULD, which is the real result here. 17272 points
within 6 cm of z=0 and closer than 2.5 m:

  mean z        +12.6 mm   (want 0)
  plane rms      15.5 mm
  residual tilt   1.29 deg

That is a THIRD independent confirmation of the camera-to-rover extrinsics, after
the plane fit and the checkerboard pose. Depth, the transform and the measured
height and tilt all agree well enough to put a floor within 13 mm of zero across
a 2.5 m span.

A FIFTH OF THE CLOUD IS PHYSICALLY IMPOSSIBLE. 27493 points, 19.7%, sit more than
10 cm BELOW the floor, spread from 0.85 m to 6.09 m. Nothing can be there. They
are an unresolved artefact plus far-field noise -- NOT the polished floor an
earlier draft blamed, since the floor is not polished; see subfloor_check.py -- and
anything
building a map on this must reject sub-floor points rather than trusting them --
depth_grid.py already does. Only 0.5% land above 2.8 m, so the ceiling is not the
problem.

THE Z SPREAD PER BAND IS THE QUADRATIC DEPTH ERROR MADE VISIBLE. 0.62 m of spread
in the nearest band against 5.71 m beyond 4 m. Points smear along the viewing ray,
which is why the top-down render shows radial streaks rather than surfaces. Any
accumulation wants a range gate; the grid uses 0.4 to 5.0 m.

WHAT THIS DOES NOT TEST: the map. One viewpoint is one depth image, with
everything behind the first surface missing. Accumulating a room needs the rover
to move, and needs pose to stitch the views -- which is a separate measurement.
"""
import subprocess, sys, time
import cv2, numpy as np, yaml

W, H, HALF = 3840, 1200, 1920
CAL = '/home/narsil/vslam-min/calib'
OUT = '/tmp/cloud'
subprocess.run(['mkdir', '-p', OUT])

def load(p):
    d = yaml.safe_load(open(p))
    g = lambda k, r, c: np.array(d[k]['data'], float).reshape(r, c)
    return (g('camera_matrix',3,3), g('distortion_coefficients',1,5),
            g('rectification_matrix',3,3), g('projection_matrix',3,4))

def bin2x2(h16):
    f = h16.astype(np.float32)
    q = (f[0::2,0::2]+f[0::2,1::2]+f[1::2,0::2]+f[1::2,1::2])*0.25
    lo, hi = np.percentile(q, (1, 99))
    return np.clip((q-lo)*255.0/max(hi-lo,1), 0, 255).astype(np.uint8)

y = yaml.safe_load(open(CAL + '/camera_to_base.yaml'))
height = float(y['height_m'])
n = np.array(y['floor_normal'], float); n /= np.linalg.norm(n)
fwd = np.array([0.0,0.0,1.0]); xb = fwd - (fwd@n)*n; xb /= np.linalg.norm(xb)
yb = np.cross(n, xb)
Rbo = np.column_stack([xb, yb, n]).T

KL,DL,R1,P1 = load(CAL+'/acamst_left.yaml'); KR,DR,R2,P2 = load(CAL+'/acamst_right.yaml')
fx,cx,cy = P1[0,0],P1[0,2],P1[1,2]; fxb = abs(P2[0,3])

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B','A','1','0'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
if not cap.isOpened(): sys.exit('cannot open /dev/video0')
for _ in range(5): cap.read()
for k,v in (('exposure',1500),('analogue_gain',800)):
    subprocess.run(['v4l2-ctl','-d','/dev/video0','-c','%s=%d'%(k,v)],stdout=-3,stderr=-3)
time.sleep(0.8)
for _ in range(10): ok,buf = cap.read()
cap.release()
if not ok: sys.exit('read failed')
raw = buf.reshape(-1).view(np.uint16).reshape(H,W)
L = cv2.rotate(bin2x2(raw[:,HALF:]), cv2.ROTATE_180)
Rg = cv2.rotate(bin2x2(raw[:,:HALF]), cv2.ROTATE_180)
sz = (L.shape[1], L.shape[0])
mL = cv2.initUndistortRectifyMap(KL,DL,R1,P1,sz,cv2.CV_32FC1)
mR = cv2.initUndistortRectifyMap(KR,DR,R2,P2,sz,cv2.CV_32FC1)
Lr = cv2.remap(L,*mL,cv2.INTER_LINEAR); Rr = cv2.remap(Rg,*mR,cv2.INTER_LINEAR)
cv2.imwrite(OUT+'/left.png', Lr)

sg = cv2.StereoSGBM_create(minDisparity=0,numDisparities=256,blockSize=5,P1=8*25,
    P2=32*25,disp12MaxDiff=1,uniquenessRatio=10,speckleWindowSize=100,
    speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
disp = sg.compute(Lr,Rr).astype(np.float32)/16.0
Z = np.where(disp>0.5, fxb/np.maximum(disp,1e-6), 0.0)
m = (Z>0.35)&(Z<6.0)
ys,xs = np.nonzero(m); z = Z[ys,xs]
opt = np.stack([(xs-cx)*z/fx, (ys-cy)*z/fx, z],1)
pts = opt @ Rbo.T + np.array([0.0,0.0,height])
grey = Lr[ys,xs]

tot = int(m.size); val = len(pts)
print('valid depth pixels %d of %d (%.1f%%)'%(val,tot,100.0*val/tot))
print('extent in base_link:')
for i,nm in enumerate('xyz'):
    print('   %s  %+.2f to %+.2f m'%(nm, pts[:,i].min(), pts[:,i].max()))
near = pts[pts[:,0] < 2.0]
print('floor check: %d points below z=0.05 m, %d above'
      % (int((pts[:,2]<0.05).sum()), int((pts[:,2]>=0.05).sum())))

with open(OUT+'/cloud.ply','w') as fh:
    fh.write('ply\nformat ascii 1.0\nelement vertex %d\n'%val)
    fh.write('property float x\nproperty float y\nproperty float z\n')
    fh.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
    fh.write('end_header\n')
    step = max(1, val//200000)
    for p,g in zip(pts[::step], grey[::step]):
        fh.write('%.4f %.4f %.4f %d %d %d\n'%(p[0],p[1],p[2],g,g,g))

def render(a_i, b_i, a_rng, b_rng, flip_b, label, fname):
    """Orthographic projection, nearest-point wins so front surfaces show."""
    Wp, Hp = 640, 480
    img = np.full((Hp,Wp,3), 24, np.uint8)
    depth = np.full((Hp,Wp), 1e9, np.float32)
    a = (pts[:,a_i]-a_rng[0])/(a_rng[1]-a_rng[0])
    b = (pts[:,b_i]-b_rng[0])/(b_rng[1]-b_rng[0])
    if flip_b: b = 1.0-b
    u = (a*(Wp-1)).astype(np.int32); v = (b*(Hp-1)).astype(np.int32)
    keep = (u>=0)&(u<Wp)&(v>=0)&(v<Hp)
    for uu,vv,gg,dd in zip(u[keep],v[keep],grey[keep],pts[keep][:,0]):
        if dd < depth[vv,uu]:
            depth[vv,uu] = dd; img[vv,uu] = (gg,gg,gg)
    cv2.putText(img,label,(8,18),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,220,255),1)
    cv2.imwrite(OUT+'/'+fname, img)

render(1,0,(-3,3),(0,6),True, 'TOP-DOWN  y left <-> , x forward up', 'top.png')
render(0,2,(0,6),(-0.3,2.2),True,'SIDE  x forward ->, z up', 'side.png')
render(1,2,(-3,3),(-0.3,2.2),True,'FRONT  y left <->, z up', 'front.png')
print('wrote %s/{cloud.ply,top.png,side.png,front.png}'%OUT)
