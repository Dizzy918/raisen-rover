#!/usr/bin/env python3
"""Is the live rectified pair still epipolar aligned? A 30 second health check.

Run it after anything touches the camera bar, after a reflash, or when depth
starts looking wrong. Calibration RMS cannot catch a knocked mount or a lens ring
that has rotated -- the model still fits the old images perfectly. This measures
the runtime path instead.

    python3 check_rectification.py      # on the Jetson, camera must be free

MEASURED 2026-08-17, known-good baseline to compare against:

                 median dy    MAD    within 2px   disparity
  unrectified      -6.000    0.000        1%      all positive
  rectified        +0.000    1.000       93%      all positive

The unrectified row is the control and it is the important half: it shows the two
sensors sit a systematic 6 px apart vertically, and rectification takes that to
zero. Without the control, a rectified median of 0.0 could just mean the test is
insensitive. The residual MAD of 1.0 px is ORB's integer-pixel localisation on
binned images, not real misalignment.

FAIL LOOKS LIKE: rectified median dy of several px, or any negative disparity.
Negative disparity specifically means the left/right swap is inverted.


Epipolar check, outlier-robust.

The first attempt used crossCheck matching over all octaves and judged on the
mean, which an office full of near-identical chairs and ceiling tiles defeats:
false matches land anywhere vertically and drag the mean and std around while
leaving the median at zero. This version:

  ratio test instead of crossCheck   drops ambiguous matches, which is exactly
                                     what repeated structure produces
  octave 0 only                      higher octaves are localised on blurred
                                     pyramid levels, so their y is worth 2-3 px
  median and MAD, not mean and std   one bad match cannot move them

A correctly rectified pair should give |median dy| well under 0.5 px with a MAD
of about a pixel, and every disparity positive.
"""
import subprocess, sys, time
import cv2, numpy as np, yaml

W, H, HALF = 3840, 1200, 1920
CAL = '/home/narsil/vslam-min/calib'

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

KL,DL,R1,P1 = load(CAL+'/acamst_left.yaml')
KR,DR,R2,P2 = load(CAL+'/acamst_right.yaml')

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('B','A','1','0'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
if not cap.isOpened(): sys.exit('cannot open /dev/video0')
for _ in range(5): cap.read()
for k,v in (('exposure',1500),('analogue_gain',800)):
    subprocess.run(['v4l2-ctl','-d','/dev/video0','-c','%s=%d'%(k,v)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(0.8)
for _ in range(8): ok,buf = cap.read()
cap.release()
if not ok: sys.exit('read failed')
raw = buf.reshape(-1).view(np.uint16).reshape(H,W)

L = cv2.rotate(bin2x2(raw[:,HALF:]), cv2.ROTATE_180)
R = cv2.rotate(bin2x2(raw[:,:HALF]), cv2.ROTATE_180)
sz = (L.shape[1], L.shape[0])
mL = cv2.initUndistortRectifyMap(KL,DL,R1,P1,sz,cv2.CV_32FC1)
mR = cv2.initUndistortRectifyMap(KR,DR,R2,P2,sz,cv2.CV_32FC1)
Lr = cv2.remap(L,*mL,cv2.INTER_LINEAR)
Rr = cv2.remap(R,*mR,cv2.INTER_LINEAR)

def report(a, b, label):
    orb = cv2.ORB_create(4000)
    kA,dA = orb.detectAndCompute(a,None)
    kB,dB = orb.detectAndCompute(b,None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    pairs = bf.knnMatch(dA,dB,k=2)
    dxs,dys = [],[]
    for pr in pairs:
        if len(pr) < 2: continue
        m,n = pr
        if m.distance > 0.75*n.distance: continue      # ratio test
        if kA[m.queryIdx].octave != 0 or kB[m.trainIdx].octave != 0: continue
        pa,pb = kA[m.queryIdx].pt, kB[m.trainIdx].pt
        dxs.append(pa[0]-pb[0]); dys.append(pa[1]-pb[1])
    dxs,dys = np.array(dxs), np.array(dys)
    keep = (dxs > 1) & (dxs < 300) & (np.abs(dys) < 10)
    if keep.sum() < 15:
        print('%s: only %d matches, cannot judge' % (label, keep.sum())); return
    dx,dy = dxs[keep], dys[keep]
    med = np.median(dy)
    mad = np.median(np.abs(dy-med))
    print('%s  n=%d' % (label, keep.sum()))
    print('   dy   median %+.3f px   MAD %.3f   within 1px %.0f%%   within 2px %.0f%%'
          % (med, mad, 100*(np.abs(dy)<1).mean(), 100*(np.abs(dy)<2).mean()))
    print('   dx   median %.1f px  min %.1f  all positive: %s'
          % (np.median(dx), dx.min(), bool((dx>0).all())))

report(Lr, Rr, 'RECTIFIED  ')
# Control: the same match on UNRECTIFIED halves. If rectification is doing real
# work, its dy must be visibly better than this baseline.
report(L, R, 'unrectified')
