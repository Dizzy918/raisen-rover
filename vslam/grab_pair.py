#!/usr/bin/env python3
"""Grab one rectified mono8 stereo pair from the AR0234 for the cuVSLAM smoke test.
2x2 Bayer bin (never demosaic), rotate 180 + swap for the inverted mount, then
rectify with the 960x600 calibration."""
import subprocess, time, sys
import numpy as np, cv2, yaml

W,H,HALF = 3840,1200,1920
OUT = '/home/narsil/vslam-min/testpair'

def sh(*a): subprocess.run(a, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
def ctl(k,v): sh('v4l2-ctl','-d','/dev/video0','-c','%s=%d'%(k,v))

def load(p):
    d=yaml.safe_load(open(p))
    g=lambda k,r,c: np.array(d[k]['data'],float).reshape(r,c)
    return g('camera_matrix',3,3),g('distortion_coefficients',1,5),g('rectification_matrix',3,3),g('projection_matrix',3,4)

def bin2x2(h16):
    f=h16.astype(np.float32)
    q=(f[0::2,0::2]+f[0::2,1::2]+f[1::2,0::2]+f[1::2,1::2])*0.25
    lo,hi=np.percentile(q,1),np.percentile(q,99.5)
    return np.clip((q-lo)*255.0/max(hi-lo,1),0,255).astype(np.uint8)

KL,DL,R1,P1 = load('/home/narsil/vslam-min/calib/acamst_left.yaml')
KR,DR,R2,P2 = load('/home/narsil/vslam-min/calib/acamst_right.yaml')

cap=cv2.VideoCapture('/dev/video0',cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('B','A','1','0'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH,W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
if not cap.isOpened(): sys.exit('cannot open /dev/video0')
for _ in range(5): cap.read()
ctl('exposure',8000); ctl('analogue_gain',300); time.sleep(0.7)
ok=False
for _ in range(6): ok,buf=cap.read()
if not ok: sys.exit('read failed')
f=buf.reshape(-1).view(np.uint16).reshape(H,W)
L=cv2.rotate(bin2x2(f[:,HALF:]),cv2.ROTATE_180)
R=cv2.rotate(bin2x2(f[:,:HALF]),cv2.ROTATE_180)
ctl('exposure',700); ctl('analogue_gain',100); cap.release()

sz=(L.shape[1],L.shape[0])
mLx,mLy=cv2.initUndistortRectifyMap(KL,DL,R1,P1,sz,cv2.CV_32FC1)
mRx,mRy=cv2.initUndistortRectifyMap(KR,DR,R2,P2,sz,cv2.CV_32FC1)
Lr=cv2.remap(L,mLx,mLy,cv2.INTER_LINEAR)
Rr=cv2.remap(R,mRx,mRy,cv2.INTER_LINEAR)
cv2.imwrite(OUT+'/left_rect.png',Lr); cv2.imwrite(OUT+'/right_rect.png',Rr)

# camera_info for RECTIFIED images: K = P[:3,:3], D = 0, R = I, P unchanged
for side,P in (('left',P1),('right',P2)):
    ci={'image_width':sz[0],'image_height':sz[1],
        'camera_name':'ar0234_%s_rect'%side,
        'camera_matrix':{'rows':3,'cols':3,'data':P[:3,:3].reshape(-1).tolist()},
        'distortion_model':'plumb_bob',
        'distortion_coefficients':{'rows':1,'cols':5,'data':[0.0]*5},
        'rectification_matrix':{'rows':3,'cols':3,'data':np.eye(3).reshape(-1).tolist()},
        'projection_matrix':{'rows':3,'cols':4,'data':P.reshape(-1).tolist()}}
    yaml.safe_dump(ci,open(OUT+'/%s_rect_info.yaml'%side,'w'),sort_keys=False)

# texture check: cuVSLAM needs corners to track
for n,im in (('L',Lr),('R',Rr)):
    k=cv2.goodFeaturesToTrack(im,500,0.01,7)
    print(f'{n}: {im.shape} mean={im.mean():.0f} corners={0 if k is None else len(k)}')
print('saved to', OUT)
