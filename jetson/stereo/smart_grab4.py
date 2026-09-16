#!/usr/bin/env python3
# Final stereo-pair collector for MATLAB calibration.
#  - ONE persistent OpenCV/V4L2 stream (repeated open/close wedges the Tegra VI channel)
#  - exposure set MID-STREAM (this driver ignores it before streaming starts)
#  - corrects the inverted mount: rotate each half 180 deg, then SWAP halves
#    (verified by disparity sign: +19.3px swapped vs -19.4px unswapped)
#  - keeps only frames with the 9x6 board in BOTH eyes and a NEW pose
import os, glob, time, subprocess
import numpy as np, cv2

BASE=os.path.expanduser('~/stereo/matlab_pairs')
LDIR,RDIR=BASE+'/left',BASE+'/right'
STOPFILE='/tmp/smartgrab.stop'
W,H,HALF=3840,1200,1920
GOAL=31; TIME_BUDGET=300; CHECK_EVERY=0.30; SMALL=(768,480)
EXPOSURE=8000
GRID=(9,6)

def sh(*a): subprocess.run(a,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
def setexp(v): sh('v4l2-ctl','-d','/dev/video0','-c','exposure=%d'%v)

def gray(half16):
    lo,hi=np.percentile(half16,1),np.percentile(half16,99.5)
    b=np.clip((half16.astype(np.float32)-lo)*255/max(hi-lo,1),0,255).astype(np.uint8)
    return cv2.cvtColor(cv2.cvtColor(b,cv2.COLOR_BayerGR2BGR),cv2.COLOR_BGR2GRAY)

def main():
    for d in (LDIR,RDIR):
        os.makedirs(d,exist_ok=True)
    if os.path.exists(STOPFILE): os.remove(STOPFILE)
    sh('sudo','docker','stop','-t','20','narsil-vision'); time.sleep(3)

    cap=cv2.VideoCapture('/dev/video0',cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('B','A','1','0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
    if not cap.isOpened():
        print('FAILED to open /dev/video0'); sh('sudo','docker','start','narsil-vision'); return
    for _ in range(5): cap.read()
    setexp(EXPOSURE); time.sleep(0.5)
    print('streaming, exposure=%d. goal %d pairs, %ds. Move the board through poses.'
          %(EXPOSURE,GOAL,TIME_BUDGET),flush=True)

    det=cv2.createCLAHE(3.0,(8,8)); sav=cv2.createCLAHE(2.0,(8,8))
    kept=len(glob.glob(LDIR+'/*.png')); last=None; reads=0; checks=0; seen=0; cells=set(); t0=time.time(); tlast=0.0
    try:
        while kept<GOAL and (time.time()-t0)<TIME_BUDGET:
            if os.path.exists(STOPFILE): print('stop requested.',flush=True); break
            ok,buf=cap.read()
            if not ok or buf is None: continue
            reads+=1
            now=time.time()
            if now-tlast<CHECK_EVERY: continue
            tlast=now; checks+=1
            try: f=buf.reshape(-1).view(np.uint16).reshape(H,W)
            except Exception: continue
            L=cv2.rotate(gray(f[:,HALF:]),cv2.ROTATE_180)   # new LEFT  = half1
            R=cv2.rotate(gray(f[:,:HALF]),cv2.ROTATE_180)   # new RIGHT = half0
            okL,cL=cv2.findChessboardCornersSB(det.apply(cv2.resize(L,SMALL)),GRID)
            if not okL: continue
            okR,_=cv2.findChessboardCornersSB(det.apply(cv2.resize(R,SMALL)),GRID)
            if not okR: continue
            seen+=1
            p=cL.reshape(-1,2); cx,cy=p[:,0].mean(),p[:,1].mean()
            sp=(p[:,0].max()-p[:,0].min())/SMALL[0]
            if last is not None and abs(cx-last[0])<40 and abs(cy-last[1])<40 and abs(sp-last[2])<0.07:
                continue
            kept+=1; last=(cx,cy,sp)
            cv2.imwrite('%s/img_%02d.png'%(LDIR,kept),sav.apply(L))
            cv2.imwrite('%s/img_%02d.png'%(RDIR,kept),sav.apply(R))
            col='L' if cx<SMALL[0]*0.36 else ('R' if cx>SMALL[0]*0.64 else 'C')
            row='T' if cy<SMALL[1]*0.36 else ('B' if cy>SMALL[1]*0.64 else 'M')
            cells.add(row+col)
            print('KEPT %2d/%d cell=%s fill=%.0f%% (%ds) cells: %s'
                  %(kept,GOAL,row+col,sp*100,int(time.time()-t0),' '.join(sorted(cells))),flush=True)
        print('DONE: kept %d/%d in %ds (reads=%d checks=%d both-eyes-hits=%d). cells: %s'
              %(kept,GOAL,int(time.time()-t0),reads,checks,seen,' '.join(sorted(cells))),flush=True)
    finally:
        setexp(700); cap.release()
        sh('sudo','docker','start','narsil-vision')
        print('vision restarted.',flush=True)

if __name__=='__main__':
    main()
