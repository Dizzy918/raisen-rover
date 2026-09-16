#!/usr/bin/env python3
# Stereo-pair collector, final. Guarantees every saved pair is detectable at
# FULL resolution in BOTH eyes -- the earlier versions only checked a 768x480
# downscale, which hid motion blur, so most saves were unusable by MATLAB.
#  - ONE persistent V4L2 stream (repeated open/close wedges the Tegra VI channel)
#  - exposure set mid-stream (ignored if set before streaming)
#  - inverted mount: rotate each half 180 deg then SWAP (verified by disparity sign)
#  - NO CLAHE on saved images (exposure is correct now; CLAHE added noise)
#  - sharpness gate, then full-res verification, then save
import os, glob, time, subprocess
import numpy as np, cv2

BASE=os.path.expanduser('~/stereo/matlab_pairs')
LDIR,RDIR=BASE+'/left',BASE+'/right'
STOPFILE='/tmp/smartgrab.stop'
W,H,HALF=3840,1200,1920
GOAL=20; TIME_BUDGET=420; CHECK_EVERY=0.25
SMALL=(768,480); EXPOSURE=8000; GRID=(9,6)
SHARP_MIN=45.0            # Laplacian variance on the downscale; rejects blur

def sh(*a): subprocess.run(a,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

def gray(half16):
    lo,hi=np.percentile(half16,1),np.percentile(half16,99.5)
    b=np.clip((half16.astype(np.float32)-lo)*255/max(hi-lo,1),0,255).astype(np.uint8)
    return cv2.cvtColor(cv2.cvtColor(b,cv2.COLOR_BayerGR2BGR),cv2.COLOR_BGR2GRAY)

def main():
    for d in (LDIR,RDIR):
        os.makedirs(d,exist_ok=True)
        for f in glob.glob(d+'/*'): os.remove(f)
    if os.path.exists(STOPFILE): os.remove(STOPFILE)
    sh('sudo','docker','stop','-t','20','narsil-vision'); time.sleep(3)

    cap=cv2.VideoCapture('/dev/video0',cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_CONVERT_RGB,0)
    cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('B','A','1','0'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT,H)
    if not cap.isOpened():
        print('cannot open /dev/video0'); sh('sudo','docker','start','narsil-vision'); return
    for _ in range(5): cap.read()
    sh('v4l2-ctl','-d','/dev/video0','-c','exposure=%d'%EXPOSURE); time.sleep(0.5)
    print('streaming exp=%d. goal %d, %ds. HOLD STILL ~1.5s at each pose.'
          %(EXPOSURE,GOAL,TIME_BUDGET),flush=True)

    det=cv2.createCLAHE(3.0,(8,8))
    kept=0; last=None; blurry=0; smallhit=0; fullfail=0; cells=set()
    t0=time.time(); tlast=0.0
    try:
        while kept<GOAL and (time.time()-t0)<TIME_BUDGET:
            if os.path.exists(STOPFILE): print('stopped.',flush=True); break
            ok,buf=cap.read()
            if not ok or buf is None: continue
            now=time.time()
            if now-tlast<CHECK_EVERY: continue
            tlast=now
            try: f=buf.reshape(-1).view(np.uint16).reshape(H,W)
            except Exception: continue
            L=cv2.rotate(gray(f[:,HALF:]),cv2.ROTATE_180)
            R=cv2.rotate(gray(f[:,:HALF]),cv2.ROTATE_180)
            ls=cv2.resize(L,SMALL)
            if cv2.Laplacian(ls,cv2.CV_64F).var()<SHARP_MIN:
                blurry+=1; continue
            okL,cL=cv2.findChessboardCornersSB(det.apply(ls),GRID)
            if not okL: continue
            if not cv2.findChessboardCornersSB(det.apply(cv2.resize(R,SMALL)),GRID)[0]: continue
            smallhit+=1
            p=cL.reshape(-1,2); cx,cy=p[:,0].mean(),p[:,1].mean()
            sp=(p[:,0].max()-p[:,0].min())/SMALL[0]
            if last is not None and abs(cx-last[0])<40 and abs(cy-last[1])<40 and abs(sp-last[2])<0.07:
                continue
            # FULL-RES verification -- the whole point of this version
            if not cv2.findChessboardCornersSB(L,GRID)[0] or not cv2.findChessboardCornersSB(R,GRID)[0]:
                fullfail+=1
                if fullfail%5==0: print('  (%d rejected: not sharp enough at full res)'%fullfail,flush=True)
                continue
            kept+=1; last=(cx,cy,sp)
            cv2.imwrite('%s/img_%02d.png'%(LDIR,kept),L)     # plain, no CLAHE
            cv2.imwrite('%s/img_%02d.png'%(RDIR,kept),R)
            col='L' if cx<SMALL[0]*0.36 else ('R' if cx>SMALL[0]*0.64 else 'C')
            row='T' if cy<SMALL[1]*0.36 else ('B' if cy>SMALL[1]*0.64 else 'M')
            cells.add(row+col)
            print('KEPT %2d/%d cell=%s fill=%.0f%% VERIFIED-FULLRES (%ds) cells: %s'
                  %(kept,GOAL,row+col,sp*100,int(time.time()-t0),' '.join(sorted(cells))),flush=True)
        print('DONE: %d/%d kept in %ds | blur-rejected=%d small-hits=%d fullres-rejected=%d | cells: %s'
              %(kept,GOAL,int(time.time()-t0),blurry,smallhit,fullfail,' '.join(sorted(cells))),flush=True)
    finally:
        sh('v4l2-ctl','-d','/dev/video0','-c','exposure=700'); cap.release()
        sh('sudo','docker','start','narsil-vision')
        print('vision restarted.',flush=True)

if __name__=='__main__':
    main()
