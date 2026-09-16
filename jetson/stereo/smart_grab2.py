#!/usr/bin/env python3
# Stereo-pair collector v2: opens /dev/video0 ONCE and streams continuously
# (repeated open/close was destabilising the Tegra VI channel). Reads every
# frame to keep the queue drained, but only runs board detection every
# CHECK_EVERY seconds. Saves full-res pairs where the 9x6 board is found in
# BOTH eyes and the pose differs from the last kept one.
import os, glob, time, subprocess
import numpy as np, cv2

BASE=os.path.expanduser('~/stereo/matlab_pairs')
LDIR,RDIR=BASE+'/left',BASE+'/right'
STOPFILE='/tmp/smartgrab.stop'
W,H,HALF=3840,1200,1920
GOAL=20; TIME_BUDGET=300; CHECK_EVERY=0.35; SMALL=768
GRID=(9,6)

def sh(*a): subprocess.run(a,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

def full_gray(half16):
    lo,hi=np.percentile(half16,1),np.percentile(half16,99.5)
    b=np.clip((half16.astype(np.float32)-lo)*255/max(hi-lo,1),0,255).astype(np.uint8)
    return cv2.cvtColor(cv2.cvtColor(b,cv2.COLOR_BayerGR2BGR),cv2.COLOR_BGR2GRAY)

def find_small(fullg):
    s=cv2.resize(fullg,(SMALL,int(SMALL*H/W/2*2)))  # keep aspect of the half
    s=cv2.createCLAHE(3.0,(8,8)).apply(s)
    ok,c=cv2.findChessboardCornersSB(s,GRID)
    return ok,(c.reshape(-1,2) if ok else None),s.shape

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
        print('FAILED to open /dev/video0'); sh('sudo','docker','start','narsil-vision'); return
    print('streaming (single open). goal %d pairs, up to %ds. Keep the board moving.'
          %(GOAL,TIME_BUDGET),flush=True)
    kept=0; last=None; checks=0; reads=0; bad=0; cells=set(); t0=time.time()
    clahe=cv2.createCLAHE(2.0,(8,8))
    try:
        tlast=0.0
        while kept<GOAL and (time.time()-t0)<TIME_BUDGET:
            if os.path.exists(STOPFILE): print('stop requested.',flush=True); break
            ok,buf=cap.read()
            if not ok or buf is None:
                bad+=1
                if bad%50==0: print('  (%d failed reads)'%bad,flush=True)
                continue
            reads+=1
            now=time.time()
            if now-tlast<CHECK_EVERY: continue
            tlast=now; checks+=1
            try:
                f=buf.reshape(-1).view(np.uint16).reshape(H,W)
            except Exception:
                bad+=1; continue
            gl=full_gray(f[:,:HALF])
            okL,cL,shp=find_small(gl)
            if not okL: continue
            gr=full_gray(f[:,HALF:])
            okR,_,_=find_small(gr)
            if not okR: continue
            cx,cy=cL[:,0].mean(),cL[:,1].mean()
            sp=(cL[:,0].max()-cL[:,0].min())/SMALL
            if last is not None and abs(cx-last[0])<45 and abs(cy-last[1])<45 and abs(sp-last[2])<0.08:
                continue
            kept+=1; last=(cx,cy,sp)
            cv2.imwrite('%s/img_%02d.png'%(LDIR,kept),clahe.apply(gl))
            cv2.imwrite('%s/img_%02d.png'%(RDIR,kept),clahe.apply(gr))
            col='L' if cx<SMALL*0.36 else ('R' if cx>SMALL*0.64 else 'C')
            row='T' if cy<shp[0]*0.36 else ('B' if cy>shp[0]*0.64 else 'M')
            cells.add(row+col)
            print('KEPT %2d/%d cell=%s fill=%.0f%% (reads=%d checks=%d %ds) cells:%s'
                  %(kept,GOAL,row+col,sp*100,reads,checks,int(time.time()-t0),
                    ' '.join(sorted(cells))),flush=True)
        print('DONE: kept %d in %ds (reads=%d checks=%d failed=%d). cells: %s'
              %(kept,int(time.time()-t0),reads,checks,bad,' '.join(sorted(cells))),flush=True)
    finally:
        cap.release()
        sh('sudo','docker','start','narsil-vision')
        print('vision restarted.',flush=True)

if __name__=='__main__':
    main()
