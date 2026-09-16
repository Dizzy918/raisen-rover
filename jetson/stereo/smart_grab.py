#!/usr/bin/env python3
# Smart stereo-pair collector: grabs continuously, keeps ONLY frames where the
# 9x6 board is found in BOTH eyes and the pose differs from the last kept one.
# Runs until GOAL good pairs or TIME_BUDGET seconds. Stop early: touch STOPFILE.
import os, glob, time, subprocess
import numpy as np, cv2

BASE=os.path.expanduser('~/stereo/matlab_pairs')
LDIR,RDIR=BASE+'/left',BASE+'/right'
STOPFILE='/tmp/smartgrab.stop'
W,H,HALF=3840,1200,1920
GOAL=20; TIME_BUDGET=240; BURST=4; SMALL=768
GRID=(9,6); RAW='/tmp/sgburst.raw'

def sh(*a): subprocess.run(a,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
def grab():
    if os.path.exists(RAW): os.remove(RAW)
    sh('timeout','15','v4l2-ctl','-d','/dev/video0',
       '--set-fmt-video=width=%d,height=%d,pixelformat=BA10'%(W,H),
       '--stream-mmap','--stream-count=%d'%BURST,'--stream-to='+RAW)
    if not os.path.exists(RAW): return []
    fb=W*H*2; r=np.fromfile(RAW,dtype=np.uint8); n=len(r)//fb
    return [r[i*fb:(i+1)*fb].view(np.uint16).reshape(H,W) for i in range(n)]
def full_gray(half16):
    lo,hi=np.percentile(half16,1),np.percentile(half16,99.5)
    b=np.clip((half16.astype(np.float32)-lo)*255/max(hi-lo,1),0,255).astype(np.uint8)
    return cv2.cvtColor(cv2.cvtColor(b,cv2.COLOR_BayerGR2BGR),cv2.COLOR_BGR2GRAY)
def sharp(g): return cv2.Laplacian(cv2.resize(g,(400,250)),cv2.CV_64F).var()
def find_small(fullg):
    s=cv2.resize(fullg,(SMALL,int(SMALL*H/W)))
    s=cv2.createCLAHE(3.0,(8,8)).apply(s)
    ok,c=cv2.findChessboardCornersSB(s,GRID)
    return ok,(c.reshape(-1,2) if ok else None),s.shape

def main():
    for d in (LDIR,RDIR):
        os.makedirs(d,exist_ok=True)
        for f in glob.glob(d+'/*'): os.remove(f)
    if os.path.exists(STOPFILE): os.remove(STOPFILE)
    sh('sudo','docker','stop','-t','20','narsil-vision')
    time.sleep(2)
    kept=0; last=None; t0=time.time(); grabs=0; cells=set()
    try:
        print('COLLECTING: goal %d good pairs, up to %ds. Keep the board moving.'%(GOAL,TIME_BUDGET),flush=True)
        while kept<GOAL and (time.time()-t0)<TIME_BUDGET:
            if os.path.exists(STOPFILE): print('stop requested.',flush=True); break
            frames=grab(); grabs+=1
            if not frames: continue
            # sharpest frame of the burst by left half
            gls=[full_gray(f[:,:HALF]) for f in frames]
            i=max(range(len(frames)),key=lambda j:sharp(gls[j]))
            gl=gls[i]
            okL,cL,shp=find_small(gl)
            if not okL: continue
            gr=full_gray(frames[i][:,HALF:])
            okR,_,_=find_small(gr)
            if not okR: continue
            cx,cy=cL[:,0].mean(),cL[:,1].mean()
            sw=(SMALL); sp=(cL[:,0].max()-cL[:,0].min())/sw
            if last is not None:
                if (abs(cx-last[0])<45 and abs(cy-last[1])<45 and abs(sp-last[2])<0.08):
                    continue  # too similar to last kept -> skip
            kept+=1; last=(cx,cy,sp)
            _cl=cv2.createCLAHE(2.0,(8,8))
            cv2.imwrite('%s/img_%02d.png'%(LDIR,kept),_cl.apply(gl))
            cv2.imwrite('%s/img_%02d.png'%(RDIR,kept),_cl.apply(gr))
            col='L' if cx<sw*0.36 else ('R' if cx>sw*0.64 else 'C')
            row='T' if cy<shp[0]*0.36 else ('B' if cy>shp[0]*0.64 else 'M')
            cells.add(row+col)
            print('KEPT %2d/%d  cell=%s fill=%.0f%%  (grabs=%d, %ds)  cells:%s'
                  %(kept,GOAL,row+col,sp*100,grabs,int(time.time()-t0),''.join(sorted(cells))),flush=True)
        print('DONE: kept %d pairs in %ds (%d grabs). cells hit: %s'
              %(kept,int(time.time()-t0),grabs,' '.join(sorted(cells))),flush=True)
    finally:
        sh('sudo','docker','start','narsil-vision')
        print('vision restarted.',flush=True)

if __name__=='__main__':
    main()
