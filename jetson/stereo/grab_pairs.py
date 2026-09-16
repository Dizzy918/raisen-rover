#!/usr/bin/env python3
# Grab synced L/R pairs off the AR0234 Camarray at full 3840x1200 for offline
# MATLAB stereo calibration. No detection here (MATLAB does that) -> fast.
import os, glob, time, subprocess
import numpy as np, cv2

BASE = os.path.expanduser('~/stereo/matlab_pairs')
LDIR, RDIR = BASE+'/left', BASE+'/right'
W, H, HALF = 3840, 1200, 1920
NSHOTS  = 30
CADENCE = 2.0
BURST   = 5
LEAD_IN = 10
RAW = '/tmp/pairburst.raw'

def sh(*a): subprocess.run(a, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def grab():
    if os.path.exists(RAW): os.remove(RAW)
    sh('timeout','20','v4l2-ctl','-d','/dev/video0',
       '--set-fmt-video=width=%d,height=%d,pixelformat=BA10'%(W,H),
       '--stream-mmap','--stream-count=%d'%BURST,'--stream-to='+RAW)
    fb=W*H*2; r=np.fromfile(RAW,dtype=np.uint8); n=len(r)//fb
    return [r[i*fb:(i+1)*fb].view(np.uint16).reshape(H,W) for i in range(n)]

def gray(half16):
    lo,hi=np.percentile(half16,1),np.percentile(half16,99.5)
    b=np.clip((half16.astype(np.float32)-lo)*255/max(hi-lo,1),0,255).astype(np.uint8)
    return cv2.cvtColor(cv2.cvtColor(b,cv2.COLOR_BayerGR2BGR),cv2.COLOR_BGR2GRAY)

def sharp(g): return cv2.Laplacian(cv2.resize(g,(480,300)),cv2.CV_64F).var()

def main():
    for d in (LDIR,RDIR):
        os.makedirs(d,exist_ok=True)
        for f in glob.glob(d+'/*'): os.remove(f)
    sh('sudo','docker','stop','-t','20','narsil-vision')
    try:
        print('LEAD-IN %ds: get the board UP in front of both cameras NOW.'%LEAD_IN, flush=True)
        time.sleep(LEAD_IN)
        for k in range(1,NSHOTS+1):
            t0=time.time()
            frames=grab()
            if not frames:
                print('shot %2d/%d: capture failed'%(k,NSHOTS), flush=True); continue
            gl=[gray(f[:,:HALF]) for f in frames]
            i=max(range(len(frames)),key=lambda j: sharp(gl[j]))
            cv2.imwrite('%s/img_%02d.png'%(LDIR,k), gl[i])
            cv2.imwrite('%s/img_%02d.png'%(RDIR,k), gray(frames[i][:,HALF:]))
            print('shot %2d/%d saved (sharp=%.0f)'%(k,NSHOTS,sharp(gl[i])), flush=True)
            dt=CADENCE-(time.time()-t0)
            if dt>0: time.sleep(dt)
        print('DONE: %d pairs saved.'%NSHOTS, flush=True)
    finally:
        sh('sudo','docker','start','narsil-vision')
        print('vision restarted.', flush=True)

if __name__=='__main__':
    main()
