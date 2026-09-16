#!/usr/bin/env python3
# Stereo calibration capture helper for the AR0234 Camarray (3840x1200 SBS).
# Usage: calib_capture.py <pose_index>   (index 1 clears the session dir)
import sys, os, json, glob, subprocess
import numpy as np, cv2

DIR = os.path.expanduser('~/stereo/calib3840')
W, H = 3840, 1200
HALF = 1920
GRID = (9, 6)          # inner corners
RAW  = '/tmp/calib_burst.raw'
NBURST = 12

def grab():
    if os.path.exists(RAW): os.remove(RAW)
    subprocess.run(['timeout','30','v4l2-ctl','-d','/dev/video0',
        '--set-fmt-video=width=%d,height=%d,pixelformat=BA10'%(W,H),
        '--stream-mmap','--stream-count=%d'%NBURST,'--stream-to='+RAW],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    fb = W*H*2
    r = np.fromfile(RAW, dtype=np.uint8)
    n = len(r)//fb
    return [r[i*fb:(i+1)*fb].view(np.uint16).reshape(H,W) for i in range(n)]

def to_gray(half16):
    lo,hi = np.percentile(half16,1), np.percentile(half16,99.5)
    b = np.clip((half16.astype(np.float32)-lo)*255/max(hi-lo,1),0,255).astype(np.uint8)
    g = cv2.cvtColor(cv2.cvtColor(b, cv2.COLOR_BayerGR2BGR), cv2.COLOR_BGR2GRAY)
    return cv2.createCLAHE(3.0,(8,8)).apply(g)

def sharp(g):
    return cv2.Laplacian(g, cv2.CV_64F).var()

def detect(g):
    ok,c = cv2.findChessboardCornersSB(g, GRID, cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)
    if not ok:
        ok,c = cv2.findChessboardCorners(g, GRID,
               cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK)
        if ok:
            c = cv2.cornerSubPix(g, c, (7,7), (-1,-1),
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.01))
    return (c.reshape(-1,2) if ok else None)

def fix_corr(cl, cr):
    # near-identical viewpoints: pick ordering (as-is vs reversed right) with lower dy spread
    a = np.std((cl-cr)[:,1]); b = np.std((cl-cr[::-1])[:,1])
    return (cl, cr) if a<=b else (cl, cr[::-1])

def coverage(cl):
    xs,ys = cl[:,0], cl[:,1]
    cx,cy = xs.mean(), ys.mean()
    col = 'LEFT' if cx<HALF/3 else ('RIGHT' if cx>2*HALF/3 else 'center')
    row = 'TOP'  if cy<H/3   else ('BOTTOM' if cy>2*H/3   else 'middle')
    span = (xs.max()-xs.min())/HALF
    r0,r5 = cl[0:9], cl[45:54]
    c0,c5 = cl[0::9], cl[8::9]
    tv = np.hypot(*(r0[-1]-r0[0]))/max(np.hypot(*(r5[-1]-r5[0])),1)   # top/bottom width ratio
    th = np.hypot(*(c0[-1]-c0[0]))/max(np.hypot(*(c5[-1]-c5[0])),1)
    return dict(cell=f'{row}-{col}', span=round(float(span),2),
                tilt_v=round(float(tv),2), tilt_h=round(float(th),2))

def main():
    idx = int(sys.argv[1])
    os.makedirs(DIR, exist_ok=True)
    subprocess.run(['sudo','docker','stop','-t','20','narsil-vision'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if idx==1:
        for f in glob.glob(DIR+'/*'): os.remove(f)

    frames = grab()
    if not frames:
        print('CAPTURE FAILED: no frames (device busy?)'); return
    # try frames sharpest-first, accept first where BOTH halves detect
    order = sorted(range(len(frames)), key=lambda i: -sharp(to_gray(frames[i][:,:HALF])))
    best = None
    diagL = diagR = 0
    for i in order:
        gl = to_gray(frames[i][:,:HALF]); gr = to_gray(frames[i][:,HALF:])
        cl = detect(gl); cr = detect(gr)
        diagL += cl is not None; diagR += cr is not None
        if cl is not None and cr is not None:
            best = (i,gl,gr,cl,cr); break
    if best is None:
        print(f'NOT DETECTED in both eyes across {len(frames)} frames '
              f'(left hits={diagL}, right hits={diagR}).')
        print('Brightness L=%.0f R=%.0f'%(to_gray(frames[0][:,:HALF]).mean(),
                                          to_gray(frames[0][:,HALF:]).mean()))
        return
    i,gl,gr,cl,cr = best
    cl,cr = fix_corr(cl,cr)
    np.savez(f'{DIR}/pose_{idx:02d}.npz', left=cl, right=cr)
    cov = coverage(cl)
    dy = float(np.median(cl[:,1]-cr[:,1])); dx = float(np.median(cl[:,0]-cr[:,0]))
    # preview
    vis = cv2.cvtColor(np.hstack([gl,gr]), cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(vis[:, :HALF], GRID, cl.reshape(-1,1,2).astype(np.float32), True)
    cv2.imwrite(f'{DIR}/pose_{idx:02d}.jpg',
                cv2.resize(vis,(1700,int(1700*H/W)),interpolation=cv2.INTER_AREA))

    # session coverage summary
    st = f'{DIR}/state.json'
    hist = json.load(open(st)) if os.path.exists(st) else []
    hist = [h for h in hist if h['idx']!=idx]
    hist.append(dict(idx=idx, **cov)); json.dump(hist, open(st,'w'))
    cells = sorted({h['cell'] for h in hist})
    spans = [h['span'] for h in hist]
    print(f'POSE {idx} OK  ->  saved.')
    print(f'  position: {cov["cell"]}   fill: {int(cov["span"]*100)}% of width')
    print(f'  tilt: vert-ratio {cov["tilt_v"]}  horiz-ratio {cov["tilt_h"]}  (1.0 = fronto-parallel)')
    print(f'  sanity: median disparity {dx:+.1f}px  vertical offset {dy:+.1f}px')
    print(f'  SESSION: {len(hist)} poses | cells seen: {", ".join(cells)}')
    print(f'           fill range {min(spans):.2f}-{max(spans):.2f}')

if __name__ == "__main__":
    main()
