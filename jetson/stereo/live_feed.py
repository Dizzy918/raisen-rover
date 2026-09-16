#!/usr/bin/env python3
"""Live MJPEG view of the AR0234 stereo pair, watchable in a browser.

    ./run_live_feed.sh              # on the Jetson, frees the camera first
    open http://jetson9.local:8080/ # from the Mac

Modes (buttons on the page, or /stream?mode=NAME):

  pair    both eyes side by side, 2x2 binned grey -- what the stereo and vSLAM
          pipelines actually consume
  left    left eye alone, larger
  clahe   left eye with local contrast equalisation, which is the cheap answer
          to the blown-window problem: a single exposure cannot hold a lit
          window and an indoor foreground, but CLAHE recovers usable texture
          in both by normalising in tiles instead of globally
  color   left eye demosaiced and white balanced

COLOUR IS NOT FREE, three reasons worth knowing before switching anything to it:

  The pattern constant is ambiguous. The sensor reports GRBG, but OpenCV's
  Bayer names are relative to a different origin, and GB vs GR differ only by
  a red/blue swap that no amount of staring at an office settles. Put something
  strongly red in frame: if it renders blue, flip BAYER below to the other one.

  There is no white balance in raw. Every variant comes out green or magenta
  because green has twice the photosites and a higher response. Grey-world
  (equalise the channel means) fixes it well enough to look at, and it is what
  this uses -- but it fails on scenes with a genuinely dominant colour, since
  it assumes the world averages to grey.

  Demosaicing costs resolution the binning does not. Binning averages each 2x2
  quad, so every output pixel is real measured light. Demosaicing INTERPOLATES
  the two missing channels at every pixel, which invents detail and aliases on
  fine repeated structure -- the exact trap that cost us a whole checkerboard
  capture round. For stereo matching and for cuVSLAM, grey is not a compromise,
  it is the correct input; cuVSLAM refuses anything but mono8 outright.

narsil-vision holds /dev/video0, so run this through run_live_feed.sh rather
than directly.
"""
import socket
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

W, H, HALF = 3840, 1200, 1920
BAYER = cv2.COLOR_BayerGB2BGR   # flip to COLOR_BayerGR2BGR if red renders blue

PAGE = b"""<!doctype html><meta charset=utf-8><title>AR0234 live</title>
<style>
 body{background:#111;color:#ddd;font:14px system-ui;margin:0;padding:16px}
 a{display:inline-block;padding:6px 14px;margin:0 6px 12px 0;background:#222;
   color:#8cf;text-decoration:none;border-radius:4px}
 a:hover{background:#333} img{max-width:100%;display:block;border:1px solid #333}
 code{color:#9c9}
</style>
<h3>AR0234 stereo, live</h3>
<a href="/?mode=pair">pair (grey)</a><a href="/?mode=left">left</a>
<a href="/?mode=clahe">left + CLAHE</a><a href="/?mode=color">colour</a>
<img id=v src="">
<p id=s>connecting...</p>
<script>
 var m = new URLSearchParams(location.search).get('mode') || 'pair';
 document.getElementById('v').src = '/stream?mode=' + m;
 document.getElementById('s').textContent = 'mode: ' + m;
</script>
"""


def bin2x2(half16):
    f = half16.astype(np.float32)
    q = (f[0::2, 0::2] + f[0::2, 1::2] + f[1::2, 0::2] + f[1::2, 1::2]) * 0.25
    lo, hi = np.percentile(q[::6, ::6], (1, 99))
    return np.clip((q - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)


def grey_world(bgr):
    """Equalise channel means. Assumes the scene averages to grey -- fine for a
    cluttered office, wrong for a close-up of one saturated colour."""
    f = bgr.astype(np.float32)
    means = [max(f[:, :, i].mean(), 1.0) for i in range(3)]
    target = sum(means) / 3.0
    for i in range(3):
        f[:, :, i] *= target / means[i]
    return np.clip(f, 0, 255).astype(np.uint8)


class Camera:
    """One reader thread; every HTTP client encodes from the same latest frame,
    so N viewers cost N JPEG encodes but only one camera read."""

    def __init__(self, dev, exposure, gain, ae=None):
        # v4l2cap: OpenCV first, v4l2-ctl if it cannot open the device.
        import v4l2cap
        self.cap = v4l2cap.open_stereo(dev, W, H)
        if self.cap is None:
            raise SystemExit('cannot open %s -- is narsil-vision still running? '
                             'use run_live_feed.sh' % dev)
        for _ in range(5):
            self.cap.read()
        import subprocess
        for k, v in (('exposure', exposure), ('analogue_gain', gain)):
            subprocess.run(['v4l2-ctl', '-d', dev, '-c', '%s=%d' % (k, v)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.6)
        self.ae = ae
        self.frame = None
        self.n = 0
        self.t0 = time.time()
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ok, buf = self.cap.read()
            if not ok or buf is None:
                time.sleep(0.01)
                continue
            try:
                f = buf.reshape(-1).view(np.uint16).reshape(H, W)
            except ValueError:
                continue
            # AE meters the RAW frame, before bin2x2's per-frame stretch erases
            # the evidence that exposure changed at all.
            if self.ae is not None:
                self.ae.update(f)
            with self.lock:
                self.frame = f
                self.n += 1

    def ae_label(self):
        if self.ae is None:
            return ''
        s = self.ae.state()
        return '  exp=%d gain=%d%s' % (s['exposure'], s['gain'],
                                       ' SAT-LIM' if self.ae.sat_limited else '')

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def fps(self):
        el = time.time() - self.t0
        return self.n / el if el > 0 else 0.0


def render(raw, mode):
    """raw is the full side-by-side uint16 frame. The mount is inverted, so each
    half is rotated 180 and the halves are swapped: the LEFT eye is the second
    half of the buffer."""
    if mode == 'color':
        eight = (raw[:, HALF:] >> 8).astype(np.uint8)
        bgr = cv2.rotate(cv2.cvtColor(eight, BAYER), cv2.ROTATE_180)
        return grey_world(cv2.resize(bgr, (960, 600)))

    left = cv2.rotate(bin2x2(raw[:, HALF:]), cv2.ROTATE_180)
    if mode == 'left':
        return left
    if mode == 'clahe':
        return cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(left)
    right = cv2.rotate(bin2x2(raw[:, :HALF]), cv2.ROTATE_180)
    return np.hstack([left, right])


class Handler(BaseHTTPRequestHandler):
    cam = None
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path != '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return

        mode = parse_qs(u.query).get('mode', ['pair'])[0]
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=f')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        enc = [int(cv2.IMWRITE_JPEG_QUALITY), 72]
        try:
            while True:
                raw = self.cam.latest()
                if raw is None:
                    time.sleep(0.05)
                    continue
                img = render(raw, mode)
                label = '%s  %.1f fps%s' % (mode, self.cam.fps(),
                                            self.cam.ae_label())
                cv2.putText(img, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, 255 if img.ndim == 2 else (0, 255, 255), 2)
                ok, jpg = cv2.imencode('.jpg', img, enc)
                if not ok:
                    continue
                b = jpg.tobytes()
                self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                 b'Content-Length: ' + str(len(b)).encode()
                                 + b'\r\n\r\n' + b + b'\r\n')
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--exposure', type=int, default=1500,
                    help='sensor row-times; this caps the frame rate')
    ap.add_argument('--gain', type=int, default=800)
    ap.add_argument('--auto', action='store_true',
                    help='auto-exposure instead of the fixed values')
    ap.add_argument('--min-fps', type=float, default=20.0,
                    help='AE will not spend exposure below this frame rate')
    a = ap.parse_args()
    ae = None
    if a.auto:
        from auto_exposure import AutoExposure
        ae = AutoExposure(a.device, min_fps=a.min_fps,
                          exposure=a.exposure, gain=a.gain)
        print('auto-exposure on, exposure capped at %d for >=%.0f fps'
              % (ae.exp_max, a.min_fps))
    Handler.cam = Camera(a.device, a.exposure, a.gain, ae)
    srv = ThreadingHTTPServer(('0.0.0.0', a.port), Handler)
    # the host's own name, not a hardcoded jetson9 -- this runs on both
    print('open http://%s.local:%d/  (ctrl-c to stop)'
          % (socket.gethostname(), a.port), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
