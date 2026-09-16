#!/usr/bin/env python3
"""Live sonar view in a browser: top-down beams, numbers, and a rolling trace.

    python3 rover/sonar_view.py                 # then open http://localhost:8081/
    python3 rover/sonar_view.py --no-imu        # sonar only, leave the rover alone

Runs on the Mac or on the Jetson -- it only needs HTTP reach to the sonar box.
It cannot be a hosted page: a published artifact is forbidden by CSP from
contacting a device on your LAN, so the poller has to live next to the hardware.

WHY THE POLL RATE IS WHAT IT IS. The firmware triggers the four sensors
sequentially with 60 ms between them, deliberately, so they never hear each
other's pulses -- plus a 200 ms tail, so a full cycle is about 440 ms. Asking
faster than about 2 Hz just returns the same numbers again. The browser polls
this server instead, which serves a cached reading, so the page stays smooth
without hammering the ESP32.

CHANNEL MAP, confirmed on hardware 2026-08-18:
    sonar4 = FRONT      sonar2 = LEFT      sonar1 = RIGHT      sonar3 = dead
PROJECT_CONTEXT.md says sonar1 is the front. It is wrong for this rover.
"""
import argparse
import json
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SONAR_IP = '172.10.10.192'
ROVER_IP = '172.10.10.184'
CHANNELS = {'front': 'sonar4', 'left': 'sonar2', 'right': 'sonar1'}
SPARE = 'sonar3'
POLL_S = 0.45              # one firmware cycle
TRACE_N = 240              # ~110 s of history
STOP_CM = 40.0             # drive_test refuses forward motion inside this
WARN_CM = 100.0

state = {'t': 0.0, 'front': None, 'left': None, 'right': None, 'spare': None,
         'yaw': None, 'gz': None, 'fails': 0, 'ok': 0, 'trace': []}
lock = threading.Lock()


def fetch(url, timeout=2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode())


def poller(use_imu):
    imu_url = ('http://%s/js?json=%s'
               % (ROVER_IP, urllib.parse.quote('{"T":126}')))
    while True:
        t0 = time.time()
        try:
            d = fetch('http://%s/sonar' % SONAR_IP)
            vals = {}
            for name, ch in CHANNELS.items():
                v = float(d.get(ch, -1))
                vals[name] = None if v < 0 else v      # None means no echo
            sp = float(d.get(SPARE, -1))
            with lock:
                state.update(t=time.time(), spare=(None if sp < 0 else sp), **vals)
                state['ok'] += 1
                state['trace'].append([round(time.time(), 2),
                                       vals['front'], vals['left'], vals['right']])
                del state['trace'][:-TRACE_N]
        except Exception:
            with lock:
                state['fails'] += 1
        if use_imu:
            try:
                d = fetch(imu_url)
                with lock:
                    state['yaw'] = float(d.get('y', 0.0))
                    state['gz'] = float(d.get('gz', 0.0))
            except Exception:
                pass
        time.sleep(max(0.0, POLL_S - (time.time() - t0)))


PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<title>Sonar</title>
<style>
:root{
  --bg:#f4f5f6; --panel:#ffffff; --ink:#14181a; --dim:#6b7478; --line:#dfe3e5;
  --ok:#2f8078; --warn:#c08a2e; --bad:#c0492e; --grid:#e7eaec; --sweep:#45a79e;
}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){
  --bg:#0d1417; --panel:#141c20; --ink:#eef1f2; --dim:#8b979c; --line:#243035;
  --ok:#45a79e; --warn:#d6a24a; --bad:#e0654a; --grid:#1c2529;
  --sweep:#45a79e;
}}
:root[data-theme="dark"]{
  --bg:#0d1417; --panel:#141c20; --ink:#eef1f2; --dim:#8b979c; --line:#243035;
  --ok:#45a79e; --warn:#d6a24a; --bad:#e0654a; --grid:#1c2529; --sweep:#45a79e;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{padding:14px 18px;border-bottom:1px solid var(--line);
  display:flex;gap:18px;align-items:baseline;flex-wrap:wrap}
h1{margin:0;font-size:15px;letter-spacing:.08em;text-transform:uppercase;font-weight:600}
.meta{color:var(--dim);font-size:12px}
main{display:grid;grid-template-columns:minmax(320px,1fr) minmax(280px,420px);
  gap:16px;padding:16px;align-items:start}
@media (max-width:820px){main{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px}
.card h2{margin:0 0 10px;font-size:11px;letter-spacing:.1em;text-transform:uppercase;
  color:var(--dim);font-weight:600}
canvas{width:100%;height:auto;display:block}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
td{padding:7px 0;border-bottom:1px solid var(--line)}
td:last-child{text-align:right;font-weight:600}
tr:last-child td{border-bottom:none}
.tag{display:inline-block;padding:1px 7px;border-radius:999px;font-size:11px;
  border:1px solid currentColor}
.ok{color:var(--ok)} .warn{color:var(--warn)} .bad{color:var(--bad)} .dim{color:var(--dim)}
footer{padding:0 18px 20px;color:var(--dim);font-size:12px;max-width:70ch}
</style></head><body>
<header>
  <h1>Sonar -- rover-01</h1>
  <span class="meta" id="meta">connecting</span>
</header>
<main>
  <div class="card">
    <h2>Top-down, rover facing up</h2>
    <canvas id="radar" width="640" height="520"></canvas>
  </div>
  <div>
    <div class="card" style="margin-bottom:16px">
      <h2>Readings</h2>
      <table id="tbl"></table>
    </div>
    <div class="card">
      <h2>Last 110 s</h2>
      <canvas id="trace" width="640" height="260"></canvas>
    </div>
  </div>
</main>
<footer id="note"></footer>
<script>
const STOP=40, WARN=100, MAXR=400;
const q=s=>document.querySelector(s);
function band(v){ if(v===null) return 'dim'; if(v<STOP) return 'bad';
  if(v<WARN) return 'warn'; return 'ok'; }
function css(n){ return getComputedStyle(document.documentElement)
  .getPropertyValue('--'+n).trim(); }

function radar(d){
  const c=q('#radar'), x=c.getContext('2d'), W=c.width, H=c.height;
  const cx=W/2, cy=H*0.62, px=(H*0.56)/MAXR;      // pixels per cm
  x.clearRect(0,0,W,H);
  x.strokeStyle=css('grid'); x.fillStyle=css('dim'); x.lineWidth=1;
  x.font='11px ui-monospace';
  for(let r=100;r<=MAXR;r+=100){
    x.beginPath(); x.arc(cx,cy,r*px,0,Math.PI*2); x.stroke();
    // label on the 135 deg diagonal: the three beams point up, left and right,
    // so a vertical label would sit underneath the front cone.
    const la=-135*Math.PI/180;
    x.fillText((r/100)+' m', cx+Math.cos(la)*r*px+3, cy+Math.sin(la)*r*px-3);
  }
  const beams=[['front',0],['left',-90],['right',90]];
  for(const [k,adeg] of beams){
    const v=d[k], half=7.5*Math.PI/180;
    const a=(adeg-90)*Math.PI/180;                 // screen: up is forward
    const r=(v===null?MAXR:Math.min(v,MAXR))*px;
    x.beginPath(); x.moveTo(cx,cy);
    x.arc(cx,cy,r,a-half,a+half); x.closePath();
    const col=css(band(v));
    x.fillStyle=col+(v===null?'18':'33'); x.fill();
    x.strokeStyle=col; x.lineWidth=v===null?1:2;
    if(v===null){x.setLineDash([4,4]);} x.stroke(); x.setLineDash([]);
    if(v!==null){
      x.fillStyle=col; x.font='600 12px ui-monospace';
      const lx=cx+Math.cos(a)*(r+16), ly=cy+Math.sin(a)*(r+16);
      x.textAlign = adeg===0?'center':(adeg<0?'right':'left');
      x.fillText(v.toFixed(0)+' cm', lx, ly+4); x.textAlign='left';
    }
  }
  // chassis: 218 mm long, 187 mm wide, from algorythm/rover.py
  const L=21.8*px, Wd=18.7*px;
  x.fillStyle=css('panel'); x.strokeStyle=css('ink'); x.lineWidth=1.5;
  x.beginPath(); x.rect(cx-Wd/2, cy-L/2, Wd, L); x.fill(); x.stroke();
  x.beginPath(); x.moveTo(cx,cy-L/2-9); x.lineTo(cx-5,cy-L/2+1);
  x.lineTo(cx+5,cy-L/2+1); x.closePath(); x.fillStyle=css('ink'); x.fill();
  x.fillStyle=css('bad'); x.globalAlpha=.5;
  x.beginPath(); x.arc(cx,cy,STOP*px,0,Math.PI*2); x.globalAlpha=1;
  x.setLineDash([3,4]); x.strokeStyle=css('bad'); x.stroke(); x.setLineDash([]);
}

function trace(rows){
  const c=q('#trace'), x=c.getContext('2d'), W=c.width, H=c.height;
  const padL=40, padT=26, padB=14, padR=8;
  x.clearRect(0,0,W,H);
  const keys=[['front',1],['left',2],['right',3]];
  const cols={front:css('sweep'),left:css('warn'),right:css('bad')};

  // Autoscale to what is actually on screen. A fixed 0-400 cm axis squashed a
  // 60-130 cm scene into the bottom sixth and hid every change worth seeing.
  // Floor the span at 60 cm so a steady reading does not turn noise into drama.
  let lo=Infinity, hi=-Infinity;
  for(const r of rows) for(const [,i] of keys){
    const v=r[i]; if(v===null||v>=999) continue;
    if(v<lo) lo=v; if(v>hi) hi=v;
  }
  if(!isFinite(lo)){ lo=0; hi=MAXR; }
  const mid=(lo+hi)/2, span=Math.max(hi-lo, 60)*1.25;
  lo=Math.max(0, mid-span/2); hi=mid+span/2;
  const yOf=v=>padT+(H-padT-padB)*(1-(Math.min(v,hi)-lo)/(hi-lo));

  x.strokeStyle=css('grid'); x.fillStyle=css('dim'); x.font='10px ui-monospace';
  for(let i=0;i<=4;i++){
    const v=lo+(hi-lo)*i/4, y=yOf(v);
    x.beginPath(); x.moveTo(padL,y); x.lineTo(W-padR,y); x.stroke();
    x.fillText(v.toFixed(0)+' cm', 2, y+3);
  }
  // The 40 cm threshold, drawn only when it is inside the visible window.
  if(STOP>lo && STOP<hi){
    x.setLineDash([3,4]); x.strokeStyle=css('bad');
    x.beginPath(); x.moveTo(padL,yOf(STOP)); x.lineTo(W-padR,yOf(STOP)); x.stroke();
    x.setLineDash([]);
  }
  if(rows.length>=2){
    for(const [k,idx] of keys){
      x.beginPath(); x.strokeStyle=cols[k]; x.lineWidth=1.6; let started=false;
      rows.forEach((r,i)=>{
        const v=r[idx];
        if(v===null||v>=999){started=false;return;}
        const px_=padL+(W-padL-padR)*i/(rows.length-1);
        if(!started){x.moveTo(px_,yOf(v));started=true;} else x.lineTo(px_,yOf(v));
      });
      x.stroke();
    }
  }
  // Legend along the top, right of the axis gutter so it clears the labels.
  x.font='600 11px ui-monospace'; let lx=padL+2;
  for(const [k] of keys){ x.fillStyle=cols[k]; x.fillText(k,lx,13); lx+=48; }
}

async function tick(){
  try{
    const d=await (await fetch('/data',{cache:'no-store'})).json();
    const rows=[['front','FRONT'],['left','left'],['right','right']];
    q('#tbl').innerHTML = rows.map(([k,label])=>{
      const v=d[k], b=band(v);
      const txt = v===null ? 'no echo' : v.toFixed(1)+' cm';
      return `<tr><td>${label}</td><td class="${b}">${txt}</td></tr>`;
    }).join('')
      + `<tr><td>sonar3</td><td class="dim">${d.spare===null?'dead':d.spare.toFixed(0)}</td></tr>`
      + (d.yaw===null?'':`<tr><td>yaw</td><td>${d.yaw.toFixed(2)} deg</td></tr>`)
      + (d.gz===null?'':`<tr><td>gz raw</td><td>${d.gz.toFixed(2)} deg/s</td></tr>`);
    radar(d); trace(d.trace);
    const age=(Date.now()/1000-d.t);
    q('#meta').innerHTML = `<span class="tag ${age<2?'ok':'bad'}">`
      + (age<2?'live':'stale '+age.toFixed(0)+'s') + `</span> `
      + `${d.ok} reads, ${d.fails} failed`;
    q('#note').textContent = d.note;
  }catch(e){ q('#meta').innerHTML='<span class="tag bad">server gone</span>'; }
}
setInterval(tick,330); tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith('/data'):
            with lock:
                d = dict(state)
                d['trace'] = list(state['trace'])
            d['note'] = ('Channel map confirmed on hardware: sonar4 front, '
                         'sonar2 left, sonar1 right, sonar3 dead. The dashed '
                         'circle is the 40 cm threshold below which drive_test '
                         'refuses forward motion. Polled every %.0f ms, one '
                         'firmware cycle -- the four sensors fire 60 ms apart '
                         'so they do not hear each other.' % (POLL_S * 1000))
            self._send(json.dumps(d).encode(), 'application/json')
        else:
            self._send(PAGE.encode(), 'text/html; charset=utf-8')

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8081)
    ap.add_argument('--sonar-ip', default=SONAR_IP)
    ap.add_argument('--rover-ip', default=ROVER_IP)
    ap.add_argument('--no-imu', action='store_true',
                    help='do not poll the rover at all, sonar box only')
    a = ap.parse_args()
    globals()['SONAR_IP'] = a.sonar_ip
    globals()['ROVER_IP'] = a.rover_ip
    threading.Thread(target=poller, args=(not a.no_imu,), daemon=True).start()
    print('sonar view on http://localhost:%d/   (Ctrl-C to stop)' % a.port)
    ThreadingHTTPServer(('0.0.0.0', a.port), H).serve_forever()


if __name__ == '__main__':
    main()
