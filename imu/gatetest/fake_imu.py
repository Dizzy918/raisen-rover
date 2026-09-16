#!/usr/bin/env python3
"""Stand-in for the ESP32's T:126 endpoint, serving the scripted profile.

Same URL shape fuse_heading.py builds: GET /js?json={"T":126}
"""
import json
import math
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import gate_profile as gp

T0 = float(sys.argv[1])
PORT = int(sys.argv[2])


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        t = time.time() - T0
        # A little noise, so a gate that only works on clean input fails here.
        n = 0.25 * math.sin(t * 7.3)
        body = json.dumps({
            'T': 1002,
            'r': -0.48, 'p': 0.73,
            'y': gp.wrap(gp.imu_yaw_at(t) + n),
            'ax': -12.8, 'ay': -12.7, 'az': 984.9,
            'gx': 0.31, 'gy': 0.56, 'gz': gp.gz_at(t) + 0.15 * math.sin(t * 11.0),
            'mx': 31, 'my': -48, 'mz': -5, 'temp': 68.9,
        }).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


ThreadingHTTPServer(('127.0.0.1', PORT), H).serve_forever()
