#!/usr/bin/env python3
"""Preview the actual embedded dashboard with explicitly simulated telemetry.
Run python3 tools/dashboard_preview.py and open http://127.0.0.1:8767/.
This server does not communicate with a device or write project data.
"""
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import struct
import time

ROOT = Path(__file__).resolve().parents[1]
START = time.monotonic()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def do_GET(self):
        elapsed = time.monotonic() - START
        t = elapsed * 2 * math.pi / 30
        radius = 90
        speed = radius * 2 * math.pi / 30
        east, north = radius * math.cos(t), radius * math.sin(t)
        lat, lon = 37 + north/111320, -122 + east/(111320*math.cos(math.radians(37)))
        course = (-t) % (2*math.pi)
        if self.path == '/':
            page = (ROOT/'sensors/blackbox/web/dashboard.html').read_text()
            page = page.replace('<title>Blackbox</title>', '<title>Blackbox — simulated preview</title>')
            page = page.replace('BLACKBOX<span', 'DEMO / BLACKBOX<span')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(page.encode())
            return
        if self.path.startswith('/api/telemetry'):
            values = [4, 0xAA55, int(elapsed*1000), 0, speed**2/radius, 0, 2*math.pi/30,
                      0, 0, t+math.pi/2, east, north, -speed*math.sin(t), speed*math.cos(t),
                      speed*3.6, 2.5, 4, lat, lon, 1, course, 0, 0, 0, 0]
            packet = bytearray(struct.pack('<BHI13fBffBfIHBH', *values))
            struct.pack_into('<H', packet, 80, sum(packet[:-2]) & 0xffff)
            result = {'seq': int(elapsed*25), 'data': base64.b64encode(packet).decode(),
                      'timing': {'state': 'idle', 'revision': 0, 'last_lap_ms': None,
                                 'best_lap_ms': None, 'lat': lat, 'lon': lon,
                                 'gps_valid': True, 'gps_age_ms': 0},
                      'f': {'li': 0, 'lg': 0, 'gw': .3, 'gr': 25, 'pc': 0,
                            'pf': 95, 'rc': 0, 'rf': 95, 'tx': 0, 'ty': 0}}
        elif self.path.startswith('/api/diagnostics'):
            result = {'sensor_rates': {'imu_hz': 200, 'gps_hz': 25, 'loop_hz': 900,
                       'imu_expected': 200, 'gps_expected': 25, 'zupt_per_min': 0, 'ekf_per_gps': 8},
                      'ekf': {'pos_sigma': 2.5, 'vel_sigma': .2, 'yaw_sigma_deg': 2, 'bias_x': 0, 'bias_y': 0},
                      'system': {'heap_free': 100000, 'uptime_s': int(elapsed), 'tx_ok': 100, 'tx_fail': 0},
                      'gps': {'model': 'SIMULATED', 'rate_hz': 25, 'fix': True, 'warmup': True,
                              'satellites': 12, 'hdop': 1, 'ref_lat': 37, 'ref_lon': -122},
                      'wifi': {'mode': 'DEMO', 'ssid': 'SIMULATED DATA'},
                      'config': {'telemetry_hz': 30, 'gps_model': 'demo', 'warmup_fixes': 5},
                      'fusion': {'lon_raw': 0, 'lon_filtered': 0, 'lon_blended': 0,
                                 'gps_weight': .3, 'gps_accel': 0, 'gps_rate': 25, 'gps_rejected': False,
                                 'pitch_corr': 0, 'roll_corr': 0, 'pitch_conf': 95, 'roll_conf': 95,
                                 'yaw_bias': 0, 'yaw_calibrated': True, 'tilt_x': 0, 'tilt_y': 0, 'tilt_valid': True}}
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(json.dumps(result, allow_nan=False).encode())


if __name__ == '__main__':
    print('Simulated dashboard: http://127.0.0.1:8767/', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 8767), Handler).serve_forever()
