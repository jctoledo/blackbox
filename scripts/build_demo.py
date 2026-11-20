#!/usr/bin/env python3
"""Publish the actual dashboard with a separate, explicitly synthetic data source."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def main():
    page = (ROOT/'sensors/blackbox/web/dashboard.html').read_text()
    page = page.replace('<title>Blackbox</title>', '<title>Try Blackbox — interactive GPS lap timer demo</title><meta name="description" content="Explore the actual Blackbox dashboard with simulated telemetry, lap timing, recording, and connection-loss controls. No hardware required."><link rel="canonical" href="https://jctoledo.github.io/blackbox/demo.html">')
    page = page.replace("'blackbox-rec'", "'blackbox-demo-rec'").replace("'blackbox-tracks'", "'blackbox-demo-tracks'")
    page = page.replace('<body>', '''<body><aside style="flex-shrink:0;padding:10px 16px;background:#d7ff45;color:#111820;display:grid;grid-template-columns:1fr auto;gap:8px 12px;align-items:center;font:13px/1.4 system-ui"><a href="./" style="color:inherit;font-weight:800">BLACKBOX DEMO</a><span id="demo-note" style="grid-column:1/-1;grid-row:2">Simulated circuit and reference. No measured driving data.</span><button id="demo-connection" style="padding:9px;border:1px solid #111820;background:transparent;border-radius:4px;font:inherit;cursor:pointer">Simulate disconnect</button></aside>''')
    page = page.replace('<script>', '<script src="demo.js"></script><script>', 1)
    (ROOT/'docs/demo.html').write_text(page)
    print('Generated isolated synthetic demo from the device dashboard')

if __name__ == '__main__':
    main()
