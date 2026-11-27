#!/usr/bin/env python3
"""Publish the actual dashboard with a separate, explicitly synthetic data source."""
from pathlib import Path
import hashlib
ROOT = Path(__file__).resolve().parents[1]

REPLAY_ROUTE = """
  <section class="bbDemoRoute" id="demo-replay-route" hidden aria-label="Simulated circuit route">
   <svg id="demo-route-map" viewBox="0 0 320 230" role="img" aria-label="Simulated circuit with acceleration and braking sections"></svg>
   <div class="bbDemoRouteKey"><span class="bbDemoAccel">Acceleration</span><span class="bbDemoBrake">Braking</span><span>±0.10 g</span></div>
   <label>Jump to<select id="demo-route-event">
    <option value="">Choose a moment</option>
    <option value="bend">Strongest bend</option>
    <option value="brake">Hardest braking</option>
    <option value="accel">Strongest acceleration</option>
   </select></label>
  </section>"""


def replace_once(page, anchor, replacement):
    if page.count(anchor) != 1:
        raise ValueError(f'Demo integration anchor missing or ambiguous: {anchor}')
    return page.replace(anchor, replacement, 1)


def main():
    page = (ROOT/'sensors/blackbox/web/dashboard.html').read_text()
    page = page.replace('<title>Blackbox</title>', '<title>Try Blackbox — interactive GPS lap timer demo</title><meta name="description" content="Explore the actual Blackbox dashboard with simulated telemetry, lap timing, recording, and connection-loss controls. No hardware required."><link rel="canonical" href="https://jctoledo.github.io/blackbox/demo.html">')
    page = page.replace("'blackbox-rec'", "'blackbox-demo-rec'").replace("'blackbox-tracks'", "'blackbox-demo-tracks'")
    page = page.replace('<div class="bbMenuPanel">', '<div class="bbMenuPanel"><p id="demo-note" style="padding:14px 18px;font-size:12px">Synthetic circuit, sensor texture and reference. No measured driving data.</p><button class="bbMenuItem" id="demo-connection">Simulate disconnect</button><button class="bbMenuItem" id="demo-gps-log">Pause GPS timestamp log</button><button class="bbMenuItem" id="demo-drive">Preview Drive recording panel</button><button class="bbMenuItem" id="demo-interval">Simulate rolling interval</button><button class="bbMenuItem" id="demo-launch">Simulate standing start</button>')
    for brand in ('BLACKBOX', 'Diagnostics'):
        page = page.replace(f'<span class="bbBrand">{brand}', f'<span class="bbBrand">{brand} <small style="font-size:9px">SIM</small>')
    revision = hashlib.sha256((ROOT/'docs/demo.js').read_bytes()).hexdigest()[:12]
    route_revision = hashlib.sha256((ROOT/'docs/demo-route.css').read_bytes()).hexdigest()[:12]
    route_body = '<div class="bbReplayBody">'
    page = replace_once(page, route_body, route_body + REPLAY_ROUTE)
    update_meter = "    drawForceMap($('replay-canvas'),frame.trail,Number($('replay-scale').value),frame.time);"
    page = replace_once(page, update_meter, update_meter + '\n    window.renderDemoRoute?.(replayLoadedId,frame.time);')
    page = replace_once(page, '</head>', f'<link rel="stylesheet" href="demo-route.css?v={route_revision}">\n</head>')
    page = page.replace('</style>', '.bbClusterHealth::before{content:"SIM";color:var(--text-tertiary)}</style>', 1)
    page = page.replace('<script>', f'<script src="demo.js?v={revision}"></script><script>', 1)
    (ROOT/'docs/demo.html').write_text(page)
    print('Generated isolated synthetic demo from the device dashboard')

if __name__ == '__main__':
    main()
