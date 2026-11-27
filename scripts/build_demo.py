#!/usr/bin/env python3
"""Publish the actual dashboard with a separate, explicitly synthetic data source."""
from pathlib import Path
import hashlib
ROOT = Path(__file__).resolve().parents[1]

def main():
    page = (ROOT/'sensors/blackbox/web/dashboard.html').read_text()
    page = page.replace('<title>Blackbox</title>', '<title>Try Blackbox — interactive GPS lap timer demo</title><meta name="description" content="Explore the actual Blackbox dashboard with simulated telemetry, lap timing, recording, and connection-loss controls. No hardware required."><link rel="canonical" href="https://jctoledo.github.io/blackbox/demo.html">')
    page = page.replace("'blackbox-rec'", "'blackbox-demo-rec'").replace("'blackbox-tracks'", "'blackbox-demo-tracks'")
    page = page.replace('<div class="bbMenuPanel">', '<div class="bbMenuPanel"><p id="demo-note" style="padding:14px 18px;font-size:12px">Synthetic circuit, sensor texture and reference. No measured driving data.</p><button class="bbMenuItem" id="demo-connection">Simulate disconnect</button><button class="bbMenuItem" id="demo-gps-log">Pause GPS timestamp log</button><button class="bbMenuItem" id="demo-drive">Preview Drive recording panel</button><button class="bbMenuItem" id="demo-interval">Simulate rolling interval</button><button class="bbMenuItem" id="demo-launch">Simulate standing start</button>')
    for brand in ('BLACKBOX', 'Diagnostics'):
        page = page.replace(f'<span class="bbBrand">{brand}', f'<span class="bbBrand">{brand} <small style="font-size:9px">SIM</small>')
    revision = hashlib.sha256((ROOT/'docs/demo.js').read_bytes()).hexdigest()[:12]
    page = page.replace('</style>', '.bbClusterHealth::before{content:"SIM";color:var(--text-tertiary)}</style>', 1)
    page = page.replace('<script>', f'<script src="demo.js?v={revision}"></script><script>', 1)
    (ROOT/'docs/demo.html').write_text(page)
    print('Generated isolated synthetic demo from the device dashboard')

if __name__ == '__main__':
    main()
