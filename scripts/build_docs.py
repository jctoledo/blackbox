#!/usr/bin/env python3
"""Render repository guides as static HTML; no client JavaScript needed to read them."""
import html
from pathlib import Path
import re
import markdown
from build_demo import main as build_demo

ROOT = Path(__file__).resolve().parents[1]
BASE = 'https://jctoledo.github.io/blackbox/'
PAGES = {'BACKUP.md': ('backup', 'Back up and restore long Blackbox recordings safely'),
         'RECORDING_SUMMARY.md': ('recording-summary', 'Saved drive summaries: speed, sustained forces and evidence gaps'),
         'DRIVE.md': ('drive-recording', 'Drive recording: elapsed duration, partial GPS distance and browser recovery'),
         'FIRST_DRIVE.md': ('first-drive', 'First Blackbox drive: NEO-M9N setup, recording, exports and photos'),
         'DIY_GPS_LAP_TIMER.md': ('diy-gps-lap-timer', 'Build a DIY GPS lap timer with ESP32-C3 and a phone dashboard'),
         'M9N_GPS_LOGGER.md': ('neo-m9n-esp32-gps-logger', 'NEO-M9N GPS logging with ESP32: capture and inspect 25 Hz evidence'),
         'GPS_TIMING_ACCURACY.md': ('gps-lap-timer-accuracy', 'GPS lap timer accuracy: sample rates, interpolation and measured evidence'),
         'BUILD.md': ('build', 'Build Blackbox: parts, wiring, mounting and the assembly journal'),
         'RUNS.md': ('rolling-runs', 'Receiver-timestamped rolling acceleration estimates and evidence'),
         'MATHS_AND_RATES.md': ('maths-and-rates', 'GPS lap timing maths, G-meter estimates and hardware update rates'),
         'GPS_LOGGING.md': ('gps-logging', 'GPS timestamp logging and drive-data analysis'),
         'CALIBRATION.md': ('mount-alignment', 'Experimental horizontal mount alignment: setup and validation'),
         'COMPARISON.md': ('comparison', 'Blackbox vs paid GPS lap timers: capabilities, costs, and evidence'),
         'FAQ.md': ('faq', 'GPS lap timer compatibility, accuracy, and ownership FAQ'),
         'HARDWARE_VALIDATION.md': ('hardware-validation', 'Blackbox hardware and lap timing acceptance checklist'),
         'LAP_TIMER.md': ('lap-timer', 'GPS lap timer setup and troubleshooting'),
         'SENSOR_FUSION.md': ('sensor-fusion', 'GPS and IMU sensor fusion in Rust'),
         'ENGINEERING.md': ('engineering-review', 'How Blackbox works: timing, telemetry, and data ownership')}
STYLE = '''body{margin:0;background:#090d12;color:#edf3f8;font:17px/1.8 system-ui,sans-serif}header,main,footer{max-width:880px;margin:auto;padding:28px}header{border-bottom:1px solid #344554;display:flex;gap:24px;flex-wrap:wrap}a{color:#d7ff45}header a{font-size:14px}h1,h2,h3{line-height:1.2;letter-spacing:-.025em}h1{font-size:clamp(32px,5vw,48px)}h2{margin-top:2em}pre{background:#151f29;color:#e5eddd;overflow:auto;padding:20px;border-radius:8px;font-size:13px}code{font-size:.85em}table{display:block;overflow:auto;border-collapse:collapse;font-size:14px}th,td{padding:12px;border:1px solid #344554;text-align:left;min-width:100px}blockquote{border-left:3px solid #7b9c60;padding-left:20px;color:#b1c0cc}img{max-width:100%;height:auto}footer{font-size:13px;color:#b1c0cc}:focus-visible{outline:3px solid #c78029;outline-offset:3px}h2,h3{scroll-margin-top:20px}'''


def link(match):
    target = match.group(1)
    if '://' in target or target.startswith('#'):
        return match.group(0)
    filename = target.split('#')[0].split('/')[-1]
    if filename in PAGES:
        suffix = '#' + target.split('#', 1)[1] if '#' in target else ''
        return f'href="{PAGES[filename][0]}.html{suffix}"'
    return f'href="https://github.com/jctoledo/blackbox/blob/main/docs/{target}"'


def main():
    directory = ROOT / 'docs/guides'
    directory.mkdir(exist_ok=True)
    build_demo()
    urls = [BASE, BASE + "demo.html"]
    for source, (slug, description) in PAGES.items():
        content = markdown.markdown((ROOT / 'docs' / source).read_text(),
                                    extensions=['fenced_code', 'tables', 'toc'])
        content = re.sub(r'href="([^"]+)"', link, content)
        content = content.replace('src="assets/', 'src="../assets/')
        url = BASE + f'guides/{slug}.html'
        title = html.escape(description + ' | Blackbox')
        document = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title><meta name="description" content="{html.escape(description)}: implementation, setup, known limits, and reproducible checks for the open-source ESP32-C3 telemetry project."><link rel="canonical" href="{url}"><meta name="robots" content="index,follow,max-image-preview:large"><meta property="og:type" content="article"><meta property="og:title" content="{title}"><meta property="og:url" content="{url}"><meta property="og:description" content="{html.escape(description)}"><meta property="og:image" content="{BASE}dashboard-cluster-v12.jpg"><meta name="twitter:card" content="summary_large_image"><style>{STYLE}</style></head><body><header><a href="../">BLACKBOX /</a><a href="diy-gps-lap-timer.html">Start here</a><a href="lap-timer.html">Lap timing</a><a href="sensor-fusion.html">Sensor fusion</a><a href="engineering-review.html">How it works</a><a href="build.html">Build</a><a href="maths-and-rates.html">Maths</a></header><main>{content}</main><footer>Open source under MIT. <a href="https://github.com/jctoledo/blackbox/blob/main/docs/{source}">View the maintained source</a>. Published firmware may lag source changes.</footer></body></html>'''
        (directory / f'{slug}.html').write_text(document)
        urls.append(url)
    sitemap = '<?xml version="1.0" encoding="UTF-8"?>\n<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">\n'
    sitemap += ''.join(f'  <url><loc>{url}</loc></url>\n' for url in urls)
    (ROOT / 'docs/sitemap.xml').write_text(sitemap + '</urlset>\n')
    print(f'Rendered {len(PAGES)} guides and sitemap')


if __name__ == '__main__':
    main()
