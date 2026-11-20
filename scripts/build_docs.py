#!/usr/bin/env python3
"""Render repository guides as static HTML; no client JavaScript needed to read them."""
import html
from pathlib import Path
import re
import markdown

ROOT = Path(__file__).resolve().parents[1]
BASE = 'https://jctoledo.github.io/blackbox/'
PAGES = {'LAP_TIMER.md': ('lap-timer', 'GPS lap timer setup and troubleshooting'),
         'SENSOR_FUSION.md': ('sensor-fusion', 'GPS and IMU sensor fusion in Rust'),
         'REVIEW.md': ('engineering-review', 'Engineering review, accuracy claims, and validation')}
STYLE = '''body{margin:0;background:#f5f5ec;color:#172b23;font:17px/1.8 system-ui,sans-serif}header,main,footer{max-width:880px;margin:auto;padding:28px}header{border-bottom:1px solid #ced6c7;display:flex;gap:24px;flex-wrap:wrap}a{color:#356d31}header a{font-size:14px}h1,h2,h3{line-height:1.2;letter-spacing:-.025em}h1{font-size:clamp(32px,5vw,48px)}h2{margin-top:2em}pre{background:#172b23;color:#e5eddd;overflow:auto;padding:20px;border-radius:8px;font-size:13px}code{font-size:.85em}table{display:block;overflow:auto;border-collapse:collapse;font-size:14px}th,td{padding:12px;border:1px solid #ced6c7;text-align:left;min-width:100px}blockquote{border-left:3px solid #7b9c60;padding-left:20px;color:#526458}img{max-width:100%;height:auto}footer{font-size:13px;color:#526458}:focus-visible{outline:3px solid #c78029;outline-offset:3px}h2,h3{scroll-margin-top:20px}'''


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
    urls = [BASE]
    for source, (slug, description) in PAGES.items():
        content = markdown.markdown((ROOT / 'docs' / source).read_text(),
                                    extensions=['fenced_code', 'tables', 'toc'])
        content = re.sub(r'href="([^"]+)"', link, content)
        url = BASE + f'guides/{slug}.html'
        title = html.escape(description + ' | Blackbox')
        document = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title><meta name="description" content="{html.escape(description)}: implementation, setup, known limits, and reproducible checks for the open-source ESP32-C3 telemetry project."><link rel="canonical" href="{url}"><meta name="robots" content="index,follow,max-image-preview:large"><meta property="og:type" content="article"><meta property="og:title" content="{title}"><meta property="og:url" content="{url}"><style>{STYLE}</style></head><body><header><a href="../">BLACKBOX /</a><a href="lap-timer.html">Lap timing</a><a href="sensor-fusion.html">Sensor fusion</a><a href="engineering-review.html">Validation</a></header><main>{content}</main><footer>Open source under MIT. <a href="https://github.com/jctoledo/blackbox/blob/main/docs/{source}">View the maintained source</a>. Published firmware may lag source changes.</footer></body></html>'''
        (directory / f'{slug}.html').write_text(document)
        urls.append(url)
    sitemap = '<?xml version="1.0" encoding="UTF-8"?>\n<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">\n'
    sitemap += ''.join(f'  <url><loc>{url}</loc></url>\n' for url in urls)
    (ROOT / 'docs/sitemap.xml').write_text(sitemap + '</urlset>\n')
    print(f'Rendered {len(PAGES)} guides and sitemap')


if __name__ == '__main__':
    main()
