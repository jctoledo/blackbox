#!/usr/bin/env python3
"""Bundle shared instrument fonts into the single device-served dashboard."""
import argparse
import base64
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
START = '/* BEGIN BUNDLED INSTRUMENT FONTS */'
END = '/* END BUNDLED INSTRUMENT FONTS */'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    directory = ROOT / 'docs/assets/fonts'
    metadata = json.loads((directory / 'sources.json').read_text())
    rules = [START]
    for slug, spec in metadata.items():
        data = (directory / f'{slug}.woff2').read_bytes()
        if data[:4] != b'wOF2':
            raise ValueError(f'Invalid WOFF2: {slug}')
        encoded = base64.b64encode(data).decode('ascii')
        rules.append(f"@font-face{{font-family:'{spec['family']}';font-style:normal;"
                     f"font-weight:{spec['weight']};font-display:swap;"
                     f"src:url(data:font/woff2;base64,{encoded}) format('woff2')}}")
    rules.append(END)
    page = ROOT / 'sensors/blackbox/web/dashboard.html'
    original = page.read_text()
    generated, count = re.subn(re.escape(START) + '.*?' + re.escape(END),
                              '\n'.join(rules), original, flags=re.S)
    if count != 1:
        raise ValueError('Expected exactly one font embedding block')
    if args.check:
        if generated != original:
            raise SystemExit('Bundled fonts are stale; run scripts/embed_dashboard_fonts.py')
        print('Device fonts match the shared local font assets')
    else:
        page.write_text(generated)
        print('Embedded offline instrument fonts')


if __name__ == '__main__':
    main()
