#!/usr/bin/env python3
"""Rebuild the pinned ESP Web Tools Android USB adapter on lat."""
from pathlib import Path
import subprocess
import tempfile
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
REVISION = '8db1c6f565e10b1dff028ba8c29dd24ce69f049b'
BASE = f'https://raw.githubusercontent.com/esphome/esp-web-tools/{REVISION}'
FILES = ('index', 'driver', 'port', 'cdc-acm', 'ch34x', 'cp210x', 'ftdi')


def main():
    destination = ROOT / 'docs/vendor'
    destination.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='blackbox-usb-') as temporary:
        source = Path(temporary)
        for name in FILES:
            with urlopen(f'{BASE}/src/util/usb-serial/{name}.ts', timeout=30) as response:
                (source / f'{name}.ts').write_bytes(response.read())
        subprocess.run([
            'npm', 'exec', '--yes', '--package=esbuild@0.25.12', '--', 'esbuild',
            'index.ts', '--bundle', '--format=esm', '--target=es2020',
            '--legal-comments=inline',
            f'--banner:js=// ESP Web Tools USB serial adapter, Apache-2.0; upstream {REVISION}. See usb-serial.LICENSE and README.md.',
            f'--outfile={destination / "usb-serial.mjs"}',
        ], check=True, cwd=source)
    with urlopen(f'{BASE}/LICENSE', timeout=30) as response:
        (destination / 'usb-serial.LICENSE').write_bytes(response.read())


if __name__ == '__main__':
    main()
