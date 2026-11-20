#!/usr/bin/env python3
"""Stage a checksum-verified release on the same origin as the web flasher."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    repo = os.environ.get('GITHUB_REPOSITORY', 'jctoledo/blackbox')
    version = os.environ.get('VERSION', 'latest')
    endpoint = 'latest' if version == 'latest' else f'tags/{version}'
    release = json.loads(subprocess.check_output(['gh', 'api', f'repos/{repo}/releases/{endpoint}']))
    asset = next(a for a in release['assets'] if a['name'] == 'blackbox.bin')
    digest = asset.get('digest', '')
    if not digest.startswith('sha256:'):
        raise RuntimeError('Release asset has no SHA-256 digest; refusing to stage unverified firmware')
    destination = Path(__file__).resolve().parents[1] / 'docs/firmware'
    destination.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as folder:
        subprocess.run(['gh', 'release', 'download', release['tag_name'], '--repo', repo,
                        '--pattern', 'blackbox.bin', '--dir', folder], check=True)
        data = (Path(folder) / 'blackbox.bin').read_bytes()
        if hashlib.sha256(data).hexdigest() != digest.removeprefix('sha256:'):
            raise RuntimeError('Firmware digest mismatch')
        if len(data) != asset['size'] or len(data) > 4 * 1024 * 1024 or data[:1] != b'\xe9':
            raise RuntimeError('Unexpected merged ESP32 image')
        (destination / 'blackbox.bin').write_bytes(data)
    manifest = dict(version=release['tag_name'], chip='esp32c3', offset=0,
                    size=len(data), sha256=digest.removeprefix('sha256:'),
                    source=asset['browser_download_url'], release=release['html_url'])
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f"Staged verified {manifest['version']} firmware ({len(data)} bytes)")


if __name__ == '__main__':
    main()
