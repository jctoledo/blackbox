#!/usr/bin/env python3
"""Check a merged C3 release's application identity before uploading it."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def validate_release_image(data, expected_version):
    """Return metadata only for a bounded C3 image with the expected app version."""
    if not 0x10000 < len(data) <= 4 * 1024 * 1024:
        raise ValueError('Merged image must fit the supported 4 MiB flash layout')
    if data[0] != 0xe9 or data[12:14] != b'\x05\x00':
        raise ValueError('Expected an ESP32-C3 merged bootloader at offset zero')
    applications = []
    for position in range(0x8000, 0x9000, 32):
        magic, kind, subtype, offset, size, name, flags = struct.unpack_from('<HBBII16sI', data, position)
        if magic in (0xffff, 0xebeb):
            break
        if magic != 0x50aa:
            raise ValueError('Invalid partition table entry')
        if kind == 0:
            applications.append((offset, size))
    if len(applications) != 1:
        raise ValueError('Expected the supported single-application partition layout')
    offset, size = applications[0]
    if offset < 0x9000 or size < 288 or offset + size > len(data):
        raise ValueError('Application partition exceeds the merged image')
    if data[offset] != 0xe9 or data[offset + 12:offset + 14] != b'\x05\x00':
        raise ValueError('Expected an ESP32-C3 application header')
    segment_size = struct.unpack_from('<I', data, offset + 28)[0]
    descriptor = offset + 32
    if segment_size < 256 or descriptor + segment_size > offset + size:
        raise ValueError('Application descriptor segment exceeds its partition')
    if struct.unpack_from('<I', data, descriptor)[0] != 0xabcd5432:
        raise ValueError('Application descriptor magic is missing')

    def string_at(start, length):
        """Read a fixed-size ASCII descriptor field without trailing padding."""
        return data[start:start + length].split(b'\0', 1)[0].decode('ascii')

    version = string_at(descriptor + 16, 32)
    if version != expected_version:
        raise ValueError(f'Application version {version!r} does not match release {expected_version!r}')
    return {'version': version, 'project': string_at(descriptor + 48, 32),
            'idf': string_at(descriptor + 112, 32), 'app_offset': offset,
            'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def main():
    """Validate the selected release artifact and print non-secret metadata."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--expected-version', required=True)
    args = parser.parse_args()
    print(json.dumps(validate_release_image(args.image.read_bytes(), args.expected_version), sort_keys=True))


if __name__ == '__main__':
    main()
