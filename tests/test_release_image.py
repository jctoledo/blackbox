"""Release metadata regressions; full firmware is also checked on lat."""
import importlib.util
from pathlib import Path
import struct
import unittest

SPEC = importlib.util.spec_from_file_location('check_release_image',
                                             Path(__file__).resolve().parents[1] / 'scripts/check_release_image.py')
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


def image():
    """Construct the supported merged layout with a minimal descriptor segment."""
    data = bytearray(b'\xff' * 0x20000)
    data[0] = data[0x10000] = 0xe9
    data[12:14] = data[0x1000c:0x1000e] = b'\x05\x00'
    struct.pack_into('<HBBII16sI', data, 0x8000, 0x50aa, 0, 0, 0x10000, 0x10000, b'factory', 0)
    struct.pack_into('<I', data, 0x1001c, 256)
    data[0x10020:0x10120] = bytes(256)
    struct.pack_into('<I', data, 0x10020, 0xabcd5432)
    data[0x10030:0x1003d] = b'v0.0.4-rc.10\0'
    data[0x10050:0x1005a] = b'libespidf\0'
    return data


class ReleaseImageTests(unittest.TestCase):
    """Reject mislabeled releases and malformed/unsupported image layouts."""
    def test_expected_identity(self):
        """Keep the actual SDK project name separate from the release version."""
        result = CHECK.validate_release_image(image(), 'v0.0.4-rc.10')
        self.assertEqual(result['project'], 'libespidf')
        self.assertEqual(result['app_offset'], 0x10000)
        self.assertEqual(len(result['sha256']), 64)

    def test_wrong_tag(self):
        """A valid image cannot be uploaded under a different release tag."""
        with self.assertRaisesRegex(ValueError, 'does not match'):
            CHECK.validate_release_image(image(), 'v0.0.4-rc.11')

    def test_bad_layout(self):
        """Reject wrong chip, malformed partitions and missing app descriptors."""
        for offset, value in [(12, 9), (0x8000, 0), (0x10000, 0), (0x1000c, 9), (0x10020, 0)]:
            with self.subTest(offset=offset):
                data = image()
                data[offset] = value
                with self.assertRaises(ValueError):
                    CHECK.validate_release_image(data, 'v0.0.4-rc.10')

    def test_partition_and_segment_bounds(self):
        """Reject a descriptor/segment or partition extending beyond the image."""
        for offset in [0x8008, 0x1001c]:
            data = image()
            struct.pack_into('<I', data, offset, 0x20000)
            with self.assertRaises(ValueError):
                CHECK.validate_release_image(data, 'v0.0.4-rc.10')
