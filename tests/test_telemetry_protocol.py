import pathlib
import struct
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools/python'))
from telemetry_protocol import TelemetryDecoder, TelemetryStream


class ProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.packet = bytes.fromhex(pathlib.Path('tests/fixtures/telemetry-v4.hex').read_text())

    def test_actual_firmware_packet_layout(self):
        decoded = TelemetryDecoder().decode(self.packet)
        self.assertEqual(len(self.packet), 82)
        self.assertEqual(decoded['speed_kmh'], 72)
        self.assertEqual(decoded['pos_sigma'], 2.5)
        self.assertEqual(decoded['mode'], 'CORNER')
        self.assertEqual(decoded['lon'], -122)
        self.assertEqual(decoded['gps_course'], 1.25)
        self.assertEqual(decoded['lap_time_ms'], 60023)
        self.assertEqual(decoded['lap_count'], 2)
        self.assertEqual(decoded['lap_flags'], 4)

    def test_corrupt_or_unknown_packets_are_rejected(self):
        for index in [0, 1, 15, 81]:
            corrupted = bytearray(self.packet)
            corrupted[index] ^= 0x80
            self.assertIsNone(TelemetryDecoder().decode(corrupted))
        self.assertIsNone(TelemetryDecoder().decode(self.packet[:-1]))

    def test_stream_handles_fragments_and_recovers_after_corruption(self):
        stream = TelemetryStream()
        corrupted = bytearray(self.packet)
        corrupted[-1] ^= 1
        results = []
        for byte in b'garbage' + corrupted + self.packet + self.packet:
            results.extend(stream.feed(bytes([byte])))
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0]['lap_time_ms'], 60023)
        self.assertLess(len(stream.buffer), 3)

    def test_legacy_v1_remains_supported(self):
        packet = bytearray(struct.pack('<BHI12fBffBH', 1, 0xAA55, 42,
                                       *([0.0] * 12), 0, 37.0, -122.0, 1, 0))
        struct.pack_into('<H', packet, len(packet)-2, sum(packet[:-2]) & 0xffff)
        self.assertEqual(TelemetryDecoder().decode(packet)['timestamp_ms'], 42)


if __name__ == '__main__':
    unittest.main()
