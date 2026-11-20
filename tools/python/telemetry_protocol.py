"""Little-endian Blackbox wire protocol, shared by UDP, TCP, and MQTT tools."""
import struct
import time


class TelemetryDecoder:
    FORMATS = {1: struct.Struct('<BHI12fBffBH'), 4: struct.Struct('<BHI13fBffBfIHBH')}
    SIZE = FORMATS[4].size
    MODE_NAMES = {0: 'IDLE', 1: 'ACCEL', 2: 'BRAKE', 4: 'CORNER',
                  5: 'ACCEL+CORNER', 6: 'BRAKE+CORNER'}

    def __init__(self):
        self.packet_count = 0
        self.error_count = 0
        self.last_time = time.monotonic()
        self.packets_per_second = 0

    def decode(self, payload):
        layout = self.FORMATS.get(payload[0]) if payload else None
        if layout is None or len(payload) != layout.size:
            self.error_count += 1
            return None
        data = layout.unpack(payload)
        if data[1] != 0xAA55 or (sum(payload[:-2]) & 0xFFFF) != data[-1]:
            self.error_count += 1
            return None
        self.packet_count += 1
        now = time.monotonic()
        if now - self.last_time >= 1:
            self.packets_per_second = self.packet_count / (now - self.last_time)
            self.packet_count = 0
            self.last_time = now
        names = ('ax', 'ay', 'az', 'wz', 'roll', 'pitch', 'yaw', 'x', 'y',
                 'vx', 'vy', 'speed_kmh')
        result = dict(zip(names, data[3:15]))
        result.update(version=data[0], timestamp_ms=data[2])
        offset = 16 if data[0] == 4 else 15
        result.update(mode=self.MODE_NAMES.get(data[offset], 'UNKNOWN'),
                      lat=data[offset + 1], lon=data[offset + 2],
                      gps_valid=bool(data[offset + 3]))
        if data[0] == 4:
            result.update(pos_sigma=data[15], gps_course=data[20],
                          lap_time_ms=data[21], lap_count=data[22], lap_flags=data[23])
        return result


class TelemetryStream:
    """Reassemble fragmented TCP frames and recover after corrupted data."""
    def __init__(self, decoder=None):
        self.decoder = decoder or TelemetryDecoder()
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        results = []
        while len(self.buffer) >= 3:
            layout = self.decoder.FORMATS.get(self.buffer[0])
            if layout is None or self.buffer[1:3] != b'\x55\xaa':
                del self.buffer[0]
                continue
            if len(self.buffer) < layout.size:
                break
            result = self.decoder.decode(self.buffer[:layout.size])
            if result is None:
                del self.buffer[0]
            else:
                del self.buffer[:layout.size]
                results.append(result)
        return results
