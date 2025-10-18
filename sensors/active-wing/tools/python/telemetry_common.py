#!/usr/bin/env python3
"""
Shared telemetry decoding and display functionality
Eliminates code duplication across MQTT and TCP receivers
"""

import struct
import time
from datetime import datetime
from abc import ABC, abstractmethod
from typing import Optional, Dict


class TelemetryDecoder:
    """Decoder for binary telemetry packets (v1 protocol)"""

    # Protocol version 1: 66 bytes
    FORMAT = '=BHIffffffffffffBffBH'  # Added version byte
    SIZE = 67  # Updated for version field

    MODE_NAMES = ['IDLE', 'ACCEL', 'BRAKE', 'CORNER']

    def __init__(self):
        self.packet_count = 0
        self.error_count = 0
        self.last_time = time.time()
        self.packets_per_second = 0
        print(f"📦 Packet size: {self.SIZE} bytes (v1 protocol)")

    def decode(self, payload: bytes) -> Optional[Dict]:
        """Decode binary packet to dictionary"""
        if len(payload) != self.SIZE:
            self.error_count += 1
            print(f"❌ Invalid packet size: {len(payload)} (expected {self.SIZE})")
            return None

        try:
            data = struct.unpack(self.FORMAT, payload)
        except struct.error as e:
            self.error_count += 1
            print(f"❌ Unpack error: {e}")
            return None

        # Verify version
        version = data[0]
        if version != 1:
            self.error_count += 1
            print(f"❌ Unsupported protocol version: {version}")
            return None

        # Verify header
        if data[1] != 0xAA55:
            self.error_count += 1
            print(f"❌ Invalid header: 0x{data[1]:04X} (expected 0xAA55)")
            return None

        # Verify checksum (last field in struct)
        checksum_calc = sum(payload[:-2]) & 0xFFFF
        checksum_recv = data[-1]
        if checksum_calc != checksum_recv:
            self.error_count += 1
            print(f"⚠️  Checksum mismatch: calc=0x{checksum_calc:04X} recv=0x{checksum_recv:04X}")
            # Continue anyway for debugging

        # Update packet rate
        self.packet_count += 1
        now = time.time()
        if now - self.last_time >= 1.0:
            self.packets_per_second = self.packet_count
            self.packet_count = 0
            self.last_time = now

        # Build dictionary (indices match struct order)
        return {
            'version': version,
            'timestamp_ms': data[2],
            'ax': data[3],
            'ay': data[4],
            'az': data[5],
            'wz': data[6],
            'roll': data[7],
            'pitch': data[8],
            'yaw': data[9],
            'x': data[10],
            'y': data[11],
            'vx': data[12],
            'vy': data[13],
            'speed_kmh': data[14],
            'mode': self.MODE_NAMES[data[15]] if data[15] < 4 else 'UNKNOWN',
            'lat': data[16],
            'lon': data[17],
            'gps_valid': bool(data[18]),
        }

    def get_stats(self) -> Dict[str, int]:
        """Get decoder statistics"""
        return {
            'packets': self.packet_count,
            'errors': self.error_count,
            'rate_hz': self.packets_per_second,
        }


class TelemetryFormatter(ABC):
    """Abstract base class for telemetry formatters"""

    @abstractmethod
    def format(self, data: Dict, rate: int) -> str:
        """Format telemetry data as string"""
        pass


class CompactFormatter(TelemetryFormatter):
    """Single-line compact telemetry formatter"""

    def format(self, data: Dict, rate: int) -> str:
        gps_str = f"{data['lat']:.6f},{data['lon']:.6f}" if data['gps_valid'] else "NO_GPS"

        return (f"[{rate:2d}Hz] "
                f"Speed:{data['speed_kmh']:5.1f}km/h "
                f"Mode:{data['mode']:6s} "
                f"Pos:({data['x']:6.1f},{data['y']:6.1f}) "
                f"Yaw:{data['yaw']*57.3:+5.0f}° "
                f"GPS:{gps_str}")


class DetailedFormatter(TelemetryFormatter):
    """Multi-line detailed telemetry formatter"""

    def format(self, data: Dict, rate: int) -> str:
        lines = [
            "\n" + "="*80,
            f"📡 Telemetry @ {rate} Hz | Time: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}",
            "="*80,
            f"\n🚗 Motion:",
            f"  Speed:  {data['speed_kmh']:6.1f} km/h",
            f"  Mode:   {data['mode']}",
            f"  Vel:    vx={data['vx']:+6.2f} vy={data['vy']:+6.2f} m/s",
            f"\n📐 Orientation:",
            f"  Roll:   {data['roll']*57.3:+6.1f}°",
            f"  Pitch:  {data['pitch']*57.3:+6.1f}°",
            f"  Yaw:    {data['yaw']*57.3:+6.1f}°",
            f"\n⚡ Acceleration:",
            f"  ax: {data['ax']:+6.2f}  ay: {data['ay']:+6.2f}  az: {data['az']:+6.2f} m/s²",
            f"  wz: {data['wz']*57.3:+6.1f} °/s",
            f"\n📍 Position:",
            f"  x: {data['x']:8.2f}  y: {data['y']:8.2f} m",
        ]

        if data['gps_valid']:
            lines.append(f"  GPS: {data['lat']:.6f}, {data['lon']:.6f}")
        else:
            lines.append(f"  GPS: No fix")

        return '\n'.join(lines)


class RateLimiter:
    """Rate limiter for display updates"""

    def __init__(self, interval_sec: float):
        self.interval = interval_sec
        self.last_time = time.time()

    def should_update(self) -> bool:
        """Check if enough time has passed for next update"""
        now = time.time()
        if now - self.last_time >= self.interval:
            self.last_time = now
            return True
        return False


class TelemetryDisplay:
    """Display manager with rate limiting and formatting"""

    def __init__(self, formatter: TelemetryFormatter, update_interval: float = 0.05):
        self.formatter = formatter
        self.rate_limiter = RateLimiter(update_interval)

    def display(self, data: Dict, rate: int) -> None:
        """Display telemetry if rate limit allows"""
        if self.rate_limiter.should_update():
            print(self.formatter.format(data, rate))

    def set_formatter(self, formatter: TelemetryFormatter) -> None:
        """Change display format at runtime"""
        self.formatter = formatter


class TelemetryReceiver(ABC):
    """Abstract base class for telemetry receivers"""

    def __init__(self, decoder: TelemetryDecoder, display: TelemetryDisplay):
        self.decoder = decoder
        self.display = display

    @abstractmethod
    def connect(self, *args, **kwargs) -> None:
        """Connect to telemetry source"""
        pass

    @abstractmethod
    def run(self) -> None:
        """Start receiving telemetry"""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from telemetry source"""
        pass
