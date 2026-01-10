#!/usr/bin/env python3
"""
High-speed UDP telemetry server
Receives binary packets from ESP32 at 20+ Hz
No connection management - just receive packets!
"""

import socket
import struct
import time
from datetime import datetime


class TelemetryDecoder:
    # Format: B = version, H = header, I = timestamp, then floats, bytes, checksum
    FORMAT = "=BHIffffffffffffBffBH"
    SIZE = 67

    MODE_NAMES = {
        0: "IDLE",
        1: "ACCEL",
        2: "BRAKE",
        4: "CORNER",
        5: "ACCEL+CORNER",
        6: "BRAKE+CORNER",
    }

    def __init__(self):
        self.packet_count = 0
        self.error_count = 0
        self.last_time = time.time()
        self.packets_per_second = 0
        print(f"📦 Packet size: {self.SIZE} bytes")

    def decode(self, payload):
        if len(payload) != self.SIZE:
            self.error_count += 1
            return None

        try:
            data = struct.unpack(self.FORMAT, payload)
        except struct.error as e:
            self.error_count += 1
            print(f"❌ Unpack error: {e}")
            return None

        # Verify version
        if data[0] != 1:
            self.error_count += 1
            print(f"⚠️  Unknown version: {data[0]} (expected 1)")

        # Verify header
        if data[1] != 0xAA55:
            self.error_count += 1
            print(f"❌ Invalid header: 0x{data[1]:04X} (expected 0xAA55)")
            return None

        # Verify checksum
        checksum_calc = sum(payload[:-2]) & 0xFFFF
        checksum_recv = data[-1]
        if checksum_calc != checksum_recv:
            self.error_count += 1
            print(
                f"⚠️  Checksum mismatch: calc=0x{checksum_calc:04X} recv=0x{checksum_recv:04X}"
            )

        # Update rate
        self.packet_count += 1
        now = time.time()
        if now - self.last_time >= 1.0:
            self.packets_per_second = self.packet_count
            self.packet_count = 0
            self.last_time = now

        return {
            "version": data[0],
            "timestamp_ms": data[2],
            "ax": data[3],
            "ay": data[4],
            "az": data[5],
            "wz": data[6],
            "roll": data[7],
            "pitch": data[8],
            "yaw": data[9],
            "x": data[10],
            "y": data[11],
            "vx": data[12],
            "vy": data[13],
            "speed_kmh": data[14],
            "mode": self.MODE_NAMES.get(data[15], "UNKNOWN"),
            "lat": data[16],
            "lon": data[17],
            "gps_valid": bool(data[18]),
        }


class TelemetryDisplay:
    def __init__(self):
        self.last_print = time.time()
        self.print_full_every = 20
        self.packet_counter = 0

    def print_full(self, data, rate):
        print("\n" + "=" * 100)
        print(
            f"📡 FULL TELEMETRY @ {rate} Hz | {datetime.now().strftime('%H:%M:%S.%f')[:-3]}"
        )
        print("=" * 100)

        print(
            f"🚗 Motion:     Speed: {data['speed_kmh']:6.1f} km/h  |  Mode: {data['mode']}"
        )
        print(f"               Vel:   vx={data['vx']:+6.2f}  vy={data['vy']:+6.2f} m/s")

        roll_deg = data['roll'] * 57.3
        pitch_deg = data['pitch'] * 57.3
        yaw_deg = data['yaw'] * 57.3
        print(f"📐 Orientation: Roll: {roll_deg:+6.1f}°  Pitch: {pitch_deg:+6.1f}°")
        print(f"                Yaw:  {yaw_deg:+6.1f}°")

        wz_deg = data['wz'] * 57.3
        ax, ay, az = data['ax'], data['ay'], data['az']
        print(f"⚡ Accel:       ax={ax:+6.2f}  ay={ay:+6.2f}  az={az:+6.2f} m/s²")
        print(f"               wz={wz_deg:+5.1f}°/s")

        print(f"📍 Position:    x={data['x']:8.2f}  y={data['y']:8.2f} m")
        if data["gps_valid"]:
            print(f"🛰️  GPS:         {data['lat']:.6f}, {data['lon']:.6f}")
        else:
            print("🛰️  GPS:         NO FIX")

    def print_compact(self, data, rate):
        now = time.time()
        if now - self.last_print < 0.05:
            return
        self.last_print = now

        self.packet_counter += 1

        if self.packet_counter % self.print_full_every == 0:
            self.print_full(data, rate)
            return

        print(
            f"[{rate:2d}Hz] "
            f"Spd:{data['speed_kmh']:5.1f}km/h "
            f"Pos:({data['x']:7.1f},{data['y']:7.1f})m "
            f"Vel:({data['vx']:+5.2f},{data['vy']:+5.2f})m/s "
            f"Acc:({data['ax']:+5.2f},{data['ay']:+5.2f},{data['az']:+5.2f})m/s² "
            f"Yaw:{data['yaw'] * 57.3:+5.0f}° "
            f"wz:{data['wz'] * 57.3:+4.0f}°/s "
            f"{data['mode']:6s}"
        )


def main():
    HOST = "0.0.0.0"
    PORT = 9000

    print("🚀 High-Speed UDP Telemetry Server")
    print("=" * 80)
    print(f"📡 Listening on {HOST}:{PORT}")
    print("⚡ Connectionless UDP - no handshakes, just data!")
    print("=" * 80)

    decoder = TelemetryDecoder()
    display = TelemetryDisplay()

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))

    print("✅ Server ready - waiting for UDP packets...\n")

    first_packet = True
    last_addr = None

    try:
        while True:
            # Receive UDP packet (blocks until data arrives)
            data, addr = sock.recvfrom(1024)

            # Log new senders
            if addr != last_addr:
                print(f"📨 Receiving from {addr}")
                last_addr = addr

            # Show first packet hex
            if first_packet:
                print(f"First packet (hex): {data.hex()}")
                first_packet = False

            # Decode and display
            decoded = decoder.decode(data)
            if decoded:
                display.print_compact(decoded, decoder.packets_per_second)

    except KeyboardInterrupt:
        print("\n\n👋 Shutting down...")
        sock.close()


if __name__ == "__main__":
    main()
