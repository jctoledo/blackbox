#!/usr/bin/env python3
"""
Probe WT901 IMU to find its current baud rate.

Usage:
    1. Disconnect IMU from ESP32
    2. Connect IMU to USB-serial adapter (TX->RX, RX->TX, GND, 5V)
    3. Run: python3 probe_wt901.py /dev/ttyUSB0
"""

import sys
import serial
import time

BAUD_RATES = [9600, 115200, 19200, 38400, 57600, 230400]

def probe_baud(port, baud):
    """Try to read IMU data at a specific baud rate."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.3)
        ser.reset_input_buffer()

        # Read for 1 second
        data = ser.read(200)
        ser.close()

        # Count WT901 header bytes (0x55)
        header_count = data.count(0x55)

        if header_count >= 3:
            # Estimate rate (3 packets per IMU cycle: accel, gyro, angle)
            estimated_hz = header_count / 3
            return True, header_count, estimated_hz
        return False, header_count, 0

    except Exception as e:
        return False, 0, str(e)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 probe_wt901.py <serial_port>")
        print("\nAvailable ports:")
        import glob
        for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'):
            print(f"  {p}")
        sys.exit(1)

    port = sys.argv[1]
    print(f"Probing WT901 on {port}...")
    print("=" * 50)

    found = False
    for baud in BAUD_RATES:
        print(f"Trying {baud:6d} baud... ", end="", flush=True)
        success, count, rate = probe_baud(port, baud)

        if success:
            print(f"FOUND! ({count} headers, ~{rate:.0f} Hz)")
            found = True
            print("\n" + "=" * 50)
            print(f"Your WT901 is running at {baud} baud, ~{rate:.0f} Hz")
            print("=" * 50)
            break
        else:
            print(f"no data ({count} headers)")

    if not found:
        print("\n" + "=" * 50)
        print("Could not find IMU at any baud rate.")
        print("Check wiring: TX->RX, RX->TX, GND, 5V/3.3V")
        print("=" * 50)

if __name__ == "__main__":
    main()
