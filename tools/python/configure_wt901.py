#!/usr/bin/env python3
"""
Configure WitMotion WT901 IMU for 200Hz output at 115200 baud.

This script sends configuration commands to the WT901 and saves them to EEPROM.
Run this ONCE to permanently configure your IMU.

Usage:
    python3 configure_wt901.py /dev/ttyUSB0

After running:
    1. The IMU will switch to 115200 baud
    2. Output rate will be 200 Hz
    3. Settings are saved to EEPROM (persist across power cycles)

Wiring (connect IMU to USB-serial adapter, NOT to ESP32):
    IMU TX  -> Adapter RX
    IMU RX  -> Adapter TX
    IMU GND -> Adapter GND
    IMU VCC -> Adapter 5V (or 3.3V)
"""

import sys
import time
import serial

# WT901 Configuration Commands (from datasheet)
# Format: 0xFF 0xAA [register] [value_low] [value_high]

CMD_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])  # Unlock for configuration
CMD_SAVE = bytes([0xFF, 0xAA, 0x00, 0x00, 0x00])    # Save to EEPROM

# Rate register (0x03): 0x0B = 200Hz
CMD_RATE_200HZ = bytes([0xFF, 0xAA, 0x03, 0x0B, 0x00])
CMD_RATE_100HZ = bytes([0xFF, 0xAA, 0x03, 0x09, 0x00])
CMD_RATE_50HZ = bytes([0xFF, 0xAA, 0x03, 0x08, 0x00])
CMD_RATE_20HZ = bytes([0xFF, 0xAA, 0x03, 0x07, 0x00])
CMD_RATE_10HZ = bytes([0xFF, 0xAA, 0x03, 0x06, 0x00])

# Baud rate register (0x04): 0x06 = 115200
CMD_BAUD_115200 = bytes([0xFF, 0xAA, 0x04, 0x06, 0x00])
CMD_BAUD_9600 = bytes([0xFF, 0xAA, 0x04, 0x02, 0x00])


def send_command(ser, cmd, description, delay=0.1):
    """Send a command and wait for it to be processed."""
    print(f"  Sending: {description}... ", end="", flush=True)
    ser.write(cmd)
    ser.flush()
    time.sleep(delay)
    print("OK")


def try_configure_at_baud(port, baud, target_rate_cmd, target_rate_name, target_baud_cmd=None, target_baud=None):
    """Try to configure the IMU at a specific baud rate."""
    print(f"\nTrying to connect at {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)  # Wait for connection to stabilize

        # Clear any pending data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Read a bit to check if we're getting valid data
        print("  Checking for IMU data... ", end="", flush=True)
        data = ser.read(100)

        # Look for WT901 header byte (0x55)
        if 0x55 in data:
            print(f"Found IMU data! ({len(data)} bytes, header found)")

            # Send configuration sequence
            print("\nConfiguring IMU:")

            # 1. Unlock
            send_command(ser, CMD_UNLOCK, "Unlock command", delay=0.2)

            # 2. Set baud rate (if changing)
            if target_baud_cmd and target_baud and target_baud != baud:
                send_command(ser, target_baud_cmd, f"Set baud to {target_baud}", delay=0.3)
                # After baud change, need to reconnect
                ser.close()
                print(f"  Reconnecting at {target_baud} baud...")
                time.sleep(0.5)
                ser = serial.Serial(port, target_baud, timeout=1)
                time.sleep(0.5)
                # Unlock again after baud change
                send_command(ser, CMD_UNLOCK, "Unlock command (after baud change)", delay=0.2)

            # 3. Set output rate
            send_command(ser, target_rate_cmd, f"Set rate to {target_rate_name}", delay=0.2)

            # 4. Save configuration
            send_command(ser, CMD_SAVE, "Save to EEPROM", delay=0.5)

            print("\nConfiguration complete!")

            # Verify by reading data
            print("\nVerifying configuration (reading data for 2 seconds)...")
            ser.reset_input_buffer()
            start = time.time()
            packet_count = 0

            while time.time() - start < 2.0:
                data = ser.read(100)
                packet_count += data.count(0x55)

            measured_hz = packet_count / 2.0 / 3  # Divide by 3 because WT901 sends 3 packet types per cycle
            final_baud = target_baud if target_baud else baud
            print(f"  Measured rate: ~{measured_hz:.0f} Hz at {final_baud} baud")

            ser.close()
            return True
        else:
            print(f"No valid data (got {len(data)} bytes, no header)")
            ser.close()
            return False

    except serial.SerialException as e:
        print(f"  Failed: {e}")
        return False


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 configure_wt901.py <serial_port> [rate]")
        print("")
        print("Examples:")
        print("  python3 configure_wt901.py /dev/ttyUSB0          # Configure for 200Hz")
        print("  python3 configure_wt901.py /dev/ttyUSB0 100      # Configure for 100Hz")
        print("  python3 configure_wt901.py /dev/ttyUSB0 50       # Configure for 50Hz")
        print("")
        print("Available serial ports:")
        import glob
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        for p in ports:
            print(f"  {p}")
        if not ports:
            print("  (none found)")
        sys.exit(1)

    port = sys.argv[1]

    # Parse target rate
    target_rate = 200
    if len(sys.argv) >= 3:
        target_rate = int(sys.argv[2])

    rate_commands = {
        10: (CMD_RATE_10HZ, "10Hz"),
        20: (CMD_RATE_20HZ, "20Hz"),
        50: (CMD_RATE_50HZ, "50Hz"),
        100: (CMD_RATE_100HZ, "100Hz"),
        200: (CMD_RATE_200HZ, "200Hz"),
    }

    if target_rate not in rate_commands:
        print(f"Error: Invalid rate {target_rate}. Choose from: {list(rate_commands.keys())}")
        sys.exit(1)

    target_rate_cmd, target_rate_name = rate_commands[target_rate]

    print("=" * 50)
    print("WitMotion WT901 Configuration Tool")
    print("=" * 50)
    print(f"Port: {port}")
    print(f"Target rate: {target_rate_name}")
    print(f"Target baud: 115200")
    print("")
    print("This will permanently configure your IMU.")
    print("The settings will persist across power cycles.")
    print("")

    # Try common baud rates (IMU might be at factory default 9600 or already at 115200)
    baud_rates_to_try = [9600, 115200, 19200, 38400, 57600]

    for baud in baud_rates_to_try:
        # If we're at 9600 and want 200Hz, we need to switch to 115200
        if target_rate >= 100:
            success = try_configure_at_baud(
                port, baud,
                target_rate_cmd, target_rate_name,
                CMD_BAUD_115200, 115200
            )
        else:
            # For lower rates, just set the rate without changing baud
            success = try_configure_at_baud(
                port, baud,
                target_rate_cmd, target_rate_name
            )

        if success:
            print("\n" + "=" * 50)
            print("SUCCESS! Your WT901 is now configured for:")
            print(f"  - Output rate: {target_rate_name}")
            print(f"  - Baud rate: 115200" if target_rate >= 100 else f"  - Baud rate: {baud}")
            print("")
            print("You can now use it with your ESP32 at 115200 baud.")
            print("=" * 50)
            sys.exit(0)

    print("\n" + "=" * 50)
    print("FAILED: Could not communicate with IMU at any baud rate.")
    print("")
    print("Troubleshooting:")
    print("  1. Check that the serial port is correct")
    print("  2. Check wiring (TX->RX, RX->TX, GND->GND, 5V->VCC)")
    print("  3. Try unplugging and replugging the USB adapter")
    print("  4. Make sure no other program is using the port")
    print("=" * 50)
    sys.exit(1)


if __name__ == "__main__":
    main()
