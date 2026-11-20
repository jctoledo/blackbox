#!/usr/bin/env python3
"""
Binary telemetry decoder for ESP32-C3 Blackbox telemetry system
Decodes version-1 and version-4 binary packets published to car/telemetry_bin
"""

import paho.mqtt.client as mqtt
import time
from datetime import datetime
from telemetry_protocol import TelemetryDecoder


class TelemetryDisplay:
    """Pretty printer for telemetry data"""

    def __init__(self):
        self.last_print = time.time()
        self.print_interval = 0.1  # Print every 100ms

    def print_full(self, data, rate):
        """Print full telemetry data"""
        print("\n" + "=" * 80)
        print(
            f"📡 Telemetry @ {rate} Hz | Time: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}"
        )
        print("=" * 80)

        print(f"\n🚗 Motion:")
        print(f"  Speed:  {data['speed_kmh']:6.1f} km/h")
        print(f"  Mode:   {data['mode']}")
        print(f"  Vel:    vx={data['vx']:+6.2f} vy={data['vy']:+6.2f} m/s")

        print(f"\n📐 Orientation:")
        print(f"  Roll:   {data['roll'] * 57.3:+6.1f}°")
        print(f"  Pitch:  {data['pitch'] * 57.3:+6.1f}°")
        print(f"  Yaw:    {data['yaw'] * 57.3:+6.1f}°")

        print(f"\n⚡ Acceleration:")
        print(
            f"  ax: {data['ax']:+6.2f}  ay: {data['ay']:+6.2f}  az: {data['az']:+6.2f} m/s²"
        )
        print(f"  wz: {data['wz'] * 57.3:+6.1f} °/s")

        print(f"\n📍 Position:")
        print(f"  x: {data['x']:8.2f}  y: {data['y']:8.2f} m")
        if data["gps_valid"]:
            print(f"  GPS: {data['lat']:.6f}, {data['lon']:.6f}")
        else:
            print(f"  GPS: No fix")

    def print_compact(self, data, rate):
        """Print compact one-line telemetry"""
        now = time.time()
        if now - self.last_print < self.print_interval:
            return
        self.last_print = now

        gps_str = (
            f"{data['lat']:.6f},{data['lon']:.6f}" if data["gps_valid"] else "NO_GPS"
        )

        print(
            f"[{rate:2d}Hz] "
            f"Speed:{data['speed_kmh']:5.1f}km/h "
            f"Mode:{data['mode']:6s} "
            f"Pos:({data['x']:6.1f},{data['y']:6.1f}) "
            f"Yaw:{data['yaw'] * 57.3:+5.0f}° "
            f"GPS:{gps_str}"
        )


def on_connect(client, userdata, flags, rc, properties=None):
    """Callback when connected to MQTT broker"""
    if rc == 0:
        print("✅ Connected to MQTT broker")
        client.subscribe("car/telemetry_bin")
        client.subscribe("car/status")
        print("📥 Subscribed to car/telemetry_bin and car/status")
    else:
        print(f"❌ Connection failed with code {rc}")


def on_message(client, userdata, msg):
    """Callback when message received"""
    decoder = userdata["decoder"]
    display = userdata["display"]

    if msg.topic == "car/telemetry_bin":
        print(f"🔍 Received {len(msg.payload)} bytes on {msg.topic}")
        # Print first 16 bytes as hex for debugging
        hex_preview = " ".join(f"{b:02X}" for b in msg.payload[:16])
        print(f"   Hex: {hex_preview}...")

        data = decoder.decode(msg.payload)
        if data:
            # Print compact by default, full on demand
            display.print_compact(data, decoder.packets_per_second)
            # Uncomment for full display:
            # display.print_full(data, decoder.packets_per_second)

    elif msg.topic == "car/status":
        # Print status messages in JSON
        try:
            print(f"\n📢 Status: {msg.payload.decode('utf-8')}")
        except Exception:
            pass


def main():
    """Main entry point"""
    print("🚀 ESP32-C3 Binary Telemetry Decoder")
    print("=" * 80)

    # Create decoder and display
    decoder = TelemetryDecoder()
    display = TelemetryDisplay()

    # Create MQTT client (v2 API)
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.user_data_set({"decoder": decoder, "display": display})
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to broker
    broker = "192.168.50.46"
    port = 1883

    print(f"\n🔌 Connecting to MQTT broker at {broker}:{port}...")
    try:
        client.connect(broker, port, 60)
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return

    # Start loop
    print("🎧 Listening for telemetry... (Ctrl+C to exit)\n")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n\n👋 Disconnecting...")
        client.disconnect()
        print(f"📊 Statistics:")
        print(f"  Total packets: {decoder.packet_count}")
        print(f"  Errors: {decoder.error_count}")


if __name__ == "__main__":
    main()
