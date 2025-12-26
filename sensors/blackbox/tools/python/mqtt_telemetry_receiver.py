#!/usr/bin/env python3
"""
MQTT telemetry receiver - uses shared telemetry module
Supports binary and JSON protocols
"""

import argparse
import paho.mqtt.client as mqtt
from telemetry_common import (
    TelemetryDecoder, TelemetryDisplay, CompactFormatter, DetailedFormatter
)


class MqttTelemetryReceiver:
    """MQTT receiver implementation"""

    def __init__(self, broker: str, port: int, decoder: TelemetryDecoder, display: TelemetryDisplay):
        self.broker = broker
        self.port = port
        self.decoder = decoder
        self.display = display
        self.client = None

    def connect(self) -> None:
        """Connect to MQTT broker"""
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.user_data_set({
            'decoder': self.decoder,
            'display': self.display,
        })
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        print(f"🔌 Connecting to MQTT broker at {self.broker}:{self.port}...")
        try:
            self.client.connect(self.broker, self.port, 60)
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            raise

    def run(self) -> None:
        """Start receiving telemetry"""
        print("🎧 Listening for telemetry... (Ctrl+C to exit)\n")
        try:
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n\n👋 Disconnecting...")
            self.disconnect()

    def disconnect(self) -> None:
        """Disconnect from MQTT broker"""
        if self.client:
            self.client.disconnect()
            stats = self.decoder.get_stats()
            print(f"📊 Statistics:")
            print(f"  Total packets: {stats['packets']}")
            print(f"  Errors: {stats['errors']}")

    @staticmethod
    def _on_connect(client, userdata, flags, rc, properties=None):
        """Callback when connected to MQTT broker"""
        if rc == 0:
            print("✅ Connected to MQTT broker")
            client.subscribe("car/telemetry_bin")
            client.subscribe("car/status")
            print("📥 Subscribed to car/telemetry_bin and car/status")
        else:
            print(f"❌ Connection failed with code {rc}")

    @staticmethod
    def _on_message(client, userdata, msg):
        """Callback when message received"""
        decoder = userdata['decoder']
        display = userdata['display']

        if msg.topic == "car/telemetry_bin":
            data = decoder.decode(msg.payload)
            if data:
                display.display(data, decoder.packets_per_second)

        elif msg.topic == "car/status":
            try:
                print(f"\n📢 Status: {msg.payload.decode('utf-8')}")
            except:
                pass


def main():
    """Main entry point with argument parsing"""
    parser = argparse.ArgumentParser(
        description='ESP32-C3 MQTT Telemetry Receiver',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--broker', '-b',
        default='192.168.50.46',
        help='MQTT broker IP address'
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=1883,
        help='MQTT broker port'
    )
    parser.add_argument(
        '--format', '-f',
        choices=['compact', 'detailed'],
        default='compact',
        help='Display format'
    )
    parser.add_argument(
        '--update-rate', '-r',
        type=float,
        default=0.1,
        help='Display update interval in seconds'
    )

    args = parser.parse_args()

    print("🚀 ESP32-C3 MQTT Telemetry Receiver")
    print("="*80)

    # Create components
    decoder = TelemetryDecoder()
    formatter = DetailedFormatter() if args.format == 'detailed' else CompactFormatter()
    display = TelemetryDisplay(formatter, args.update_rate)

    # Create and run receiver
    receiver = MqttTelemetryReceiver(args.broker, args.port, decoder, display)
    receiver.connect()
    receiver.run()


if __name__ == "__main__":
    main()
