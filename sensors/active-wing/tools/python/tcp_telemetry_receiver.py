#!/usr/bin/env python3
"""
TCP telemetry receiver - uses shared telemetry module
High-speed streaming (20+ Hz)
"""

import argparse
import socket
import threading
from telemetry_common import (
    TelemetryDecoder, TelemetryDisplay, CompactFormatter, DetailedFormatter
)


class TcpTelemetryReceiver:
    """TCP receiver implementation"""

    def __init__(self, host: str, port: int, decoder: TelemetryDecoder, display: TelemetryDisplay):
        self.host = host
        self.port = port
        self.decoder = decoder
        self.display = display
        self.server_socket = None
        self.running = False

    def connect(self) -> None:
        """Initialize TCP server socket"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        print(f"✅ TCP server listening on {self.host}:{self.port}")

    def run(self) -> None:
        """Start receiving telemetry"""
        self.running = True
        print(f"⚡ Waiting for ESP32 connection...\n")

        try:
            while self.running:
                conn, addr = self.server_socket.accept()
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True
                )
                client_thread.start()
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
            self.disconnect()

    def disconnect(self) -> None:
        """Close server socket"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
            stats = self.decoder.get_stats()
            print(f"📊 Statistics:")
            print(f"  Total packets: {stats['packets']}")
            print(f"  Errors: {stats['errors']}")
            print(f"  Final rate: {stats['rate_hz']} Hz")

    def _handle_client(self, conn: socket.socket, addr: tuple) -> None:
        """Handle individual client connection"""
        print(f"✅ Client connected from {addr}")
        buffer = b''

        try:
            while self.running:
                chunk = conn.recv(4096)
                if not chunk:
                    print(f"❌ Client {addr} disconnected")
                    break

                buffer += chunk

                # Process complete packets
                while len(buffer) >= self.decoder.SIZE:
                    packet = buffer[:self.decoder.SIZE]
                    buffer = buffer[self.decoder.SIZE:]

                    data = self.decoder.decode(packet)
                    if data:
                        self.display.display(data, self.decoder.packets_per_second)

        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            conn.close()


def main():
    """Main entry point with argument parsing"""
    parser = argparse.ArgumentParser(
        description='ESP32-C3 TCP Telemetry Receiver (High-Speed)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--host', '-H',
        default='0.0.0.0',
        help='TCP server bind address (0.0.0.0 = all interfaces)'
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=9000,
        help='TCP server port'
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
        default=0.05,
        help='Display update interval in seconds (50ms default for 20Hz)'
    )

    args = parser.parse_args()

    print("🚀 ESP32-C3 TCP Telemetry Receiver")
    print("="*80)
    print(f"📡 Optimized for 20+ Hz streaming")
    print("="*80)

    # Create components
    decoder = TelemetryDecoder()
    formatter = DetailedFormatter() if args.format == 'detailed' else CompactFormatter()
    display = TelemetryDisplay(formatter, args.update_rate)

    # Create and run receiver
    receiver = TcpTelemetryReceiver(args.host, args.port, decoder, display)
    receiver.connect()
    receiver.run()


if __name__ == "__main__":
    main()
