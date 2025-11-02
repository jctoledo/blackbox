# Python Telemetry Tools

Telemetry receivers and decoders for Active Wing.

## Installation

```bash
# Install required dependencies
pip install paho-mqtt

# Or use a virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install paho-mqtt
```

## Tools

### tcp_telemetry_server.py

**Recommended** - High-speed TCP receiver for real-time telemetry at 20+ Hz.

```bash
python3 tcp_telemetry_server.py
```

**Features:**
- Receives binary telemetry packets over TCP
- 20 Hz sustained data rate
- Minimal latency
- Compact display output

**Usage:**
1. Start this script first
2. Boot the ESP32 (it will connect automatically)
3. Watch live telemetry stream

### mqtt_binary_decoder.py

MQTT subscriber for binary telemetry packets.

```bash
python3 mqtt_binary_decoder.py
```

**Features:**
- Subscribes to `car/telemetry_bin` and `car/status` topics
- Decodes binary packets (66 bytes)
- Shows both status messages and telemetry

**Configuration:**
Edit the broker address in the script:
```python
broker = "192.168.50.46"  # Change to your MQTT broker IP
```

### mqtt_decoder.py

Legacy JSON telemetry decoder (for reference).

```bash
python3 mqtt_decoder.py
```

**Note:** The current firmware uses binary telemetry for efficiency. This decoder is kept for reference and compatibility with older firmware versions.

## Telemetry Packet Format

Binary packets are 66 bytes with this structure:

```python
FORMAT = '=HIffffffffffffBffBH'
# H - header (0xAA55)
# I - timestamp_ms
# f - ax, ay, az, wz, roll, pitch (6 floats)
# f - yaw, x, y, vx, vy, speed_kmh (6 floats)
# B - mode (0=IDLE, 1=ACCEL, 2=BRAKE, 3=CORNER)
# f - lat, lon (2 floats)
# B - gps_valid
# H - checksum
```

## Configuration

### TCP Server

Edit `tcp_telemetry_server.py`:
```python
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9000       # TCP port
```

Make sure this matches the `TCP_SERVER` setting in the ESP32 firmware (`src/main.rs`).

### MQTT Broker

Edit the MQTT scripts:
```python
broker = "192.168.50.46"  # Your broker IP
port = 1883               # MQTT port
```

## Custom Integration

Want to build your own dashboard or logger?

### Example: Log to CSV

```python
import struct
import socket
import csv

FORMAT = '=HIffffffffffffBffBH'
SIZE = 66

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', 9000))
    s.listen(1)

    print("Waiting for connection...")
    conn, addr = s.accept()

    with open('telemetry.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'x', 'y', 'vx', 'vy', 'speed_kmh',
                        'ax', 'ay', 'az', 'yaw', 'mode'])

        buffer = b''
        while True:
            buffer += conn.recv(4096)
            while len(buffer) >= SIZE:
                packet = buffer[:SIZE]
                buffer = buffer[SIZE:]

                data = struct.unpack(FORMAT, packet)
                if data[0] == 0xAA55:  # Valid header
                    writer.writerow([
                        data[1],   # timestamp
                        data[9],   # x
                        data[10],  # y
                        data[11],  # vx
                        data[12],  # vy
                        data[13],  # speed_kmh
                        data[2],   # ax
                        data[3],   # ay
                        data[4],   # az
                        data[8],   # yaw
                        data[14]   # mode
                    ])
                    csvfile.flush()
```

### Example: WebSocket Bridge

Forward TCP stream to WebSocket for web dashboards:

```python
import asyncio
import websockets
import socket
import struct

async def tcp_to_websocket(websocket, path):
    # Connect to ESP32
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp:
        tcp.connect(('192.168.1.X', 9000))  # ESP32 IP

        buffer = b''
        while True:
            buffer += tcp.recv(4096)
            while len(buffer) >= 66:
                packet = buffer[:66]
                buffer = buffer[66:]

                # Forward binary packet to WebSocket clients
                await websocket.send(packet)

start_server = websockets.serve(tcp_to_websocket, "localhost", 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
```

## Troubleshooting

### TCP receiver shows no data

**Check:**
- ESP32 and laptop on same network
- TCP_SERVER IP in ESP32 firmware matches laptop IP
- Start receiver before booting ESP32
- Firewall not blocking port 9000

**Test connection:**
```bash
# On laptop, check if port is open
netstat -an | grep 9000

# Try telnet to ESP32 (won't decode, but tests connectivity)
telnet <ESP32_IP> 9000
```

### MQTT not connecting

**Check:**
- MQTT broker is running
- Broker IP correct in script
- ESP32 on same network as broker
- Port 1883 not blocked

**Test MQTT:**
```bash
# Subscribe to topics manually
mosquitto_sub -h 192.168.1.X -t "car/#" -v
```

### Data looks corrupted

**Check:**
- Packet size is exactly 66 bytes
- Header is 0xAA55
- Checksum matches (sum of first 64 bytes)

**Debug:**
```python
# Print packet details
print(f"Size: {len(packet)} bytes")
print(f"Header: 0x{data[0]:04X}")
print(f"Checksum calc: 0x{sum(packet[:-2]) & 0xFFFF:04X}")
print(f"Checksum recv: 0x{data[-1]:04X}")
```

## Contributing

Improvements to these tools are welcome!

- Add new output formats (JSON, InfluxDB, etc.)
- Build visualization dashboards
- Add data analysis features
- Improve error handling

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for guidelines.

## License

MIT License - same as the main Active Wing project.
