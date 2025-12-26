# Frequently Asked Questions (FAQ)

## General Questions

### What is Blackbox?

Blackbox is an open-source vehicle telemetry system that runs on an ESP32 microcontroller. It combines data from an IMU (accelerometer/gyroscope) and GPS using an Extended Kalman Filter to provide accurate position, velocity, acceleration, and driving dynamics in real-time.

Think of it as a professional-grade data logger that costs $30 instead of $500+.

### Who is this for?

- **Track day enthusiasts** who want telemetry data
- **Engineering students** learning sensor fusion and vehicle dynamics
- **DIY car builders** who want to add data logging
- **Rally drivers** who need navigation and dynamics data
- **Anyone curious** about what their car is actually doing

### Is this legal to use in my car?

Yes, it's a data logging device (observes only, doesn't control anything). However:
- Don't interact with displays while driving
- Mount it securely so it doesn't become a projectile
- Obey all traffic laws
- Check your local regulations about in-car electronics

### How is this different from a phone app?

Phone apps typically:
- Just log GPS (no sensor fusion)
- Low sample rates (1-5 Hz)
- Inaccurate IMU data (phone not rigidly mounted)
- Can't run other apps while logging

Blackbox:
- Fuses GPS + IMU with Kalman filter
- 20 Hz output (200 Hz IMU internal sampling)
- Rigidly mounted = accurate measurements
- Dedicated hardware, always reliable

### How is this different from commercial data loggers?

**Blackbox ($30):**
- Open source (hack it, modify it, learn from it)
- 20 Hz telemetry
- IMU + GPS sensor fusion
- WiFi streaming
- No subscription fees
- Community support

**Commercial loggers ($500-5000):**
- Proprietary hardware/software
- Often 10-20 Hz
- Some do sensor fusion, many don't
- Some have screens/SD cards
- Sometimes subscription required
- Professional support

Blackbox is 90% of the functionality at 5% of the price.

## Hardware Questions

### What ESP32 board should I buy?

Any ESP32-C3 development board works. Common options:
- ESP32-C3-DevKitC-02 (official Espressif board)
- ESP32-C3 Super Mini (tiny, cheap)
- Seeed XIAO ESP32C3

**Requirements:**
- ESP32-C3 (other variants need code changes)
- 2 UARTs available (for IMU and GPS)
- USB programming capability

### Can I use a different IMU?

The current code is written for the WT901. To use a different IMU:
- Modify `src/imu.rs` with your IMU's protocol
- Common alternatives: MPU6050, BMI088, LSM6DSO
- Contributions welcome! See [CONTRIBUTING.md](CONTRIBUTING.md)

### Can I use a different GPS?

The code parses NMEA GPRMC/GNRMC sentences, which are standard. Most GPS modules will work:
- NEO-6M (tested, cheap, works well)
- NEO-M8N (faster lock, better accuracy)
- NEO-M9N (multi-band, even better)

Higher-end GPS modules give faster locks and better accuracy but aren't required.

### Do I need the RGB LED?

No, it's optional. The LED provides visual status feedback which is helpful for debugging, but the system works fine without it. Just remove the LED initialization code from `main.rs`.

### How do I mount this in my car?

**Critical requirements:**
- **Rigid mounting** - Any flex corrupts IMU readings
- **Known orientation** - Code assumes specific IMU orientation
- **Secure** - High g-forces in cornering/braking
- **Protected** - From heat, water, vibration

**Suggested mounting:**
- 3D printed enclosure with screw mounts
- Near center of vehicle (near center of mass)
- Away from heat sources (engine, exhaust)
- Protected from weather if open cockpit

### Can I power this from my car's 12V?

Yes, but you need a regulator. Options:
1. **USB adapter** - Plug into car's USB port (easiest)
2. **12V→5V buck converter** - Hardwire to car's power
3. **Battery bank** - For testing without car power

Never connect ESP32 directly to 12V!

### What about power consumption?

- ESP32-C3: ~160mA active (WiFi on)
- WT901 IMU: ~35mA
- GPS: ~25mA
- LED: ~20mA (optional)

**Total: ~240mA @ 5V = 1.2W**

A typical phone charger (2A / 10W) is more than enough.

## Software Questions

### Do I need to know Rust?

To use it as-is: **No**
- Just edit WiFi credentials in `main.rs`
- Flash the firmware
- Run the Python receiver

To modify it: **Some Rust helps**
- But the code is well-commented
- Start with simple changes (thresholds, rates)
- Check [CLAUDE.md](CLAUDE.md) for architecture

### Can I use this without WiFi?

Not currently - telemetry streams over WiFi. But you could:
- Add SD card logging (modify firmware)
- Use BLE instead of WiFi (would need significant changes)
- Store data in ESP32 flash (limited space)

Contributions welcome for these features!

### How do I change the telemetry rate?

Edit `src/main.rs`:
```rust
const TELEMETRY_INTERVAL_MS: u32 = 50;  // 20 Hz
```

Change to:
- `25` for 40 Hz (might be unstable)
- `100` for 10 Hz (more stable, lower bandwidth)

Higher rates require more CPU/network bandwidth.

### Can I log the data to a file?

The Python receiver (`tools/python/tcp_telemetry_server.py`) currently just displays data. To log it:

**Option 1:** Modify the Python script to write CSV
```python
with open('telemetry.csv', 'w') as f:
    # Write headers
    # Write data rows in the decode callback
```

**Option 2:** Use tee on Linux/Mac
```bash
python3 tools/python/tcp_telemetry_server.py | tee telemetry.log
```

**Option 3:** Contribute an SD card logging feature!

### How accurate is the position tracking?

**With GPS lock:**
- ±1m between GPS updates (5 Hz)
- ±5m from GPS itself (depends on conditions)
- Better with better GPS module (RTK = cm-level)

**Without GPS lock:**
- Dead reckoning for ~30 seconds before drift
- Then increasingly inaccurate
- ZUPT prevents drift when stationary

### What's a Kalman Filter and why do I need it?

Simple explanation:
- GPS is accurate but slow (5 Hz)
- IMU is fast but drifts (200 Hz)
- Kalman filter combines both to get "best of both worlds"

Without sensor fusion, you're either:
- Stuck with 5 Hz GPS (too slow for dynamics)
- Using raw IMU (accumulates huge errors)

The math is in `src/ekf.rs`, explained in [CLAUDE.md](CLAUDE.md).

## Troubleshooting

### GPS never gets a lock

**Check:**
- Antenna has clear view of sky (won't work indoors usually)
- GPS module has power (3.3V, check with multimeter)
- TX/RX wires not swapped (GPS TX → ESP32 RX)
- Baudrate is 9600 (default for NEO-6M)

**Tips:**
- Cold start takes 30-60 seconds outdoors
- Warm start is faster (if GPS hasn't lost power)
- LED will pulse cyan when GPS is locked

### WiFi won't connect

**Check:**
- SSID and password correct in `main.rs`
- WiFi is 2.4 GHz (ESP32 doesn't support 5 GHz)
- Network allows new devices (not restricted)
- Try your phone's hotspot to isolate router issues

**LED status:**
- Green blinks = WiFi connected
- Red blinks = WiFi failed

### TCP receiver shows no data

**Check:**
- TCP server IP in `main.rs` matches your laptop IP
- Start `tools/python/tcp_telemetry_server.py` **before** booting ESP32
- Laptop and ESP32 on same network
- Firewall not blocking port 9000

**Test:**
```bash
# On laptop, see if you can reach ESP32
ping <ESP32_IP>

# Check if port is open (on laptop)
netstat -an | grep 9000
```

### Calibration fails

**Problem:** IMU must be completely stationary during calibration.

**Solution:**
- Don't touch the device during yellow LED phase
- Place on stable surface (not a car with engine running)
- Takes ~10 seconds to collect 500 samples
- If it fails, reset and try again

### Data looks noisy/wrong

**Check:**
- IMU is rigidly mounted (any flex = bad data)
- Calibration completed successfully
- GPS has valid fix (cyan LED pulsing)
- Orientation matches code assumptions

**Tuning:**
- Increase Kalman filter measurement noise (R_* in `ekf.rs`)
- Adjust mode detection thresholds (`mode.rs`)
- Add more EMA filtering (`mode.rs` alpha parameter)

### ESP32 keeps crashing

**Check:**
- Serial output for error messages
- Free heap (logged every 5 seconds)
- Power supply adequate (weak power = crashes)

**Try:**
- Lower telemetry rate (less CPU/RAM load)
- Simplify (disable features to isolate issue)
- Check for memory leaks (heap should be stable)

## Performance Questions

### What's the latency?

**Sensor to output:**
- IMU sampling: ~5ms (200 Hz)
- EKF processing: <1ms
- Packet serialization: <1ms
- TCP transmission: ~5-10ms on local WiFi

**Total: <20ms typical, <50ms worst case**

Much faster than phone apps (100-500ms typical).

### Can I increase the telemetry rate beyond 20 Hz?

Theoretically yes, but:
- ESP32 CPU becomes the bottleneck
- WiFi bandwidth becomes an issue
- Diminishing returns (GPS still only 5 Hz)

40 Hz might be achievable with optimization. 100 Hz unlikely without significant changes.

### How far can I go from the WiFi?

**WiFi range:**
- Depends on environment
- ~30m indoors (through walls)
- ~100m outdoors (line of sight)
- Use WiFi repeater or hotspot in car for longer range

**Future:** LoRa support could enable km-range telemetry.

### How much data does it use?

**Bandwidth:**
- 66 bytes per packet
- 20 packets/second
- = 1,320 bytes/second = 1.3 KB/s

**Per hour:**
- 1.3 KB/s × 3600 s = 4.7 MB/hour

Basically nothing. A 1 GB hotspot could log for 200+ hours.

## Project Questions

### Is this project maintained?

Yes! Active development as of January 2025.

Check the [CHANGELOG.md](CHANGELOG.md) for recent updates.

### Can I use this commercially?

Yes! It's MIT licensed. You can:
- Build products based on it
- Sell hardware with this firmware
- Modify it for commercial use

**BUT:**
- Include the LICENSE file
- No warranty/liability from the authors
- Give credit where credit is due (appreciated but not required)

### How can I contribute?

See [CONTRIBUTING.md](CONTRIBUTING.md) for full details.

Quick version:
1. Fork the repo
2. Make your changes
3. Test on hardware
4. Submit a pull request

Or help by:
- Reporting bugs
- Improving documentation
- Answering questions from other users

### Where can I get help?

1. **Documentation** - Start with [README.md](README.md) and [SUPPORT.md](SUPPORT.md)
2. **Search issues** - Someone might have asked already
3. **Open an issue** - Use templates, provide details
4. **Community** - Help each other in issue discussions

### Can I fork this project?

Absolutely! It's MIT licensed - fork away.

If you're going in a different direction, that's totally fine. If you make improvements, we'd love to see them as PRs, but you're not obligated.

### How can I support the project?

- ⭐ Star the repo on GitHub
- 📢 Tell others about it (blog, forums, social media)
- 🐛 Report bugs and suggest improvements
- 💻 Contribute code or documentation
- 💰 Sponsor development (GitHub Sponsors, if set up)
- 🙏 Say thanks! (It's motivating to hear success stories)

## Still have questions?

- **Not covered here?** Open an issue with the `question` label
- **Want to discuss?** Start a GitHub Discussion
- **Found an error?** Submit a PR to update this FAQ!

---

**Remember:** There are no dumb questions. If you're confused, others probably are too. Asking helps improve the documentation for everyone! 🏁
