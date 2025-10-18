# Security Policy

## Overview

Active Wing is a data acquisition system for vehicles. While it's designed to be safe and reliable, it's important to understand the security considerations when using embedded systems in automotive applications.

## Supported Versions

We release security updates for the following versions:

| Version | Supported          |
| ------- | ------------------ |
| main    | :white_check_mark: |
| 0.1.x   | :white_check_mark: |
| < 0.1   | :x:                |

## Security Considerations

### What Active Wing Does
- **Observes** vehicle dynamics (acceleration, position, velocity)
- **Streams** telemetry data over WiFi
- **Does NOT** control vehicle systems or safety-critical functions

### What Active Wing Does NOT Do
- Does not interface with vehicle control systems (brakes, steering, throttle)
- Does not store or transmit personal/sensitive information
- Does not require internet connectivity (local network only)
- Does not execute remote code or accept commands over the network

## Potential Security Concerns

### Network Security

**Issue**: Telemetry streams over unencrypted TCP/WiFi
- Data includes GPS coordinates and vehicle dynamics
- Anyone on the same network can intercept packets

**Mitigation**:
- Use a private WiFi network (not public hotspots)
- Consider VPN if streaming over untrusted networks
- Future versions may include TLS encryption

### WiFi Credentials

**Issue**: WiFi credentials stored in firmware (`main.rs`)
- Visible in source code and binary

**Mitigation**:
- Don't commit credentials to public repositories
- Use a separate WiFi network for telemetry
- Consider implementing secure credential storage (future)

### Physical Access

**Issue**: Physical access to ESP32 allows firmware extraction
- Someone with physical access can read flash memory

**Mitigation**:
- Don't store sensitive data on the device
- Use flash encryption if needed (ESP-IDF feature)
- Mount securely to prevent theft

## Reporting a Vulnerability

We take security seriously. If you discover a security vulnerability, please help us responsibly:

### What to Report

Security issues include:
- Buffer overflows or memory corruption
- Potential for remote code execution
- Privacy leaks (unintended data transmission)
- Authentication/authorization bypasses
- Denial of service vulnerabilities

### How to Report

**DO:**
1. Email security reports to: [INSERT SECURITY EMAIL]
2. Include detailed description of the vulnerability
3. Provide steps to reproduce if possible
4. Give us reasonable time to fix before public disclosure (90 days)

**DON'T:**
- Don't open public GitHub issues for security vulnerabilities
- Don't disclose publicly before a fix is available
- Don't exploit vulnerabilities maliciously

### What to Expect

1. **Acknowledgment** within 48 hours
2. **Initial assessment** within 1 week
3. **Fix timeline** communicated based on severity
4. **Credit** in release notes (if you want it)

## Security Best Practices

### For Users

1. **Use a dedicated WiFi network** for telemetry
   - Separate from your home/business network
   - Use WPA3 or WPA2 encryption

2. **Don't use on public WiFi** (coffee shops, airports, etc.)
   - Your telemetry and GPS coordinates would be visible

3. **Update firmware regularly**
   - We release security patches as needed
   - Subscribe to GitHub releases for notifications

4. **Physical security**
   - Mount the device securely
   - Consider hiding the installation
   - Use tamper-evident enclosures for competitions

5. **Review your code changes**
   - If you modify the firmware, review security implications
   - Don't add features that accept network commands without authentication

### For Developers

1. **Input validation**
   - Validate all sensor data before processing
   - Check buffer bounds rigorously
   - Handle malformed UART packets gracefully

2. **Memory safety**
   - Avoid `unsafe` blocks unless absolutely necessary
   - Use Rust's type system to prevent memory errors
   - Test with various edge cases

3. **Network security**
   - Consider adding TLS/encryption for new features
   - Implement authentication for any control interfaces
   - Rate-limit network operations

4. **Secrets management**
   - Never commit WiFi passwords or API keys
   - Use environment variables or config files (gitignored)
   - Document credential management in PRs

## Known Limitations

### By Design

1. **No authentication** on TCP telemetry stream
   - Anyone on the network can connect
   - Intentional for simplicity and low latency

2. **No encryption** on telemetry data
   - Would add latency and complexity
   - Trade-off for real-time performance

3. **No remote control** interface
   - Device only transmits, doesn't accept commands
   - Reduces attack surface significantly

### Future Improvements

We're considering:
- [ ] TLS encryption for telemetry (optional)
- [ ] Secure credential storage (NVS encryption)
- [ ] Authentication tokens for telemetry streams
- [ ] Signed firmware updates
- [ ] SD card logging with encryption

## Compliance

### Automotive Safety

**Important**: Active Wing is NOT:
- A safety system
- Certified for use in safety-critical applications
- Compliant with ISO 26262 or automotive safety standards

**Use Active Wing**:
- For data logging and analysis only
- In conjunction with proper safety equipment
- With understanding that it's an observational tool

### Radio Regulations

The ESP32-C3 contains WiFi radio:
- Complies with FCC Part 15 (USA)
- Complies with CE (Europe)
- Use only in countries where 2.4 GHz WiFi is legal
- Don't modify RF parameters without testing/certification

### Data Privacy

Active Wing may collect:
- GPS coordinates (your vehicle's location)
- Driving patterns (acceleration, braking, cornering)

**Your responsibility**:
- Inform passengers if logging data
- Comply with local privacy laws
- Secure logged data appropriately
- Don't share others' data without consent

## Questions?

If you have security questions that aren't sensitive enough for private disclosure:
- Open a GitHub issue with the `security` label
- We'll discuss publicly (helps everyone learn)

For sensitive security concerns, always use private disclosure.

---

**Remember**: Active Wing is a hobby project. While we strive for security, it's not designed for adversarial environments. Use appropriate operational security based on your threat model. 🔒
