# Blackbox - ESP32 Vehicle Telemetry System

## Overview
A comprehensive vehicle telemetry system built around the ESP32-S3 microcontroller with integrated power management, CAN bus communication, GPS tracking, and IMU sensing.

**Board:** powerBox v1.0 | **Created:** 2026-01-13 | **Updated:** 2026-01-24

---

## Main Components

### Power Management

#### USB-C Input & Protection
- **Connector:** TYPE-C-31-M-12 USB-C receptacle
- **Protection:** SRV05-4 (D2) TVS diodes, SMBJ17A (D6) overvoltage protection, SMD2920P075TF (F1) polyfuse

#### Battery Charger
- **IC:** BQ25606RGER (U1) - Switch-mode battery charge controller
  - Input: 3.9V - 13.5V (21V max)
  - Integrated power path management
  - NTC temperature monitoring
  - Status LEDs

#### 3.3V Regulator
- **IC:** TPS631000DRLR (U2) - High-efficiency buck converter
  - Input: SYS rail from battery charger
  - Output: 3.3V for all logic circuits

---

### Microcontroller

#### ESP32-S3 Module
- **Module:** ESP32-S3-WROOM-1-N16R8
  - 16MB Flash, 8MB PSRAM
  - Dual-core Xtensa LX7 @ 240MHz
  - Wi-Fi and Bluetooth integrated
- **Programming:** UART0 interface with reset/boot buttons

---

### Communication Interfaces

#### CAN Bus
- **Controller:** MCP2515T-E/ST (U5) - SPI-based CAN controller with 8MHz crystal
- **Transceiver:** SN65HVD230QDR (U6) - 3.3V CAN transceiver
- **Protection:** NUP2105LT1G (D1) TVS diodes, 120Ω termination
- **Connector:** 4-pin CAN interface

---

### Sensors

#### GPS Module
- **Module:** Sparkfun NEO M9N (U11) - u-blox multi-GNSS receiver
  - GPS, GLONASS, Galileo, BeiDou support
  - UART, I²C, and SPI interfaces -> currently in UART configuration
  - PPS output

#### IMU
- **Module:** WT901 (U10) - 9-axis IMU
  - 3-axis accelerometer, gyroscope, magnetometer
  - UART and I²C interfaces -> currently in UART configuration

#### Battery Monitor
- **IC:** LM339QT (U9) - Quad comparator for battery status (TBD configuration)

---

## System Architecture

### Power Rails
- **VBUS:** USB-C input (fused and protected)
- **12V:** Primary input (3.9V - 13.5V operating)
- **SYS:** Battery charger output
- **3.3V:** Main logic supply

### Communication Buses
- **SPI:** ESP32 ↔ MCP2515 CAN controller
- **I²C:** ESP32 ↔ GPS/IMU for future applications (with SP0502BAHTG protection)
- **UART0:** Programming/debug
- **UART (GPS):** ESP32 ↔ NEO M9N
- **UART (IMU):** ESP32 ↔ WT901

### Connectors
- **USB-C:** Power input and programming
- **BATCon:** 4-pin battery connector with voltage monitoring
- **CAN:** 4-pin automotive CAN bus

---

## Key Features
- Dual power input (USB-C or 12V battery)
- Automatic power path management
- Li-Ion/Li-Po battery charging with temperature monitoring
- Automotive-grade CAN bus interface
- Multi-GNSS positioning
- 9-axis inertial measurement
- Wi-Fi and Bluetooth connectivity

---

## Open Items
- Status LEDs
- Battery SOC LED (4 LEDs, analog circuit -> press button and see the SOC as 0% - 35% - 70% - 100%)
- Extra I/O for future applications
- Wakeup IC
- PCB Layout
