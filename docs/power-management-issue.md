# Power management: automatic sleep for battery conservation

## Background

Currently, Blackbox is powered by an external USB powerbank. The long-term goal is to create a **self-contained unit with an integrated battery** - mount it in your car, forget about it, and it just works.

The problem: if you finish a track session and forget to turn it off, your battery dies. Since there's no connection to the vehicle's ignition, Blackbox needs to be smart about managing its own power.

## The Goal

Blackbox should automatically sleep when the car hasn't moved for a while, and wake up when driving resumes - no user intervention required.

## Measured Power Consumption

Real measurements from the current hardware:

| State | Current | Notes |
|-------|---------|-------|
| Full system active | 160-185mA | WiFi AP + telemetry streaming |
| Without IMU | 150-175mA | GPS adds ~35-45mA |
| Without GPS | 125-140mA | IMU adds ~10-15mA |
| ESP32 only | 100-130mA | WiFi dominates power draw |

**Key insight:** The ESP32 with WiFi uses ~70% of total power. Turning off sensors saves little. The only real win is putting the ESP32 into deep sleep (~10-50µA) - a 3000x reduction.

### Battery Life Estimates (2000mAh LiPo)

| State | Current | Runtime |
|-------|---------|---------|
| Active | ~170mA | ~12 hours |
| Full shutdown | ~0mA | Forever (until next power-on) |
| Deep sleep (future) | ~20-30µA | ~1 year standby |

## Proposed Implementation

### Phase 1: Timeout-Based Shutdown (No Additional Hardware)

Simple approach for the first PCB revision:

1. Detect stationary state using existing sensors (low acceleration, low gyro, low GPS speed)
2. After **20 minutes** of no movement → **full shutdown**
3. User manually powers on when needed (button or power cycle)

```
ACTIVE ──[no movement 20min]──► SHUTDOWN (full power off)
                                    │
                    [manual button press or power cycle]
                                    │
                                    ▼
                                 ACTIVE
```

**Pros:** No hardware changes, simple to implement
**Cons:** Requires manual wake-up

### Phase 2: Wake-on-Motion with LIS3DH (Future PCB)

Add a dedicated ultra-low-power accelerometer (~$1.50) that stays on and wakes the system:

1. Same stationary detection as Phase 1
2. After 20 minutes → **deep sleep** (ESP32 sleeping, LIS3DH watching)
3. LIS3DH detects motion → **automatic wake**

```
ACTIVE ──[no movement 20min]──► DEEP SLEEP (~20-30µA)
   ▲                                │
   │                    [LIS3DH motion interrupt]
   │                                │
   └────────────────────────────────┘
```

**Why LIS3DH?**
- Ultra-low power: 2-6µA (can watch for motion indefinitely)
- Configurable threshold: detect car door, engine start, driving
- Interrupt pin: wakes ESP32 instantly
- The WT901 IMU draws ~10-15mA and can't wake the ESP32 from sleep

## Open Questions

### Calibration on Wake

The IMU calibration currently runs at boot and requires the vehicle to be **completely stationary** for ~10 seconds. What happens if the system wakes while already driving?

Options to consider:
- **Store calibration in flash (NVS)** - reload last known biases on wake
- **Skip calibration, accept degraded accuracy** - until next stationary period
- **Require stationary start** - don't transition to active until stopped (bad UX)

### EKF Convergence

After wake, the EKF state is reset. How quickly will it converge to accurate estimates?
- Position: Should lock on quickly once GPS has a fix (~2-30s depending on warm/cold start)
- Velocity: GPS provides this directly
- Yaw/heading: May drift until GPS velocity gives direction of travel

Is this acceptable, or do we need to persist EKF state across sleep cycles?

### GPS Warm Start

The GPS backup battery keeps ephemeris data alive during sleep. Warm start is ~2 seconds vs cold start ~24 seconds. Need to verify the SparkFun NEO-M9N backup battery stays charged during deep sleep.

### False Wakes (Phase 2)

In a busy parking lot, nearby cars or pedestrians might trigger the LIS3DH. Mitigations:
- Higher threshold (0.2g instead of 0.1g)
- Require sustained motion (multiple triggers within 1 second)
- Quick "is this real movement?" check after wake - go back to sleep if not

### 20-Minute Timeout

Is 20 minutes the right value? Considerations:
- Too short: annoying at gas stations, red lights in traffic
- Too long: wasted battery in parking lots
- Should this be user-configurable via the dashboard?

## Hardware Requirements (Phase 2)

| Component | Purpose | Cost |
|-----------|---------|------|
| LIS3DH accelerometer | Wake-on-motion detection | ~$1.50 |
| Load switches (x2) | Cut power to IMU and GPS | ~$1.60 |
| Passives | Decoupling, pull-ups | ~$0.20 |
| **Total** | | **~$3.30** |

## Summary

| Phase | Hardware | Sleep Mode | Wake Method | Complexity |
|-------|----------|------------|-------------|------------|
| 1 | None | Full shutdown | Manual | Low |
| 2 | LIS3DH + load switches | Deep sleep (~25µA) | Automatic (motion) | Medium |

Phase 1 is good enough for initial testing and proves the detection logic. Phase 2 delivers the "mount and forget" experience.

---

*Measurements taken 2025-01-11 with bench power supply. System: ESP32-C3 DevKitC-02 + WT901 IMU + SparkFun NEO-M9N GPS.*
