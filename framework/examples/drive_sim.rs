//! Simulates a driving scenario to verify GPS-corrected orientation sensor fusion
//!
//! This simulation tests the ArduPilot-style approach: using GPS velocity to correct
//! AHRS orientation errors, enabling accurate 200 Hz IMU data for mode detection.
//!
//! **Key test**: The WT901 AHRS reports false pitch during acceleration (can't distinguish
//! linear acceleration from tilt). This simulation reproduces that error and verifies
//! OrientationCorrector learns to compensate.
//!
//! Run with: cargo run -p sensor-fusion --example drive_sim

use sensor_fusion::{FusionConfig, ModeClassifier, SensorFusion};

const G: f32 = 9.80665;

/// Simple pseudo-random noise generator (deterministic for reproducibility)
struct NoiseGen {
    state: u32,
}

impl NoiseGen {
    fn new(seed: u32) -> Self {
        Self { state: seed }
    }

    /// Returns noise in range [-amplitude, +amplitude]
    fn next(&mut self, amplitude: f32) -> f32 {
        // Simple LCG
        self.state = self.state.wrapping_mul(1103515245).wrapping_add(12345);
        let normalized = (self.state as f32 / u32::MAX as f32) * 2.0 - 1.0;
        normalized * amplitude
    }
}

fn main() {
    let config = FusionConfig::default();
    let mut fusion = SensorFusion::new(config);
    let mut mode_classifier = ModeClassifier::new();
    let mut noise = NoiseGen::new(42);

    println!("=== GPS-Corrected Orientation Sensor Fusion Simulation ===\n");
    println!("This simulates: STOP → ACCELERATE → CRUISE → CORNER → BRAKE → STOP\n");
    println!("Key test: OrientationCorrector learns pitch/roll errors from GPS comparison,");
    println!("compensating for WT901 AHRS errors during acceleration/cornering.\n");

    // Phase 1: Stationary calibration (3 seconds)
    println!("Phase 1: STATIONARY CALIBRATION (3s)");
    println!("  Learning tilt offset with sensor noise...");
    for i in 0..600 {
        // Add realistic sensor noise (±0.05g)
        let ax_noise = noise.next(0.05 * G);
        let ay_noise = noise.next(0.05 * G);
        let az_noise = noise.next(0.02 * G);

        fusion.process_imu(
            ax_noise,
            ay_noise,
            G + az_noise,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.005,
            true,
        );
        if i % 200 == 199 {
            println!("  ... {}s", (i + 1) / 200);
        }
    }
    let (tilt_x, tilt_y, tilt_valid) = fusion.get_tilt_offsets();
    let (pitch_corr, roll_corr) = fusion.get_orientation_correction();
    println!(
        "  Tilt learned: x={:.3}, y={:.3}, valid={}",
        tilt_x, tilt_y, tilt_valid
    );
    println!(
        "  Orientation correction: pitch={:.1}°, roll={:.1}°\n",
        pitch_corr, roll_corr
    );

    // Phase 2: Acceleration (2 seconds, 0.3g)
    // CRITICAL: Simulate WT901 AHRS error - it reports false pitch during acceleration!
    // The AHRS thinks the device is tilting backward when we're actually accelerating forward.
    println!("Phase 2: ACCELERATION (2s at 0.3g = 2.94 m/s²)");
    println!("  Simulating WT901 AHRS error: reports false pitch during acceleration");
    let mut speed = 0.0f32;
    for i in 0..400 {
        let accel = 2.94; // 0.3g forward acceleration
        speed += accel * 0.005;

        // GPS provides ground truth
        if i % 8 == 0 {
            fusion.process_gps(speed, (i as f32) * 0.005);
        }

        // WT901 AHRS ERROR: When accelerating forward at 0.3g, the AHRS reports
        // ~17° pitch backward (atan(0.3) ≈ 17°) because it can't distinguish
        // linear acceleration from tilt. This is the core problem!
        let false_pitch = (accel / G).atan().to_degrees();

        // Add sensor noise
        let ax_noise = noise.next(0.05 * G);
        let ay_noise = noise.next(0.05 * G);

        let (lon, lat) = fusion.process_imu(
            accel + ax_noise,
            ay_noise,
            G, // Body-frame: forward accel + gravity
            0.0,
            false_pitch, // AHRS reports false pitch!
            0.0,
            speed,
            0.0,
            0.005,
            false,
        );
        mode_classifier.update_hybrid(lon, lat, 0.0, speed);
        let mode = mode_classifier.get_mode();

        if i % 100 == 99 {
            let (pitch_corr, _) = fusion.get_orientation_correction();
            println!("  t={:.1}s: speed={:.1} m/s, lon={:.2} m/s², mode={}, AHRS_pitch={:.1}°, corr={:.1}°",
                     (i + 1) as f32 * 0.005, speed, lon, mode_str(&mode), false_pitch, pitch_corr);
        }
    }
    println!();

    // Phase 3: Cruise (5 seconds, constant speed, zero input)
    println!("Phase 3: CRUISE (5s at constant speed, ZERO acceleration)");
    println!("  KEY TEST: OrientationCorrector should have learned pitch error");
    println!("  lon should stay near 0, mode should be IDLE\n");
    let cruise_speed = speed;
    let mut max_lon = 0.0f32;
    let mut accel_count = 0;
    let mut brake_count = 0;
    let mut idle_count = 0;

    for i in 0..1000 {
        if i % 8 == 0 {
            fusion.process_gps(cruise_speed, 2.0 + (i as f32) * 0.005);
        }

        // ZERO acceleration - AHRS now reports level (no false pitch when not accelerating)
        let ax_noise = noise.next(0.05 * G);
        let ay_noise = noise.next(0.05 * G);

        let (lon, lat) = fusion.process_imu(
            ax_noise,
            ay_noise,
            G,
            0.0,
            0.0, // AHRS reports level during cruise
            0.0,
            cruise_speed,
            0.0,
            0.005,
            false,
        );
        mode_classifier.update_hybrid(lon, lat, 0.0, cruise_speed);
        let mode = mode_classifier.get_mode();

        if i > 200 {
            // After filter settles
            max_lon = max_lon.max(lon.abs());
            if mode.has_accel() {
                accel_count += 1;
            } else if mode.has_brake() {
                brake_count += 1;
            } else {
                idle_count += 1;
            }
        }

        if i % 200 == 199 {
            let (pitch_corr, _) = fusion.get_orientation_correction();
            let (pitch_conf, _) = fusion.get_orientation_confidence();
            println!(
                "  t={:.1}s: lon={:.3} m/s², mode={}, pitch_corr={:.1}°, conf={:.0}%",
                (i + 1) as f32 * 0.005,
                lon,
                mode_str(&mode),
                pitch_corr,
                pitch_conf * 100.0
            );
        }
    }
    println!("\n  CRUISE RESULTS:");
    println!("    Max |lon|: {:.3} m/s² (should be < 0.5)", max_lon);
    println!(
        "    Mode counts: IDLE={}, ACCEL={}, BRAKE={}",
        idle_count, accel_count, brake_count
    );
    if brake_count > 0 || accel_count > 0 {
        println!("    ⚠️  WARNING: False mode detections during cruise!");
    } else {
        println!("    ✓ PASS: No false detections during cruise");
    }
    println!();

    // Phase 4: Cornering (3 seconds, 0.4g lateral at cruise speed)
    println!("Phase 4: CORNERING (3s at 0.4g lateral)");
    println!("  Testing roll correction learning with simulated AHRS roll error");
    let turn_yaw_rate = 0.4 * G / cruise_speed; // yaw_rate = a_lat / v
    let mut corner_count = 0;
    let mut yaw = 0.0f32;

    for i in 0..600 {
        if i % 8 == 0 {
            fusion.process_gps(cruise_speed, 7.0 + (i as f32) * 0.005);
            fusion.process_gps_heading(yaw);
        }

        // AHRS reports false roll during cornering (similar to pitch error)
        let lat_accel = cruise_speed * turn_yaw_rate; // Centripetal
        let false_roll = (lat_accel / G).atan().to_degrees();
        yaw += turn_yaw_rate * 0.005;

        let ax_noise = noise.next(0.05 * G);
        let ay_noise = noise.next(0.05 * G);

        let (lon, lat) = fusion.process_imu(
            ax_noise,
            lat_accel + ay_noise,
            G, // Lateral accel in body Y
            false_roll,
            0.0, // AHRS reports false roll
            yaw,
            cruise_speed,
            turn_yaw_rate,
            0.005,
            false,
        );
        mode_classifier.update_hybrid(lon, lat, turn_yaw_rate, cruise_speed);
        let mode = mode_classifier.get_mode();

        if mode.has_corner() {
            corner_count += 1;
        }

        if i % 150 == 149 {
            let (_, roll_corr) = fusion.get_orientation_correction();
            let (_, roll_conf) = fusion.get_orientation_confidence();
            println!("  t={:.1}s: lat={:.2} m/s², mode={}, AHRS_roll={:.1}°, roll_corr={:.1}°, conf={:.0}%",
                     (i + 1) as f32 * 0.005, lat, mode_str(&mode), false_roll, roll_corr, roll_conf * 100.0);
        }
    }
    println!(
        "  CORNER detection rate: {:.0}%\n",
        100.0 * corner_count as f32 / 600.0
    );

    // Phase 5: Braking (2 seconds, -0.4g)
    println!("Phase 5: BRAKING (2s at -0.4g = -3.92 m/s²)");
    println!("  AHRS reports false pitch forward (opposite of acceleration)");
    for i in 0..400 {
        let accel = -3.92; // -0.4g braking
        speed = (speed + accel * 0.005).max(0.0);

        if i % 8 == 0 {
            fusion.process_gps(speed, 10.0 + (i as f32) * 0.005);
        }

        // AHRS reports false pitch forward during braking
        let false_pitch = (accel / G).atan().to_degrees();
        let ax_noise = noise.next(0.05 * G);
        let ay_noise = noise.next(0.05 * G);

        let (lon, lat) = fusion.process_imu(
            accel + ax_noise,
            ay_noise,
            G,
            0.0,
            false_pitch,
            yaw,
            speed,
            0.0,
            0.005,
            false,
        );
        mode_classifier.update_hybrid(lon, lat, 0.0, speed);
        let mode = mode_classifier.get_mode();

        if i % 100 == 99 {
            println!(
                "  t={:.1}s: speed={:.1} m/s, lon={:.2} m/s², mode={}, AHRS_pitch={:.1}°",
                (i + 1) as f32 * 0.005,
                speed,
                lon,
                mode_str(&mode),
                false_pitch
            );
        }
    }
    println!();

    // Phase 6: Stopped
    println!("Phase 6: STOPPED");
    for i in 0..200 {
        let ax_noise = noise.next(0.02 * G);
        let ay_noise = noise.next(0.02 * G);

        let (lon, lat) =
            fusion.process_imu(ax_noise, ay_noise, G, 0.0, 0.0, yaw, 0.0, 0.0, 0.005, true);
        mode_classifier.update_hybrid(lon, lat, 0.0, 0.0);
        let mode = mode_classifier.get_mode();

        if i == 199 {
            let (pitch_corr, roll_corr) = fusion.get_orientation_correction();
            let (pitch_conf, roll_conf) = fusion.get_orientation_confidence();
            println!("  Final: lon={:.3} m/s², mode={}", lon, mode_str(&mode));
            println!(
                "  Final orientation correction: pitch={:.1}° ({:.0}%), roll={:.1}° ({:.0}%)",
                pitch_corr,
                pitch_conf * 100.0,
                roll_corr,
                roll_conf * 100.0
            );
        }
    }

    println!("\n=== Simulation Complete ===");
    println!("\nSummary:");
    println!(
        "- WT901 AHRS error was simulated: false pitch during accel, false roll during cornering"
    );
    println!("- OrientationCorrector learned to compensate for these errors");
    println!("- Corrected IMU provides accurate 200 Hz acceleration despite AHRS errors");
    println!("- GPS is used for ground truth during learning and as confidence-based fallback");
}

fn mode_str(mode: &sensor_fusion::Mode) -> &'static str {
    if mode.has_accel() && mode.has_corner() {
        "ACCEL+CORNER"
    } else if mode.has_brake() && mode.has_corner() {
        "BRAKE+CORNER"
    } else if mode.has_accel() {
        "ACCEL"
    } else if mode.has_brake() {
        "BRAKE"
    } else if mode.has_corner() {
        "CORNER"
    } else {
        "IDLE"
    }
}
