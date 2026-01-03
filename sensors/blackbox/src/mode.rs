#![allow(dead_code)] // API methods for future use

/// Driving mode classifier
/// Detects IDLE, ACCEL, BRAKE, and CORNER modes based on acceleration and yaw
/// rate
use sensor_fusion::transforms::earth_to_car;

const G: f32 = 9.80665; // m/s²

/// Driving mode flags - can be combined
pub struct Mode {
    flags: u8,
}

// Mode bit flags
const IDLE: u8 = 0;
const ACCEL: u8 = 1;
const BRAKE: u8 = 2;
const CORNER: u8 = 4;

impl Mode {
    pub fn idle() -> Self {
        Self { flags: IDLE }
    }

    pub fn is_idle(&self) -> bool {
        self.flags == IDLE
    }

    pub fn has_accel(&self) -> bool {
        self.flags & ACCEL != 0
    }

    pub fn has_brake(&self) -> bool {
        self.flags & BRAKE != 0
    }

    pub fn has_corner(&self) -> bool {
        self.flags & CORNER != 0
    }

    pub fn set_accel(&mut self, enabled: bool) {
        if enabled {
            self.flags |= ACCEL;
            self.flags &= !BRAKE; // Clear brake (mutually exclusive)
        } else {
            self.flags &= !ACCEL;
        }
    }

    pub fn set_brake(&mut self, enabled: bool) {
        if enabled {
            self.flags |= BRAKE;
            self.flags &= !ACCEL; // Clear accel (mutually exclusive)
        } else {
            self.flags &= !BRAKE;
        }
    }

    pub fn set_corner(&mut self, enabled: bool) {
        if enabled {
            self.flags |= CORNER;
        } else {
            self.flags &= !CORNER;
        }
    }

    pub fn clear(&mut self) {
        self.flags = IDLE;
    }

    /// Get mode as string for display
    pub fn as_str(&self) -> &'static str {
        match self.flags {
            0 => "IDLE",
            1 => "ACCEL",
            2 => "BRAKE",
            4 => "CORNER",
            5 => "ACCEL+CORNER",
            6 => "BRAKE+CORNER",
            _ => "UNKNOWN",
        }
    }

    /// Get mode as u8 for binary telemetry
    pub fn as_u8(&self) -> u8 {
        self.flags
    }
}

impl Default for Mode {
    fn default() -> Self {
        Self::idle()
    }
}

/// Configurable thresholds for mode detection
#[derive(Debug, Clone, Copy)]
pub struct ModeConfig {
    pub min_speed: f32,  // m/s (minimum speed for maneuvers)
    pub acc_thr: f32,    // g (acceleration threshold)
    pub acc_exit: f32,   // g (acceleration exit threshold)
    pub brake_thr: f32,  // g (braking threshold, negative)
    pub brake_exit: f32, // g (braking exit threshold, negative)
    pub lat_thr: f32,    // g (lateral acceleration threshold)
    pub lat_exit: f32,   // g (lateral acceleration exit threshold)
    pub yaw_thr: f32,    // rad/s (yaw rate threshold for cornering)
    pub alpha: f32,      // EMA smoothing factor
}

impl Default for ModeConfig {
    fn default() -> Self {
        Self {
            min_speed: 2.0,    // ~7 km/h - must be moving for accel/brake/corner
            acc_thr: 0.10,     // 0.10g - city driving (gentle acceleration)
            acc_exit: 0.05,    // exit when below 0.05g
            brake_thr: -0.18,  // -0.18g - city driving (normal braking)
            brake_exit: -0.09, // exit when above -0.09g
            lat_thr: 0.12,     // 0.12g lateral - city turns
            lat_exit: 0.06,    // exit when below 0.06g
            yaw_thr: 0.05,     // ~2.9°/s yaw rate
            alpha: 0.35,       // EMA smoothing - balanced responsiveness vs bump rejection
        }
    }
}

/// Mode classifier with EMA filtering
pub struct ModeClassifier {
    pub mode: Mode,
    pub config: ModeConfig,

    // EMA filtered values (all accelerations filtered to reject road bumps)
    a_lon_ema: f32,
    a_lat_ema: f32,
    yaw_ema: f32,

    // Display speed with separate smoothing
    v_disp: f32,
    v_alpha: f32, // Speed display smoothing factor
}

impl ModeClassifier {
    pub fn new() -> Self {
        Self {
            mode: Mode::default(),
            config: ModeConfig::default(),
            a_lon_ema: 0.0,
            a_lat_ema: 0.0,
            yaw_ema: 0.0,
            v_disp: 0.0,
            v_alpha: 0.85, // High alpha = fast response to GPS speed changes
        }
    }

    /// Update mode based on current vehicle state
    ///
    /// # Arguments
    /// * `ax_earth` - Longitudinal acceleration in earth frame (m/s²)
    /// * `ay_earth` - Lateral acceleration in earth frame (m/s²)
    /// * `yaw_rad` - Vehicle yaw angle (rad)
    /// * `wz` - Yaw rate (rad/s)
    /// * `speed` - Vehicle speed (m/s)
    pub fn update(&mut self, ax_earth: f32, ay_earth: f32, yaw_rad: f32, wz: f32, speed: f32) {
        // Update display speed with EMA
        self.v_disp = (1.0 - self.v_alpha) * self.v_disp + self.v_alpha * speed;
        if self.v_disp < 3.0 / 3.6 {
            // < 3 km/h (below walking speed, treat as stationary for display)
            self.v_disp = 0.0;
        }

        // Transform earth-frame acceleration to car-frame
        let (a_lon_ms2, a_lat_ms2) = earth_to_car(ax_earth, ay_earth, yaw_rad);

        // Convert to g units
        let a_lon = a_lon_ms2 / G;
        let a_lat = a_lat_ms2 / G;

        // Update EMA filters for ALL values (critical for rejecting road bump noise)
        let alpha = self.config.alpha;
        self.a_lon_ema = (1.0 - alpha) * self.a_lon_ema + alpha * a_lon;
        self.a_lat_ema = (1.0 - alpha) * self.a_lat_ema + alpha * a_lat;
        self.yaw_ema = (1.0 - alpha) * self.yaw_ema + alpha * wz;

        // Independent state detection for each mode component
        // Modes can be combined (e.g., ACCEL+CORNER, BRAKE+CORNER)

        // If stopped, clear all modes
        if speed < self.config.min_speed {
            self.mode.clear();
            return;
        }

        // Check cornering (independent of accel/brake)
        let cornering_active = self.a_lat_ema.abs() > self.config.lat_thr
            && self.yaw_ema.abs() > self.config.yaw_thr
            && (self.a_lat_ema * self.yaw_ema) > 0.0;

        let cornering_exit = self.a_lat_ema.abs() < self.config.lat_exit
            && self.yaw_ema.abs() < self.config.yaw_thr * 0.5;

        if self.mode.has_corner() {
            // Exit corner if conditions no longer met
            if cornering_exit {
                self.mode.set_corner(false);
            }
        } else {
            // Enter corner if conditions met
            if cornering_active {
                self.mode.set_corner(true);
            }
        }

        // Check acceleration (mutually exclusive with braking)
        // Uses filtered a_lon_ema to reject road bump noise
        if self.mode.has_accel() {
            // Exit if acceleration dropped
            if self.a_lon_ema < self.config.acc_exit {
                self.mode.set_accel(false);
            }
        } else if !self.mode.has_brake() {
            // Enter accel if not braking and threshold met
            if self.a_lon_ema > self.config.acc_thr {
                self.mode.set_accel(true);
            }
        }

        // Check braking (mutually exclusive with acceleration)
        // Uses filtered a_lon_ema to reject road bump noise
        if self.mode.has_brake() {
            // Exit if braking force dropped
            if self.a_lon_ema > self.config.brake_exit {
                self.mode.set_brake(false);
            }
        } else if !self.mode.has_accel() {
            // Enter brake if not accelerating and threshold met
            if self.a_lon_ema < self.config.brake_thr {
                self.mode.set_brake(true);
            }
        }
    }

    /// Get current mode
    pub fn get_mode(&self) -> &Mode {
        &self.mode
    }

    /// Get current mode as u8 for telemetry
    pub fn get_mode_u8(&self) -> u8 {
        self.mode.as_u8()
    }

    /// Get display speed in km/h
    pub fn get_speed_kmh(&self) -> f32 {
        self.v_disp * 3.6
    }

    /// Force speed to zero (for ZUPT)
    pub fn reset_speed(&mut self) {
        self.v_disp = 0.0;
    }

    /// Check if display speed is near zero
    pub fn is_stopped(&self) -> bool {
        self.v_disp < 0.1 // < 0.36 km/h
    }

    /// Get EMA filtered lateral acceleration (for diagnostics)
    #[allow(dead_code)]
    pub fn get_a_lat_ema(&self) -> f32 {
        self.a_lat_ema
    }

    /// Get EMA filtered yaw rate (for diagnostics)
    #[allow(dead_code)]
    pub fn get_yaw_ema(&self) -> f32 {
        self.yaw_ema
    }

    /// Update configuration (e.g., from MQTT)
    pub fn update_config(&mut self, config: ModeConfig) {
        self.config = config;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mode_idle_by_default() {
        let classifier = ModeClassifier::new();
        assert!(classifier.get_mode().is_idle());
    }

    #[test]
    fn test_acceleration_detection() {
        let mut classifier = ModeClassifier::new();

        // Simulate forward acceleration at 5 m/s (above min_speed of 2.0)
        let ax = 0.25 * G; // 0.25g forward
        let ay = 0.0;
        let yaw = 0.0;
        let wz = 0.0;
        let speed = 5.0;

        // Multiple updates needed for EMA to converge (alpha=0.25)
        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed);
        }
        assert!(classifier.get_mode().has_accel());
    }

    #[test]
    fn test_braking_detection() {
        let mut classifier = ModeClassifier::new();

        // Simulate braking at 5 m/s (above min_speed of 2.0)
        let ax = -0.3 * G; // 0.3g deceleration
        let ay = 0.0;
        let yaw = 0.0;
        let wz = 0.0;
        let speed = 5.0;

        // Multiple updates needed for EMA to converge
        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed);
        }
        assert!(classifier.get_mode().has_brake());
    }

    #[test]
    fn test_no_mode_when_stopped() {
        let mut classifier = ModeClassifier::new();

        // Even with high acceleration, should stay IDLE when speed is 0
        let ax = -0.5 * G; // 0.5g deceleration (would trigger brake)
        let ay = 0.0;
        let yaw = 0.0;
        let wz = 0.0;
        let speed = 0.0; // STOPPED

        classifier.update(ax, ay, yaw, wz, speed);
        assert!(classifier.get_mode().is_idle()); // Must stay IDLE when stopped
    }

    #[test]
    fn test_combined_accel_corner() {
        let mut classifier = ModeClassifier::new();

        // Simulate acceleration while cornering (corner exit)
        let ax = 0.20 * G; // 0.20g forward acceleration
        let ay = 0.15 * G; // 0.15g lateral
        let yaw = 0.0;
        let wz = 0.08; // 0.08 rad/s yaw rate
        let speed = 8.0; // 8 m/s

        // Multiple updates for EMA convergence
        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed);
        }

        // Should detect both acceleration and cornering
        assert!(
            classifier.get_mode().has_accel(),
            "Should detect acceleration"
        );
        assert!(
            classifier.get_mode().has_corner(),
            "Should detect cornering"
        );
        assert_eq!(classifier.get_mode().as_str(), "ACCEL+CORNER");
    }

    #[test]
    fn test_combined_brake_corner() {
        let mut classifier = ModeClassifier::new();

        // Simulate braking while cornering (corner entry)
        let ax = -0.25 * G; // 0.25g braking
        let ay = -0.15 * G; // 0.15g lateral (left turn)
        let yaw = 0.0;
        let wz = -0.08; // -0.08 rad/s yaw rate (left)
        let speed = 8.0; // 8 m/s

        // Multiple updates for EMA convergence
        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed);
        }

        // Should detect both braking and cornering
        assert!(classifier.get_mode().has_brake(), "Should detect braking");
        assert!(
            classifier.get_mode().has_corner(),
            "Should detect cornering"
        );
        assert_eq!(classifier.get_mode().as_str(), "BRAKE+CORNER");
    }

    #[test]
    fn test_hysteresis_prevents_oscillation() {
        let mut classifier = ModeClassifier::new();

        let speed = 5.0;
        let yaw = 0.0;
        let wz = 0.0;
        let ay = 0.0;

        // Acceleration above entry threshold - multiple updates for EMA
        let ax1 = 0.20 * G; // Use higher value to ensure EMA exceeds threshold
        for _ in 0..10 {
            classifier.update(ax1, ay, yaw, wz, speed);
        }
        assert!(classifier.get_mode().has_accel(), "Should enter ACCEL");

        // Drop to 0.06g (above 0.05g exit threshold) - EMA should stay above exit
        let ax2 = 0.06 * G;
        for _ in 0..5 {
            classifier.update(ax2, ay, yaw, wz, speed);
        }
        assert!(
            classifier.get_mode().has_accel(),
            "Should stay ACCEL (EMA still above exit)"
        );

        // Drop to 0.00g - EMA will gradually fall below exit threshold
        let ax3 = 0.00 * G;
        for _ in 0..20 {
            classifier.update(ax3, ay, yaw, wz, speed);
        }
        assert!(
            classifier.get_mode().is_idle(),
            "Should exit to IDLE (EMA below exit)"
        );
    }

    #[test]
    fn test_speed_threshold_prevents_false_modes() {
        let mut classifier = ModeClassifier::new();

        // High acceleration but speed below threshold
        let ax = 0.30 * G;
        let ay = 0.0;
        let yaw = 0.0;
        let wz = 0.0;
        let speed_low = 1.5; // Below 2.0 m/s threshold

        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed_low);
        }
        assert!(
            classifier.get_mode().is_idle(),
            "Should stay IDLE when speed < min_speed"
        );

        // Same acceleration, speed above threshold
        let speed_high = 2.5;
        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed_high);
        }
        assert!(
            classifier.get_mode().has_accel(),
            "Should detect ACCEL when speed > min_speed"
        );
    }

    #[test]
    fn test_corner_independent_persistence() {
        let mut classifier = ModeClassifier::new();

        // Start cornering - multiple updates for EMA
        let ax = 0.0;
        let ay = 0.15 * G;
        let yaw = 0.0;
        let wz = 0.08;
        let speed = 8.0;

        for _ in 0..10 {
            classifier.update(ax, ay, yaw, wz, speed);
        }
        assert!(classifier.get_mode().has_corner(), "Should detect corner");

        // Add acceleration while still cornering
        let ax2 = 0.20 * G;
        for _ in 0..10 {
            classifier.update(ax2, ay, yaw, wz, speed);
        }
        assert!(classifier.get_mode().has_accel(), "Should detect accel");
        assert!(
            classifier.get_mode().has_corner(),
            "Should still detect corner"
        );
        assert_eq!(classifier.get_mode().as_u8(), 5, "Should be ACCEL+CORNER");

        // Stop accelerating but keep cornering - EMA needs to drop
        let ax3 = 0.0;
        for _ in 0..20 {
            classifier.update(ax3, ay, yaw, wz, speed);
        }
        assert!(!classifier.get_mode().has_accel(), "Should exit accel");
        assert!(classifier.get_mode().has_corner(), "Should still corner");
    }
}
