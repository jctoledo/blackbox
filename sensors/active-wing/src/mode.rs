/// Driving mode classifier
/// Detects IDLE, ACCEL, BRAKE, and CORNER modes based on acceleration and yaw
/// rate
use motorsport_telemetry::transforms::earth_to_car;

const G: f32 = 9.80665; // m/s²

/// Driving modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Idle,
    Accel,
    Brake,
    Corner,
}

impl Mode {
    pub fn as_str(&self) -> &'static str {
        match self {
            Mode::Idle => "IDLE",
            Mode::Accel => "ACCEL",
            Mode::Brake => "BRAKE",
            Mode::Corner => "CORNER",
        }
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
            min_speed: 2.0, // ~7 km/h
            acc_thr: 0.21,
            acc_exit: 0.11,
            brake_thr: -0.25,
            brake_exit: -0.12,
            lat_thr: 0.20,
            lat_exit: 0.10,
            yaw_thr: 0.07, // rad/s
            alpha: 0.20,
        }
    }
}

/// Mode classifier with EMA filtering
pub struct ModeClassifier {
    pub mode: Mode,
    pub config: ModeConfig,

    // EMA filtered values
    a_lat_ema: f32,
    yaw_ema: f32,

    // Display speed with separate smoothing
    v_disp: f32,
    v_alpha: f32, // Speed display smoothing factor
}

impl ModeClassifier {
    pub fn new() -> Self {
        Self {
            mode: Mode::Idle,
            config: ModeConfig::default(),
            a_lat_ema: 0.0,
            yaw_ema: 0.0,
            v_disp: 0.0,
            v_alpha: 0.6, // Faster smoothing for speed display
        }
    }

    /// Update mode based on current vehicle state
    ///
    /// # Arguments
    /// * `ax_earth` - Longitudinal acceleration in earth frame (m/s²)
    /// * `ay_earth` - Lateral acceleration in earth frame (m/s²)
    /// * `yaw_rad` - Vehicle yaw angle (rad)
    /// * `wz` - Yaw rate (rad/s)
    /// * `vx` - Velocity X in earth frame (m/s)
    /// * `vy` - Velocity Y in earth frame (m/s)
    pub fn update(
        &mut self,
        ax_earth: f32,
        ay_earth: f32,
        yaw_rad: f32,
        wz: f32,
        vx: f32,
        vy: f32,
    ) {
        // Calculate speed
        let speed = (vx * vx + vy * vy).sqrt();

        // Update display speed with EMA
        self.v_disp = (1.0 - self.v_alpha) * self.v_disp + self.v_alpha * speed;
        if self.v_disp < 0.5 / 3.6 {
            // < 0.5 km/h
            self.v_disp = 0.0;
        }

        // Transform earth-frame acceleration to car-frame
        let (a_lon_ms2, a_lat_ms2) = earth_to_car(ax_earth, ay_earth, yaw_rad);

        // Convert to g units
        let a_lon = a_lon_ms2 / G;
        let a_lat = a_lat_ms2 / G;

        // Update EMA filters
        let alpha = self.config.alpha;
        self.a_lat_ema = (1.0 - alpha) * self.a_lat_ema + alpha * a_lat;
        self.yaw_ema = (1.0 - alpha) * self.yaw_ema + alpha * wz;

        // State machine for mode detection
        match self.mode {
            Mode::Idle => {
                // Check for corner entry (high lateral accel + yaw rate, same sign)
                if speed > self.config.min_speed
                    && self.a_lat_ema.abs() > self.config.lat_thr
                    && self.yaw_ema.abs() > self.config.yaw_thr
                    && (self.a_lat_ema * self.yaw_ema) > 0.0
                {
                    self.mode = Mode::Corner;
                }
                // Check for acceleration
                else if a_lon > self.config.acc_thr {
                    self.mode = Mode::Accel;
                }
                // Check for braking
                else if a_lon < self.config.brake_thr {
                    self.mode = Mode::Brake;
                }
            }

            Mode::Accel => {
                if a_lon < self.config.acc_exit {
                    self.mode = Mode::Idle;
                }
            }

            Mode::Brake => {
                if a_lon > self.config.brake_exit {
                    self.mode = Mode::Idle;
                }
            }

            Mode::Corner => {
                // Exit corner if speed drops or lateral forces subside
                if speed < self.config.min_speed
                    || (self.a_lat_ema.abs() < self.config.lat_exit
                        && self.yaw_ema.abs() < self.config.yaw_thr * 0.5)
                {
                    self.mode = Mode::Idle;
                }
            }
        }
    }

    /// Get current mode
    pub fn get_mode(&self) -> Mode {
        self.mode
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
        assert_eq!(classifier.get_mode(), Mode::Idle);
    }

    #[test]
    fn test_acceleration_detection() {
        let mut classifier = ModeClassifier::new();

        // Simulate forward acceleration at 3 m/s
        let ax = 0.25 * G; // 0.25g forward
        let ay = 0.0;
        let yaw = 0.0;
        let wz = 0.0;
        let vx = 3.0;
        let vy = 0.0;

        classifier.update(ax, ay, yaw, wz, vx, vy);
        assert_eq!(classifier.get_mode(), Mode::Accel);
    }

    #[test]
    fn test_braking_detection() {
        let mut classifier = ModeClassifier::new();

        // Simulate braking at 5 m/s
        let ax = -0.3 * G; // 0.3g deceleration
        let ay = 0.0;
        let yaw = 0.0;
        let wz = 0.0;
        let vx = 5.0;
        let vy = 0.0;

        classifier.update(ax, ay, yaw, wz, vx, vy);
        assert_eq!(classifier.get_mode(), Mode::Brake);
    }
}
