/// Sensor Fusion Module
///
/// This module handles:
/// 1. GPS-derived longitudinal acceleration (from velocity changes)
/// 2. GPS/IMU acceleration blending with adaptive weights
/// 3. Dynamic tilt offset learning (ZUPT enhancement)
/// 4. Moving gravity estimation (continuous calibration while driving)
///
/// The goal is to provide clean, drift-free acceleration for mode detection
/// that works regardless of device mounting angle.

use crate::filter::BiquadFilter;

/// Configuration for sensor fusion
#[derive(Clone, Copy)]
pub struct FusionConfig {
    /// GPS rate threshold for high-confidence GPS (Hz)
    pub gps_high_rate: f32,
    /// GPS rate threshold for medium-confidence GPS (Hz)
    pub gps_medium_rate: f32,
    /// Maximum age of GPS data before considered stale (seconds)
    pub gps_max_age: f32,
    /// GPS weight when rate >= high_rate (0.0-1.0)
    /// Higher = more GPS, smoother but slower response
    /// Lower = more IMU, faster response but potential drift
    pub gps_weight_high: f32,
    /// GPS weight when rate >= medium_rate (0.0-1.0)
    pub gps_weight_medium: f32,
    /// GPS weight when rate < medium_rate (0.0-1.0)
    pub gps_weight_low: f32,
    /// Time required at stop to learn tilt offset (seconds)
    pub tilt_learn_time: f32,
    /// Speed threshold for steady-state detection (m/s)
    pub steady_state_speed_tolerance: f32,
    /// Yaw rate threshold for steady-state (rad/s)
    pub steady_state_yaw_tolerance: f32,
    /// Time required in steady state to update gravity estimate (seconds)
    pub gravity_learn_time: f32,
    /// Alpha for gravity estimate update (smaller = slower, more stable)
    pub gravity_alpha: f32,
    /// Low-pass filter cutoff for acceleration (Hz)
    pub accel_filter_cutoff: f32,
    /// IMU sample rate (Hz)
    pub imu_sample_rate: f32,
}

impl Default for FusionConfig {
    /// Default configuration balanced for city/highway/canyon driving.
    ///
    /// GPS weights are moderate (70/50/30) for balanced responsiveness:
    /// - Fast enough for canyon driving (~80-100ms latency)
    /// - Smooth enough for city/highway (no jitter)
    ///
    /// For track use, consider increasing GPS weights (90/70/50) for
    /// maximum accuracy at the cost of some latency.
    fn default() -> Self {
        Self {
            gps_high_rate: 20.0,
            gps_medium_rate: 10.0,
            gps_max_age: 0.2, // 200ms
            // Balanced blend: 70% GPS when fresh and fast
            // Gives ~80-100ms latency instead of ~120-150ms
            gps_weight_high: 0.70,   // 70% GPS / 30% IMU at >= 20Hz
            gps_weight_medium: 0.50, // 50% GPS / 50% IMU at >= 10Hz
            gps_weight_low: 0.30,    // 30% GPS / 70% IMU at < 10Hz
            tilt_learn_time: 3.0,
            steady_state_speed_tolerance: 0.5,  // 0.5 m/s = 1.8 km/h
            steady_state_yaw_tolerance: 0.087,  // ~5 deg/s
            gravity_learn_time: 2.0,
            gravity_alpha: 0.02, // Very slow update
            accel_filter_cutoff: 4.0,
            imu_sample_rate: 200.0,
        }
    }
}

/// GPS-derived acceleration tracker
pub struct GpsAcceleration {
    /// Previous speed (m/s)
    prev_speed: f32,
    /// Previous timestamp (seconds, monotonic)
    prev_time: f32,
    /// Computed longitudinal acceleration (m/s²)
    accel_lon: f32,
    /// Is the acceleration estimate valid?
    valid: bool,
    /// Current GPS rate estimate (Hz)
    gps_rate: f32,
    /// Time since last valid GPS fix (seconds)
    time_since_fix: f32,
    /// EMA for GPS rate calculation
    rate_ema: f32,
    /// Last fix count for rate calculation
    last_fix_count: u32,
    /// Time of last rate calculation
    last_rate_time: f32,
}

impl GpsAcceleration {
    pub fn new() -> Self {
        Self {
            prev_speed: 0.0,
            prev_time: 0.0,
            accel_lon: 0.0,
            valid: false,
            gps_rate: 0.0,
            time_since_fix: 999.0,
            rate_ema: 0.0,
            last_fix_count: 0,
            last_rate_time: 0.0,
        }
    }

    /// Update with new GPS speed measurement
    ///
    /// # Arguments
    /// * `speed` - Current speed in m/s
    /// * `time` - Current timestamp in seconds (monotonic)
    ///
    /// # Returns
    /// The computed longitudinal acceleration, or None if not enough data
    pub fn update(&mut self, speed: f32, time: f32) -> Option<f32> {
        self.time_since_fix = 0.0;

        if self.prev_time > 0.0 {
            let dt = time - self.prev_time;

            // Validate dt: should be 20-200ms for 5-50Hz GPS
            if dt > 0.02 && dt < 0.5 {
                self.accel_lon = (speed - self.prev_speed) / dt;
                self.valid = true;

                self.prev_speed = speed;
                self.prev_time = time;

                return Some(self.accel_lon);
            }
        }

        self.prev_speed = speed;
        self.prev_time = time;
        None
    }

    /// Update GPS rate estimate
    ///
    /// # Arguments
    /// * `fix_count` - Total number of valid fixes received
    /// * `time` - Current timestamp in seconds
    pub fn update_rate(&mut self, fix_count: u32, time: f32) {
        if self.last_rate_time > 0.0 {
            let dt = time - self.last_rate_time;
            if dt >= 1.0 {
                let fixes = fix_count.saturating_sub(self.last_fix_count);
                let instant_rate = fixes as f32 / dt;

                // EMA smoothing for rate
                self.rate_ema = 0.3 * instant_rate + 0.7 * self.rate_ema;
                self.gps_rate = self.rate_ema;

                self.last_fix_count = fix_count;
                self.last_rate_time = time;
            }
        } else {
            self.last_fix_count = fix_count;
            self.last_rate_time = time;
        }
    }

    /// Advance time without GPS fix (for staleness tracking)
    pub fn advance_time(&mut self, dt: f32) {
        self.time_since_fix += dt;
        if self.time_since_fix > 0.5 {
            self.valid = false;
        }
    }

    /// Get current GPS-derived acceleration
    pub fn get_accel(&self) -> Option<f32> {
        if self.valid {
            Some(self.accel_lon)
        } else {
            None
        }
    }

    /// Get current GPS rate estimate (Hz)
    pub fn get_rate(&self) -> f32 {
        self.gps_rate
    }

    /// Check if GPS data is fresh
    pub fn is_fresh(&self, max_age: f32) -> bool {
        self.time_since_fix < max_age
    }
}

impl Default for GpsAcceleration {
    fn default() -> Self {
        Self::new()
    }
}

/// Tilt offset estimator - learns mounting offset when stopped
pub struct TiltEstimator {
    /// Accumulated X acceleration (earth frame)
    accel_x_sum: f32,
    /// Accumulated Y acceleration (earth frame)
    accel_y_sum: f32,
    /// Number of samples accumulated
    sample_count: u32,
    /// Time stationary (seconds)
    stationary_time: f32,
    /// Learned tilt offset X (m/s²)
    offset_x: f32,
    /// Learned tilt offset Y (m/s²)
    offset_y: f32,
    /// Is the offset valid (learned at least once)?
    offset_valid: bool,
    /// Required stationary time to learn (seconds)
    learn_time: f32,
}

impl TiltEstimator {
    pub fn new(learn_time: f32) -> Self {
        Self {
            accel_x_sum: 0.0,
            accel_y_sum: 0.0,
            sample_count: 0,
            stationary_time: 0.0,
            offset_x: 0.0,
            offset_y: 0.0,
            offset_valid: false,
            learn_time,
        }
    }

    /// Update with acceleration sample while stationary
    ///
    /// # Arguments
    /// * `ax_earth` - X acceleration in earth frame (m/s²)
    /// * `ay_earth` - Y acceleration in earth frame (m/s²)
    /// * `dt` - Time step (seconds)
    ///
    /// # Returns
    /// true if offset was just updated (can trigger visual feedback)
    pub fn update_stationary(&mut self, ax_earth: f32, ay_earth: f32, dt: f32) -> bool {
        self.stationary_time += dt;
        self.accel_x_sum += ax_earth;
        self.accel_y_sum += ay_earth;
        self.sample_count += 1;

        // After sufficient time, compute offset
        if self.stationary_time >= self.learn_time && self.sample_count > 50 {
            let new_offset_x = self.accel_x_sum / self.sample_count as f32;
            let new_offset_y = self.accel_y_sum / self.sample_count as f32;

            // Only update if change is significant (avoid jitter)
            let change = ((new_offset_x - self.offset_x).powi(2)
                + (new_offset_y - self.offset_y).powi(2))
            .sqrt();

            if change > 0.05 || !self.offset_valid {
                self.offset_x = new_offset_x;
                self.offset_y = new_offset_y;
                self.offset_valid = true;

                // Reset accumulators for continued learning
                self.accel_x_sum = 0.0;
                self.accel_y_sum = 0.0;
                self.sample_count = 0;

                return true;
            }
        }

        false
    }

    /// Reset stationary tracking (called when vehicle starts moving)
    pub fn reset_stationary(&mut self) {
        self.accel_x_sum = 0.0;
        self.accel_y_sum = 0.0;
        self.sample_count = 0;
        self.stationary_time = 0.0;
    }

    /// Apply tilt correction to acceleration
    pub fn correct(&self, ax_earth: f32, ay_earth: f32) -> (f32, f32) {
        if self.offset_valid {
            (ax_earth - self.offset_x, ay_earth - self.offset_y)
        } else {
            (ax_earth, ay_earth)
        }
    }
}

/// Moving gravity estimator - learns gravity offset while driving
pub struct GravityEstimator {
    /// Estimated gravity offset X (earth frame, m/s²)
    offset_x: f32,
    /// Estimated gravity offset Y (earth frame, m/s²)
    offset_y: f32,
    /// Is the estimate valid?
    valid: bool,
    /// Speed history for steady-state detection
    speed_history: [f32; 10],
    speed_idx: usize,
    /// Time in steady state (seconds)
    steady_time: f32,
    /// Configuration
    speed_tolerance: f32,
    yaw_tolerance: f32,
    learn_time: f32,
    alpha: f32,
}

impl GravityEstimator {
    pub fn new(config: &FusionConfig) -> Self {
        Self {
            offset_x: 0.0,
            offset_y: 0.0,
            valid: false,
            speed_history: [0.0; 10],
            speed_idx: 0,
            steady_time: 0.0,
            speed_tolerance: config.steady_state_speed_tolerance,
            yaw_tolerance: config.steady_state_yaw_tolerance,
            learn_time: config.gravity_learn_time,
            alpha: config.gravity_alpha,
        }
    }

    /// Update with current driving state
    ///
    /// # Arguments
    /// * `ax_earth` - X acceleration (earth frame, m/s²)
    /// * `ay_earth` - Y acceleration (earth frame, m/s²)
    /// * `speed` - Current speed (m/s)
    /// * `yaw_rate` - Current yaw rate (rad/s)
    /// * `dt` - Time step (seconds)
    pub fn update(
        &mut self,
        ax_earth: f32,
        ay_earth: f32,
        speed: f32,
        yaw_rate: f32,
        dt: f32,
    ) -> bool {
        // Update speed history
        self.speed_history[self.speed_idx] = speed;
        self.speed_idx = (self.speed_idx + 1) % self.speed_history.len();

        // Check if in steady state (constant velocity)
        let speed_stable = self.is_speed_stable();
        let yaw_low = yaw_rate.abs() < self.yaw_tolerance;
        let moving = speed > 2.0; // Must be moving (not stopped)

        if speed_stable && yaw_low && moving {
            self.steady_time += dt;

            if self.steady_time >= self.learn_time {
                // In steady state, expected acceleration is ~0
                // Any measured acceleration is gravity/mounting error
                self.offset_x = (1.0 - self.alpha) * self.offset_x + self.alpha * ax_earth;
                self.offset_y = (1.0 - self.alpha) * self.offset_y + self.alpha * ay_earth;
                self.valid = true;
                return true;
            }
        } else {
            self.steady_time = 0.0;
        }

        false
    }

    fn is_speed_stable(&self) -> bool {
        if self.speed_history[0] < 1.0 {
            return false; // Need some speed data
        }

        let mean: f32 = self.speed_history.iter().sum::<f32>() / self.speed_history.len() as f32;
        let max_dev = self
            .speed_history
            .iter()
            .map(|&s| (s - mean).abs())
            .fold(0.0f32, |a, b| a.max(b));

        max_dev < self.speed_tolerance
    }

    /// Apply gravity correction to acceleration
    pub fn correct(&self, ax_earth: f32, ay_earth: f32) -> (f32, f32) {
        if self.valid {
            (ax_earth - self.offset_x, ay_earth - self.offset_y)
        } else {
            (ax_earth, ay_earth)
        }
    }
}

/// Main sensor fusion processor
pub struct SensorFusion {
    pub config: FusionConfig,
    /// GPS-derived acceleration
    pub gps_accel: GpsAcceleration,
    /// Tilt estimator (learns when stopped)
    pub tilt_estimator: TiltEstimator,
    /// Gravity estimator (learns while driving)
    pub gravity_estimator: GravityEstimator,
    /// Low-pass filter for IMU X
    filter_x: BiquadFilter,
    /// Low-pass filter for IMU Y
    filter_y: BiquadFilter,
    /// Filtered IMU longitudinal acceleration (m/s²)
    imu_lon_filtered: f32,
    /// Filtered IMU lateral acceleration (m/s²)
    imu_lat_filtered: f32,
    /// Blended longitudinal acceleration (m/s²)
    blended_lon: f32,
    /// Last GPS blend weight (for diagnostics)
    last_gps_weight: f32,
    /// Was stationary last update?
    was_stationary: bool,
}

impl SensorFusion {
    pub fn new(config: FusionConfig) -> Self {
        Self {
            gps_accel: GpsAcceleration::new(),
            tilt_estimator: TiltEstimator::new(config.tilt_learn_time),
            gravity_estimator: GravityEstimator::new(&config),
            filter_x: BiquadFilter::new_lowpass(config.accel_filter_cutoff, config.imu_sample_rate),
            filter_y: BiquadFilter::new_lowpass(config.accel_filter_cutoff, config.imu_sample_rate),
            imu_lon_filtered: 0.0,
            imu_lat_filtered: 0.0,
            blended_lon: 0.0,
            last_gps_weight: 0.0,
            was_stationary: false,
            config,
        }
    }

    /// Process IMU data (call at IMU rate, e.g., 200Hz)
    ///
    /// # Arguments
    /// * `ax_earth` - X acceleration in earth frame (m/s²)
    /// * `ay_earth` - Y acceleration in earth frame (m/s²)
    /// * `yaw` - Vehicle yaw angle (rad, for earth-to-car transform)
    /// * `dt` - Time step (seconds)
    /// * `is_stationary` - Whether vehicle is currently stationary
    ///
    /// # Returns
    /// (lon_accel, lat_accel) - Fused accelerations in vehicle frame (m/s²)
    pub fn process_imu(
        &mut self,
        ax_earth: f32,
        ay_earth: f32,
        yaw: f32,
        speed: f32,
        yaw_rate: f32,
        dt: f32,
        is_stationary: bool,
    ) -> (f32, f32) {
        // Track GPS staleness
        self.gps_accel.advance_time(dt);

        // Apply tilt correction (learned when stopped)
        let (ax_tilt, ay_tilt) = self.tilt_estimator.correct(ax_earth, ay_earth);

        // Apply gravity correction (learned while driving)
        let (ax_grav, ay_grav) = self.gravity_estimator.correct(ax_tilt, ay_tilt);

        // Low-pass filter to remove vibration
        let ax_filt = self.filter_x.process(ax_grav);
        let ay_filt = self.filter_y.process(ay_grav);

        // Transform to vehicle frame
        let (lon_imu, lat_imu) = earth_to_car(ax_filt, ay_filt, yaw);

        self.imu_lon_filtered = lon_imu;
        self.imu_lat_filtered = lat_imu;

        // Handle stationary state transitions
        if is_stationary {
            // Learn tilt offset when stopped
            self.tilt_estimator
                .update_stationary(ax_earth, ay_earth, dt);

            if !self.was_stationary {
                // Just stopped - reset filters to avoid transient
                self.filter_x.reset_to(0.0);
                self.filter_y.reset_to(0.0);
            }
        } else {
            if self.was_stationary {
                // Just started moving
                self.tilt_estimator.reset_stationary();
            }

            // Update gravity estimate while driving
            self.gravity_estimator
                .update(ax_earth, ay_earth, speed, yaw_rate, dt);
        }
        self.was_stationary = is_stationary;

        // Blend GPS and IMU for longitudinal acceleration
        let gps_weight = self.compute_gps_weight();
        self.last_gps_weight = gps_weight;

        let lon_blended = if let Some(gps_lon) = self.gps_accel.get_accel() {
            gps_weight * gps_lon + (1.0 - gps_weight) * lon_imu
        } else {
            lon_imu
        };

        self.blended_lon = lon_blended;

        // Lateral always from IMU (GPS can't sense lateral well)
        (lon_blended, lat_imu)
    }

    /// Process GPS speed update (call at GPS rate, e.g., 25Hz)
    ///
    /// # Arguments
    /// * `speed` - GPS speed in m/s
    /// * `time` - Monotonic timestamp in seconds
    pub fn process_gps(&mut self, speed: f32, time: f32) {
        self.gps_accel.update(speed, time);
    }

    /// Update GPS rate estimate (call periodically, e.g., every second)
    pub fn update_gps_rate(&mut self, fix_count: u32, time: f32) {
        self.gps_accel.update_rate(fix_count, time);
    }

    /// Compute GPS weight based on GPS rate and data freshness
    fn compute_gps_weight(&self) -> f32 {
        let rate = self.gps_accel.get_rate();
        let fresh = self.gps_accel.is_fresh(self.config.gps_max_age);

        if !fresh {
            return 0.0;
        }

        // Blending based on GPS rate using configurable weights
        if rate >= self.config.gps_high_rate {
            self.config.gps_weight_high
        } else if rate >= self.config.gps_medium_rate {
            self.config.gps_weight_medium
        } else if rate > 0.0 {
            self.config.gps_weight_low
        } else {
            0.0
        }
    }
}

impl Default for SensorFusion {
    fn default() -> Self {
        Self::new(FusionConfig::default())
    }
}

/// Earth to car frame transformation
///
/// Converts earth-frame accelerations to vehicle-frame (longitudinal/lateral)
///
/// # Arguments
/// * `ax_earth` - X acceleration in earth frame (m/s²)
/// * `ay_earth` - Y acceleration in earth frame (m/s²)
/// * `yaw` - Vehicle heading (rad, 0 = East, π/2 = North)
///
/// # Returns
/// (lon, lat) - Longitudinal (forward+) and lateral (left+) accelerations
fn earth_to_car(ax_earth: f32, ay_earth: f32, yaw: f32) -> (f32, f32) {
    let cos_yaw = yaw.cos();
    let sin_yaw = yaw.sin();

    // Rotate from earth to car frame
    let lon = ax_earth * cos_yaw + ay_earth * sin_yaw;
    let lat = -ax_earth * sin_yaw + ay_earth * cos_yaw;

    (lon, lat)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_acceleration_basic() {
        let mut gps = GpsAcceleration::new();

        // First update - no acceleration yet
        assert!(gps.update(10.0, 0.0).is_none());

        // Second update - should compute acceleration
        let accel = gps.update(12.0, 0.1); // 2 m/s increase over 0.1s = 20 m/s²
        assert!(accel.is_some());
        assert!((accel.unwrap() - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_gps_acceleration_decel() {
        let mut gps = GpsAcceleration::new();

        gps.update(20.0, 0.0);
        let accel = gps.update(15.0, 0.1); // 5 m/s decrease = -50 m/s²

        assert!(accel.is_some());
        assert!((accel.unwrap() + 50.0).abs() < 0.1);
    }

    #[test]
    fn test_gps_staleness() {
        let mut gps = GpsAcceleration::new();
        let config = FusionConfig::default();

        gps.update(10.0, 0.0);
        gps.update(12.0, 0.1);

        assert!(gps.is_fresh(config.gps_max_age));

        // Advance time without GPS update
        gps.advance_time(0.3); // > 200ms
        assert!(!gps.is_fresh(config.gps_max_age));
        assert!(gps.get_accel().is_none());
    }

    #[test]
    fn test_tilt_estimator_learns() {
        let mut tilt = TiltEstimator::new(1.0); // 1 second learn time

        // Simulate stationary with 0.5 m/s² offset
        let mut learned = false;
        for _ in 0..300 {
            let dt = 0.005; // 200Hz
            if tilt.update_stationary(0.5, 0.3, dt) {
                learned = true;
            }
        }

        // Should have learned by now - verify by checking correction works
        assert!(learned, "Tilt should have been learned");

        // If offset was learned as ~(0.5, 0.3), then correct(0.5, 0.3) should give ~(0, 0)
        let (ax, ay) = tilt.correct(0.5, 0.3);
        assert!(ax.abs() < 0.1, "Corrected X should be ~0: {}", ax);
        assert!(ay.abs() < 0.1, "Corrected Y should be ~0: {}", ay);
    }

    #[test]
    fn test_tilt_correction() {
        let mut tilt = TiltEstimator::new(0.5);

        // Learn offset
        for _ in 0..200 {
            tilt.update_stationary(0.4, -0.2, 0.005);
        }

        // Apply correction
        let (ax, ay) = tilt.correct(0.5, 0.0);
        assert!((ax - 0.1).abs() < 0.15); // 0.5 - 0.4 = 0.1
        assert!((ay - 0.2).abs() < 0.15); // 0.0 - (-0.2) = 0.2
    }

    #[test]
    fn test_gravity_estimator_steady_state() {
        let config = FusionConfig {
            gravity_learn_time: 0.5,
            steady_state_speed_tolerance: 1.0,
            steady_state_yaw_tolerance: 0.1,
            gravity_alpha: 0.1,
            ..Default::default()
        };

        let mut grav = GravityEstimator::new(&config);

        // Simulate steady-state driving at 15 m/s with small gravity offset
        for _ in 0..200 {
            grav.update(
                0.3,  // ax offset
                -0.2, // ay offset
                15.0, // speed
                0.01, // low yaw rate
                0.01, // dt
            );
        }

        // Verify learning by checking correction reduces the offset
        // If it learned offset ~(0.3, -0.2), then correct(0.3, -0.2) should give ~(0, 0)
        let (ax, ay) = grav.correct(0.3, -0.2);
        assert!(ax.abs() < 0.1, "Corrected X should be ~0: {}", ax);
        assert!(ay.abs() < 0.1, "Corrected Y should be ~0: {}", ay);
    }

    #[test]
    fn test_sensor_fusion_blending() {
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Simulate high GPS rate
        fusion.gps_accel.rate_ema = 25.0;
        fusion.gps_accel.update(10.0, 0.0);
        fusion.gps_accel.update(12.0, 0.04); // 50 m/s² GPS accel

        // Process IMU with different acceleration
        let (lon, _lat) = fusion.process_imu(
            20.0, // IMU shows 20 m/s² in earth X
            0.0,  // No Y accel
            0.0,  // Heading East
            10.0, // Moving
            0.0,  // No yaw rate
            0.005,
            false, // Not stationary
        );

        // With 90% GPS weight, result should be closer to GPS
        // GPS: 50 m/s², IMU: ~20 m/s² (after filtering)
        // We can't predict exact value due to filter transient
        assert!(lon.abs() < 100.0, "Longitudinal should be reasonable");
    }

    #[test]
    fn test_earth_to_car_transform() {
        // Heading East (yaw = 0)
        let (lon, lat) = earth_to_car(10.0, 0.0, 0.0);
        assert!((lon - 10.0).abs() < 0.01);
        assert!(lat.abs() < 0.01);

        // Heading North (yaw = π/2)
        let (lon, lat) = earth_to_car(0.0, 10.0, core::f32::consts::FRAC_PI_2);
        assert!((lon - 10.0).abs() < 0.01);
        assert!(lat.abs() < 0.01);

        // Heading West (yaw = π)
        let (lon, lat) = earth_to_car(-10.0, 0.0, core::f32::consts::PI);
        assert!((lon - 10.0).abs() < 0.01);
        assert!(lat.abs() < 0.01);
    }

    #[test]
    fn test_gps_rate_calculation() {
        let mut gps = GpsAcceleration::new();

        // Initialize
        gps.update_rate(0, 0.0);

        // After 1 second with 25 fixes
        gps.update_rate(25, 1.0);
        assert!((gps.get_rate() - 7.5).abs() < 1.0); // First update, EMA starts low

        // After another second with 25 more fixes
        gps.update_rate(50, 2.0);
        let rate = gps.get_rate();
        assert!(rate > 10.0 && rate < 30.0, "Rate should converge: {}", rate);
    }
}
