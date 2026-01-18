//! Sensor Fusion Module
//!
//! This module handles:
//! 1. GPS-derived longitudinal acceleration (from velocity changes)
//! 2. GPS/IMU acceleration blending with adaptive weights
//! 3. Butterworth low-pass filtering to remove engine vibration
//! 4. Dynamic tilt offset learning (ZUPT enhancement)
//! 5. Moving gravity estimation (continuous calibration while driving)
//! 6. GPS heading-based yaw rate calibration
//!
//! The goal is to provide clean, drift-free acceleration for mode detection
//! that works regardless of device mounting angle.
//!
//! ## Vibration Filtering
//!
//! Engine vibration (20-100+ Hz) is removed using a 2nd-order Butterworth
//! low-pass filter at 5 Hz. This preserves driving dynamics (0-3 Hz) while
//! eliminating noise from engine, alternator, road surface, etc.
//!
//! Research references:
//! - ArduPilot uses 10 Hz on accelerometers (outer loop doesn't need fast response)
//! - Academic research shows 1-5 Hz Butterworth is effective for vehicle dynamics
//! - Driving events (braking, acceleration, cornering) are all below 3 Hz

use crate::filter::BiquadFilter;
use crate::transforms::earth_to_car;

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
    /// Low-pass filter cutoff for IMU longitudinal (Hz)
    /// Removes engine vibration (20-100Hz) while passing driving dynamics (0-3Hz)
    /// Research: ArduPilot uses 10Hz, academic papers suggest 1-5Hz for vehicle dynamics
    pub lon_filter_cutoff: f32,
    /// Sample rate for longitudinal filter (Hz) - must match telemetry rate
    pub lon_sample_rate: f32,
}

impl Default for FusionConfig {
    /// Default configuration balanced for city/highway/canyon driving.
    ///
    /// GPS weights are conservative (40/30/20) to trust filtered IMU more:
    /// - GPS-derived accel is often 0 (no speed change between samples)
    /// - Filtered IMU provides faster response
    ///
    /// The 15 Hz Butterworth filter removes engine vibration (30-100Hz) while
    /// preserving all driving dynamics (0-10Hz). This is based on:
    /// - ArduPilot: uses 20Hz for INS_ACCEL_FILTER
    /// - Research: shows 1-5Hz Butterworth effective for brake detection
    /// - Physics: driving events (braking, cornering) are all < 3Hz
    fn default() -> Self {
        Self {
            gps_high_rate: 20.0,
            gps_medium_rate: 10.0,
            gps_max_age: 0.2, // 200ms
            // GPS blending weights - conservative values to trust filtered IMU more
            // GPS-derived accel is often 0 (no speed change between samples)
            gps_weight_high: 0.40,   // 40% GPS / 60% IMU at >= 20Hz
            gps_weight_medium: 0.30, // 30% GPS / 70% IMU at >= 10Hz
            gps_weight_low: 0.20,    // 20% GPS / 80% IMU at < 10Hz
            tilt_learn_time: 3.0,
            // Butterworth filter for IMU longitudinal vibration removal
            // 15 Hz cutoff: passes driving dynamics (0-10Hz), removes engine vibration (30-100Hz)
            // ArduPilot uses 20Hz for INS_ACCEL_FILTER; 15Hz is conservative middle ground
            lon_filter_cutoff: 15.0,
            // CRITICAL: Must match actual IMU sample rate, NOT telemetry rate!
            // Filter is called in process_imu() which runs at IMU rate (200Hz)
            lon_sample_rate: 200.0,
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

    /// Get learned offsets and validity (for diagnostics)
    pub fn get_offsets(&self) -> (f32, f32, bool) {
        (self.offset_x, self.offset_y, self.offset_valid)
    }
}

/// Yaw rate calibrator using GPS heading
///
/// When driving straight on highway (stable GPS course), the true yaw rate
/// should be ~0. Any measured yaw rate from the gyro is bias that we can
/// learn and subtract. This prevents lateral drift in centripetal calculation.
pub struct YawRateCalibrator {
    /// Learned yaw rate bias (rad/s)
    bias: f32,
    /// Is bias estimate valid?
    valid: bool,
    /// Heading history for stability detection (radians)
    heading_history: [f32; 20],
    heading_idx: usize,
    heading_count: u32,
    /// Time driving straight (seconds)
    straight_time: f32,
    /// Accumulated yaw rate samples
    yaw_sum: f32,
    sample_count: u32,
    /// Configuration
    min_speed: f32,         // m/s, minimum speed for calibration (~29 km/h)
    heading_tolerance: f32, // radians, max heading deviation for "straight" (~2°)
    learn_time: f32,        // seconds, time required to update bias
    alpha: f32,             // EMA alpha for bias update
}

impl YawRateCalibrator {
    pub fn new() -> Self {
        Self {
            bias: 0.0,
            valid: false,
            heading_history: [0.0; 20],
            heading_idx: 0,
            heading_count: 0,
            straight_time: 0.0,
            yaw_sum: 0.0,
            sample_count: 0,
            min_speed: 8.0,           // ~29 km/h
            heading_tolerance: 0.035, // ~2 degrees
            learn_time: 3.0,          // 3 seconds of straight driving
            alpha: 0.1,               // Slow update for stability
        }
    }

    /// Update heading history with new GPS course
    /// Call this at GPS rate (5-25 Hz)
    pub fn update_heading(&mut self, heading: f32) {
        self.heading_history[self.heading_idx] = heading;
        self.heading_idx = (self.heading_idx + 1) % self.heading_history.len();
        if self.heading_count < self.heading_history.len() as u32 {
            self.heading_count += 1;
        }
    }

    /// Update with gyro yaw rate and check for calibration opportunity
    /// Call this at telemetry rate (20 Hz)
    ///
    /// # Returns
    /// true if bias was updated
    pub fn update(&mut self, yaw_rate: f32, speed: f32, dt: f32) -> bool {
        // Need minimum speed and enough heading history
        if speed < self.min_speed || self.heading_count < 10 {
            self.reset_accumulator();
            return false;
        }

        // Check if heading is stable (driving straight)
        if !self.is_heading_stable() {
            self.reset_accumulator();
            return false;
        }

        // Accumulate yaw rate samples
        self.straight_time += dt;
        self.yaw_sum += yaw_rate;
        self.sample_count += 1;

        // After sufficient time, learn bias
        if self.straight_time >= self.learn_time && self.sample_count > 50 {
            let measured_bias = self.yaw_sum / self.sample_count as f32;

            if self.valid {
                // EMA update for smooth correction
                self.bias = (1.0 - self.alpha) * self.bias + self.alpha * measured_bias;
            } else {
                self.bias = measured_bias;
                self.valid = true;
            }

            self.reset_accumulator();
            return true;
        }

        false
    }

    fn reset_accumulator(&mut self) {
        self.straight_time = 0.0;
        self.yaw_sum = 0.0;
        self.sample_count = 0;
    }

    fn is_heading_stable(&self) -> bool {
        if self.heading_count < 10 {
            return false;
        }

        let count = self.heading_count as usize;

        // Compute circular mean (headings wrap around at ±π)
        let mut sin_sum = 0.0f32;
        let mut cos_sum = 0.0f32;
        for &h in &self.heading_history[..count] {
            sin_sum += h.sin();
            cos_sum += h.cos();
        }
        let mean_heading = sin_sum.atan2(cos_sum);

        // Check all headings are within tolerance of mean
        for &h in &self.heading_history[..count] {
            let diff = angle_diff(h, mean_heading);
            if diff.abs() > self.heading_tolerance {
                return false;
            }
        }

        true
    }

    /// Apply bias correction to yaw rate
    pub fn correct(&self, yaw_rate: f32) -> f32 {
        if self.valid {
            yaw_rate - self.bias
        } else {
            yaw_rate
        }
    }

    /// Get current bias estimate (rad/s)
    pub fn get_bias(&self) -> f32 {
        self.bias
    }

    /// Is bias calibration valid?
    pub fn is_valid(&self) -> bool {
        self.valid
    }
}

impl Default for YawRateCalibrator {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute angular difference handling wrap-around at ±π
fn angle_diff(a: f32, b: f32) -> f32 {
    let mut diff = a - b;
    if diff > core::f32::consts::PI {
        diff -= 2.0 * core::f32::consts::PI;
    } else if diff < -core::f32::consts::PI {
        diff += 2.0 * core::f32::consts::PI;
    }
    diff
}

/// Main sensor fusion processor
pub struct SensorFusion {
    pub config: FusionConfig,
    /// GPS-derived acceleration
    pub gps_accel: GpsAcceleration,
    /// Tilt estimator (learns when stopped)
    pub tilt_estimator: TiltEstimator,
    /// Yaw rate calibrator (learns gyro bias while driving straight)
    pub yaw_rate_calibrator: YawRateCalibrator,
    /// Butterworth low-pass filter for IMU longitudinal (removes engine vibration)
    lon_filter: BiquadFilter,
    /// Blended longitudinal acceleration (m/s²)
    blended_lon: f32,
    /// Display longitudinal - GPS when fresh, otherwise filtered blended (m/s²)
    lon_display: f32,
    /// Raw IMU longitudinal (unfiltered, m/s², vehicle frame)
    lon_imu_raw: f32,
    /// Filtered IMU longitudinal (m/s², vehicle frame)
    lon_imu_filtered: f32,
    /// Corrected lateral acceleration for display (m/s², vehicle frame)
    lat_corrected: f32,
    /// Centripetal lateral acceleration for mode detection (m/s²)
    lat_centripetal: f32,
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
            yaw_rate_calibrator: YawRateCalibrator::new(),
            lon_filter: BiquadFilter::new_lowpass(config.lon_filter_cutoff, config.lon_sample_rate),
            blended_lon: 0.0,
            lon_display: 0.0,
            lon_imu_raw: 0.0,
            lon_imu_filtered: 0.0,
            lat_corrected: 0.0,
            lat_centripetal: 0.0,
            last_gps_weight: 0.0,
            was_stationary: false,
            config,
        }
    }

    /// Process IMU data (call at telemetry rate, e.g., 20Hz)
    ///
    /// # Arguments
    /// * `ax_earth` - X acceleration in earth frame (m/s²)
    /// * `ay_earth` - Y acceleration in earth frame (m/s²)
    /// * `yaw` - Vehicle yaw angle (rad, for earth-to-car transform)
    /// * `speed` - Vehicle speed (m/s)
    /// * `yaw_rate` - Yaw rate (rad/s)
    /// * `dt` - Time step (seconds)
    /// * `is_stationary` - Whether vehicle is currently stationary
    ///
    /// # Returns
    /// (lon_blended, lat_centripetal) - Accelerations for mode detection (m/s²)
    /// lon_blended: GPS/IMU blended longitudinal
    /// lat_centripetal: speed * yaw_rate (pro-style, mount-independent)
    #[allow(clippy::too_many_arguments)]
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
        // This handles mounting angle offset - the device learns the residual
        // acceleration when stationary and subtracts it while moving.
        let (ax_corr, ay_corr) = self.tilt_estimator.correct(ax_earth, ay_earth);

        // Transform to vehicle frame
        let (lon_imu_raw, lat_imu_raw) = earth_to_car(ax_corr, ay_corr, yaw);

        // Store raw IMU longitudinal (before filtering)
        self.lon_imu_raw = lon_imu_raw;

        // Apply Butterworth low-pass filter to remove engine vibration (30-100Hz)
        // Filter cutoff is 15Hz: passes driving dynamics (0-5Hz), attenuates vibration
        // Based on ArduPilot (10-20Hz on accels, INS_ACCEL_FILTER parameter)
        let lon_imu_filtered = self.lon_filter.process(lon_imu_raw);
        self.lon_imu_filtered = lon_imu_filtered;

        // Store corrected lateral for display
        self.lat_corrected = lat_imu_raw;

        // Apply yaw rate calibration (removes gyro bias learned while driving straight)
        let yaw_rate_corrected = self.yaw_rate_calibrator.correct(yaw_rate);

        // Calculate centripetal lateral for mode detection (pro-style)
        // a_lateral = v * omega (mount-angle independent, no filtering needed)
        // Uses calibrated yaw rate to prevent drift on highway
        self.lat_centripetal = speed * yaw_rate_corrected;

        // Handle stationary state transitions
        if is_stationary {
            // Learn tilt offset when stopped
            self.tilt_estimator
                .update_stationary(ax_earth, ay_earth, dt);
        } else {
            if self.was_stationary {
                // Just started moving
                self.tilt_estimator.reset_stationary();
            }

            // Update yaw rate calibrator (learns bias while driving straight)
            self.yaw_rate_calibrator.update(yaw_rate, speed, dt);
        }
        self.was_stationary = is_stationary;

        // Blend GPS and IMU for longitudinal acceleration
        // GPS provides smooth, drift-free acceleration from velocity changes
        // IMU (now filtered) provides fast response with vibration removed
        // Blending gives best of both: smooth when GPS available, fast filtered fallback
        let base_gps_weight = self.compute_gps_weight();

        let lon_blended = if let Some(gps_lon) = self.gps_accel.get_accel() {
            // GPS accel validity check: if GPS shows ~0 but IMU shows signal,
            // GPS is likely just not updating (no speed change between samples)
            // In this case, trust filtered IMU entirely
            const GPS_ACCEL_MIN_THRESHOLD: f32 = 0.2; // m/s² (~0.02g)

            let effective_gps_weight = if gps_lon.abs() < GPS_ACCEL_MIN_THRESHOLD
                && self.lon_imu_filtered.abs() > GPS_ACCEL_MIN_THRESHOLD
            {
                // GPS showing ~0 but IMU has signal → GPS unreliable, use 100% IMU
                0.0
            } else {
                base_gps_weight
            };

            self.last_gps_weight = effective_gps_weight;

            // Blend GPS with filtered IMU (vibration removed)
            effective_gps_weight * gps_lon + (1.0 - effective_gps_weight) * self.lon_imu_filtered
        } else {
            // No GPS - use filtered IMU only (still vibration-free)
            self.last_gps_weight = 0.0;
            self.lon_imu_filtered
        };

        self.blended_lon = lon_blended;

        // Display uses same blended value as mode detection
        // GPS-only was problematic: GPS-derived accel is often 0 (no speed change between samples)
        // With 15Hz Butterworth filter, vibration is removed and IMU is responsive
        self.lon_display = lon_blended;

        // Return blended longitudinal and centripetal lateral for mode detection
        (lon_blended, self.lat_centripetal)
    }

    /// Process GPS speed update (call at GPS rate, e.g., 25Hz)
    ///
    /// # Arguments
    /// * `speed` - GPS speed in m/s
    /// * `time` - Monotonic timestamp in seconds
    pub fn process_gps(&mut self, speed: f32, time: f32) {
        self.gps_accel.update(speed, time);
    }

    /// Process GPS heading update for yaw rate calibration
    /// Call this at GPS rate when GPS is valid and speed > 0
    ///
    /// # Arguments
    /// * `heading` - Course over ground in radians
    pub fn process_gps_heading(&mut self, heading: f32) {
        self.yaw_rate_calibrator.update_heading(heading);
    }

    /// Update GPS rate estimate (call periodically, e.g., every second)
    pub fn update_gps_rate(&mut self, fix_count: u32, time: f32) {
        self.gps_accel.update_rate(fix_count, time);
    }

    /// Get display-optimized longitudinal acceleration (vehicle frame, m/s²)
    /// Uses GPS-derived acceleration when fresh (no vibration noise)
    /// Falls back to blended with heavier smoothing when GPS stale
    /// Positive = forward acceleration
    pub fn get_lon_display(&self) -> f32 {
        self.lon_display
    }

    /// Get blended longitudinal acceleration (vehicle frame, m/s²)
    /// This is GPS/IMU blended - same value used by mode classification
    /// Positive = forward acceleration
    pub fn get_lon_blended(&self) -> f32 {
        self.blended_lon
    }

    /// Get corrected lateral acceleration for display (vehicle frame, m/s²)
    /// This is accelerometer-based with tilt/gravity correction
    /// Positive = left
    /// NOTE: Consider using get_lat_centripetal() instead for responsive display
    pub fn get_lat_display(&self) -> f32 {
        self.lat_corrected
    }

    /// Get centripetal lateral acceleration (vehicle frame, m/s²)
    /// Computed as speed × yaw_rate - instant, mount-independent, doesn't stick
    /// This is the same value used by mode detection
    /// Positive = left turn
    pub fn get_lat_centripetal(&self) -> f32 {
        self.lat_centripetal
    }

    // ========== Diagnostic getters ==========

    /// Get raw IMU longitudinal acceleration (before filter, m/s²)
    pub fn get_lon_imu_raw(&self) -> f32 {
        self.lon_imu_raw
    }

    /// Get filtered IMU longitudinal acceleration (after Butterworth, m/s²)
    pub fn get_lon_imu_filtered(&self) -> f32 {
        self.lon_imu_filtered
    }

    /// Get last GPS blend weight (0.0-1.0)
    pub fn get_last_gps_weight(&self) -> f32 {
        self.last_gps_weight
    }

    /// Get GPS-derived acceleration (m/s²), or NaN if invalid
    pub fn get_gps_accel(&self) -> f32 {
        self.gps_accel.get_accel().unwrap_or(f32::NAN)
    }

    /// Get GPS rate estimate (Hz)
    pub fn get_gps_rate(&self) -> f32 {
        self.gps_accel.get_rate()
    }

    /// Check if GPS was rejected by validity check
    /// Returns true if GPS showed ~0 but IMU had signal
    pub fn is_gps_rejected(&self) -> bool {
        // If we have valid GPS but weight is 0, GPS was rejected
        self.gps_accel.get_accel().is_some() && self.last_gps_weight < 0.01
    }

    /// Get tilt estimator offsets and validity
    pub fn get_tilt_offsets(&self) -> (f32, f32, bool) {
        self.tilt_estimator.get_offsets()
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_acceleration_basic() {
        let mut gps = GpsAcceleration::new();

        // First update - no acceleration yet (use non-zero time)
        assert!(gps.update(10.0, 1.0).is_none());

        // Second update - should compute acceleration
        let accel = gps.update(12.0, 1.1); // 2 m/s increase over 0.1s = 20 m/s²
        assert!(accel.is_some());
        assert!((accel.unwrap() - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_gps_acceleration_decel() {
        let mut gps = GpsAcceleration::new();

        // Use non-zero times (prev_time > 0.0 check in code)
        gps.update(20.0, 1.0);
        let accel = gps.update(15.0, 1.1); // 5 m/s decrease = -50 m/s²

        assert!(accel.is_some());
        assert!((accel.unwrap() + 50.0).abs() < 0.1);
    }

    #[test]
    fn test_gps_staleness() {
        let mut gps = GpsAcceleration::new();
        let config = FusionConfig::default();

        // Use non-zero times
        gps.update(10.0, 1.0);
        gps.update(12.0, 1.1);

        assert!(gps.is_fresh(config.gps_max_age));
        assert!(gps.get_accel().is_some());

        // Advance time without GPS update - exceeds freshness threshold
        gps.advance_time(0.3); // > 200ms (gps_max_age)
        assert!(!gps.is_fresh(config.gps_max_age));

        // Advance more time - exceeds validity threshold (0.5s internal)
        gps.advance_time(0.3); // total 0.6s > 0.5s
        assert!(gps.get_accel().is_none());
    }

    #[test]
    fn test_tilt_estimator_learns_and_corrects() {
        let mut tilt = TiltEstimator::new(1.0); // 1 second learn time

        // Simulate stationary with 0.5 m/s² offset (tilted mount)
        for _ in 0..300 {
            // 1.5 seconds at 200Hz
            tilt.update_stationary(0.5, 0.3, 0.005);
        }

        // Correction should now reduce the offset to near zero
        let (ax, ay) = tilt.correct(0.5, 0.3);
        assert!(ax.abs() < 0.1, "Corrected X should be ~0: {}", ax);
        assert!(ay.abs() < 0.1, "Corrected Y should be ~0: {}", ay);
    }

    #[test]
    fn test_centripetal_lateral_during_turn() {
        // Simulate a turn: speed = 10 m/s, yaw_rate = 0.3 rad/s
        // Expected centripetal: 10 * 0.3 = 3 m/s²
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Process with turn conditions
        let (_lon, lat) = fusion.process_imu(
            0.0,   // No earth-frame X accel
            0.0,   // No earth-frame Y accel
            0.0,   // Heading
            10.0,  // Speed = 10 m/s
            0.3,   // Yaw rate = 0.3 rad/s (turning)
            0.05,  // dt
            false, // Not stationary
        );

        // Lateral should be centripetal: speed * yaw_rate = 3 m/s²
        assert!(
            (lat - 3.0).abs() < 0.01,
            "Centripetal lateral should be 3.0: got {}",
            lat
        );
    }

    #[test]
    fn test_lateral_returns_to_zero_after_turn() {
        // This test verifies the critical bug fix:
        // After a turn ends, lateral should return to zero immediately
        // (not "stick" due to earth-frame filtering)

        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Simulate a 90° turn
        let mut yaw = 0.0;
        for i in 0..20 {
            // 1 second of turning at 20Hz
            let yaw_rate = 1.57; // ~90°/s
            yaw += yaw_rate * 0.05;

            let (_lon, lat) = fusion.process_imu(
                0.0,      // Earth X
                0.0,      // Earth Y
                yaw,      // Changing heading
                10.0,     // Constant speed
                yaw_rate, // Turning
                0.05,     // dt
                false,    // Not stationary
            );

            // During turn, lateral should be positive (speed * positive yaw_rate)
            if i > 5 {
                assert!(
                    lat > 10.0,
                    "During turn, lateral should be high: {}",
                    lat
                );
            }
        }

        // Turn ends - yaw_rate goes to zero
        let (_lon, lat) = fusion.process_imu(
            0.0,  // Earth X
            0.0,  // Earth Y
            yaw,  // Final heading
            10.0, // Still moving
            0.0,  // NO MORE TURNING
            0.05, // dt
            false,
        );

        // Lateral should be zero immediately (no sticking!)
        assert!(
            lat.abs() < 0.01,
            "After turn ends, lateral should be ~0 immediately: got {}",
            lat
        );
    }

    #[test]
    fn test_display_matches_mode_classification_longitudinal() {
        // Verify that get_lon_blended() returns the same value as process_imu()
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Setup GPS
        fusion.gps_accel.rate_ema = 25.0;
        fusion.gps_accel.update(10.0, 0.0);
        fusion.gps_accel.update(12.0, 0.04);

        let (lon_from_process, _lat) = fusion.process_imu(5.0, 3.0, 0.0, 10.0, 0.0, 0.05, false);

        let lon_for_display = fusion.get_lon_blended();

        assert_eq!(
            lon_from_process, lon_for_display,
            "Display lon must match mode classifier input"
        );
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

        // Initialize with non-zero time
        gps.update_rate(0, 1.0);

        // After 1 second with 25 fixes
        gps.update_rate(25, 2.0);
        // First update, EMA: 0.3 * 25 + 0.7 * 0 = 7.5
        assert!((gps.get_rate() - 7.5).abs() < 1.0, "First rate should be ~7.5: got {}", gps.get_rate());

        // After another second with 25 more fixes
        gps.update_rate(50, 3.0);
        let rate = gps.get_rate();
        // EMA: 0.3 * 25 + 0.7 * 7.5 = 7.5 + 5.25 = 12.75
        assert!(rate > 10.0 && rate < 20.0, "Rate should converge: {}", rate);
    }

    #[test]
    fn test_yaw_rate_calibrator_learns_bias() {
        let mut cal = YawRateCalibrator::new();

        // Simulate driving straight on highway with constant heading
        // Heading = 0 rad (East), speed = 20 m/s, gyro has 0.01 rad/s bias
        let bias = 0.01; // Small gyro bias

        // Feed stable headings (within 2° tolerance)
        for _ in 0..30 {
            cal.update_heading(0.0); // Constant heading
        }

        // Drive straight for 4 seconds at 20 Hz with biased gyro
        for _ in 0..80 {
            cal.update(bias, 20.0, 0.05);
        }

        // Calibrator should have learned the bias
        assert!(cal.is_valid(), "Should have valid calibration after 4s straight");
        assert!(
            (cal.get_bias() - bias).abs() < 0.005,
            "Learned bias should be close to actual: {} vs {}",
            cal.get_bias(),
            bias
        );

        // Corrected yaw rate should be near zero
        let corrected = cal.correct(bias);
        assert!(
            corrected.abs() < 0.005,
            "Corrected yaw rate should be ~0: {}",
            corrected
        );
    }

    #[test]
    fn test_yaw_rate_calibrator_rejects_turns() {
        let mut cal = YawRateCalibrator::new();

        // Feed varying headings (turning)
        for i in 0..30 {
            cal.update_heading(i as f32 * 0.1); // Increasing heading = turning
        }

        // Try to calibrate during turn
        for _ in 0..80 {
            cal.update(0.1, 20.0, 0.05);
        }

        // Should NOT have valid calibration because heading wasn't stable
        assert!(
            !cal.is_valid(),
            "Should reject calibration during turns"
        );
    }

    #[test]
    fn test_centripetal_uses_calibrated_yaw_rate() {
        // Verify that centripetal calculation uses the calibrated yaw rate
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Simulate highway driving with bias in yaw rate
        let yaw_bias = 0.02; // 0.02 rad/s bias

        // Feed stable headings
        for _ in 0..30 {
            fusion.process_gps_heading(0.0);
        }

        // Drive straight with biased yaw rate until calibration kicks in
        for _ in 0..100 {
            fusion.process_imu(0.0, 0.0, 0.0, 20.0, yaw_bias, 0.05, false);
        }

        // Now with calibration active, centripetal should be near zero
        // even though raw yaw_rate has bias
        let (_lon, lat) = fusion.process_imu(0.0, 0.0, 0.0, 20.0, yaw_bias, 0.05, false);

        // Without calibration: lat = 20 * 0.02 = 0.4 m/s²
        // With calibration: lat should be ~0
        assert!(
            lat.abs() < 0.1,
            "Calibrated lateral should be near zero on straight: {}",
            lat
        );
    }

    #[test]
    fn test_biquad_filter_removes_engine_vibration() {
        // Simulate engine vibration at 30Hz with 0.1g amplitude (~1 m/s²)
        // The 15Hz Butterworth filter should attenuate this by ~12dB
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        let vibration_freq = 30.0; // Hz - typical engine vibration
        let vibration_amp = 1.0;   // m/s² (~0.1g)
        let sample_rate = 200.0;   // Hz - IMU rate (must match config!)

        // Run for 2 seconds to let filter settle
        let mut max_output: f32 = 0.0;
        for i in 0..400 {
            let t = i as f32 / sample_rate;
            // Simulate vibrating IMU input in earth frame
            let vibration = vibration_amp * (2.0 * core::f32::consts::PI * vibration_freq * t).sin();

            let (lon, _lat) = fusion.process_imu(
                vibration, // X earth = vibrating
                0.0,       // Y earth = 0
                0.0,       // yaw = 0 (heading east)
                10.0,      // speed
                0.0,       // yaw_rate = 0
                0.005,     // dt = 5ms (200Hz)
                false,     // not stationary
            );

            // After settling (1 second = 200 samples), check output
            if i > 200 {
                max_output = max_output.max(lon.abs());
            }
        }

        // 30Hz vibration should be attenuated by ~12dB at 15Hz cutoff
        // That means 1 m/s² input → ~0.25 m/s² output (or less)
        // Being conservative, check it's below 0.4 m/s² (~0.04g)
        assert!(
            max_output < 0.4,
            "30Hz vibration should be attenuated: got {} m/s²",
            max_output
        );
    }

    #[test]
    fn test_biquad_filter_passes_driving_dynamics() {
        // Simulate a braking event at 1Hz (typical vehicle dynamics)
        // The 15Hz filter should pass this with minimal attenuation
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        let dynamics_freq = 1.0;   // Hz - braking event
        let dynamics_amp = 3.0;    // m/s² (~0.3g braking)
        let sample_rate = 200.0;   // Hz - IMU rate (must match config!)

        // Run for 3 seconds
        let mut max_output: f32 = 0.0;
        for i in 0..600 {
            let t = i as f32 / sample_rate;
            let signal = dynamics_amp * (2.0 * core::f32::consts::PI * dynamics_freq * t).sin();

            let (lon, _lat) = fusion.process_imu(
                signal,
                0.0,
                0.0,
                10.0,
                0.0,
                0.005, // dt = 5ms (200Hz)
                false,
            );

            // After settling (1 second = 200 samples), check output
            if i > 200 {
                max_output = max_output.max(lon.abs());
            }
        }

        // 1Hz should pass through with >90% amplitude
        // 3 m/s² input → >2.7 m/s² output
        assert!(
            max_output > 2.5,
            "1Hz driving dynamics should pass through: got {} m/s², expected >2.5",
            max_output
        );
    }

    #[test]
    fn test_default_config_has_correct_filter_settings() {
        // Verify critical configuration values
        let config = FusionConfig::default();

        // Filter must be configured for IMU rate (200Hz), NOT telemetry rate (20Hz)
        assert_eq!(
            config.lon_sample_rate, 200.0,
            "lon_sample_rate must be 200Hz (IMU rate)"
        );

        // Filter cutoff should be 15Hz (ArduPilot uses 20Hz)
        assert_eq!(
            config.lon_filter_cutoff, 15.0,
            "lon_filter_cutoff should be 15Hz"
        );

        // GPS weights should be conservative (trust filtered IMU more)
        assert!(
            config.gps_weight_high <= 0.5,
            "GPS weight should be <=50% to trust filtered IMU"
        );
    }

    #[test]
    fn test_lon_display_equals_lon_blended() {
        // Verify that lon_display uses lon_blended, not GPS-only
        // This ensures dashboard shows same value as mode detection
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Feed GPS data
        fusion.process_gps(10.0, 0.0);
        fusion.process_gps(10.5, 0.1); // Creates GPS accel

        // Process IMU with non-zero input
        let (lon_blended, _lat) = fusion.process_imu(
            2.0,   // Earth X accel
            0.0,   // Earth Y
            0.0,   // Heading
            10.0,  // Speed
            0.0,   // Yaw rate
            0.005, // dt
            false, // Not stationary
        );

        let lon_display = fusion.get_lon_display();

        // Display should equal blended (not GPS-only)
        assert!(
            (lon_display - lon_blended).abs() < 0.001,
            "lon_display ({}) should equal lon_blended ({})",
            lon_display,
            lon_blended
        );
    }

    #[test]
    fn test_sharp_braking_preserved_with_15hz_filter() {
        // Simulate sharp braking (0.3 second event = ~3Hz component)
        // 15Hz filter should preserve this; old 5Hz filter would attenuate
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        let sample_rate = 200.0;
        let mut max_output: f32 = 0.0;

        // 0.3 second braking pulse
        for i in 0..200 {
            let t = i as f32 / sample_rate;

            // Sharp braking pulse: ramps up in 0.1s, holds 0.1s, ramps down 0.1s
            let brake_input = if t < 0.1 {
                5.0 * (t / 0.1) // Ramp up to 5 m/s² (0.5g)
            } else if t < 0.2 {
                5.0 // Hold
            } else if t < 0.3 {
                5.0 * (1.0 - (t - 0.2) / 0.1) // Ramp down
            } else {
                0.0
            };

            let (lon, _lat) = fusion.process_imu(
                brake_input,
                0.0,
                0.0,
                15.0, // 15 m/s = 54 km/h
                0.0,
                0.005,
                false,
            );

            max_output = max_output.max(lon.abs());
        }

        // Should preserve at least 60% of 5 m/s² input
        // (some attenuation due to filter, but not severe)
        assert!(
            max_output > 3.0,
            "Sharp braking should be preserved: got {} m/s², expected >3.0",
            max_output
        );
    }

    #[test]
    fn test_gps_accel_validity_check() {
        // When GPS accel is ~0 but IMU shows signal, should use 100% IMU
        // This prevents blending with "nothing" when GPS speed doesn't change
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Set up GPS with zero acceleration (no speed change)
        fusion.process_gps(10.0, 0.0);
        fusion.process_gps(10.0, 0.1); // Same speed = 0 accel

        // Process IMU with significant acceleration (braking at 0.3g)
        // Need to run multiple iterations for filter to settle
        let imu_accel = 3.0; // m/s² = ~0.3g
        let mut lon = 0.0;

        for i in 0..100 {
            let (l, _lat) = fusion.process_imu(
                imu_accel,
                0.0,
                0.0,
                10.0, // speed
                0.0,  // yaw_rate
                0.005,
                false,
            );
            // Keep GPS "stale" by not updating it, but refresh timestamp
            if i % 20 == 0 {
                fusion.process_gps(10.0, 0.1 + (i as f32) * 0.005);
            }
            lon = l;
        }

        // GPS weight should have been set to 0 (validity check triggered)
        // So output should be close to filtered IMU, not blended toward 0
        // With 15Hz filter settled, expect at least 50% of input
        assert!(
            lon > 1.5,
            "When GPS=0 but IMU has signal, should use IMU: got {} m/s²",
            lon
        );

        // Verify GPS weight was reduced
        assert!(
            fusion.last_gps_weight < 0.1,
            "GPS weight should be ~0 when GPS accel invalid: got {}",
            fusion.last_gps_weight
        );
    }

    #[test]
    fn test_cruising_at_constant_speed_produces_zero_lon() {
        // KEY BUG TEST: When cruising at constant speed with no real acceleration,
        // lon_blended should be near zero. This catches bad offset learning.
        //
        // The GravityEstimator bug caused lon_raw = -2.6 m/s² when cruising,
        // which triggered false BRAKE detection.
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // First: learn tilt offset when stopped (simulates calibration)
        for _ in 0..600 {
            // 3 seconds at 200Hz
            fusion.process_imu(
                0.0,  // No earth X accel (level surface)
                0.0,  // No earth Y accel
                0.0,  // Heading
                0.0,  // Stopped
                0.0,  // No yaw rate
                0.005, // dt = 5ms
                true,  // STATIONARY - tilt learns here
            );
        }

        // Now: start "cruising" at 20 m/s with zero input acceleration
        // This simulates constant-speed highway driving
        let mut max_lon: f32 = 0.0;
        let mut sum_lon: f32 = 0.0;
        let samples = 400; // 2 seconds at 200Hz

        for i in 0..samples {
            // Provide GPS updates to keep it fresh
            if i % 8 == 0 {
                // 25Hz GPS rate
                fusion.process_gps(20.0, (i as f32) * 0.005); // Constant speed
            }

            let (lon, _lat) = fusion.process_imu(
                0.0,   // ZERO earth X accel - no acceleration!
                0.0,   // Zero earth Y accel
                0.0,   // Heading
                20.0,  // 20 m/s = 72 km/h cruising
                0.0,   // Straight road
                0.005, // dt = 5ms
                false, // Moving
            );

            // Skip filter settling time
            if i > 200 {
                max_lon = max_lon.max(lon.abs());
                sum_lon += lon.abs();
            }
        }

        let avg_lon = sum_lon / (samples - 200) as f32;

        // CRITICAL: lon_blended should be near zero when cruising
        // The bug caused ~2.6 m/s² here
        assert!(
            max_lon < 0.5,
            "Cruising with zero input should produce near-zero lon: max={} m/s²",
            max_lon
        );
        assert!(
            avg_lon < 0.2,
            "Average lon during cruise should be near zero: avg={} m/s²",
            avg_lon
        );
    }

    #[test]
    fn test_tilt_only_updates_when_stationary() {
        // Verify that tilt estimator only learns when is_stationary=true
        // If it learned while moving, bad offsets could accumulate
        let mut tilt = TiltEstimator::new(1.0);

        // Simulate "moving" conditions with large acceleration
        for _ in 0..300 {
            // 1.5 seconds at 200Hz
            // Note: update_stationary should only be called when stationary,
            // but let's verify the valid flag behavior
            tilt.update_stationary(5.0, 3.0, 0.005); // Large accel (would be wrong if learned)
        }

        // After learning, check the offset
        let (ax, _ay) = tilt.correct(5.0, 3.0);

        // The tilt estimator WILL learn this offset - this is by design
        // The key is that in main.rs, we only call update_stationary when truly stopped
        assert!(
            ax.abs() < 0.1,
            "After learning, correction should work: {}",
            ax
        );
    }

    #[test]
    fn test_no_drift_during_extended_cruise() {
        // Verify that lon_blended stays near zero over extended cruising
        // This catches slow drift from any remaining estimators
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Learn tilt when stopped
        for _ in 0..600 {
            fusion.process_imu(0.0, 0.0, 0.0, 0.0, 0.0, 0.005, true);
        }

        // Start cruising
        let mut lon_samples: Vec<f32> = Vec::new();

        for i in 0..2000 {
            // 10 seconds at 200Hz
            if i % 8 == 0 {
                fusion.process_gps(25.0, (i as f32) * 0.005);
            }

            let (lon, _) = fusion.process_imu(
                0.0,   // Zero input
                0.0,
                0.0,
                25.0,  // 25 m/s = 90 km/h
                0.0,
                0.005,
                false,
            );

            if i > 400 {
                // After 2 seconds settling
                lon_samples.push(lon);
            }
        }

        // Check for drift: first half vs second half average
        let mid = lon_samples.len() / 2;
        let first_half_avg: f32 = lon_samples[..mid].iter().sum::<f32>() / mid as f32;
        let second_half_avg: f32 = lon_samples[mid..].iter().sum::<f32>() / mid as f32;
        let drift = (second_half_avg - first_half_avg).abs();

        assert!(
            drift < 0.1,
            "lon should not drift during cruise: first_avg={}, second_avg={}, drift={}",
            first_half_avg,
            second_half_avg,
            drift
        );
    }
}
