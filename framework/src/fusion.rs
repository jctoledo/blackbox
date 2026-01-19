//! Sensor Fusion Module
//!
//! This module handles:
//! 1. GPS-derived longitudinal acceleration (from velocity changes)
//! 2. GPS-corrected orientation for accurate IMU acceleration
//! 3. Butterworth low-pass filtering to remove engine vibration
//! 4. Dynamic tilt offset learning (ZUPT enhancement)
//! 5. GPS heading-based yaw rate calibration
//!
//! The goal is to provide clean, drift-free acceleration for mode detection
//! that works regardless of device mounting angle.
//!
//! ## Key Innovation: GPS-Corrected Orientation
//!
//! The WT901 AHRS cannot distinguish linear acceleration from tilt. During
//! forward acceleration, it reports false pitch. This corrupts gravity removal
//! and produces wrong earth-frame acceleration.
//!
//! Solution: Use GPS velocity (ground truth) to learn orientation corrections.
//! - Compare IMU-predicted accel with GPS-derived accel
//! - The difference reveals orientation error: pitch_error ≈ (ax_imu - ax_gps) / G
//! - Apply corrections BEFORE gravity removal
//!
//! This enables accurate 200 Hz IMU data instead of being limited to GPS rate.
//!
//! ## Vibration Filtering
//!
//! Engine vibration (20-100+ Hz) is removed using a 2nd-order Butterworth
//! low-pass filter at 10 Hz. This preserves driving dynamics (0-5 Hz) while
//! eliminating noise from engine, alternator, road surface, etc.

use crate::filter::BiquadFilter;
use crate::transforms::{body_to_earth, earth_to_car, remove_gravity};

const G: f32 = 9.80665;

/// Orientation Corrector - learns pitch/roll corrections from GPS velocity
///
/// The WT901 AHRS can't distinguish linear acceleration from tilt. During
/// acceleration, it reports wrong pitch/roll, which corrupts gravity removal.
///
/// This corrector compares IMU-derived acceleration (using AHRS angles) with
/// GPS-derived acceleration (ground truth) to learn orientation corrections.
///
/// ## How it works
/// 1. Compute IMU acceleration using AHRS + current correction
/// 2. Compare with GPS-derived acceleration (dv/dt)
/// 3. The difference reveals orientation error: pitch_error ≈ (ax_imu - ax_gps) / G
/// 4. Apply EMA filter to learn smooth correction
/// 5. Only learn when vehicle is accelerating (signal present) and GPS is valid
///
/// ## Why this works
/// - During acceleration, pitch error causes ax_earth to be wrong
/// - GPS velocity gives us true horizontal acceleration
/// - The innovation (difference) is directly proportional to pitch error
pub struct OrientationCorrector {
    /// Pitch correction in radians (added to AHRS pitch before gravity removal)
    pitch_correction: f32,
    /// Roll correction in radians (added to AHRS roll before gravity removal)
    roll_correction: f32,
    /// Confidence in pitch correction (0-1, based on learning samples)
    pitch_confidence: f32,
    /// Confidence in roll correction (0-1)
    roll_confidence: f32,
    /// Cruise bias - offset learned when driving at constant speed (m/s²)
    /// This catches mounting offsets that show up during cruise but not at stops
    cruise_bias: f32,
    /// Cruise bias accumulator for averaging
    cruise_bias_sum: f32,
    /// Cruise bias sample count
    cruise_bias_count: u32,
    /// Time spent in cruise state (seconds)
    cruise_time: f32,
    /// EMA alpha for learning (smaller = slower but more stable)
    alpha: f32,
    /// Minimum speed for learning (m/s) - need motion for GPS accuracy
    min_speed: f32,
    /// Minimum acceleration to trigger pitch/roll learning (m/s²)
    /// Below this threshold, cruise bias learning kicks in instead
    min_accel: f32,
    /// Maximum correction magnitude (radians) - safety cap
    max_correction: f32,
    /// Number of learning updates (for confidence calculation)
    update_count: u32,
}

impl OrientationCorrector {
    pub fn new() -> Self {
        Self {
            pitch_correction: 0.0,
            roll_correction: 0.0,
            pitch_confidence: 0.0,
            roll_confidence: 0.0,
            cruise_bias: 0.0,
            cruise_bias_sum: 0.0,
            cruise_bias_count: 0,
            cruise_time: 0.0,
            alpha: 0.05,          // Slow learning for stability
            min_speed: 3.0,       // ~11 km/h
            min_accel: 0.3,       // ~0.03g - lowered to learn from more events
            max_correction: 0.26, // ~15 degrees max
            update_count: 0,
        }
    }

    /// Update orientation corrections based on GPS vs IMU comparison
    ///
    /// # Arguments
    /// * `lon_imu` - IMU-derived longitudinal accel (using AHRS + current correction)
    /// * `lon_gps` - GPS-derived longitudinal accel (ground truth)
    /// * `lat_imu` - IMU-derived lateral accel
    /// * `lat_gps` - GPS-derived lateral accel (from centripetal: speed × yaw_rate)
    /// * `speed` - Vehicle speed in m/s
    /// * `is_gps_fresh` - Whether GPS data is recent and valid
    /// * `dt` - Time step in seconds (for cruise learning timing)
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        lon_imu: f32,
        lon_gps: f32,
        lat_imu: f32,
        lat_gps: f32,
        speed: f32,
        is_gps_fresh: bool,
        dt: f32,
    ) {
        // Only learn when conditions are good
        if speed < self.min_speed || !is_gps_fresh {
            self.cruise_time = 0.0; // Reset cruise accumulator
            self.cruise_bias_sum = 0.0;
            self.cruise_bias_count = 0;
            return;
        }

        // Apply current cruise bias before comparison
        let lon_imu_corrected = lon_imu - self.cruise_bias;
        let lon_error = lon_imu_corrected - lon_gps;

        // Learn pitch correction from longitudinal acceleration difference
        // During acceleration: pitch error → wrong ax_earth
        if lon_gps.abs() > self.min_accel {
            // Signal present - can learn pitch
            let pitch_innovation = lon_error / G;

            // EMA update with clamping
            self.pitch_correction =
                (1.0 - self.alpha) * self.pitch_correction + self.alpha * pitch_innovation;
            self.pitch_correction = self
                .pitch_correction
                .clamp(-self.max_correction, self.max_correction);

            // Update confidence
            self.update_count = self.update_count.saturating_add(1);
            self.pitch_confidence = (self.update_count as f32 / 100.0).min(1.0);

            // Reset cruise accumulator when accelerating
            self.cruise_time = 0.0;
            self.cruise_bias_sum = 0.0;
            self.cruise_bias_count = 0;
        } else {
            // Cruising (lon_gps near zero) - learn cruise bias
            // If GPS says ~0 accel but IMU shows offset, that's mounting bias
            self.cruise_time += dt;
            self.cruise_bias_sum += lon_imu_corrected; // Should be ~0 when cruising
            self.cruise_bias_count += 1;

            // After 2 seconds of stable cruising, update cruise bias
            if self.cruise_time >= 2.0 && self.cruise_bias_count > 30 {
                let measured_bias = self.cruise_bias_sum / self.cruise_bias_count as f32;

                // EMA update for cruise bias (slower alpha for stability)
                self.cruise_bias = 0.9 * self.cruise_bias + 0.1 * measured_bias;
                // Clamp cruise bias to reasonable range (±1.5 m/s² = ±0.15g)
                self.cruise_bias = self.cruise_bias.clamp(-1.5, 1.5);

                // Reset accumulator
                self.cruise_time = 0.0;
                self.cruise_bias_sum = 0.0;
                self.cruise_bias_count = 0;
            }
        }

        // Learn roll correction from lateral acceleration difference
        // During cornering: roll error → wrong ay_earth
        let lat_error = lat_imu - lat_gps;
        if lat_gps.abs() > self.min_accel {
            let roll_innovation = lat_error / G;

            self.roll_correction =
                (1.0 - self.alpha) * self.roll_correction + self.alpha * roll_innovation;
            self.roll_correction = self
                .roll_correction
                .clamp(-self.max_correction, self.max_correction);

            self.roll_confidence = (self.update_count as f32 / 100.0).min(1.0);
        }
    }

    /// Get current cruise bias (m/s²) for diagnostics
    pub fn get_cruise_bias(&self) -> f32 {
        self.cruise_bias
    }

    /// Apply corrections to AHRS orientation
    ///
    /// Returns corrected (pitch_deg, roll_deg) for use in gravity removal
    pub fn correct(&self, ahrs_pitch_deg: f32, ahrs_roll_deg: f32) -> (f32, f32) {
        // Convert corrections from radians to degrees
        let pitch_corr_deg = self.pitch_correction.to_degrees();
        let roll_corr_deg = self.roll_correction.to_degrees();

        (
            ahrs_pitch_deg - pitch_corr_deg,
            ahrs_roll_deg - roll_corr_deg,
        )
    }

    /// Get current pitch correction in degrees (for diagnostics)
    pub fn get_pitch_correction_deg(&self) -> f32 {
        self.pitch_correction.to_degrees()
    }

    /// Get current roll correction in degrees (for diagnostics)
    pub fn get_roll_correction_deg(&self) -> f32 {
        self.roll_correction.to_degrees()
    }

    /// Get pitch confidence (0-1)
    pub fn get_pitch_confidence(&self) -> f32 {
        self.pitch_confidence
    }

    /// Get roll confidence (0-1)
    pub fn get_roll_confidence(&self) -> f32 {
        self.roll_confidence
    }

    /// Reset corrections (e.g., after device remount)
    pub fn reset(&mut self) {
        self.pitch_correction = 0.0;
        self.roll_correction = 0.0;
        self.pitch_confidence = 0.0;
        self.roll_confidence = 0.0;
        self.cruise_bias = 0.0;
        self.cruise_bias_sum = 0.0;
        self.cruise_bias_count = 0;
        self.cruise_time = 0.0;
        self.update_count = 0;
    }
}

impl Default for OrientationCorrector {
    fn default() -> Self {
        Self::new()
    }
}

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
    /// Sample rate for longitudinal filter (Hz) - must match ACTUAL call rate of process_imu()
    /// CRITICAL: This is telemetry rate (~30Hz), NOT IMU rate (200Hz)!
    pub lon_sample_rate: f32,
}

impl Default for FusionConfig {
    /// Default configuration for mode detection.
    ///
    /// ## GPS-Corrected IMU Approach
    ///
    /// We use GPS velocity to correct AHRS orientation errors, then use the
    /// corrected IMU at 200 Hz for mode detection. This gives us:
    /// - 200 Hz update rate (vs 25 Hz GPS-only)
    /// - Accurate acceleration even during dynamic maneuvers
    /// - Foundation for future granular detection (soft/medium/hard accel)
    ///
    /// The OrientationCorrector learns pitch/roll errors by comparing IMU-predicted
    /// acceleration with GPS-derived acceleration (ground truth).
    ///
    /// GPS weight fields control blending between corrected IMU and GPS acceleration
    /// for robustness during correction learning phase.
    fn default() -> Self {
        Self {
            gps_high_rate: 20.0,
            gps_medium_rate: 10.0,
            gps_max_age: 0.2, // 200ms - GPS older than this is considered stale
            // GPS weight fields - blend GPS with corrected IMU based on confidence
            gps_weight_high: 0.3,   // 30% GPS, 70% corrected IMU when confident
            gps_weight_medium: 0.5, // 50/50 blend at medium confidence
            gps_weight_low: 0.8,    // 80% GPS when correction not yet learned
            tilt_learn_time: 3.0,
            // Butterworth filter for IMU
            // 10 Hz cutoff: passes driving dynamics, removes vibration
            lon_filter_cutoff: 10.0,
            // CRITICAL: Must match actual process_imu() call rate (telemetry rate ~30Hz)
            // NOT the IMU hardware rate (200Hz)!
            lon_sample_rate: 30.0,
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
    min_speed: f32, // m/s, minimum speed for calibration (~29 km/h)
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
    /// Orientation corrector (learns pitch/roll errors from GPS)
    pub orientation_corrector: OrientationCorrector,
    /// Tilt estimator (learns residual offset when stopped)
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
    /// IMU lateral (from corrected orientation) for learning
    lat_imu: f32,
    /// Last GPS blend weight (for diagnostics)
    last_gps_weight: f32,
    /// Was stationary last update?
    was_stationary: bool,
}

impl SensorFusion {
    pub fn new(config: FusionConfig) -> Self {
        Self {
            gps_accel: GpsAcceleration::new(),
            orientation_corrector: OrientationCorrector::new(),
            tilt_estimator: TiltEstimator::new(config.tilt_learn_time),
            yaw_rate_calibrator: YawRateCalibrator::new(),
            lon_filter: BiquadFilter::new_lowpass(config.lon_filter_cutoff, config.lon_sample_rate),
            blended_lon: 0.0,
            lon_display: 0.0,
            lon_imu_raw: 0.0,
            lon_imu_filtered: 0.0,
            lat_corrected: 0.0,
            lat_centripetal: 0.0,
            lat_imu: 0.0,
            last_gps_weight: 0.0,
            was_stationary: false,
            config,
        }
    }

    /// Process IMU data with GPS-corrected orientation (call at IMU rate, e.g., 200Hz)
    ///
    /// This is the ArduPilot-style approach: use GPS velocity to correct AHRS
    /// orientation errors, then use the corrected IMU for accurate 200 Hz acceleration.
    ///
    /// # Arguments
    /// * `ax_raw` - Raw X acceleration in body frame (m/s²)
    /// * `ay_raw` - Raw Y acceleration in body frame (m/s²)
    /// * `az_raw` - Raw Z acceleration in body frame (m/s²)
    /// * `ahrs_roll_deg` - AHRS roll angle in degrees
    /// * `ahrs_pitch_deg` - AHRS pitch angle in degrees
    /// * `yaw` - Vehicle yaw angle (rad, from EKF or magnetometer)
    /// * `speed` - Vehicle speed (m/s)
    /// * `yaw_rate` - Yaw rate (rad/s)
    /// * `dt` - Time step (seconds)
    /// * `is_stationary` - Whether vehicle is currently stationary
    ///
    /// # Returns
    /// (lon_blended, lat_centripetal) - Accelerations for mode detection (m/s²)
    /// lon_blended: orientation-corrected IMU blended with GPS
    /// lat_centripetal: speed * yaw_rate (pro-style, mount-independent)
    #[allow(clippy::too_many_arguments)]
    pub fn process_imu(
        &mut self,
        ax_raw: f32,
        ay_raw: f32,
        az_raw: f32,
        ahrs_roll_deg: f32,
        ahrs_pitch_deg: f32,
        yaw: f32,
        speed: f32,
        yaw_rate: f32,
        dt: f32,
        is_stationary: bool,
    ) -> (f32, f32) {
        // Track GPS staleness
        self.gps_accel.advance_time(dt);
        let gps_fresh = self.gps_accel.is_fresh(self.config.gps_max_age);

        // Step 1: Apply orientation correction (learned from GPS comparison)
        // This corrects AHRS errors caused by linear acceleration being mistaken for tilt
        let (corrected_pitch_deg, corrected_roll_deg) = self
            .orientation_corrector
            .correct(ahrs_pitch_deg, ahrs_roll_deg);

        // Step 2: Remove gravity using corrected orientation
        let (ax_nograv, ay_nograv, _az_nograv) = remove_gravity(
            ax_raw,
            ay_raw,
            az_raw,
            corrected_roll_deg,
            corrected_pitch_deg,
        );

        // Step 3: Transform to earth frame
        let (ax_earth, ay_earth) = body_to_earth(
            ax_nograv,
            ay_nograv,
            0.0,
            corrected_roll_deg,
            corrected_pitch_deg,
            yaw,
        );

        // Step 4: Apply residual tilt correction (learned when stopped)
        // This handles minor mounting offsets not captured by orientation correction
        let (ax_corr, ay_corr) = self.tilt_estimator.correct(ax_earth, ay_earth);

        // Step 5: Transform to vehicle frame
        let (lon_imu_raw, lat_imu_raw) = earth_to_car(ax_corr, ay_corr, yaw);

        // Store values
        self.lon_imu_raw = lon_imu_raw;
        self.lat_imu = lat_imu_raw;

        // Step 6: Apply Butterworth low-pass filter to remove engine vibration
        let lon_imu_filtered = self.lon_filter.process(lon_imu_raw);
        self.lon_imu_filtered = lon_imu_filtered;

        // Store corrected lateral for display
        self.lat_corrected = lat_imu_raw;

        // Step 7: Apply yaw rate calibration (removes gyro bias)
        let yaw_rate_corrected = self.yaw_rate_calibrator.correct(yaw_rate);

        // Step 8: Calculate centripetal lateral for mode detection
        // a_lateral = v * omega (mount-angle independent)
        self.lat_centripetal = speed * yaw_rate_corrected;

        // Step 9: Handle stationary state - learn residual offsets
        if is_stationary {
            self.tilt_estimator
                .update_stationary(ax_earth, ay_earth, dt);
        } else {
            if self.was_stationary {
                self.tilt_estimator.reset_stationary();
            }
            self.yaw_rate_calibrator.update(yaw_rate, speed, dt);
        }
        self.was_stationary = is_stationary;

        // Step 10: Get GPS-derived acceleration for comparison/blending
        let gps_lon = self.gps_accel.get_accel();
        let gps_lat = self.lat_centripetal; // Centripetal = speed * yaw_rate

        // Step 11: Update orientation corrector (learn from GPS vs IMU)
        // Only learn when moving and GPS is fresh
        if !is_stationary && gps_fresh {
            if let Some(lon_gps) = gps_lon {
                self.orientation_corrector.update(
                    lon_imu_filtered, // IMU (using current correction)
                    lon_gps,          // GPS (ground truth)
                    lat_imu_raw,      // IMU lateral
                    gps_lat,          // Centripetal (truth for lateral)
                    speed,
                    gps_fresh,
                    dt, // Time step for cruise bias learning
                );
            }
        }

        // Step 12: Blend IMU and GPS based on orientation correction confidence
        // Higher confidence → trust corrected IMU more
        // Lower confidence → rely more on GPS
        let lon_blended = if let Some(lon_gps) = gps_lon {
            // Compute blend weight based on correction confidence
            let confidence = self.orientation_corrector.get_pitch_confidence();

            // Interpolate between high/medium/low GPS weights based on confidence
            let gps_weight = if confidence > 0.8 {
                self.config.gps_weight_high // 30% GPS when very confident
            } else if confidence > 0.3 {
                // Linear interpolation between medium and high
                let t = (confidence - 0.3) / 0.5;
                self.config.gps_weight_medium * (1.0 - t) + self.config.gps_weight_high * t
            } else {
                // Linear interpolation between low and medium
                let t = confidence / 0.3;
                self.config.gps_weight_low * (1.0 - t) + self.config.gps_weight_medium * t
            };

            self.last_gps_weight = gps_weight;

            // Blend: (1-w)*IMU + w*GPS
            (1.0 - gps_weight) * lon_imu_filtered + gps_weight * lon_gps
        } else {
            // GPS unavailable - use corrected IMU only
            self.last_gps_weight = 0.0;
            lon_imu_filtered
        };

        self.blended_lon = lon_blended;
        self.lon_display = lon_blended;

        // Return blended longitudinal and centripetal lateral
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

    /// Get orientation correction (pitch, roll) in degrees
    pub fn get_orientation_correction(&self) -> (f32, f32) {
        (
            self.orientation_corrector.get_pitch_correction_deg(),
            self.orientation_corrector.get_roll_correction_deg(),
        )
    }

    /// Get orientation correction confidence (pitch, roll) as 0-1 values
    pub fn get_orientation_confidence(&self) -> (f32, f32) {
        (
            self.orientation_corrector.get_pitch_confidence(),
            self.orientation_corrector.get_roll_confidence(),
        )
    }

    /// Reset orientation corrector (call after device remount)
    pub fn reset_orientation_corrector(&mut self) {
        self.orientation_corrector.reset();
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

        // New API: pass raw body-frame accel
        // For level surface: ax_raw = earth_x, ay_raw = earth_y, az_raw = G
        let (_lon, lat) = fusion.process_imu(
            0.0,   // ax_raw (no forward accel)
            0.0,   // ay_raw (no lateral accel)
            G,     // az_raw (gravity)
            0.0,   // ahrs_roll_deg (level)
            0.0,   // ahrs_pitch_deg (level)
            0.0,   // yaw (heading)
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
                0.0,      // ax_raw
                0.0,      // ay_raw
                G,        // az_raw
                0.0,      // roll
                0.0,      // pitch
                yaw,      // Changing heading
                10.0,     // Constant speed
                yaw_rate, // Turning
                0.05,     // dt
                false,    // Not stationary
            );

            // During turn, lateral should be positive (speed * positive yaw_rate)
            if i > 5 {
                assert!(lat > 10.0, "During turn, lateral should be high: {}", lat);
            }
        }

        // Turn ends - yaw_rate goes to zero
        let (_lon, lat) = fusion.process_imu(
            0.0,  // ax_raw
            0.0,  // ay_raw
            G,    // az_raw
            0.0,  // roll
            0.0,  // pitch
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

        let (lon_from_process, _lat) = fusion.process_imu(
            5.0, 3.0, G, // raw body accel (forward, lateral, gravity)
            0.0, 0.0, // roll, pitch (level)
            0.0, 10.0, 0.0, 0.05, false,
        );

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
        assert!(
            (gps.get_rate() - 7.5).abs() < 1.0,
            "First rate should be ~7.5: got {}",
            gps.get_rate()
        );

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
        assert!(
            cal.is_valid(),
            "Should have valid calibration after 4s straight"
        );
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
        assert!(!cal.is_valid(), "Should reject calibration during turns");
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
            fusion.process_imu(0.0, 0.0, G, 0.0, 0.0, 0.0, 20.0, yaw_bias, 0.05, false);
        }

        // Now with calibration active, centripetal should be near zero
        // even though raw yaw_rate has bias
        let (_lon, lat) =
            fusion.process_imu(0.0, 0.0, G, 0.0, 0.0, 0.0, 20.0, yaw_bias, 0.05, false);

        // Without calibration: lat = 20 * 0.02 = 0.4 m/s²
        // With calibration: lat should be ~0
        assert!(
            lat.abs() < 0.1,
            "Calibrated lateral should be near zero on straight: {}",
            lat
        );
    }

    #[test]
    fn test_biquad_filter_removes_high_frequency_noise() {
        // Simulate noise at 12Hz with 0.1g amplitude (~1 m/s²)
        // The 10Hz cutoff Butterworth filter should attenuate this
        // (12Hz is above 10Hz cutoff but below 15Hz Nyquist for 30Hz sample rate)
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        let noise_freq = 12.0; // Hz - above 10Hz cutoff, below 15Hz Nyquist
        let noise_amp = 1.0; // m/s² (~0.1g)
        let sample_rate = 30.0; // Hz - telemetry rate (matches filter config!)

        // Run for 3 seconds to let filter settle
        let mut max_output: f32 = 0.0;
        for i in 0..90 {
            let t = i as f32 / sample_rate;
            // Simulate noisy IMU input in body frame
            let noise = noise_amp * (2.0 * core::f32::consts::PI * noise_freq * t).sin();

            let (lon, _lat) = fusion.process_imu(
                noise, // ax_raw = noisy
                0.0,   // ay_raw = 0
                G,     // az_raw = gravity
                0.0,   // roll = level
                0.0,   // pitch = level
                0.0,   // yaw = 0 (heading east)
                10.0,  // speed
                0.0,   // yaw_rate = 0
                0.033, // dt = 33ms (~30Hz)
                false, // not stationary
            );

            // After settling (1 second = 30 samples), check output
            if i > 30 {
                max_output = max_output.max(lon.abs());
            }
        }

        // 12Hz noise should be attenuated by the 10Hz cutoff filter
        // Expect ~6dB attenuation: 1 m/s² input → ~0.5 m/s² output
        // Being conservative, check it's below 0.7 m/s²
        assert!(
            max_output < 0.7,
            "12Hz noise should be attenuated: got {} m/s²",
            max_output
        );
    }

    #[test]
    fn test_biquad_filter_passes_driving_dynamics() {
        // Simulate a braking event at 1Hz (typical vehicle dynamics)
        // The 10Hz cutoff filter should pass this with minimal attenuation
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        let dynamics_freq = 1.0; // Hz - braking event
        let dynamics_amp = 3.0; // m/s² (~0.3g braking)
        let sample_rate = 30.0; // Hz - telemetry rate (matches filter config!)

        // Run for 5 seconds to let filter settle
        let mut max_output: f32 = 0.0;
        for i in 0..150 {
            let t = i as f32 / sample_rate;
            let signal = dynamics_amp * (2.0 * core::f32::consts::PI * dynamics_freq * t).sin();

            let (lon, _lat) = fusion.process_imu(
                signal, // ax_raw
                0.0,    // ay_raw
                G,      // az_raw
                0.0,    // roll
                0.0,    // pitch
                0.0,    // yaw
                10.0,   // speed
                0.0,    // yaw_rate
                0.033,  // dt = 33ms (~30Hz)
                false,
            );

            // After settling (1 second = 30 samples), check output
            if i > 30 {
                max_output = max_output.max(lon.abs());
            }
        }

        // 1Hz should pass through with >80% amplitude at 10Hz cutoff
        // 3 m/s² input → >2.4 m/s² output
        assert!(
            max_output > 2.4,
            "1Hz driving dynamics should pass through: got {} m/s², expected >2.4",
            max_output
        );
    }

    #[test]
    fn test_default_config_has_correct_filter_settings() {
        // Verify critical configuration values
        let config = FusionConfig::default();

        // Filter must be configured for TELEMETRY rate (~30Hz), not IMU hardware rate
        // process_imu() is called at telemetry rate, not IMU sample rate!
        assert_eq!(
            config.lon_sample_rate, 30.0,
            "lon_sample_rate must be 30Hz (telemetry rate, not IMU hardware rate)"
        );

        // Filter cutoff should be 10Hz for smoother output
        assert_eq!(
            config.lon_filter_cutoff, 10.0,
            "lon_filter_cutoff should be 10Hz"
        );

        // GPS max age for freshness check
        assert!(
            config.gps_max_age <= 0.3,
            "gps_max_age should be <=300ms for responsive fallback"
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
            2.0, 0.0, G, // ax, ay, az (body frame)
            0.0, 0.0, // roll, pitch
            0.0, 10.0, 0.0, 0.005, false,
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
                brake_input, // ax_raw
                0.0,         // ay_raw
                G,           // az_raw
                0.0,         // roll
                0.0,         // pitch
                0.0,         // yaw
                15.0,        // 15 m/s = 54 km/h
                0.0,         // yaw_rate
                0.005,       // dt
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
    fn test_orientation_correction_converges_during_cruise() {
        // With the new GPS-corrected orientation approach:
        // - IMU shows fake acceleration due to AHRS error
        // - OrientationCorrector learns the error from GPS comparison
        // - After learning, lon_blended converges toward GPS (truth)
        //
        // This test verifies the OrientationCorrector learns during driving.
        let config = FusionConfig::default();
        let mut fusion = SensorFusion::new(config);

        // Set up GPS with zero acceleration (cruising at constant speed)
        fusion.process_gps(10.0, 0.0);
        fusion.process_gps(10.0, 0.1); // Same speed = 0 accel

        // Process IMU with fake "acceleration" (simulates AHRS pitch error)
        // pitch_error ≈ 3° causes ~0.5 m/s² fake accel
        let imu_fake_accel = 0.5; // m/s² - this is AHRS error, not real accel
        let mut lon = 0.0;

        for i in 0..200 {
            let (l, _lat) = fusion.process_imu(
                imu_fake_accel, // Fake signal from AHRS pitch error
                0.0,            // ay_raw
                G,              // az_raw
                0.0,            // roll
                0.0,            // pitch (AHRS reports 0, but there's error we're simulating)
                0.0,            // yaw
                10.0,           // speed
                0.0,            // yaw_rate
                0.005,          // dt
                false,
            );
            // Keep GPS fresh
            if i % 20 == 0 {
                fusion.process_gps(10.0, 0.1 + (i as f32) * 0.005);
            }
            lon = l;
        }

        // With GPS-IMU blending based on confidence:
        // - Initially high GPS weight → lon close to GPS (0)
        // - As correction learns → GPS weight decreases but corrected IMU also → 0
        // Either way, lon should be small
        assert!(
            lon.abs() < 0.5,
            "During cruise, lon should converge to ~0: got {} m/s²",
            lon
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
                0.0, 0.0, G, // ax, ay, az (level, no motion)
                0.0, 0.0,   // roll, pitch
                0.0,   // yaw
                0.0,   // Stopped
                0.0,   // No yaw rate
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
                0.0, 0.0, G, // ax, ay, az - ZERO accel!
                0.0, 0.0,   // roll, pitch
                0.0,   // yaw
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
            fusion.process_imu(0.0, 0.0, G, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, true);
        }

        // Start cruising
        let mut lon_samples: Vec<f32> = Vec::new();

        for i in 0..2000 {
            // 10 seconds at 200Hz
            if i % 8 == 0 {
                fusion.process_gps(25.0, (i as f32) * 0.005);
            }

            let (lon, _) = fusion.process_imu(
                0.0, 0.0, G, // ax, ay, az - zero input
                0.0, 0.0,   // roll, pitch
                0.0,   // yaw
                25.0,  // 25 m/s = 90 km/h
                0.0,   // yaw_rate
                0.005, // dt
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

    // ============== OrientationCorrector Tests ==============

    #[test]
    fn test_orientation_corrector_init() {
        let corrector = OrientationCorrector::new();

        assert_eq!(corrector.pitch_correction, 0.0);
        assert_eq!(corrector.roll_correction, 0.0);
        assert_eq!(corrector.pitch_confidence, 0.0);
        assert_eq!(corrector.roll_confidence, 0.0);
    }

    #[test]
    fn test_orientation_corrector_learns_pitch_from_forward_accel() {
        // Simulate: true accel = 3.0 m/s², but AHRS has pitch error
        // causing IMU to show different value than GPS
        //
        // AHRS pitch error = -3° (thinks car tilting back during forward accel)
        // This causes remove_gravity to subtract wrong gravity component
        // Result: lon_imu is LESS than lon_gps
        //
        // Math: gravity error ≈ G * sin(3°) ≈ 0.51 m/s²
        // So lon_imu ≈ 3.0 - 0.51 = 2.49 m/s² (if AHRS pitch = -3°)
        // GPS shows truth: lon_gps = 3.0 m/s²
        // Innovation = (2.49 - 3.0) / 9.8 ≈ -0.052 rad ≈ -3°

        let mut corrector = OrientationCorrector::new();

        let lon_gps = 3.0; // Ground truth
        let lon_imu = 2.49; // What IMU shows with -3° pitch error
        let speed = 15.0; // 54 km/h, above min_speed

        // Simulate many updates (EMA needs time to converge)
        for _ in 0..500 {
            corrector.update(
                lon_imu, // IMU longitudinal
                lon_gps, // GPS longitudinal (truth)
                0.0,     // lat_imu
                0.0,     // lat_gps
                speed, true,  // GPS fresh
                0.033, // dt (~30Hz)
            );
        }

        // Pitch correction should converge toward the error
        // Expected: ~-3° = ~-0.052 rad
        let pitch_corr = corrector.pitch_correction;
        assert!(
            (pitch_corr - (-0.052)).abs() < 0.01,
            "Pitch correction should be ~-0.052 rad (-3°), got {} rad ({}°)",
            pitch_corr,
            pitch_corr.to_degrees()
        );

        // Confidence should be high after many updates
        assert!(
            corrector.pitch_confidence > 0.9,
            "Confidence should be >0.9 after 500 updates, got {}",
            corrector.pitch_confidence
        );
    }

    #[test]
    fn test_orientation_corrector_learns_roll_from_cornering() {
        // During left turn, centripetal acceleration is positive (leftward)
        // If AHRS has roll error, lat_imu differs from lat_gps
        let mut corrector = OrientationCorrector::new();

        let lat_gps = 5.0; // Centripetal = speed * yaw_rate = 10 * 0.5 = 5 m/s²
        let lat_imu = 4.5; // Roll error causes 0.5 m/s² difference
        let speed = 15.0;

        for _ in 0..500 {
            corrector.update(
                0.0,     // lon_imu (not accelerating)
                0.0,     // lon_gps
                lat_imu, // IMU lateral
                lat_gps, // GPS lateral (centripetal)
                speed, true, 0.005, // dt = 5ms (200Hz)
            );
        }

        // Roll correction should converge
        // Error = (4.5 - 5.0) / 9.8 = -0.051 rad
        let roll_corr = corrector.roll_correction;
        assert!(
            (roll_corr - (-0.051)).abs() < 0.01,
            "Roll correction should be ~-0.051 rad, got {}",
            roll_corr
        );
    }

    #[test]
    fn test_orientation_corrector_ignores_low_speed() {
        let mut corrector = OrientationCorrector::new();

        // At low speed, GPS is noisy - should not learn
        let low_speed = 1.0; // Below min_speed (3.0 m/s)

        for _ in 0..100 {
            corrector.update(
                5.0, // Large IMU value
                0.0, // GPS shows zero
                0.0, 0.0, low_speed, true, 0.005, // dt = 5ms
            );
        }

        // Should NOT have learned anything
        assert_eq!(
            corrector.pitch_correction, 0.0,
            "Should not learn at low speed"
        );
        assert_eq!(
            corrector.pitch_confidence, 0.0,
            "Confidence should be 0 at low speed"
        );
    }

    #[test]
    fn test_orientation_corrector_ignores_small_accel() {
        let mut corrector = OrientationCorrector::new();

        // When not accelerating (GPS shows ~0), no signal to learn from for pitch
        // BUT now this should learn cruise_bias instead!
        for _ in 0..100 {
            corrector.update(
                0.3, // Small IMU value (offset)
                0.0, // GPS shows zero (cruising)
                0.0, 0.0, 15.0, // Good speed
                true, 0.05, // dt = 50ms (shorter loop, 2s = 40 iterations, we do 100)
            );
        }

        // Pitch correction should NOT have learned - GPS accel below min_accel threshold
        assert_eq!(
            corrector.pitch_correction, 0.0,
            "Should not learn pitch when GPS accel < min_accel"
        );

        // But cruise_bias SHOULD have been learned! (new behavior)
        // IMU shows 0.3 m/s² offset during cruise, cruise bias should converge toward it
        assert!(
            corrector.cruise_bias.abs() > 0.01,
            "Should learn cruise_bias when cruising, got {}",
            corrector.cruise_bias
        );
    }

    #[test]
    fn test_orientation_corrector_clamped_at_max() {
        let mut corrector = OrientationCorrector::new();

        // Huge error that would exceed max_correction
        let lon_imu = 10.0;
        let lon_gps = 3.0; // 7 m/s² difference = huge pitch error

        for _ in 0..1000 {
            corrector.update(lon_imu, lon_gps, 0.0, 0.0, 15.0, true, 0.005);
        }

        // Should be clamped at max_correction (0.26 rad = 15°)
        assert!(
            corrector.pitch_correction.abs() <= corrector.max_correction + 0.001,
            "Correction should be clamped at max: {} > {}",
            corrector.pitch_correction.abs(),
            corrector.max_correction
        );
    }

    #[test]
    fn test_orientation_corrector_correct_applies_properly() {
        let mut corrector = OrientationCorrector::new();

        // Manually set corrections for testing
        corrector.pitch_correction = 0.05; // ~2.9°
        corrector.roll_correction = -0.03; // ~-1.7°

        // AHRS reports pitch=5°, roll=2°
        let (corrected_pitch, corrected_roll) = corrector.correct(5.0, 2.0);

        // Corrected = AHRS - correction_deg
        // pitch: 5.0 - 2.86 = 2.14
        // roll: 2.0 - (-1.72) = 3.72
        assert!(
            (corrected_pitch - 2.14).abs() < 0.1,
            "Corrected pitch should be ~2.14°, got {}",
            corrected_pitch
        );
        assert!(
            (corrected_roll - 3.72).abs() < 0.1,
            "Corrected roll should be ~3.72°, got {}",
            corrected_roll
        );
    }

    #[test]
    fn test_orientation_corrector_reset() {
        let mut corrector = OrientationCorrector::new();

        // Learn some corrections
        for _ in 0..200 {
            corrector.update(5.0, 3.0, 3.0, 2.0, 15.0, true, 0.005);
        }

        assert!(corrector.pitch_correction != 0.0);

        // Reset
        corrector.reset();

        assert_eq!(corrector.pitch_correction, 0.0);
        assert_eq!(corrector.roll_correction, 0.0);
        assert_eq!(corrector.pitch_confidence, 0.0);
        assert_eq!(corrector.update_count, 0);
    }

    #[test]
    fn test_orientation_corrector_no_gps_no_learning() {
        let mut corrector = OrientationCorrector::new();

        // GPS not fresh - should not learn
        for _ in 0..100 {
            corrector.update(
                5.0, 3.0, 0.0, 0.0, 15.0, false, // GPS NOT fresh
                0.005,
            );
        }

        assert_eq!(
            corrector.pitch_correction, 0.0,
            "Should not learn when GPS not fresh"
        );
    }

    #[test]
    fn test_orientation_corrector_cruise_bias_learning() {
        // Test that cruise bias is learned during constant-speed driving
        // when GPS shows ~0 acceleration but IMU has a consistent offset
        let mut corrector = OrientationCorrector::new();

        // Simulate 5 seconds of cruising at constant speed
        // IMU shows 0.5 m/s² offset (mounting error), GPS shows ~0 (cruising)
        let imu_offset = 0.5; // m/s² - consistent offset during cruise
        let dt = 0.05; // 50ms = 20Hz, so 2s = 40 samples

        // Run for 5 seconds (100 iterations at 50ms each)
        for _ in 0..100 {
            corrector.update(
                imu_offset, // IMU shows offset
                0.0,        // GPS shows zero (cruising)
                0.0, 0.0, 15.0, // 54 km/h - above min_speed
                true, dt,
            );
        }

        // Cruise bias should have been learned
        // With slow EMA (alpha=0.1) and 2s update cycle, after 5s we get ~2 updates
        // First update: bias = 0 + 0.1 * 0.5 = 0.05
        // Second update: corrected = 0.5 - 0.05 = 0.45, bias = 0.05 * 0.9 + 0.1 * 0.45 = 0.09
        let cruise_bias = corrector.get_cruise_bias();
        assert!(
            cruise_bias.abs() > 0.05,
            "Cruise bias should be learning (>0.05), got {}",
            cruise_bias
        );
        assert!(
            cruise_bias > 0.0,
            "Cruise bias should be positive (learning toward IMU offset): got {}",
            cruise_bias
        );

        // Pitch correction should NOT be learned (no GPS accel signal)
        assert!(
            corrector.pitch_correction.abs() < 0.01,
            "Pitch should not change during cruise"
        );
    }

    #[test]
    fn test_orientation_corrector_cruise_bias_resets_on_accel() {
        // Test that cruise bias accumulator resets when acceleration is detected
        let mut corrector = OrientationCorrector::new();

        // First, build up some cruise bias
        for _ in 0..50 {
            corrector.update(0.3, 0.0, 0.0, 0.0, 15.0, true, 0.05);
        }

        // Now accelerate - this should reset the cruise accumulator
        // but preserve the learned cruise_bias from before
        let bias_before = corrector.get_cruise_bias();

        for _ in 0..10 {
            corrector.update(
                3.5, // IMU shows acceleration
                3.0, // GPS also shows acceleration
                0.0, 0.0, 15.0, true, 0.05,
            );
        }

        // Cruise bias should be preserved (not cleared by acceleration)
        let bias_after = corrector.get_cruise_bias();
        assert!(
            (bias_after - bias_before).abs() < 0.1,
            "Cruise bias should be preserved during acceleration"
        );

        // But pitch should now be learning
        assert!(
            corrector.pitch_correction.abs() > 0.001,
            "Pitch should be learning during acceleration"
        );
    }
}
