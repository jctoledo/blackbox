//! Diagnostics module for system health monitoring
//!
//! Provides real-time statistics about sensor rates, EKF health, GPS status,
//! and system resources for the diagnostics dashboard.

use std::sync::{
    atomic::{AtomicU32, Ordering},
    Mutex,
};

/// Sensor rate statistics
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorRates {
    /// Actual IMU packet rate (Hz)
    pub imu_hz: f32,
    /// Expected IMU rate (Hz)
    pub imu_expected_hz: f32,
    /// Actual GPS fix rate (Hz) - only counts valid position fixes
    pub gps_hz: f32,
    /// Expected GPS rate (Hz)
    pub gps_expected_hz: f32,
    /// Main loop rate (Hz)
    pub loop_hz: f32,
    /// ZUPT rate (per minute) - rolling average
    pub zupt_per_min: f32,
    /// Average EKF predictions between GPS fixes
    pub ekf_predictions_per_gps: f32,
}

/// EKF health metrics (from covariance matrix diagonals)
#[derive(Debug, Clone, Copy, Default)]
pub struct EkfHealth {
    /// Position uncertainty (sqrt of P[0,0] + P[1,1]) in meters
    pub position_sigma: f32,
    /// Velocity uncertainty (sqrt of P[3,3] + P[4,4]) in m/s
    pub velocity_sigma: f32,
    /// Yaw uncertainty (sqrt of P[2,2]) in degrees
    pub yaw_sigma_deg: f32,
    /// Estimated X-axis accelerometer bias (m/s^2)
    pub bias_x: f32,
    /// Estimated Y-axis accelerometer bias (m/s^2)
    pub bias_y: f32,
}

/// GPS health status
#[derive(Debug, Clone, Default)]
pub struct GpsHealth {
    /// GPS model name (e.g., "NEO-6M", "NEO-M9N")
    pub model_name: &'static str,
    /// Configured update rate (Hz)
    pub configured_rate_hz: u8,
    /// Fix valid flag
    pub fix_valid: bool,
    /// Warmup complete flag
    pub warmup_complete: bool,
    /// Number of satellites in view
    pub satellites: u8,
    /// HDOP (horizontal dilution of precision)
    pub hdop: f32,
    /// PDOP (position dilution of precision)
    pub pdop: f32,
    /// GPS reference latitude (origin for local coordinates)
    pub ref_lat: f64,
    /// GPS reference longitude (origin for local coordinates)
    pub ref_lon: f64,
}

/// System resource usage
#[derive(Debug, Clone, Copy, Default)]
pub struct SystemHealth {
    /// Free heap memory (bytes)
    pub free_heap_bytes: u32,
    /// System uptime (seconds)
    pub uptime_seconds: u32,
    /// Total telemetry packets sent successfully
    pub telemetry_sent: u32,
    /// Total telemetry send failures
    pub telemetry_failed: u32,
}

/// WiFi connection status
#[derive(Debug, Clone, Default)]
pub struct WifiStatus {
    /// WiFi mode ("AP" or "Station")
    pub mode: &'static str,
    /// Connected SSID or AP name
    pub ssid: &'static str,
}

/// Configuration snapshot for diagnostics
#[derive(Debug, Clone, Default)]
pub struct ConfigSnapshot {
    /// Telemetry output rate (Hz)
    pub telemetry_rate_hz: u32,
    /// GPS model name
    pub gps_model: &'static str,
    /// GPS warmup fixes count
    pub gps_warmup_fixes: u8,
}

/// Sensor fusion diagnostics - filter pipeline and blending status
#[derive(Debug, Clone, Copy, Default)]
pub struct FusionDiagnostics {
    // Filter pipeline (all in m/s²)
    /// Raw IMU longitudinal acceleration (before Butterworth filter)
    pub lon_imu_raw: f32,
    /// Filtered IMU longitudinal (after 10Hz low-pass)
    pub lon_imu_filtered: f32,
    /// Final blended longitudinal (GPS/IMU mix)
    pub lon_blended: f32,

    // GPS blending status
    /// Current GPS blend weight (0.0-1.0, based on orientation correction
    /// confidence)
    pub gps_weight: f32,
    /// GPS-derived acceleration (m/s²), NaN if invalid
    pub gps_accel: f32,
    /// GPS update rate estimate (Hz)
    pub gps_rate: f32,
    /// Was GPS rejected by validity check? (GPS=0 but IMU has signal)
    pub gps_rejected: bool,

    // Orientation corrector (ArduPilot-style GPS-corrected orientation)
    /// Learned pitch correction (degrees) - corrects AHRS errors during
    /// acceleration
    pub pitch_correction_deg: f32,
    /// Learned roll correction (degrees) - corrects AHRS errors during
    /// cornering
    pub roll_correction_deg: f32,
    /// Pitch correction confidence (0.0-1.0, based on learning samples)
    pub pitch_confidence: f32,
    /// Roll correction confidence (0.0-1.0)
    pub roll_confidence: f32,

    // Yaw rate calibrator
    /// Learned gyro bias (rad/s)
    pub yaw_bias: f32,
    /// Is yaw calibration valid?
    pub yaw_calibrated: bool,

    // Tilt estimator (learned when stopped)
    /// Tilt offset X (m/s²)
    pub tilt_offset_x: f32,
    /// Tilt offset Y (m/s²)
    pub tilt_offset_y: f32,
    /// Is tilt offset valid?
    pub tilt_valid: bool,
}

/// Complete diagnostics snapshot (immutable copy for reading)
#[derive(Debug, Clone, Default)]
pub struct DiagnosticsSnapshot {
    pub sensor_rates: SensorRates,
    pub ekf_health: EkfHealth,
    pub gps_health: GpsHealth,
    pub system_health: SystemHealth,
    pub wifi_status: WifiStatus,
    pub config: ConfigSnapshot,
    pub fusion: FusionDiagnostics,
}

/// Thread-safe diagnostics state container
/// Updated by the main loop, read by the HTTP server
pub struct DiagnosticsState {
    // Atomic counters for high-frequency updates
    imu_packet_count: AtomicU32,
    gps_fix_count: AtomicU32,
    loop_count: AtomicU32,
    zupt_count: AtomicU32,
    last_rate_check_ms: AtomicU32,

    // Protected state for complex data
    inner: Mutex<DiagnosticsInner>,
}

/// Inner protected state
#[derive(Default)]
struct DiagnosticsInner {
    sensor_rates: SensorRates,
    ekf_health: EkfHealth,
    gps_health: GpsHealth,
    system_health: SystemHealth,
    wifi_status: WifiStatus,
    config: ConfigSnapshot,
    fusion: FusionDiagnostics,
    // For rate calculation (last known counts)
    last_imu_count: u32,
    last_gps_count: u32,
    last_loop_count: u32,
    last_zupt_count: u32,
}

impl DiagnosticsState {
    /// Create a new diagnostics state
    pub fn new() -> Self {
        Self {
            imu_packet_count: AtomicU32::new(0),
            gps_fix_count: AtomicU32::new(0),
            loop_count: AtomicU32::new(0),
            zupt_count: AtomicU32::new(0),
            last_rate_check_ms: AtomicU32::new(0),
            inner: Mutex::new(DiagnosticsInner::default()),
        }
    }

    /// Initialize with static configuration
    #[allow(clippy::too_many_arguments)]
    pub fn init(
        &self,
        wifi_mode: &'static str,
        wifi_ssid: &'static str,
        gps_model: &'static str,
        gps_rate_hz: u8,
        gps_warmup_fixes: u8,
        telemetry_rate_hz: u32,
        imu_expected_hz: f32,
        gps_expected_hz: f32,
    ) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.wifi_status.mode = wifi_mode;
            inner.wifi_status.ssid = wifi_ssid;
            inner.config.gps_model = gps_model;
            inner.config.gps_warmup_fixes = gps_warmup_fixes;
            inner.config.telemetry_rate_hz = telemetry_rate_hz;
            inner.gps_health.model_name = gps_model;
            inner.gps_health.configured_rate_hz = gps_rate_hz;
            inner.sensor_rates.imu_expected_hz = imu_expected_hz;
            inner.sensor_rates.gps_expected_hz = gps_expected_hz;
        }
    }

    /// Record an IMU packet received (called from main loop)
    #[inline]
    pub fn record_imu_packet(&self) {
        self.imu_packet_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a valid GPS position fix (called only for RMC with valid
    /// position)
    #[inline]
    pub fn record_gps_fix(&self) {
        self.gps_fix_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a main loop iteration
    #[inline]
    pub fn record_loop(&self) {
        self.loop_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a ZUPT (zero velocity update)
    #[inline]
    pub fn record_zupt(&self) {
        self.zupt_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Get total GPS fix count (for sensor fusion rate tracking)
    #[inline]
    pub fn gps_fix_count(&self) -> u32 {
        self.gps_fix_count.load(Ordering::Relaxed)
    }

    /// Update sensor rates (call periodically, e.g., every second)
    pub fn update_rates(&self, now_ms: u32) {
        let last_ms = self.last_rate_check_ms.swap(now_ms, Ordering::SeqCst);
        let dt_ms = now_ms.saturating_sub(last_ms);

        if dt_ms < 100 {
            return; // Too soon, skip
        }

        let imu_count = self.imu_packet_count.load(Ordering::Relaxed);
        let gps_count = self.gps_fix_count.load(Ordering::Relaxed);
        let loop_count_now = self.loop_count.load(Ordering::Relaxed);
        let zupt_total = self.zupt_count.load(Ordering::Relaxed);

        if let Ok(mut inner) = self.inner.lock() {
            let dt_s = dt_ms as f32 / 1000.0;
            let imu_delta = imu_count.saturating_sub(inner.last_imu_count);
            let gps_delta = gps_count.saturating_sub(inner.last_gps_count);
            let loop_delta = loop_count_now.saturating_sub(inner.last_loop_count);

            // Exponential moving average for smoother display
            let alpha = 0.3;
            inner.sensor_rates.imu_hz =
                alpha * (imu_delta as f32 / dt_s) + (1.0 - alpha) * inner.sensor_rates.imu_hz;
            inner.sensor_rates.gps_hz =
                alpha * (gps_delta as f32 / dt_s) + (1.0 - alpha) * inner.sensor_rates.gps_hz;
            inner.sensor_rates.loop_hz =
                alpha * (loop_delta as f32 / dt_s) + (1.0 - alpha) * inner.sensor_rates.loop_hz;

            // Calculate EKF predictions per GPS fix (IMU packets = EKF predictions)
            if gps_delta > 0 {
                let predictions_per_fix = imu_delta as f32 / gps_delta as f32;
                inner.sensor_rates.ekf_predictions_per_gps = alpha * predictions_per_fix
                    + (1.0 - alpha) * inner.sensor_rates.ekf_predictions_per_gps;
            }

            // Calculate ZUPT rate (per minute)
            let zupt_delta = zupt_total.saturating_sub(inner.last_zupt_count);
            let zupt_per_min = (zupt_delta as f32 / dt_s) * 60.0;
            inner.sensor_rates.zupt_per_min =
                alpha * zupt_per_min + (1.0 - alpha) * inner.sensor_rates.zupt_per_min;

            inner.last_imu_count = imu_count;
            inner.last_gps_count = gps_count;
            inner.last_loop_count = loop_count_now;
            inner.last_zupt_count = zupt_total;
        }
    }

    /// Update EKF health metrics
    pub fn update_ekf(
        &self,
        pos_sigma: f32,
        vel_sigma: f32,
        yaw_sigma_deg: f32,
        bias_x: f32,
        bias_y: f32,
    ) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.ekf_health.position_sigma = pos_sigma;
            inner.ekf_health.velocity_sigma = vel_sigma;
            inner.ekf_health.yaw_sigma_deg = yaw_sigma_deg;
            inner.ekf_health.bias_x = bias_x;
            inner.ekf_health.bias_y = bias_y;
        }
    }

    /// Update GPS status
    #[allow(clippy::too_many_arguments)]
    pub fn update_gps(
        &self,
        fix_valid: bool,
        warmup_complete: bool,
        satellites: u8,
        hdop: f32,
        pdop: f32,
        ref_lat: f64,
        ref_lon: f64,
    ) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.gps_health.fix_valid = fix_valid;
            inner.gps_health.warmup_complete = warmup_complete;
            inner.gps_health.satellites = satellites;
            inner.gps_health.hdop = hdop;
            inner.gps_health.pdop = pdop;
            inner.gps_health.ref_lat = ref_lat;
            inner.gps_health.ref_lon = ref_lon;
        }
    }

    /// Update system health
    pub fn update_system(&self, free_heap: u32, uptime_s: u32, tx_ok: u32, tx_fail: u32) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.system_health.free_heap_bytes = free_heap;
            inner.system_health.uptime_seconds = uptime_s;
            inner.system_health.telemetry_sent = tx_ok;
            inner.system_health.telemetry_failed = tx_fail;
        }
    }

    /// Update fusion diagnostics (filter pipeline, GPS blending, calibrators)
    #[allow(clippy::too_many_arguments)]
    pub fn update_fusion(&self, fusion: FusionDiagnostics) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.fusion = fusion;
        }
    }

    /// Get a snapshot of all diagnostics (for HTTP response)
    pub fn snapshot(&self) -> DiagnosticsSnapshot {
        if let Ok(inner) = self.inner.lock() {
            DiagnosticsSnapshot {
                sensor_rates: inner.sensor_rates,
                ekf_health: inner.ekf_health,
                gps_health: inner.gps_health.clone(),
                system_health: inner.system_health,
                wifi_status: inner.wifi_status.clone(),
                config: inner.config.clone(),
                fusion: inner.fusion,
            }
        } else {
            DiagnosticsSnapshot::default()
        }
    }
}

impl Default for DiagnosticsState {
    fn default() -> Self {
        Self::new()
    }
}
