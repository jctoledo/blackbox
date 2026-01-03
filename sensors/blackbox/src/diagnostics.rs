//! Diagnostics system for monitoring sensor health and system status
//!
//! This module provides real-time diagnostics without impacting telemetry
//! performance. Updates use try_lock() to avoid blocking the main telemetry
//! path.

use std::sync::{Arc, Mutex};

/// Sensor update rates (measured vs expected)
#[derive(Debug, Clone, Default)]
pub struct SensorRates {
    /// Measured IMU packet rate (Hz)
    pub imu_hz: f32,
    /// Measured GPS fix rate (Hz)
    pub gps_hz: f32,
    /// Expected IMU rate (200 Hz for WT901)
    pub imu_expected_hz: f32,
    /// Expected GPS rate (from config, 5-10 Hz)
    pub gps_expected_hz: f32,
}

/// EKF health indicators
#[derive(Debug, Clone, Default)]
pub struct EkfHealth {
    /// Position uncertainty (1-sigma, meters)
    pub position_sigma: f32,
    /// Velocity uncertainty (1-sigma, m/s)
    pub velocity_sigma: f32,
    /// Yaw uncertainty (degrees)
    pub yaw_sigma_deg: f32,
    /// Estimated X-axis accelerometer bias (m/s²)
    pub bias_x: f32,
    /// Estimated Y-axis accelerometer bias (m/s²)
    pub bias_y: f32,
}

/// System resource health
#[derive(Debug, Clone, Default)]
pub struct SystemHealth {
    /// Free heap memory (bytes)
    pub free_heap_bytes: u32,
    /// System uptime (seconds)
    pub uptime_seconds: u32,
    /// Total telemetry packets sent
    pub telemetry_sent: u32,
    /// Failed telemetry sends
    pub telemetry_failed: u32,
}

/// GPS module health
#[derive(Debug, Clone)]
pub struct GpsHealth {
    /// GPS module name (e.g., "NEO-7M")
    pub model_name: &'static str,
    /// Configured update rate (Hz)
    pub configured_rate_hz: u8,
    /// Whether GPS has valid fix
    pub fix_valid: bool,
    /// Whether warmup is complete (reference point set)
    pub warmup_complete: bool,
}

impl Default for GpsHealth {
    fn default() -> Self {
        Self {
            model_name: "Unknown",
            configured_rate_hz: 0,
            fix_valid: false,
            warmup_complete: false,
        }
    }
}

/// WiFi connection status
#[derive(Debug, Clone)]
pub struct WifiStatus {
    /// Operating mode ("Access Point" or "Station")
    pub mode: &'static str,
    /// Network SSID
    pub ssid: &'static str,
    /// IP address
    pub ip_address: &'static str,
}

impl Default for WifiStatus {
    fn default() -> Self {
        Self {
            mode: "Unknown",
            ssid: "",
            ip_address: "",
        }
    }
}

/// Configuration snapshot (read-only, set at startup)
#[derive(Debug, Clone)]
pub struct ConfigSnapshot {
    /// Telemetry output rate (Hz)
    pub telemetry_rate_hz: u32,
    /// GPS model name
    pub gps_model: &'static str,
    /// GPS warmup fixes count
    pub gps_warmup_fixes: u8,
}

impl Default for ConfigSnapshot {
    fn default() -> Self {
        Self {
            telemetry_rate_hz: 30,
            gps_model: "Unknown",
            gps_warmup_fixes: 10,
        }
    }
}

/// Complete diagnostics data snapshot
#[derive(Debug, Clone, Default)]
pub struct DiagnosticsData {
    /// Sensor update rates
    pub sensor_rates: SensorRates,
    /// EKF health indicators
    pub ekf_health: EkfHealth,
    /// System resource health
    pub system_health: SystemHealth,
    /// GPS module health
    pub gps_health: GpsHealth,
    /// WiFi connection status
    pub wifi_status: WifiStatus,
    /// Configuration snapshot
    pub config: ConfigSnapshot,
}

/// Thread-safe diagnostics state
///
/// Uses non-blocking updates to avoid impacting telemetry performance.
/// The main loop calls `update()` with try_lock(), which skips updates
/// if the mutex is already held (e.g., by the diagnostics endpoint).
pub struct DiagnosticsState {
    data: Mutex<DiagnosticsData>,
}

impl DiagnosticsState {
    /// Create new diagnostics state wrapped in Arc for sharing
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            data: Mutex::new(DiagnosticsData::default()),
        })
    }

    /// Non-blocking update of diagnostics data
    ///
    /// Uses try_lock() to avoid blocking the main telemetry loop.
    /// If the mutex is held (e.g., by diagnostics endpoint reading),
    /// the update is skipped. This ensures telemetry is never delayed.
    pub fn update<F>(&self, f: F)
    where
        F: FnOnce(&mut DiagnosticsData),
    {
        if let Ok(mut data) = self.data.try_lock() {
            f(&mut data);
        }
        // Silently skip if locked - telemetry path must not block
    }

    /// Get a snapshot of current diagnostics data
    ///
    /// This may briefly block if an update is in progress, but updates
    /// are fast (just copying values) so blocking is minimal.
    pub fn snapshot(&self) -> DiagnosticsData {
        self.data.lock().unwrap().clone()
    }
}
