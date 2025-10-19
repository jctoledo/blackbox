/// Configuration management for Active Wing telemetry system
/// Provides defaults and structure for future config.toml loading
use motorsport_telemetry::ekf::EkfConfig;

use crate::mode::ModeConfig;

/// Network configuration
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    pub wifi_ssid: &'static str,
    pub wifi_password: &'static str,
    pub mqtt_broker: &'static str,
    pub tcp_server: &'static str,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            wifi_ssid: "GiraffeWireless",
            wifi_password: "basicchair411",
            mqtt_broker: "mqtt://192.168.50.46:1883",
            tcp_server: "192.168.50.46:9000",
        }
    }
}

/// Telemetry configuration
#[derive(Debug, Clone, Copy)]
pub struct TelemetryConfig {
    pub interval_ms: u32,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            interval_ms: 50, // 20 Hz
        }
    }
}

/// IMU configuration
#[derive(Debug, Clone, Copy)]
pub struct ImuConfig {
    pub calib_samples: usize,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self { calib_samples: 500 }
    }
}

/// GPS configuration
#[derive(Debug, Clone, Copy)]
pub struct GpsConfig {
    pub warmup_fixes: u8,
    pub update_rate_hz: u8,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            warmup_fixes: 5,
            update_rate_hz: 5,
        }
    }
}

/// Stationary detection configuration
#[derive(Debug, Clone, Copy)]
pub struct StationaryConfig {
    pub acc_threshold: f32,            // g
    pub wz_threshold: f32,             // deg/s
    pub gps_speed_threshold: f32,      // km/h
    pub position_speed_threshold: f32, // km/h
    pub required_count: u32,
}

impl Default for StationaryConfig {
    fn default() -> Self {
        Self {
            acc_threshold: 0.18,
            wz_threshold: 12.0,
            gps_speed_threshold: 3.5,
            position_speed_threshold: 5.0,
            required_count: 5,
        }
    }
}

/// Master system configuration
#[derive(Debug, Clone)]
pub struct SystemConfig {
    pub network: NetworkConfig,
    pub telemetry: TelemetryConfig,
    pub ekf: EkfConfig,
    pub mode: ModeConfig,
    pub imu: ImuConfig,
    pub gps: GpsConfig,
    pub stationary: StationaryConfig,
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            network: NetworkConfig::default(),
            telemetry: TelemetryConfig::default(),
            ekf: EkfConfig::default(),
            mode: ModeConfig::default(),
            imu: ImuConfig::default(),
            gps: GpsConfig::default(),
            stationary: StationaryConfig::default(),
        }
    }
}

impl SystemConfig {
    /// Create configuration from environment variables (compile-time)
    /// Usage: Set environment variables before building:
    /// ```bash
    /// export WIFI_SSID="MyNetwork"
    /// export WIFI_PASSWORD="MyPassword"
    /// cargo build
    /// ```
    pub fn from_env() -> Self {
        let mut config = Self::default();

        // Override with environment variables if available
        if let Some(ssid) = option_env!("WIFI_SSID") {
            config.network.wifi_ssid = ssid;
        }
        if let Some(password) = option_env!("WIFI_PASSWORD") {
            config.network.wifi_password = password;
        }
        if let Some(broker) = option_env!("MQTT_BROKER") {
            config.network.mqtt_broker = broker;
        }
        if let Some(server) = option_env!("TCP_SERVER") {
            config.network.tcp_server = server;
        }

        config
    }

    /// Load configuration from file (future implementation)
    /// Will require adding TOML parser dependency or custom parser
    #[allow(dead_code)]
    pub fn from_file(_path: &str) -> Result<Self, ConfigError> {
        // TODO: Implement TOML parsing when needed
        // For now, return default config
        Ok(Self::default())
    }
}

/// Configuration errors
#[derive(Debug)]
pub enum ConfigError {
    FileNotFound,
    ParseError(&'static str),
    InvalidValue(&'static str),
}
