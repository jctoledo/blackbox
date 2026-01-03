#![allow(dead_code)] // Configuration structs for future use

/// Configuration management for Blackbox telemetry system
/// Provides defaults and structure for future config.toml loading
use sensor_fusion::ekf::EkfConfig;

use crate::mode::ModeConfig;

/// WiFi operating mode
/// Default: AccessPoint (standalone operation)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum WifiModeConfig {
    /// Station mode - connect to existing network (home/track WiFi)
    Station,
    /// Access Point mode - ESP32 creates own network (mobile/standalone use)
    #[default]
    AccessPoint,
}

/// Supported u-blox NEO GPS modules
/// Default: Neo7M (set via GPS_MODEL env var)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpsModel {
    /// NEO-6M: GPS only, max 5 Hz
    Neo6M,
    /// NEO-7M: GPS + GLONASS, max 10 Hz (default)
    Neo7M,
    /// NEO-8M: GPS + GLONASS + Galileo + BeiDou, max 10 Hz (18 Hz in single-GNSS mode)
    Neo8M,
}

impl Default for GpsModel {
    fn default() -> Self {
        // Check environment variable at compile time
        match option_env!("GPS_MODEL") {
            Some(m) => {
                // Note: to_lowercase() not available at compile time, so we match common variants
                match m {
                    "neo6m" | "Neo6M" | "NEO6M" | "6m" | "6M" | "6" => GpsModel::Neo6M,
                    "neo7m" | "Neo7M" | "NEO7M" | "7m" | "7M" | "7" => GpsModel::Neo7M,
                    "neo8m" | "Neo8M" | "NEO8M" | "8m" | "8M" | "8" => GpsModel::Neo8M,
                    _ => GpsModel::Neo7M, // Default
                }
            }
            None => GpsModel::Neo7M, // Default
        }
    }
}

impl GpsModel {
    /// Maximum update rate for this GPS model
    pub fn max_rate_hz(&self) -> u8 {
        match self {
            GpsModel::Neo6M => 5,
            GpsModel::Neo7M => 10,
            GpsModel::Neo8M => 10, // 18 Hz possible but 10 Hz better for multi-GNSS accuracy
        }
    }

    /// Measurement period in milliseconds for UBX CFG-RATE command
    pub fn measurement_period_ms(&self) -> u16 {
        1000 / self.max_rate_hz() as u16
    }

    /// Human-readable model name
    pub fn name(&self) -> &'static str {
        match self {
            GpsModel::Neo6M => "NEO-6M",
            GpsModel::Neo7M => "NEO-7M",
            GpsModel::Neo8M => "NEO-8M",
        }
    }
}

/// Network configuration
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    /// WiFi mode: Station (connect to network) or AccessPoint (create network)
    pub wifi_mode: WifiModeConfig,
    /// For STA mode: network to connect to. For AP mode: network name to create
    pub wifi_ssid: &'static str,
    /// WiFi password (empty for open network in AP mode)
    pub wifi_password: &'static str,
    /// MQTT broker URL (only used in STA mode)
    pub mqtt_broker: &'static str,
    /// UDP server address (only used in STA mode)
    pub udp_server: &'static str,
    /// WebSocket server port (used in AP mode, default 8080)
    pub ws_port: u16,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            wifi_mode: WifiModeConfig::AccessPoint,
            wifi_ssid: "Blackbox",
            wifi_password: "blackbox123",
            // Station mode placeholders - MUST set via environment variables
            mqtt_broker: "mqtt://192.168.1.100:1883",
            udp_server: "192.168.1.100:9000",
            ws_port: 80,
        }
    }
}

impl NetworkConfig {
    /// Configuration for Station mode (home/track use with router)
    #[allow(dead_code)]
    pub fn station(ssid: &'static str, password: &'static str) -> Self {
        Self {
            wifi_mode: WifiModeConfig::Station,
            wifi_ssid: ssid,
            wifi_password: password,
            ..Default::default()
        }
    }

    /// Configuration for Access Point mode (standalone/mobile use)
    #[allow(dead_code)]
    pub fn access_point(ssid: &'static str, password: &'static str) -> Self {
        Self {
            wifi_mode: WifiModeConfig::AccessPoint,
            wifi_ssid: ssid,
            wifi_password: password,
            ..Default::default()
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
            interval_ms: 33, // 30 Hz
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
    /// GPS module type (determines max update rate)
    pub model: GpsModel,
    /// Number of valid fixes to average for reference point (1 second at 10 Hz)
    pub warmup_fixes: u8,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            model: GpsModel::default(), // From GPS_MODEL env var
            warmup_fixes: 10,
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
#[derive(Debug, Clone, Default)]
pub struct SystemConfig {
    pub network: NetworkConfig,
    pub telemetry: TelemetryConfig,
    pub ekf: EkfConfig,
    pub mode: ModeConfig,
    pub imu: ImuConfig,
    pub gps: GpsConfig,
    pub stationary: StationaryConfig,
}

impl SystemConfig {
    /// Create configuration from environment variables (compile-time)
    ///
    /// **Access Point Mode (default):** No environment variables required
    /// - Uses default SSID "Blackbox" and password "blackbox123"
    /// - Creates standalone WiFi network at 192.168.71.1
    ///
    /// **Station Mode:** REQUIRES environment variables
    /// ```bash
    /// export WIFI_MODE="station"
    /// export WIFI_SSID="YourNetworkName"
    /// export WIFI_PASSWORD="YourPassword"
    /// export MQTT_BROKER="mqtt://192.168.1.100:1883"  # Your MQTT broker
    /// export UDP_SERVER="192.168.1.100:9000"          # Your laptop IP
    /// cargo build --release
    /// ```
    ///
    /// **WARNING:** Station mode defaults (192.168.1.100) are placeholders
    /// only. You MUST set MQTT_BROKER and UDP_SERVER for Station mode to
    /// work.
    pub fn from_env() -> Self {
        let mut config = Self::default();

        // Check for WiFi mode override
        if let Some(mode) = option_env!("WIFI_MODE") {
            config.network.wifi_mode = match mode.to_lowercase().as_str() {
                "station" | "sta" | "client" => WifiModeConfig::Station,
                "ap" | "accesspoint" | "access_point" => WifiModeConfig::AccessPoint,
                _ => WifiModeConfig::AccessPoint,
            };
        }

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
        if let Some(server) = option_env!("UDP_SERVER") {
            config.network.udp_server = server;
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
