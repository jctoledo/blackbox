//! NVS-backed settings persistence for mode detection thresholds
//! Supports multiple driving profiles (track, canyon, city, highway, custom)

use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};
use log::{info, warn};

use crate::mode::ModeConfig;

const NAMESPACE: &str = "bb_cfg";

// Key for active mode (max 15 chars for NVS keys)
const KEY_MODE: &str = "mode";

/// Driving mode profiles
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrivingMode {
    Track,
    Canyon,
    City,
    Highway,
    Custom,
}

impl DrivingMode {
    /// Get the NVS key prefix for this mode (single char to save space)
    fn prefix(&self) -> &'static str {
        match self {
            DrivingMode::Track => "t",
            DrivingMode::Canyon => "n", // 'n' for caNyon (c is taken by city)
            DrivingMode::City => "c",
            DrivingMode::Highway => "h",
            DrivingMode::Custom => "x",
        }
    }

    /// Get mode name for logging/display
    pub fn name(&self) -> &'static str {
        match self {
            DrivingMode::Track => "track",
            DrivingMode::Canyon => "canyon",
            DrivingMode::City => "city",
            DrivingMode::Highway => "highway",
            DrivingMode::Custom => "custom",
        }
    }

    /// Parse mode from string
    pub fn from_str(s: &str) -> Option<DrivingMode> {
        match s.to_lowercase().as_str() {
            "track" => Some(DrivingMode::Track),
            "canyon" => Some(DrivingMode::Canyon),
            "city" => Some(DrivingMode::City),
            "highway" => Some(DrivingMode::Highway),
            "custom" => Some(DrivingMode::Custom),
            _ => None,
        }
    }

    /// Get default config for this mode
    pub fn default_config(&self) -> ModeConfig {
        match self {
            DrivingMode::Track => ModeConfig {
                acc_thr: 0.30,
                acc_exit: 0.15,
                brake_thr: -0.50,
                brake_exit: -0.25,
                lat_thr: 0.50,
                lat_exit: 0.25,
                yaw_thr: 0.15,
                min_speed: 3.0,
                alpha: 0.35,
            },
            DrivingMode::Canyon => ModeConfig {
                acc_thr: 0.20,
                acc_exit: 0.10,
                brake_thr: -0.35,
                brake_exit: -0.17,
                lat_thr: 0.30,
                lat_exit: 0.15,
                yaw_thr: 0.10,
                min_speed: 2.5,
                alpha: 0.35,
            },
            DrivingMode::City => ModeConfig {
                acc_thr: 0.10,
                acc_exit: 0.05,
                brake_thr: -0.18,
                brake_exit: -0.09,
                lat_thr: 0.12,
                lat_exit: 0.06,
                yaw_thr: 0.05,
                min_speed: 2.0,
                alpha: 0.35,
            },
            DrivingMode::Highway => ModeConfig {
                acc_thr: 0.08,
                acc_exit: 0.04,
                brake_thr: -0.15,
                brake_exit: -0.07,
                lat_thr: 0.10,
                lat_exit: 0.05,
                yaw_thr: 0.04,
                min_speed: 4.0,
                alpha: 0.35,
            },
            DrivingMode::Custom => ModeConfig {
                // Custom defaults to City
                acc_thr: 0.10,
                acc_exit: 0.05,
                brake_thr: -0.18,
                brake_exit: -0.09,
                lat_thr: 0.12,
                lat_exit: 0.06,
                yaw_thr: 0.05,
                min_speed: 2.0,
                alpha: 0.35,
            },
        }
    }
}

/// Load the active mode from NVS, defaults to City
pub fn load_active_mode(nvs_partition: EspNvsPartition<NvsDefault>) -> DrivingMode {
    let nvs = match EspNvs::new(nvs_partition, NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(_) => return DrivingMode::City,
    };

    let mut buf = [0u8; 16];
    match nvs.get_raw(KEY_MODE, &mut buf) {
        Ok(Some(data)) => {
            // Find null terminator or use full length
            let len = data.iter().position(|&b| b == 0).unwrap_or(data.len());
            let s = core::str::from_utf8(&data[..len]).unwrap_or("city");
            DrivingMode::from_str(s).unwrap_or(DrivingMode::City)
        }
        _ => DrivingMode::City,
    }
}

/// Save the active mode to NVS
pub fn save_active_mode(nvs_partition: EspNvsPartition<NvsDefault>, mode: DrivingMode) -> bool {
    let mut nvs = match EspNvs::new(nvs_partition, NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(e) => {
            warn!("Failed to open NVS for mode save: {:?}", e);
            return false;
        }
    };

    nvs.set_raw(KEY_MODE, mode.name().as_bytes()).is_ok()
}

/// Load settings for a specific mode from NVS
/// Returns the mode's default config if not found in NVS
pub fn load_mode_settings(
    nvs_partition: EspNvsPartition<NvsDefault>,
    mode: DrivingMode,
) -> ModeConfig {
    let nvs = match EspNvs::new(nvs_partition, NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(_) => return mode.default_config(),
    };

    let prefix = mode.prefix();

    // Try to load all values - if any missing, use mode defaults
    let acc = read_f32_prefixed(&nvs, prefix, "acc");
    let acc_exit = read_f32_prefixed(&nvs, prefix, "acc_x");
    let brake = read_f32_prefixed(&nvs, prefix, "brk");
    let brake_exit = read_f32_prefixed(&nvs, prefix, "brk_x");
    let lat = read_f32_prefixed(&nvs, prefix, "lat");
    let lat_exit = read_f32_prefixed(&nvs, prefix, "lat_x");
    let yaw = read_f32_prefixed(&nvs, prefix, "yaw");
    let min_speed = read_f32_prefixed(&nvs, prefix, "spd");

    // If all values present, use them; otherwise use defaults
    if let (Some(a), Some(ae), Some(b), Some(be), Some(l), Some(le), Some(y), Some(s)) = (
        acc, acc_exit, brake, brake_exit, lat, lat_exit, yaw, min_speed,
    ) {
        info!(
            "Loaded {} profile from NVS: acc={:.2}, brake={:.2}, lat={:.2}, yaw={:.3}",
            mode.name(),
            a,
            b,
            l,
            y
        );
        ModeConfig {
            acc_thr: a,
            acc_exit: ae,
            brake_thr: b,
            brake_exit: be,
            lat_thr: l,
            lat_exit: le,
            yaw_thr: y,
            min_speed: s,
            alpha: 0.35,
        }
    } else {
        info!("Using default {} profile (not found in NVS)", mode.name());
        mode.default_config()
    }
}

/// Save settings for a specific mode to NVS
pub fn save_mode_settings(
    nvs_partition: EspNvsPartition<NvsDefault>,
    mode: DrivingMode,
    config: &ModeConfig,
) -> bool {
    let mut nvs = match EspNvs::new(nvs_partition, NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(e) => {
            warn!("Failed to open NVS for settings save: {:?}", e);
            return false;
        }
    };

    let prefix = mode.prefix();

    let success = write_f32_prefixed(&mut nvs, prefix, "acc", config.acc_thr)
        && write_f32_prefixed(&mut nvs, prefix, "acc_x", config.acc_exit)
        && write_f32_prefixed(&mut nvs, prefix, "brk", config.brake_thr)
        && write_f32_prefixed(&mut nvs, prefix, "brk_x", config.brake_exit)
        && write_f32_prefixed(&mut nvs, prefix, "lat", config.lat_thr)
        && write_f32_prefixed(&mut nvs, prefix, "lat_x", config.lat_exit)
        && write_f32_prefixed(&mut nvs, prefix, "yaw", config.yaw_thr)
        && write_f32_prefixed(&mut nvs, prefix, "spd", config.min_speed);

    if success {
        info!(
            "Saved {} profile to NVS: acc={:.2}, brake={:.2}, lat={:.2}, yaw={:.3}",
            mode.name(),
            config.acc_thr,
            config.brake_thr,
            config.lat_thr,
            config.yaw_thr
        );
    } else {
        warn!("Failed to save {} profile to NVS", mode.name());
    }

    success
}

// === Helper functions ===

fn read_f32_prefixed(nvs: &EspNvs<NvsDefault>, prefix: &str, key: &str) -> Option<f32> {
    let mut buf = [0u8; 4];
    let full_key = format!("{}_{}", prefix, key);
    match nvs.get_raw(&full_key, &mut buf) {
        Ok(Some(_)) => Some(f32::from_le_bytes(buf)),
        _ => None,
    }
}

fn write_f32_prefixed(nvs: &mut EspNvs<NvsDefault>, prefix: &str, key: &str, val: f32) -> bool {
    let full_key = format!("{}_{}", prefix, key);
    nvs.set_raw(&full_key, &val.to_le_bytes()).is_ok()
}
