//! NVS-backed settings persistence for mode detection thresholds

use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};
use log::{info, warn};

use crate::mode::ModeConfig;

const NAMESPACE: &str = "bb_cfg";

// Keys (max 15 chars for NVS)
const KEY_ACC: &str = "acc";
const KEY_ACC_EXIT: &str = "acc_x";
const KEY_BRAKE: &str = "brk";
const KEY_BRAKE_EXIT: &str = "brk_x";
const KEY_LAT: &str = "lat";
const KEY_LAT_EXIT: &str = "lat_x";
const KEY_YAW: &str = "yaw";
const KEY_MIN_SPEED: &str = "spd";

/// Load settings from NVS, returns None if not found or error
pub fn load_settings(nvs_partition: EspNvsPartition<NvsDefault>) -> Option<ModeConfig> {
    let nvs = match EspNvs::new(nvs_partition, NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(e) => {
            warn!("Failed to open NVS namespace: {:?}", e);
            return None;
        }
    };

    // Try to read all values - if any fail, return None (use defaults)
    let acc = read_f32(&nvs, KEY_ACC)?;
    let acc_exit = read_f32(&nvs, KEY_ACC_EXIT)?;
    let brake = read_f32(&nvs, KEY_BRAKE)?;
    let brake_exit = read_f32(&nvs, KEY_BRAKE_EXIT)?;
    let lat = read_f32(&nvs, KEY_LAT)?;
    let lat_exit = read_f32(&nvs, KEY_LAT_EXIT)?;
    let yaw = read_f32(&nvs, KEY_YAW)?;
    let min_speed = read_f32(&nvs, KEY_MIN_SPEED)?;

    info!(
        "Loaded settings from NVS: acc={:.2}, brake={:.2}, lat={:.2}, yaw={:.3}",
        acc, brake, lat, yaw
    );

    Some(ModeConfig {
        acc_thr: acc,
        acc_exit,
        brake_thr: brake,
        brake_exit,
        lat_thr: lat,
        lat_exit,
        yaw_thr: yaw,
        min_speed,
        alpha: 0.35, // EMA alpha is not persisted - always use default
    })
}

/// Save settings to NVS
pub fn save_settings(nvs_partition: EspNvsPartition<NvsDefault>, config: &ModeConfig) -> bool {
    let mut nvs = match EspNvs::new(nvs_partition, NAMESPACE, true) {
        Ok(nvs) => nvs,
        Err(e) => {
            warn!("Failed to open NVS namespace for write: {:?}", e);
            return false;
        }
    };

    let success = write_f32(&mut nvs, KEY_ACC, config.acc_thr)
        && write_f32(&mut nvs, KEY_ACC_EXIT, config.acc_exit)
        && write_f32(&mut nvs, KEY_BRAKE, config.brake_thr)
        && write_f32(&mut nvs, KEY_BRAKE_EXIT, config.brake_exit)
        && write_f32(&mut nvs, KEY_LAT, config.lat_thr)
        && write_f32(&mut nvs, KEY_LAT_EXIT, config.lat_exit)
        && write_f32(&mut nvs, KEY_YAW, config.yaw_thr)
        && write_f32(&mut nvs, KEY_MIN_SPEED, config.min_speed);

    if success {
        info!(
            "Saved settings to NVS: acc={:.2}, brake={:.2}, lat={:.2}, yaw={:.3}",
            config.acc_thr, config.brake_thr, config.lat_thr, config.yaw_thr
        );
    } else {
        warn!("Failed to save some settings to NVS");
    }

    success
}

fn read_f32(nvs: &EspNvs<NvsDefault>, key: &str) -> Option<f32> {
    // NVS stores as blob - read 4 bytes
    let mut buf = [0u8; 4];
    match nvs.get_raw(key, &mut buf) {
        Ok(Some(_)) => Some(f32::from_le_bytes(buf)),
        _ => None,
    }
}

fn write_f32(nvs: &mut EspNvs<NvsDefault>, key: &str, val: f32) -> bool {
    nvs.set_raw(key, &val.to_le_bytes()).is_ok()
}
