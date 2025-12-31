//! Non-volatile storage for mount calibration.

use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};
use log::info;

const NAMESPACE: &str = "blackbox";
const KEY_MOUNT_OFFSET: &str = "mount_off";
const KEY_MOUNT_VALID: &str = "mount_ok";

/// NVS-backed persistent storage for calibration data
pub struct NvsStorage {
    nvs: EspNvs<NvsDefault>,
}

impl NvsStorage {
    pub fn new(partition: EspNvsPartition<NvsDefault>) -> Result<Self, esp_idf_svc::sys::EspError> {
        let nvs = EspNvs::new(partition, NAMESPACE, true)?;
        info!("NVS storage initialized");
        Ok(Self { nvs })
    }

    fn get_u8(&self, key: &str) -> Option<u8> {
        self.nvs.get_u8(key).ok().flatten()
    }

    fn set_u8(&mut self, key: &str, val: u8) -> Result<(), esp_idf_svc::sys::EspError> {
        self.nvs.set_u8(key, val)
    }

    fn get_f32(&self, key: &str) -> Option<f32> {
        let mut buf = [0u8; 4];
        let bytes = self.nvs.get_raw(key, &mut buf).ok().flatten()?;
        if bytes.len() != 4 {
            return None;
        }
        let val = f32::from_le_bytes(buf);
        if val.is_nan() {
            return None;
        }
        Some(val)
    }

    fn set_f32(&mut self, key: &str, val: f32) -> Result<(), esp_idf_svc::sys::EspError> {
        self.nvs.set_raw(key, &val.to_le_bytes())?;
        Ok(())
    }

    /// Load mount calibration from NVS. Returns None if not calibrated or invalid.
    pub fn load_mount_calibration(&self) -> Option<crate::calibration::MountCalibration> {
        if self.get_u8(KEY_MOUNT_VALID) != Some(1) {
            return None;
        }

        let offset = self.get_f32(KEY_MOUNT_OFFSET)?;

        // Sanity check: offset should be in [-π, π]
        if offset.abs() > core::f32::consts::PI + 0.1 {
            info!("NVS: Invalid mount offset {:.2}, ignoring", offset);
            return None;
        }

        info!("NVS: Loaded mount offset: {:.1}°", offset.to_degrees());
        Some(crate::calibration::MountCalibration::new(offset))
    }

    /// Save mount calibration to NVS.
    pub fn save_mount_calibration(
        &mut self,
        cal: &crate::calibration::MountCalibration,
    ) -> Result<(), esp_idf_svc::sys::EspError> {
        if !cal.is_calibrated {
            self.set_u8(KEY_MOUNT_VALID, 0)?;
            info!("NVS: Cleared mount calibration");
            return Ok(());
        }

        self.set_f32(KEY_MOUNT_OFFSET, cal.yaw_offset)?;
        self.set_u8(KEY_MOUNT_VALID, 1)?;

        info!("NVS: Saved mount offset: {:.1}°", cal.yaw_offset.to_degrees());
        Ok(())
    }
}
