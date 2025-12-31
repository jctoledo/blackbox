//! Non-volatile storage for persistent settings and calibration data
//!
//! Provides a clean API for storing typed values that survive reboots.
//! Uses ESP-IDF NVS (Non-Volatile Storage) under the hood.

#![allow(dead_code)] // Some methods reserved for future use

use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};
use log::info;

const NAMESPACE: &str = "blackbox";

/// NVS-backed persistent storage
pub struct NvsStorage {
    nvs: EspNvs<NvsDefault>,
}

impl NvsStorage {
    /// Create a new storage handle
    pub fn new(partition: EspNvsPartition<NvsDefault>) -> Result<Self, esp_idf_svc::sys::EspError> {
        let nvs = EspNvs::new(partition, NAMESPACE, true)?;
        info!("NVS storage initialized (namespace: {})", NAMESPACE);
        Ok(Self { nvs })
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Primitive getters/setters
    // ─────────────────────────────────────────────────────────────────────────

    /// Get a u8 value
    pub fn get_u8(&self, key: &str) -> Option<u8> {
        self.nvs.get_u8(key).ok().flatten()
    }

    /// Set a u8 value
    pub fn set_u8(&mut self, key: &str, val: u8) -> Result<(), esp_idf_svc::sys::EspError> {
        self.nvs.set_u8(key, val)
    }

    /// Get an i16 value
    pub fn get_i16(&self, key: &str) -> Option<i16> {
        self.nvs.get_i16(key).ok().flatten()
    }

    /// Set an i16 value
    pub fn set_i16(&mut self, key: &str, val: i16) -> Result<(), esp_idf_svc::sys::EspError> {
        self.nvs.set_i16(key, val)
    }

    /// Get an f32 value (stored as raw bytes)
    pub fn get_f32(&self, key: &str) -> Option<f32> {
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

    /// Set an f32 value (stored as raw bytes)
    pub fn set_f32(&mut self, key: &str, val: f32) -> Result<(), esp_idf_svc::sys::EspError> {
        let bytes = val.to_le_bytes();
        self.nvs.set_raw(key, &bytes)?;
        Ok(())
    }

    /// Check if a key exists
    pub fn contains(&self, key: &str) -> bool {
        self.nvs.contains(key).unwrap_or(false)
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Mount calibration (convenience methods)
    // ─────────────────────────────────────────────────────────────────────────

    const KEY_MOUNT_OFFSET: &'static str = "mount_off";
    const KEY_MOUNT_VALID: &'static str = "mount_ok";

    /// Load mount calibration offset from NVS
    /// Returns None if not calibrated or invalid
    pub fn load_mount_calibration(&self) -> Option<crate::calibration::MountCalibration> {
        // Check validity flag
        if self.get_u8(Self::KEY_MOUNT_VALID) != Some(1) {
            return None;
        }

        // Load offset
        let offset = self.get_f32(Self::KEY_MOUNT_OFFSET)?;

        // Sanity check: offset should be in [-π, π]
        if offset.abs() > core::f32::consts::PI + 0.1 {
            info!("NVS: Invalid mount offset {:.2}, ignoring", offset);
            return None;
        }

        info!("NVS: Loaded mount offset: {:.1}°", offset.to_degrees());
        Some(crate::calibration::MountCalibration::new(offset))
    }

    /// Save mount calibration to NVS
    pub fn save_mount_calibration(
        &mut self,
        cal: &crate::calibration::MountCalibration,
    ) -> Result<(), esp_idf_svc::sys::EspError> {
        if !cal.is_calibrated {
            self.set_u8(Self::KEY_MOUNT_VALID, 0)?;
            info!("NVS: Cleared mount calibration");
            return Ok(());
        }

        self.set_f32(Self::KEY_MOUNT_OFFSET, cal.yaw_offset)?;
        self.set_u8(Self::KEY_MOUNT_VALID, 1)?;

        info!("NVS: Saved mount offset: {:.1}°", cal.yaw_offset.to_degrees());
        Ok(())
    }

    // ─────────────────────────────────────────────────────────────────────────
    // IMU bias calibration (for future use)
    // ─────────────────────────────────────────────────────────────────────────

    const KEY_IMU_BIAS_AX: &'static str = "imu_bax";
    const KEY_IMU_BIAS_AY: &'static str = "imu_bay";
    const KEY_IMU_BIAS_AZ: &'static str = "imu_baz";
    const KEY_IMU_BIAS_VALID: &'static str = "imu_ok";

    /// Load IMU bias calibration from NVS
    pub fn load_imu_bias(&self) -> Option<crate::imu::ImuBias> {
        if self.get_u8(Self::KEY_IMU_BIAS_VALID) != Some(1) {
            return None;
        }

        let ax = self.get_f32(Self::KEY_IMU_BIAS_AX)?;
        let ay = self.get_f32(Self::KEY_IMU_BIAS_AY)?;
        let az = self.get_f32(Self::KEY_IMU_BIAS_AZ)?;

        info!("NVS: Loaded IMU bias: [{:.4}, {:.4}, {:.4}]", ax, ay, az);
        Some(crate::imu::ImuBias { ax, ay, az })
    }

    /// Save IMU bias calibration to NVS
    pub fn save_imu_bias(
        &mut self,
        bias: &crate::imu::ImuBias,
    ) -> Result<(), esp_idf_svc::sys::EspError> {
        self.set_f32(Self::KEY_IMU_BIAS_AX, bias.ax)?;
        self.set_f32(Self::KEY_IMU_BIAS_AY, bias.ay)?;
        self.set_f32(Self::KEY_IMU_BIAS_AZ, bias.az)?;
        self.set_u8(Self::KEY_IMU_BIAS_VALID, 1)?;

        info!("NVS: Saved IMU bias: [{:.4}, {:.4}, {:.4}]", bias.ax, bias.ay, bias.az);
        Ok(())
    }

    /// Clear IMU bias calibration
    pub fn clear_imu_bias(&mut self) -> Result<(), esp_idf_svc::sys::EspError> {
        self.set_u8(Self::KEY_IMU_BIAS_VALID, 0)?;
        info!("NVS: Cleared IMU bias");
        Ok(())
    }
}
