//! One versioned NVS record replaces the PR's separate offset/valid keys.
//! ESP-IDF commits the single value; a checksum detects corrupt payloads.
use crate::calibration::MountCalibration;
use esp_idf_svc::nvs::{EspNvs, EspNvsPartition, NvsDefault};

pub struct NvsStorage {
    nvs: EspNvs<NvsDefault>,
}
impl NvsStorage {
    pub fn new(partition: EspNvsPartition<NvsDefault>) -> Result<Self, esp_idf_svc::sys::EspError> {
        Ok(Self {
            nvs: EspNvs::new(partition, "blackbox", true)?,
        })
    }
    pub fn load(&self) -> MountCalibration {
        let mut buf = [0u8; 16];
        self.nvs
            .get_raw("mount_v1", &mut buf)
            .ok()
            .flatten()
            .and_then(MountCalibration::decode)
            .unwrap_or_default()
    }
    pub fn save(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if MountCalibration::decode(bytes).is_none() {
            return Err(());
        }
        self.nvs
            .set_raw("mount_v1", bytes)
            .map(|_| ())
            .map_err(|_| ())
    }
}
