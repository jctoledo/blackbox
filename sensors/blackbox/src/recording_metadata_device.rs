//! ESP-IDF/config adapter. No arbitrary config/environment values cross this seam.
use crate::{
    config::GpsConfig,
    recording_metadata::{Build, Gps, RecordingMetadata},
};

/// Full monotonic controller uptime, without the acquisition loop's u32 wrap.
pub fn uptime_ms() -> u64 {
    // ESP-IDF timer is monotonic microseconds since this boot (not the u32 loop clock).
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 / 1000 }
}

/// Read only the application descriptor and allowlisted effective GPS settings.
pub fn initial(boot_id: u32, gps: &GpsConfig) -> Option<RecordingMetadata> {
    // The descriptor has static storage; read only its fixed-size arrays, never CStr.
    let desc = unsafe { esp_idf_svc::sys::esp_app_get_description().as_ref()? };
    Some(RecordingMetadata::new(
        boot_id,
        uptime_ms(),
        Build::from_descriptor(&desc.version, &desc.idf_ver, option_env!("GITHUB_SHA")),
        Gps::configured(gps.model.name(), gps.update_rate_hz, gps.effective_baud()),
    ))
}
