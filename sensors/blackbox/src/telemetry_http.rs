use crate::diagnostics::{DiagnosticsSnapshot, FusionDiagnostics};

/// Persistent timing results accompany the v4 binary packet. A browser may miss
/// individual packets without losing the exact completed lap time.
#[derive(Clone, Copy, Default, serde::Serialize)]
pub struct TimingTelemetry {
    pub state: &'static str,
    pub revision: u32,
    pub last_lap_ms: Option<u32>,
    pub best_lap_ms: Option<u32>,
    pub lat: f64,
    pub lon: f64,
    pub gps_valid: bool,
    pub gps_age_ms: Option<u32>,
    pub gps_boot_id: Option<u32>,
    pub gps_event: Option<crate::gps_log::GpsEvent>,
}

/// serde_json encodes non-finite sensor values as null and escapes text fields.
/// Hand-formatting NaN made the entire telemetry response invalid during GPS loss.
pub fn telemetry_json(
    seq: u32,
    data: &str,
    timing: &TimingTelemetry,
    fusion: Option<&FusionDiagnostics>,
) -> String {
    let f = fusion.map(|f| {
        serde_json::json!({
            "li": f.lon_imu_filtered, "lg": f.gps_accel, "gw": f.gps_weight,
            "gr": f.gps_rate, "pc": f.pitch_correction_deg, "pf": f.pitch_confidence * 100.0,
            "rc": f.roll_correction_deg, "rf": f.roll_confidence * 100.0,
            "tx": f.tilt_offset_x, "ty": f.tilt_offset_y
        })
    });
    serde_json::json!({"seq": seq, "data": data, "timing": timing, "f": f}).to_string()
}

pub fn diagnostics_json(d: &DiagnosticsSnapshot) -> String {
    serde_json::json!({
        "sensor_rates": {
            "imu_hz": d.sensor_rates.imu_hz, "gps_hz": d.sensor_rates.gps_hz,
            "loop_hz": d.sensor_rates.loop_hz, "imu_expected": d.sensor_rates.imu_expected_hz,
            "gps_expected": d.sensor_rates.gps_expected_hz, "zupt_per_min": d.sensor_rates.zupt_per_min,
            "ekf_per_gps": d.sensor_rates.ekf_predictions_per_gps
        },
        "ekf": {
            "pos_sigma": d.ekf_health.position_sigma, "vel_sigma": d.ekf_health.velocity_sigma,
            "yaw_sigma_deg": d.ekf_health.yaw_sigma_deg, "bias_x": d.ekf_health.bias_x, "bias_y": d.ekf_health.bias_y
        },
        "system": {
            "heap_free": d.system_health.free_heap_bytes, "uptime_s": d.system_health.uptime_seconds,
            "tx_ok": d.system_health.telemetry_sent, "tx_fail": d.system_health.telemetry_failed
        },
        "gps": {
            "model": d.gps_health.model_name, "rate_hz": d.gps_health.configured_rate_hz,
            "fix": d.gps_health.fix_valid, "warmup": d.gps_health.warmup_complete,
            "satellites": d.gps_health.satellites, "hdop": d.gps_health.hdop,
            "ref_lat": d.gps_health.ref_lat, "ref_lon": d.gps_health.ref_lon
        },
        "wifi": {"mode": d.wifi_status.mode, "ssid": d.wifi_status.ssid},
        "config": {"telemetry_hz": d.config.telemetry_rate_hz, "gps_model": d.config.gps_model, "warmup_fixes": d.config.gps_warmup_fixes},
        "fusion": {
            "lon_raw": d.fusion.lon_imu_raw, "lon_filtered": d.fusion.lon_imu_filtered,
            "lon_blended": d.fusion.lon_blended, "gps_weight": d.fusion.gps_weight,
            "gps_accel": d.fusion.gps_accel, "gps_rate": d.fusion.gps_rate, "gps_rejected": d.fusion.gps_rejected,
            "pitch_corr": d.fusion.pitch_correction_deg, "roll_corr": d.fusion.roll_correction_deg,
            "pitch_conf": d.fusion.pitch_confidence * 100.0, "roll_conf": d.fusion.roll_confidence * 100.0,
            "yaw_bias": d.fusion.yaw_bias, "yaw_calibrated": d.fusion.yaw_calibrated,
            "tilt_x": d.fusion.tilt_offset_x, "tilt_y": d.fusion.tilt_offset_y, "tilt_valid": d.fusion.tilt_valid
        }
    }).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unavailable_gps_acceleration_is_valid_json() {
        let fusion = FusionDiagnostics {
            gps_accel: f32::NAN,
            ..Default::default()
        };
        let text = telemetry_json(7, "AA==", &TimingTelemetry::default(), Some(&fusion));
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert!(value["f"]["lg"].is_null());
        assert_eq!(value["seq"], 7);
    }
    #[test]
    fn diagnostics_escape_ssids_and_missing_sensor_values() {
        let mut d = DiagnosticsSnapshot::default();
        d.wifi_status.ssid = "track\"wifi\\network";
        d.fusion.gps_accel = f32::NAN;
        d.ekf_health.position_sigma = f32::INFINITY;
        let value: serde_json::Value = serde_json::from_str(&diagnostics_json(&d)).unwrap();
        assert_eq!(value["wifi"]["ssid"], d.wifi_status.ssid);
        assert!(value["fusion"]["gps_accel"].is_null());
        assert!(value["ekf"]["pos_sigma"].is_null());
    }
}
