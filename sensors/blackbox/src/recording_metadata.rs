//! Versioned recording context; configured values are not receiver readback.
use serde::Serialize;

/// A bounded v1 recording context snapshot for one controller boot.
#[derive(Clone, Debug, Serialize)]
pub struct RecordingMetadata {
    version: u8,
    simulated: bool,
    pub boot_id: u32,
    pub captured_uptime_ms: u64,
    pub build: Build,
    pub gps: Gps,
    pub imu: Imu,
    pub calibration: Calibration,
}
impl RecordingMetadata {
    /// Assemble a snapshot with pending IMU selection and unknown calibration.
    pub fn new(boot_id: u32, captured_uptime_ms: u64, build: Build, gps: Gps) -> Self {
        Self {
            version: 1,
            simulated: false,
            boot_id,
            captured_uptime_ms,
            build,
            gps,
            imu: Imu::default(),
            calibration: Calibration::default(),
        }
    }
}

/// Build identity read from the application descriptor and optional source revision.
#[derive(Clone, Debug, Serialize)]
pub struct Build {
    app_version: Option<String>,
    idf_version: Option<String>,
    source_sha: Option<String>,
    target: &'static str,
}
impl Build {
    /// Decode bounded fields and accept only a complete 40-digit hexadecimal SHA.
    pub fn from_descriptor(app: &[u8], idf: &[u8], sha: Option<&str>) -> Self {
        Self {
            app_version: descriptor_text(app),
            idf_version: descriptor_text(idf),
            source_sha: sha
                .filter(|s| s.len() == 40 && s.bytes().all(|b| b.is_ascii_hexdigit()))
                .map(str::to_owned),
            target: "esp32c3",
        }
    }
}
/// Read at most the descriptor field's 32 bytes, stopping at its first NUL.
/// Empty or invalid UTF-8 fields are unknown, without reading beyond the slice.
pub fn descriptor_text(bytes: &[u8]) -> Option<String> {
    let bytes = &bytes[..bytes.len().min(32)];
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    if end == 0 {
        return None;
    }
    std::str::from_utf8(&bytes[..end]).ok().map(str::to_owned)
}

/// Effective configured GPS values, without claiming receiver verification.
#[derive(Clone, Debug, Serialize)]
pub struct Gps {
    configured_model: &'static str,
    configured_rate_hz: u8,
    configured_uart_baud: u32,
    configuration_verified: bool,
}
impl Gps {
    /// True only after ACK and matching RAM readback of the startup profile.
    pub fn set_setup(&mut self, report: ublox_gps::setup::Report) {
        self.configuration_verified = report.verified;
    }
    /// Preserve the effective controller configuration, not receiver readback.
    pub fn configured(model: &'static str, rate_hz: u8, uart_baud: u32) -> Self {
        Self {
            configured_model: model,
            configured_rate_hz: rate_hz,
            configured_uart_baud: uart_baud,
            configuration_verified: false,
        }
    }
}

/// How the controller selected the IMU UART baud.
#[derive(Clone, Copy, Debug, Default, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BaudSelection {
    #[default]
    Pending,
    HeaderProbe,
    Fallback,
}
/// Configured WT901 transport context; sensor output rate remains unknown.
#[derive(Clone, Debug, Serialize)]
pub struct Imu {
    model: &'static str,
    transport: &'static str,
    configured_uart_baud: Option<u32>,
    baud_selection: BaudSelection,
    configured_output_rate_hz: Option<u32>,
}
impl Default for Imu {
    fn default() -> Self {
        Self {
            model: "WT901",
            transport: "uart",
            configured_uart_baud: None,
            baud_selection: BaudSelection::Pending,
            configured_output_rate_hz: None,
        }
    }
}
impl Imu {
    /// Describe a completed baud selection without asserting packet validity.
    pub fn selected(baud: Option<u32>, selection: BaudSelection) -> Self {
        Self {
            configured_uart_baud: baud,
            baud_selection: selection,
            ..Self::default()
        }
    }
}
/// Successfully applied calibration only; null distinguishes unavailable from zero.
#[derive(Clone, Debug, Default, Serialize)]
pub struct Calibration {
    pub accelerometer_bias_mps2: Option<Bias>,
    pub mount_alignment: Option<MountAlignment>,
}
/// Finite accelerometer bias in metres per second squared.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct Bias {
    x: f32,
    y: f32,
    z: f32,
}
impl Bias {
    /// Reject the complete bias if any axis is nonfinite.
    pub fn finite(x: f32, y: f32, z: f32) -> Option<Self> {
        [x, y, z]
            .iter()
            .all(|v| v.is_finite())
            .then_some(Self { x, y, z })
    }
}
/// Saved/applied mounting offset in radians.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct MountAlignment {
    is_calibrated: bool,
    yaw_offset_rad: f32,
}
impl MountAlignment {
    /// Keep known zero/uncalibrated values and reject a nonfinite offset.
    pub fn finite(is_calibrated: bool, yaw_offset_rad: f32) -> Option<Self> {
        yaw_offset_rad.is_finite().then_some(Self {
            is_calibrated,
            yaw_offset_rad,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snapshot() -> RecordingMetadata {
        RecordingMetadata::new(
            7,
            u32::MAX as u64 + 9000,
            Build::from_descriptor(
                b"v1\0ignored",
                b"idf\0",
                Some("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"),
            ),
            Gps::configured("NEO-6M", 5, 9600),
        )
    }

    #[test]
    fn schema_unknowns_and_full_clock() {
        let value = serde_json::to_value(snapshot()).unwrap();
        assert_eq!(value["version"], 1);
        assert_eq!(value["simulated"], false);
        assert_eq!(value["boot_id"], 7);
        assert_eq!(value["captured_uptime_ms"], u32::MAX as u64 + 9000);
        assert_eq!(value["build"]["target"], "esp32c3");
        assert_eq!(value["imu"]["baud_selection"], "pending");
        assert!(value["imu"]["configured_uart_baud"].is_null());
        assert!(value["imu"]["configured_output_rate_hz"].is_null());
        assert!(value["calibration"]["accelerometer_bias_mps2"].is_null());
        assert!(value["calibration"]["mount_alignment"].is_null());
    }

    #[test]
    fn configured_variants_and_baud_states() {
        for (model, rate, baud) in [("NEO-6M", 5, 9600), ("NEO-M9N", 25, 115200)] {
            let gps = serde_json::to_value(Gps::configured(model, rate, baud)).unwrap();
            assert_eq!(gps["configured_model"], model);
            assert_eq!(gps["configured_rate_hz"], rate);
            assert_eq!(gps["configured_uart_baud"], baud);
            assert_eq!(gps["configuration_verified"], false);
        }
        for (baud, selection, label) in [
            (115200, BaudSelection::HeaderProbe, "header_probe"),
            (9600, BaudSelection::Fallback, "fallback"),
        ] {
            let imu = serde_json::to_value(Imu::selected(Some(baud), selection)).unwrap();
            assert_eq!(imu["configured_uart_baud"], baud);
            assert_eq!(imu["baud_selection"], label);
            assert!(imu["configured_output_rate_hz"].is_null());
        }
    }

    #[test]
    fn failed_fallback_baud_and_build_unknowns_stay_null() {
        for (selection, label) in [
            (BaudSelection::Fallback, "fallback"),
            (BaudSelection::HeaderProbe, "header_probe"),
        ] {
            let imu = serde_json::to_value(Imu::selected(None, selection)).unwrap();
            assert_eq!(imu["baud_selection"], label);
            assert!(imu["configured_uart_baud"].is_null());
        }
        let valid_sha = "A".repeat(40);
        let build =
            serde_json::to_value(Build::from_descriptor(b"v1\0", b"idf\0", Some(&valid_sha)))
                .unwrap();
        assert_eq!(
            build,
            serde_json::json!({"app_version":"v1", "idf_version":"idf", "source_sha":valid_sha, "target":"esp32c3"})
        );
        for length in [39, 41] {
            assert!(
                Build::from_descriptor(b"a", b"b", Some(&"a".repeat(length)))
                    .source_sha
                    .is_none()
            );
        }
        assert_eq!(descriptor_text(&[b'a'; 32]), Some("a".repeat(32)));
        let mut split_utf8 = [b'a'; 33];
        split_utf8[31] = 0xc3;
        split_utf8[32] = 0xa9;
        assert!(descriptor_text(&split_utf8).is_none());
    }

    #[test]
    fn applied_zero_is_known_and_nonfinite_is_unavailable() {
        let mut s = snapshot();
        s.calibration.accelerometer_bias_mps2 = Bias::finite(0.0, 0.0, 0.0);
        s.calibration.mount_alignment = MountAlignment::finite(false, 0.0);
        let value = serde_json::to_value(s).unwrap();
        assert_eq!(value["calibration"]["accelerometer_bias_mps2"]["x"], 0.0);
        assert_eq!(
            value["calibration"]["mount_alignment"]["is_calibrated"],
            false
        );
        for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert!(Bias::finite(v, 0.0, 0.0).is_none());
            assert!(Bias::finite(0.0, v, 0.0).is_none());
            assert!(Bias::finite(0.0, 0.0, v).is_none());
            assert!(MountAlignment::finite(true, v).is_none());
        }
    }

    #[test]
    fn descriptors_are_bounded_and_sha_is_exact_hex() {
        assert_eq!(descriptor_text(b"abc\0secret").as_deref(), Some("abc"));
        assert_eq!(descriptor_text(&[b'a'; 100]).unwrap().len(), 32);
        assert_eq!(descriptor_text(&[0xff, 0]), None);
        assert_eq!(descriptor_text(b""), None);
        assert_eq!(descriptor_text(b"\0hidden"), None);
        let unknown = serde_json::to_value(Build::from_descriptor(b"", &[0xff], None)).unwrap();
        assert!(unknown["app_version"].is_null());
        assert!(unknown["idf_version"].is_null());
        for sha in [
            None,
            Some(""),
            Some("abc123"),
            Some("0123456789abcdef0123456789abcdef012345678g"),
            Some(" 123456789abcdef0123456789abcdef0123456789"),
        ] {
            assert!(Build::from_descriptor(b"a", b"b", sha).source_sha.is_none());
        }
        assert_eq!(snapshot().build.source_sha.unwrap().len(), 40);
    }
}
