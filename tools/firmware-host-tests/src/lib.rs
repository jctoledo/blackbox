// Compile the same pure modules used by the ESP32, on the host for regression tests.
#[path = "../../../sensors/blackbox/src/binary_telemetry.rs"]
pub mod binary_telemetry;
#[path = "../../../sensors/blackbox/src/calibration.rs"]
pub mod calibration;
#[path = "../../../sensors/blackbox/src/diagnostics.rs"]
pub mod diagnostics;
#[path = "../../../sensors/blackbox/src/gps_log.rs"]
pub mod gps_log;
#[path = "../../../sensors/blackbox/src/gps_timing.rs"]
pub mod gps_timing;
#[path = "../../../sensors/blackbox/src/recording_metadata.rs"]
pub mod recording_metadata;
#[path = "../../../sensors/blackbox/src/telemetry_http.rs"]
pub mod telemetry_http;

#[path = "../../../sensors/blackbox/src/imu_evidence.rs"]
pub mod imu_evidence;
