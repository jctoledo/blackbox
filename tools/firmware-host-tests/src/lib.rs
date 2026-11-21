// Compile the same pure modules used by the ESP32, on the host for regression tests.
#[path = "../../../sensors/blackbox/src/binary_telemetry.rs"]
pub mod binary_telemetry;
#[path = "../../../sensors/blackbox/src/calibration.rs"]
pub mod calibration;
#[path = "../../../sensors/blackbox/src/diagnostics.rs"]
pub mod diagnostics;
#[path = "../../../sensors/blackbox/src/gps_log.rs"]
pub mod gps_log;
#[path = "../../../sensors/blackbox/src/telemetry_http.rs"]
pub mod telemetry_http;
