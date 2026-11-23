use std::sync::{
    atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering},
    Arc, Mutex,
};

/// HTTP telemetry server for dashboard and settings
/// Uses HTTP polling for reliable updates without thread blocking
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::Write;
use log::info;

use crate::diagnostics::DiagnosticsState;

/// Lap timer timing line configuration using GPS coordinates
/// Uses GPS lat/lon for session-independent timing (no coordinate frame issues)
#[derive(Clone, Copy, Debug)]
pub struct TimingLineConfig {
    pub p1_lat: f64,
    pub p1_lon: f64,
    pub p2_lat: f64,
    pub p2_lon: f64,
    pub direction: f32,
}

/// Lap timer configuration
#[derive(Clone, Debug, Default)]
pub enum LapTimerConfig {
    /// No configuration / clear timer
    #[default]
    None,
    /// Loop track with single start/finish line
    Loop(TimingLineConfig),
    /// Point-to-point with separate start and finish
    PointToPoint {
        start: TimingLineConfig,
        finish: TimingLineConfig,
    },
}

/// Mode detection settings (matching mode.rs ModeConfig)
#[derive(Clone, Copy)]
pub struct ModeSettings {
    pub acc_thr: f32,    // Acceleration threshold (g)
    pub acc_exit: f32,   // Acceleration exit threshold (g)
    pub brake_thr: f32,  // Braking threshold (g, negative)
    pub brake_exit: f32, // Braking exit threshold (g, negative)
    pub lat_thr: f32,    // Lateral threshold (g)
    pub lat_exit: f32,   // Lateral exit threshold (g)
    pub yaw_thr: f32,    // Yaw rate threshold (rad/s)
    pub min_speed: f32,  // Minimum speed for mode detection (m/s)
}

impl Default for ModeSettings {
    fn default() -> Self {
        // Matches city preset - sensitive defaults for street driving
        Self {
            acc_thr: 0.10,     // 0.10g - city driving (gentle acceleration)
            acc_exit: 0.05,    // Exit threshold
            brake_thr: -0.18,  // -0.18g - city driving (normal braking)
            brake_exit: -0.09, // Exit threshold
            lat_thr: 0.12,     // 0.12g - city turns
            lat_exit: 0.06,    // Exit threshold
            yaw_thr: 0.05,     // ~2.9°/s yaw rate
            min_speed: 2.0,    // ~7 km/h
        }
    }
}

pub use crate::telemetry_http::TimingTelemetry;

#[derive(Default)]
struct TelemetrySnapshot {
    data: Vec<u8>,
    seq: u32,
    timing: TimingTelemetry,
}

/// Shared state for telemetry server
pub struct TelemetryServerState {
    gps_log: Mutex<crate::gps_log::GpsLog>,
    recording_metadata: Mutex<Option<crate::recording_metadata::RecordingMetadata>>,
    mount_command: AtomicU8,
    mount_status: Mutex<String>,
    /// Latest telemetry packet (raw bytes)
    telemetry_data: Mutex<TelemetrySnapshot>,
    /// Calibration requested flag
    calibration_requested: AtomicBool,
    /// Mode detection settings
    mode_settings: Mutex<ModeSettings>,
    /// Settings changed flag
    settings_changed: AtomicBool,
    /// Optional diagnostics state (shared with main loop)
    diagnostics_state: Option<Arc<DiagnosticsState>>,
    /// Lap timer configuration
    lap_timer_config: Mutex<Option<(u32, LapTimerConfig)>>,
    /// Lap timer config changed flag
    lap_timer_revision: AtomicU32,
}

impl TelemetryServerState {
    pub fn new() -> Self {
        Self {
            gps_log: Mutex::new(crate::gps_log::GpsLog::new(unsafe {
                esp_idf_svc::sys::esp_random()
            })),
            recording_metadata: Mutex::new(None),
            mount_command: AtomicU8::new(0),
            mount_status: Mutex::new(String::from("{}")),
            telemetry_data: Mutex::new(TelemetrySnapshot::default()),
            calibration_requested: AtomicBool::new(false),
            mode_settings: Mutex::new(ModeSettings::default()),
            settings_changed: AtomicBool::new(false),
            diagnostics_state: None,
            lap_timer_config: Mutex::new(None),
            lap_timer_revision: AtomicU32::new(0),
        }
    }

    /// Create with diagnostics state for diagnostics page support
    pub fn with_diagnostics(diagnostics: Arc<DiagnosticsState>) -> Self {
        Self {
            gps_log: Mutex::new(crate::gps_log::GpsLog::new(unsafe {
                esp_idf_svc::sys::esp_random()
            })),
            recording_metadata: Mutex::new(None),
            mount_command: AtomicU8::new(0),
            mount_status: Mutex::new(String::from("{}")),
            telemetry_data: Mutex::new(TelemetrySnapshot::default()),
            calibration_requested: AtomicBool::new(false),
            mode_settings: Mutex::new(ModeSettings::default()),
            settings_changed: AtomicBool::new(false),
            diagnostics_state: Some(diagnostics),
            lap_timer_config: Mutex::new(None),
            lap_timer_revision: AtomicU32::new(0),
        }
    }

    /// Reuse the GPS log identity rather than generating another boot identifier.
    pub fn boot_id(&self) -> Option<u32> {
        self.gps_log.lock().ok().map(|log| log.latest().0)
    }

    /// Replace the latest typed context under a brief lock; serialize only on reads.
    pub fn publish_recording_metadata(
        &self,
        snapshot: crate::recording_metadata::RecordingMetadata,
    ) {
        if let Ok(mut guard) = self.recording_metadata.lock() {
            *guard = Some(snapshot);
        }
    }

    pub fn take_mount_command(&self) -> u8 {
        self.mount_command.swap(0, Ordering::SeqCst)
    }
    pub fn publish_mount_status(&self, status: &crate::calibration::CalibrationStatus) {
        if let (Ok(json), Ok(mut guard)) = (serde_json::to_string(status), self.mount_status.lock())
        {
            *guard = json;
        }
    }

    /// Get diagnostics state reference
    pub fn diagnostics(&self) -> Option<&Arc<DiagnosticsState>> {
        self.diagnostics_state.as_ref()
    }

    /// Request calibration from the UI
    pub fn request_calibration(&self) {
        self.calibration_requested.store(true, Ordering::SeqCst);
    }

    /// Check if calibration was requested and clear the flag
    pub fn take_calibration_request(&self) -> bool {
        self.calibration_requested.swap(false, Ordering::SeqCst)
    }

    /// Get current mode settings
    pub fn get_settings(&self) -> ModeSettings {
        self.mode_settings.lock().map(|g| *g).unwrap_or_default()
    }

    /// Update mode settings
    pub fn set_settings(&self, settings: ModeSettings) {
        if let Ok(mut guard) = self.mode_settings.lock() {
            *guard = settings;
            self.settings_changed.store(true, Ordering::SeqCst);
        }
    }

    /// Check if settings were changed and clear the flag
    pub fn take_settings_change(&self) -> Option<ModeSettings> {
        if self.settings_changed.swap(false, Ordering::SeqCst) {
            Some(self.get_settings())
        } else {
            None
        }
    }

    /// Queue a versioned configuration so clients can wait for it to be applied.
    pub fn set_lap_timer_config(&self, config: LapTimerConfig) -> u32 {
        if let Ok(mut guard) = self.lap_timer_config.lock() {
            let revision = self
                .lap_timer_revision
                .fetch_add(1, Ordering::SeqCst)
                .wrapping_add(1);
            *guard = Some((revision, config));
            revision
        } else {
            0
        }
    }

    pub fn take_lap_timer_config(&self) -> Option<(u32, LapTimerConfig)> {
        self.lap_timer_config
            .lock()
            .ok()
            .and_then(|mut guard| guard.take())
    }

    /// Store data, sequence and persistent timing metadata atomically.
    pub fn record_gps_event(
        &self,
        timing: ublox_gps::RmcTiming,
        fix: &ublox_gps::GpsFix,
        host_us: u64,
    ) {
        if let Ok(mut log) = self.gps_log.lock() {
            log.push(timing, fix, host_us);
        }
    }

    pub fn update_telemetry(&self, data: &[u8], mut timing: TimingTelemetry) {
        if let Ok(log) = self.gps_log.lock() {
            let (boot_id, event) = log.latest();
            timing.gps_boot_id = Some(boot_id);
            timing.gps_event = event;
        }
        if let Ok(mut guard) = self.telemetry_data.lock() {
            guard.data.clear();
            guard.data.extend_from_slice(data);
            guard.timing = timing;
            guard.seq = guard.seq.wrapping_add(1);
        }
    }

    fn telemetry_snapshot(&self) -> Option<(u32, String, TimingTelemetry)> {
        self.telemetry_data.lock().ok().and_then(|snapshot| {
            if snapshot.data.is_empty() {
                None
            } else {
                Some((snapshot.seq, base64_encode(&snapshot.data), snapshot.timing))
            }
        })
    }
}

impl Default for TelemetryServerState {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple base64 encoder
fn base64_encode(data: &[u8]) -> String {
    const CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::with_capacity(data.len().div_ceil(3) * 4);

    for chunk in data.chunks(3) {
        let b0 = chunk[0] as usize;
        let b1 = chunk.get(1).copied().unwrap_or(0) as usize;
        let b2 = chunk.get(2).copied().unwrap_or(0) as usize;

        result.push(CHARS[b0 >> 2] as char);
        result.push(CHARS[((b0 & 0x03) << 4) | (b1 >> 4)] as char);

        if chunk.len() > 1 {
            result.push(CHARS[((b1 & 0x0f) << 2) | (b2 >> 6)] as char);
        } else {
            result.push('=');
        }

        if chunk.len() > 2 {
            result.push(CHARS[b2 & 0x3f] as char);
        } else {
            result.push('=');
        }
    }

    result
}

fn valid_timing_line(line: &TimingLineConfig) -> bool {
    [line.p1_lat, line.p2_lat]
        .iter()
        .all(|v| v.is_finite() && v.abs() <= 90.0)
        && [line.p1_lon, line.p2_lon]
            .iter()
            .all(|v| v.is_finite() && v.abs() <= 180.0)
        && line.direction.is_finite()
        && line.direction.abs() <= core::f32::consts::TAU
        && (line.p1_lat != line.p2_lat || line.p1_lon != line.p2_lon)
}

/// Parse lap timer configuration from query string
fn parse_lap_timer_config(uri: &str, state: &TelemetryServerState) -> (String, u16) {
    let query = match uri.split('?').nth(1) {
        Some(q) => q,
        None => return (r#"{"error":"missing query parameters"}"#.to_string(), 400),
    };

    // Parse query parameters into a simple map
    let mut params = std::collections::HashMap::new();
    for param in query.split('&') {
        let mut parts = param.split('=');
        if let (Some(key), Some(val)) = (parts.next(), parts.next()) {
            params.insert(key, val);
        }
    }

    // Get config type
    let config_type = match params.get("type") {
        Some(&t) => t,
        None => return (r#"{"error":"missing 'type' parameter"}"#.to_string(), 400),
    };

    match config_type {
        "clear" | "none" => {
            state.set_lap_timer_config(LapTimerConfig::None);
            info!("Lap timer cleared");
            (r#"{"status":"ok","config":"cleared"}"#.to_string(), 200)
        }
        "loop" => {
            // Parse timing line parameters (GPS coordinates)
            let p1_lat = params.get("p1_lat").and_then(|v| v.parse::<f64>().ok());
            let p1_lon = params.get("p1_lon").and_then(|v| v.parse::<f64>().ok());
            let p2_lat = params.get("p2_lat").and_then(|v| v.parse::<f64>().ok());
            let p2_lon = params.get("p2_lon").and_then(|v| v.parse::<f64>().ok());
            let dir = params.get("dir").and_then(|v| v.parse::<f32>().ok());

            match (p1_lat, p1_lon, p2_lat, p2_lon, dir) {
                (Some(p1_lat), Some(p1_lon), Some(p2_lat), Some(p2_lon), Some(direction)) => {
                    let line = TimingLineConfig {
                        p1_lat,
                        p1_lon,
                        p2_lat,
                        p2_lon,
                        direction,
                    };
                    if !valid_timing_line(&line) {
                        return (r#"{"error":"invalid timing line coordinates or direction"}"#.to_string(), 400);
                    }
                    let revision = state.set_lap_timer_config(LapTimerConfig::Loop(line));
                    info!(
                        "Lap timer configured: loop track, line ({:.6},{:.6})-({:.6},{:.6}) dir={:.2}",
                        p1_lat, p1_lon, p2_lat, p2_lon, direction
                    );
                    (
                        format!(
                            r#"{{"status":"ok","config":"loop","revision":{},"line":{{"p1":{{"lat":{},"lon":{}}},"p2":{{"lat":{},"lon":{}}},"dir":{}}}}}"#,
                            revision, p1_lat, p1_lon, p2_lat, p2_lon, direction
                        ),
                        200,
                    )
                }
                _ => (
                    r#"{"error":"missing parameters for loop config (p1_lat, p1_lon, p2_lat, p2_lon, dir)"}"#
                        .to_string(),
                    400,
                ),
            }
        }
        "point_to_point" => {
            // Parse start line (GPS coordinates)
            let p1_lat = params.get("p1_lat").and_then(|v| v.parse::<f64>().ok());
            let p1_lon = params.get("p1_lon").and_then(|v| v.parse::<f64>().ok());
            let p2_lat = params.get("p2_lat").and_then(|v| v.parse::<f64>().ok());
            let p2_lon = params.get("p2_lon").and_then(|v| v.parse::<f64>().ok());
            let dir = params.get("dir").and_then(|v| v.parse::<f32>().ok());
            // Parse finish line (f_ prefix)
            let f_p1_lat = params.get("f_p1_lat").and_then(|v| v.parse::<f64>().ok());
            let f_p1_lon = params.get("f_p1_lon").and_then(|v| v.parse::<f64>().ok());
            let f_p2_lat = params.get("f_p2_lat").and_then(|v| v.parse::<f64>().ok());
            let f_p2_lon = params.get("f_p2_lon").and_then(|v| v.parse::<f64>().ok());
            let f_dir = params.get("f_dir").and_then(|v| v.parse::<f32>().ok());

            match (
                p1_lat, p1_lon, p2_lat, p2_lon, dir,
                f_p1_lat, f_p1_lon, f_p2_lat, f_p2_lon, f_dir,
            ) {
                (
                    Some(p1_lat),
                    Some(p1_lon),
                    Some(p2_lat),
                    Some(p2_lon),
                    Some(direction),
                    Some(f_p1_lat),
                    Some(f_p1_lon),
                    Some(f_p2_lat),
                    Some(f_p2_lon),
                    Some(f_direction),
                ) => {
                    let start = TimingLineConfig {
                        p1_lat,
                        p1_lon,
                        p2_lat,
                        p2_lon,
                        direction,
                    };
                    let finish = TimingLineConfig {
                        p1_lat: f_p1_lat,
                        p1_lon: f_p1_lon,
                        p2_lat: f_p2_lat,
                        p2_lon: f_p2_lon,
                        direction: f_direction,
                    };
                    if !valid_timing_line(&start) || !valid_timing_line(&finish) {
                        return (r#"{"error":"invalid timing line coordinates or direction"}"#.to_string(), 400);
                    }
                    let revision = state.set_lap_timer_config(LapTimerConfig::PointToPoint { start, finish });
                    info!(
                        "Lap timer configured: point-to-point, start ({:.6},{:.6})-({:.6},{:.6}), finish ({:.6},{:.6})-({:.6},{:.6})",
                        p1_lat, p1_lon, p2_lat, p2_lon, f_p1_lat, f_p1_lon, f_p2_lat, f_p2_lon
                    );
                    (
                        format!(
                            r#"{{"status":"ok","config":"point_to_point","revision":{},"start":{{"p1":{{"lat":{},"lon":{}}},"p2":{{"lat":{},"lon":{}}},"dir":{}}},"finish":{{"p1":{{"lat":{},"lon":{}}},"p2":{{"lat":{},"lon":{}}},"dir":{}}}}}"#,
                            revision, p1_lat, p1_lon, p2_lat, p2_lon, direction,
                            f_p1_lat, f_p1_lon, f_p2_lat, f_p2_lon, f_direction
                        ),
                        200,
                    )
                }
                _ => (
                    r#"{"error":"missing parameters for point_to_point config (start: p1_lat, p1_lon, p2_lat, p2_lon, dir; finish: f_p1_lat, f_p1_lon, f_p2_lat, f_p2_lon, f_dir)"}"#
                        .to_string(),
                    400,
                ),
            }
        }
        _ => (
            format!(
                r#"{{"error":"unknown type '{}', expected: clear, loop, point_to_point"}}"#,
                config_type
            ),
            400,
        ),
    }
}

/// Telemetry HTTP server
pub struct TelemetryServer {
    _server: EspHttpServer<'static>,
    state: Arc<TelemetryServerState>,
}

/// Combined dashboard + diagnostics single-page app with IndexedDB recording
const DASHBOARD_HTML: &str = include_str!("../web/dashboard.html");

impl TelemetryServer {
    /// Create and start the telemetry server with HTTP polling
    ///
    /// # Arguments
    /// * `port` - HTTP port to listen on
    /// * `diagnostics` - Optional diagnostics state for /diagnostics endpoint
    pub fn new(
        port: u16,
        diagnostics: Option<Arc<DiagnosticsState>>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        info!("Starting Telemetry server on port {}", port);

        let server_config = Configuration {
            http_port: port,
            max_uri_handlers: 12,
            max_open_sockets: 8,
            stack_size: 10240,
            ..Default::default()
        };

        let mut server = EspHttpServer::new(&server_config)?;
        let state = Arc::new(match diagnostics {
            Some(diag) => TelemetryServerState::with_diagnostics(diag),
            None => TelemetryServerState::new(),
        });

        // Serve the combined dashboard + diagnostics HTML
        server.fn_handler(
            "/",
            esp_idf_svc::http::Method::Get,
            |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let html_bytes = DASHBOARD_HTML.as_bytes();
                let content_length = html_bytes.len().to_string();
                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "text/html; charset=utf-8"),
                        ("Content-Length", &content_length),
                        ("Connection", "close"),
                    ],
                )?;
                response.write_all(html_bytes)?;
                Ok(())
            },
        )?;

        // A small RAM window, fetched once per second while recording. Clone under
        // the lock, serialize outside it so UART acquisition is not blocked by JSON.
        let gps_state = state.clone();
        server.fn_handler(
            "/api/gps-log",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let query = req.uri().split_once('?').map_or("", |(_, q)| q);
                let parameter = |name: &str| {
                    query.split('&').find_map(|item| {
                        let (key, value) = item.split_once('=')?;
                        (key == name).then_some(value)
                    })
                };
                let cursor = parameter("boot")
                    .and_then(|b| b.parse::<u32>().ok())
                    .zip(parameter("after").and_then(|s| s.parse::<u64>().ok()));
                let snapshot = gps_state
                    .gps_log
                    .lock()
                    .ok()
                    .map(|log| log.snapshot_since(cursor));
                let json = serde_json::to_string(&snapshot).unwrap_or_else(|_| "null".into());
                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Cache-Control", "no-store"),
                    ],
                )?;
                response.write_all(json.as_bytes())?;
                Ok(())
            },
        )?;

        let metadata_state = state.clone();
        server.fn_handler(
            "/api/recording-metadata",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                // Clone the typed value under a brief lock; serialize after releasing it.
                let snapshot = metadata_state
                    .recording_metadata
                    .lock()
                    .ok()
                    .and_then(|guard| guard.clone());
                let json = snapshot.and_then(|value| serde_json::to_string(&value).ok());
                let (status, body) = match json {
                    Some(body) => (200, body),
                    None => (503, r#"{"status":"unavailable"}"#.to_owned()),
                };
                let mut response = req.into_response(
                    status,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Cache-Control", "no-store"),
                    ],
                )?;
                response.write_all(body.as_bytes())?;
                Ok(())
            },
        )?;

        // HTTP polling endpoint - includes fusion data for synchronized CSV export
        let state_poll = state.clone();
        server.fn_handler(
            "/api/telemetry",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let json = if let Some((seq, data, timing)) = state_poll.telemetry_snapshot() {
                    let fusion = state_poll.diagnostics().map(|diag| diag.snapshot().fusion);
                    crate::telemetry_http::telemetry_json(seq, &data, &timing, fusion.as_ref())
                } else {
                    r#"{"seq":0,"data":null}"#.to_string()
                };

                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Access-Control-Allow-Origin", "*"),
                        ("Cache-Control", "no-cache"),
                    ],
                )?;
                response.write_all(json.as_bytes())?;
                Ok(())
            },
        )?;

        // Health check
        server.fn_handler(
            "/api/status",
            esp_idf_svc::http::Method::Get,
            |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                #[cfg(esp_idf_httpd_ws_support)]
                let body = b"{\"status\":\"ok\",\"ws\":true}";
                #[cfg(not(esp_idf_httpd_ws_support))]
                let body = b"{\"status\":\"ok\",\"ws\":false}";

                let content_length = body.len().to_string();
                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Content-Length", &content_length),
                        ("Connection", "close"),
                    ],
                )?;
                response.write_all(body)?;
                Ok(())
            },
        )?;

        let mount_get = state.clone();
        server.fn_handler(
            "/api/mount-calibration",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let body = mount_get
                    .mount_status
                    .lock()
                    .map(|s| s.clone())
                    .unwrap_or_else(|_| String::from("{}"));
                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Cache-Control", "no-store"),
                    ],
                )?;
                response.write_all(body.as_bytes())?;
                Ok(())
            },
        )?;
        let mount_post = state.clone();
        server.fn_handler(
            "/api/mount-calibration",
            esp_idf_svc::http::Method::Post,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let command = match req.uri().split_once('?').map(|(_, q)| q) {
                    Some("action=start") => 1,
                    Some("action=cancel") => 2,
                    Some("action=save") => 3,
                    Some("action=clear") => 4,
                    _ => 0,
                };
                let (code, body) = if req.header("X-Blackbox-Action") != Some("mount") {
                    (403, "Missing action header")
                } else if command == 0 {
                    (400, "Unknown action")
                } else if mount_post
                    .mount_command
                    .compare_exchange(0, command, Ordering::SeqCst, Ordering::SeqCst)
                    .is_err()
                {
                    (409, "Command pending")
                } else {
                    (202, "Queued; read status for outcome")
                };
                let mut response = req.into_response(
                    code,
                    None,
                    &[
                        ("Content-Type", "text/plain"),
                        ("Cache-Control", "no-store"),
                    ],
                )?;
                response.write_all(body.as_bytes())?;
                Ok(())
            },
        )?;

        // Calibration trigger endpoint
        let state_calib = state.clone();
        server.fn_handler(
            "/api/calibrate",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                info!(">>> /api/calibrate endpoint called - requesting calibration");
                state_calib.request_calibration();
                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Access-Control-Allow-Origin", "*"),
                    ],
                )?;
                response.write_all(b"{\"status\":\"calibrating\"}")?;
                Ok(())
            },
        )?;

        // Get current settings
        let state_get = state.clone();
        server.fn_handler("/api/settings", esp_idf_svc::http::Method::Get, move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
            let s = state_get.get_settings();
            let json = format!(
                r#"{{"acc":{:.2},"acc_exit":{:.2},"brake":{:.2},"brake_exit":{:.2},"lat":{:.2},"lat_exit":{:.2},"yaw":{:.3},"min_speed":{:.1}}}"#,
                s.acc_thr, s.acc_exit, s.brake_thr, s.brake_exit, s.lat_thr, s.lat_exit, s.yaw_thr, s.min_speed
            );
            let content_length = json.len().to_string();
            let mut response = req.into_response(
                200,
                None,
                &[
                    ("Content-Type", "application/json"),
                    ("Content-Length", &content_length),
                    ("Connection", "close"),
                    ("Access-Control-Allow-Origin", "*"),
                ],
            )?;
            response.write_all(json.as_bytes())?;
            Ok(())
        })?;

        // Update settings via query params
        let state_set = state.clone();
        server.fn_handler("/api/settings/set", esp_idf_svc::http::Method::Get, move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
            let uri = req.uri();
            let mut settings = state_set.get_settings();

            // Parse query parameters
            if let Some(query) = uri.split('?').nth(1) {
                for param in query.split('&') {
                    let mut parts = param.split('=');
                    if let (Some(key), Some(val)) = (parts.next(), parts.next()) {
                        if let Ok(v) = val.parse::<f32>() {
                            match key {
                                "acc" => settings.acc_thr = v,
                                "acc_exit" => settings.acc_exit = v,
                                "brake" => settings.brake_thr = v,
                                "brake_exit" => settings.brake_exit = v,
                                "lat" => settings.lat_thr = v,
                                "lat_exit" => settings.lat_exit = v,
                                "yaw" => settings.yaw_thr = v,
                                "min_speed" => settings.min_speed = v,
                                _ => {}
                            }
                        }
                    }
                }
            }

            state_set.set_settings(settings);
            info!("Settings updated: acc={:.2}, acc_exit={:.2}, brake={:.2}, brake_exit={:.2}, lat={:.2}, lat_exit={:.2}, yaw={:.3}, min_speed={:.1}",
                settings.acc_thr, settings.acc_exit, settings.brake_thr, settings.brake_exit, settings.lat_thr, settings.lat_exit, settings.yaw_thr, settings.min_speed);

            let json = format!(
                r#"{{"status":"ok","acc":{:.2},"acc_exit":{:.2},"brake":{:.2},"brake_exit":{:.2},"lat":{:.2},"lat_exit":{:.2},"yaw":{:.3},"min_speed":{:.1}}}"#,
                settings.acc_thr, settings.acc_exit, settings.brake_thr, settings.brake_exit, settings.lat_thr, settings.lat_exit, settings.yaw_thr, settings.min_speed
            );
            let content_length = json.len().to_string();
            let mut response = req.into_response(
                200,
                None,
                &[
                    ("Content-Type", "application/json"),
                    ("Content-Length", &content_length),
                    ("Connection", "close"),
                    ("Access-Control-Allow-Origin", "*"),
                ],
            )?;
            response.write_all(json.as_bytes())?;
            Ok(())
        })?;

        // Diagnostics API endpoint (JSON)
        let state_diag_api = state.clone();
        server.fn_handler(
            "/api/diagnostics",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let json = if let Some(diag_state) = state_diag_api.diagnostics() {
                    crate::telemetry_http::diagnostics_json(&diag_state.snapshot())
                } else {
                    r#"{"error":"diagnostics not available"}"#.to_string()
                };

                let content_length = json.len().to_string();
                let mut response = req.into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Content-Length", &content_length),
                        ("Access-Control-Allow-Origin", "*"),
                        ("Cache-Control", "no-cache"),
                    ],
                )?;
                response.write_all(json.as_bytes())?;
                Ok(())
            },
        )?;

        // Lap timer configuration endpoint
        // GET /api/laptimer/configure?type=loop&p1_x=...&p1_y=...&p2_x=...&p2_y=...&
        // dir=... GET /api/laptimer/configure?type=clear
        // GET /api/laptimer/configure?type=point_to_point&p1_x=...&p1_y=...&p2_x=...&
        // p2_y=...&dir=...&f_p1_x=...&f_p1_y=...&f_p2_x=...&f_p2_y=...&f_dir=...
        let state_laptimer = state.clone();
        server.fn_handler(
            "/api/laptimer/configure",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let uri = req.uri();
                let (json, status) = parse_lap_timer_config(uri, &state_laptimer);

                let content_length = json.len().to_string();
                let mut response = req.into_response(
                    status,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Content-Length", &content_length),
                        ("Access-Control-Allow-Origin", "*"),
                    ],
                )?;
                response.write_all(json.as_bytes())?;
                Ok(())
            },
        )?;

        info!("Telemetry server ready on port {}", port);

        Ok(Self {
            _server: server,
            state,
        })
    }

    /// Get reference to shared state for updating telemetry
    pub fn state(&self) -> Arc<TelemetryServerState> {
        self.state.clone()
    }
}
