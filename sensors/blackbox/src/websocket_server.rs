use std::sync::{
    atomic::{AtomicBool, AtomicU32, Ordering},
    Arc, Mutex,
};

/// HTTP telemetry server for dashboard and settings
/// Uses HTTP polling for reliable updates without thread blocking
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::Write;
use log::info;

use crate::diagnostics::DiagnosticsState;

/// Lap timer timing line configuration
#[derive(Clone, Copy, Debug)]
pub struct TimingLineConfig {
    pub p1_x: f32,
    pub p1_y: f32,
    pub p2_x: f32,
    pub p2_y: f32,
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

/// Shared state for telemetry server
pub struct TelemetryServerState {
    /// Latest telemetry packet (raw bytes)
    telemetry_data: Mutex<Vec<u8>>,
    /// Packet counter for clients to detect updates
    packet_counter: AtomicU32,
    /// Calibration requested flag
    calibration_requested: AtomicBool,
    /// Mode detection settings
    mode_settings: Mutex<ModeSettings>,
    /// Settings changed flag
    settings_changed: AtomicBool,
    /// Optional diagnostics state (shared with main loop)
    diagnostics_state: Option<Arc<DiagnosticsState>>,
    /// Lap timer configuration
    lap_timer_config: Mutex<LapTimerConfig>,
    /// Lap timer config changed flag
    lap_timer_changed: AtomicBool,
}

impl TelemetryServerState {
    pub fn new() -> Self {
        Self {
            telemetry_data: Mutex::new(Vec::with_capacity(128)),
            packet_counter: AtomicU32::new(0),
            calibration_requested: AtomicBool::new(false),
            mode_settings: Mutex::new(ModeSettings::default()),
            settings_changed: AtomicBool::new(false),
            diagnostics_state: None,
            lap_timer_config: Mutex::new(LapTimerConfig::default()),
            lap_timer_changed: AtomicBool::new(false),
        }
    }

    /// Create with diagnostics state for diagnostics page support
    pub fn with_diagnostics(diagnostics: Arc<DiagnosticsState>) -> Self {
        Self {
            telemetry_data: Mutex::new(Vec::with_capacity(128)),
            packet_counter: AtomicU32::new(0),
            calibration_requested: AtomicBool::new(false),
            mode_settings: Mutex::new(ModeSettings::default()),
            settings_changed: AtomicBool::new(false),
            diagnostics_state: Some(diagnostics),
            lap_timer_config: Mutex::new(LapTimerConfig::default()),
            lap_timer_changed: AtomicBool::new(false),
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

    /// Set lap timer configuration
    pub fn set_lap_timer_config(&self, config: LapTimerConfig) {
        if let Ok(mut guard) = self.lap_timer_config.lock() {
            *guard = config;
            self.lap_timer_changed.store(true, Ordering::SeqCst);
        }
    }

    /// Check if lap timer config was changed and get the new config
    pub fn take_lap_timer_config(&self) -> Option<LapTimerConfig> {
        if self.lap_timer_changed.swap(false, Ordering::SeqCst) {
            self.lap_timer_config.lock().ok().map(|g| g.clone())
        } else {
            None
        }
    }

    /// Update telemetry data
    pub fn update_telemetry(&self, data: &[u8]) {
        if let Ok(mut guard) = self.telemetry_data.lock() {
            guard.clear();
            guard.extend_from_slice(data);
            self.packet_counter.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// Get current telemetry data as base64 for JSON transport
    pub fn get_telemetry_base64(&self) -> Option<String> {
        self.telemetry_data
            .lock()
            .ok()
            .map(|data| base64_encode(&data))
    }

    /// Get raw telemetry bytes (kept for potential future use)
    #[allow(dead_code)]
    pub fn get_telemetry_raw(&self) -> Option<Vec<u8>> {
        self.telemetry_data.lock().ok().map(|g| g.clone())
    }

    /// Get packet counter
    pub fn packet_count(&self) -> u32 {
        self.packet_counter.load(Ordering::SeqCst)
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
            // Parse timing line parameters
            let p1_x = params.get("p1_x").and_then(|v| v.parse::<f32>().ok());
            let p1_y = params.get("p1_y").and_then(|v| v.parse::<f32>().ok());
            let p2_x = params.get("p2_x").and_then(|v| v.parse::<f32>().ok());
            let p2_y = params.get("p2_y").and_then(|v| v.parse::<f32>().ok());
            let dir = params.get("dir").and_then(|v| v.parse::<f32>().ok());

            match (p1_x, p1_y, p2_x, p2_y, dir) {
                (Some(p1_x), Some(p1_y), Some(p2_x), Some(p2_y), Some(direction)) => {
                    let line = TimingLineConfig {
                        p1_x,
                        p1_y,
                        p2_x,
                        p2_y,
                        direction,
                    };
                    state.set_lap_timer_config(LapTimerConfig::Loop(line));
                    info!(
                        "Lap timer configured: loop track, line ({:.1},{:.1})-({:.1},{:.1}) dir={:.2}",
                        p1_x, p1_y, p2_x, p2_y, direction
                    );
                    (
                        format!(
                            r#"{{"status":"ok","config":"loop","line":{{"p1":[{},{}],"p2":[{},{}],"dir":{}}}}}"#,
                            p1_x, p1_y, p2_x, p2_y, direction
                        ),
                        200,
                    )
                }
                _ => (
                    r#"{"error":"missing parameters for loop config (p1_x, p1_y, p2_x, p2_y, dir)"}"#
                        .to_string(),
                    400,
                ),
            }
        }
        "point_to_point" => {
            // Parse start line
            let p1_x = params.get("p1_x").and_then(|v| v.parse::<f32>().ok());
            let p1_y = params.get("p1_y").and_then(|v| v.parse::<f32>().ok());
            let p2_x = params.get("p2_x").and_then(|v| v.parse::<f32>().ok());
            let p2_y = params.get("p2_y").and_then(|v| v.parse::<f32>().ok());
            let dir = params.get("dir").and_then(|v| v.parse::<f32>().ok());
            // Parse finish line (f_ prefix)
            let f_p1_x = params.get("f_p1_x").and_then(|v| v.parse::<f32>().ok());
            let f_p1_y = params.get("f_p1_y").and_then(|v| v.parse::<f32>().ok());
            let f_p2_x = params.get("f_p2_x").and_then(|v| v.parse::<f32>().ok());
            let f_p2_y = params.get("f_p2_y").and_then(|v| v.parse::<f32>().ok());
            let f_dir = params.get("f_dir").and_then(|v| v.parse::<f32>().ok());

            match (
                p1_x, p1_y, p2_x, p2_y, dir, f_p1_x, f_p1_y, f_p2_x, f_p2_y, f_dir,
            ) {
                (
                    Some(p1_x),
                    Some(p1_y),
                    Some(p2_x),
                    Some(p2_y),
                    Some(direction),
                    Some(f_p1_x),
                    Some(f_p1_y),
                    Some(f_p2_x),
                    Some(f_p2_y),
                    Some(f_direction),
                ) => {
                    let start = TimingLineConfig {
                        p1_x,
                        p1_y,
                        p2_x,
                        p2_y,
                        direction,
                    };
                    let finish = TimingLineConfig {
                        p1_x: f_p1_x,
                        p1_y: f_p1_y,
                        p2_x: f_p2_x,
                        p2_y: f_p2_y,
                        direction: f_direction,
                    };
                    state.set_lap_timer_config(LapTimerConfig::PointToPoint { start, finish });
                    info!(
                        "Lap timer configured: point-to-point, start ({:.1},{:.1})-({:.1},{:.1}), finish ({:.1},{:.1})-({:.1},{:.1})",
                        p1_x, p1_y, p2_x, p2_y, f_p1_x, f_p1_y, f_p2_x, f_p2_y
                    );
                    (
                        format!(
                            r#"{{"status":"ok","config":"point_to_point","start":{{"p1":[{},{}],"p2":[{},{}],"dir":{}}},"finish":{{"p1":[{},{}],"p2":[{},{}],"dir":{}}}}}"#,
                            p1_x, p1_y, p2_x, p2_y, direction,
                            f_p1_x, f_p1_y, f_p2_x, f_p2_y, f_direction
                        ),
                        200,
                    )
                }
                _ => (
                    r#"{"error":"missing parameters for point_to_point config (start: p1_x, p1_y, p2_x, p2_y, dir; finish: f_p1_x, f_p1_y, f_p2_x, f_p2_y, f_dir)"}"#
                        .to_string(),
                    400,
                ),
            }
        }
        _ => (
            format!(r#"{{"error":"unknown type '{}', expected: clear, loop, point_to_point"}}"#, config_type),
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
const DASHBOARD_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="default"><title>Blackbox</title>
<style>
:root{
    --bg:#f2f2f7;--surface:#ffffff;--text:#1c1c1e;--text-secondary:#48484a;--text-tertiary:#8e8e93;
    --divider:rgba(0,0,0,.08);--ok:#34c759;--amber:#ff9500;--red:#ff3b30;
    --mode-idle:#8e8e93;--mode-accel:#1c1c1e;--mode-brake:#ff3b30;
}
.dark{
    --bg:#000000;--surface:#1c1c1e;--text:#ffffff;--text-secondary:#aeaeb2;--text-tertiary:#636366;
    --divider:rgba(255,255,255,.08);--mode-idle:#636366;--mode-accel:#ffffff;--mode-brake:#ff3b30;
}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,'SF Pro Text','SF Pro Display',system-ui,sans-serif;background:var(--bg);color:var(--text);height:100vh;height:100dvh;display:flex;flex-direction:column;-webkit-user-select:none;user-select:none;overflow:hidden;transition:background 0.3s,color 0.3s}
.bbNum{font-variant-numeric:tabular-nums;font-feature-settings:"tnum" 1,"lnum" 1}
.bbTopbar{display:flex;justify-content:space-between;align-items:center;padding:14px 16px;background:transparent;flex-shrink:0}
.bbBrandToggle{background:none;border:0;padding:0;display:flex;align-items:baseline;gap:8px;cursor:pointer;color:inherit;font-family:inherit}
.bbBrand{font-size:11px;font-weight:600;opacity:.55;letter-spacing:.03em}
.bbModeLabel{font-size:11px;font-weight:400;opacity:.40;letter-spacing:.03em}
.bbModeIcon{font-size:10px;opacity:.40}
.bbTopRight{display:flex;align-items:center;gap:10px}
.bbStatusCluster{display:flex;align-items:center;gap:12px;opacity:.55;font-size:10px;letter-spacing:.03em}
.bbStatusItem{display:flex;align-items:center;gap:4px}
.bbDot{width:6px;height:6px;border-radius:50%;background:var(--text-tertiary);display:inline-block;opacity:.9;margin-right:2px;transition:all 0.3s}
.bbDot.on{background:var(--ok)}
.bbDot.rec{background:var(--red);animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.4}}
.bbHigh{color:var(--ok);font-weight:600}
.bbDiag{color:var(--text-tertiary);text-decoration:none;font-size:10px;opacity:.7;cursor:pointer}
.bbDiag:hover{opacity:1}
.bbBack{color:var(--text-tertiary);text-decoration:none;font-size:12px;font-weight:500;letter-spacing:.03em;padding:8px 14px;background:var(--surface);border-radius:16px;display:flex;align-items:center;gap:4px;cursor:pointer}
.bbBack:active{opacity:.7}
.bbKebab{background:none;border:0;padding:4px;font-size:24px;line-height:1;opacity:.6;cursor:pointer;color:inherit}
.bbApp{flex:1;display:flex;flex-direction:column;padding:0 12px 8px;min-height:0;overflow-y:auto}
.bbCard{background:var(--surface);border-radius:16px;overflow:hidden}
.bbGCard{flex:1;display:flex;flex-direction:column;margin-bottom:4px}
.bbGTop{display:flex;justify-content:space-between;align-items:flex-start;padding:12px 18px 6px;flex-shrink:0}
.bbSpeedLine{display:flex;align-items:baseline;gap:4px}
.bbSpeed{font-size:22px;font-weight:600;opacity:.75;transition:color 0.3s,opacity 0.3s}
.bbSpeed.peak{color:var(--amber);opacity:1}
.bbUnit{font-size:12px;opacity:.45}
.bbManeuver{font-size:14px;font-weight:600;margin-top:4px;min-width:7ch;transition:color 0.15s}
.bbGTopRight{font-size:11px;opacity:.50}
.bbGPlotWrap{display:flex;justify-content:center;padding:2px 18px 0}
.bbGPlotFrame{position:relative;width:min(560px, 100%);aspect-ratio:1/1}
.bbGPlot{display:block;width:100%;height:100%}
.bbAxis{position:absolute;font-size:10px;font-weight:500;letter-spacing:.02em;text-transform:uppercase;opacity:.50;color:var(--text-tertiary);pointer-events:none}
.bbAxis--top{top:4px;left:50%;transform:translateX(-50%)}
.bbAxis--bottom{bottom:4px;left:50%;transform:translateX(-50%)}
.bbAxis--left{left:6px;top:50%;transform:translateY(-50%)}
.bbAxis--right{right:6px;top:50%;transform:translateY(-50%)}
.bbPlotMeta{position:absolute;top:6px;right:8px;font-size:10px;color:var(--text-tertiary);opacity:.50}
.bbGReadoutRow{display:flex;justify-content:center;align-items:center;gap:0;padding:16px 18px;flex-shrink:0;margin:auto 0}
.bbGReadoutCol{flex:1;display:flex;flex-direction:column;align-items:center;text-align:center}
.bbGReadoutLbl{font-size:11px;letter-spacing:.02em;text-transform:uppercase;opacity:.55;color:var(--text-tertiary)}
.bbGReadoutVal{margin-top:8px;font-size:34px;font-weight:600;line-height:1.0;min-width:8ch;display:inline-flex;align-items:baseline;justify-content:center;gap:2px}
.bbSign{display:inline-block;width:1.2ch;text-align:right;opacity:.9;margin-right:2px}
.bbGUnit{font-size:.5em;opacity:.55;margin-left:2px}
.bbGDivider{width:1px;height:48px;background:var(--divider);margin:0 12px;flex-shrink:0}
.bbGMax{display:grid;grid-template-columns:1fr 1fr;gap:8px 16px;padding:16px 24px 20px;border-top:1px solid var(--divider);flex-shrink:0}
.bbMini{display:grid;grid-template-columns:auto 7ch;column-gap:10px;align-items:baseline}
.bbMiniLbl{font-size:11px;color:var(--text-tertiary);opacity:.75;white-space:nowrap}
.bbMiniVal{font-size:13px;font-weight:600;color:var(--text-secondary);text-align:left;justify-self:start}
.bbTelemetryStrip{display:flex;gap:14px;justify-content:center;padding:14px 0 10px;font-size:11px;color:var(--text-tertiary);flex-shrink:0}
.bbTItem{display:flex;align-items:baseline;gap:3px}
.bbTItem .bbNum{color:var(--text-secondary);font-weight:500;display:inline-block;text-align:right}
.bbNum--hz{min-width:2.5ch}
.bbNum--gps{min-width:2ch}
.bbMenuOverlay{position:fixed;inset:0;background:rgba(0,0,0,0.25);opacity:0;visibility:hidden;transition:opacity 0.15s ease;z-index:100}
.dark .bbMenuOverlay{background:rgba(0,0,0,0.5)}
.bbMenuOverlay.open{opacity:1;visibility:visible}
.bbMenuPanel{position:fixed;top:44px;right:12px;background:var(--surface);border-radius:12px;box-shadow:0 4px 20px rgba(0,0,0,0.15);min-width:180px;transform:scale(0.95);opacity:0;transition:transform 0.15s ease,opacity 0.15s ease;z-index:101;overflow:hidden}
.dark .bbMenuPanel{box-shadow:0 4px 20px rgba(0,0,0,0.4)}
.bbMenuOverlay.open .bbMenuPanel{transform:scale(1);opacity:1}
.bbMenuItem{display:block;width:100%;padding:14px 18px;font-size:15px;text-align:left;border:none;background:none;color:var(--text);cursor:pointer;border-bottom:0.5px solid var(--divider);font-family:inherit}
.bbMenuItem:last-child{border-bottom:none}
.bbMenuItem:active{background:var(--bg)}
.bbMenuItem.destructive{color:var(--red)}
.view{display:none}.view.active{display:flex;flex-direction:column;flex:1;min-height:0}
.bbDiagApp{flex:1;display:flex;flex-direction:column;padding:0 12px 20px;overflow-y:auto}
.bbDiagCard{background:var(--surface);border-radius:16px;padding:14px 16px;margin-bottom:10px}
.bbCardHead{font-size:10px;font-weight:600;color:var(--text-tertiary);text-transform:uppercase;letter-spacing:.08em;margin-bottom:10px}
.bbRow{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid var(--divider)}
.bbRow:last-child{border-bottom:none}
.bbLbl{color:var(--text-secondary);font-size:12px;font-weight:500}
.bbVal{color:var(--text);font-weight:600;font-size:13px}
.ok{color:var(--ok)}.warn{color:var(--amber)}.err{color:var(--red)}
.bbGrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:500px){.bbGrid{grid-template-columns:1fr}}
.bbUptime{text-align:center;margin-top:6px;padding:14px;font-size:12px;font-weight:500;color:var(--text-tertiary);letter-spacing:.03em;background:var(--surface);border-radius:16px}
.bbRecBtn{display:inline-flex;align-items:center;gap:8px;padding:10px 16px;background:var(--surface);border:none;border-radius:12px;font-size:13px;font-weight:600;color:var(--text);cursor:pointer;font-family:inherit;margin-right:8px}
.bbRecBtn:active{opacity:.7}
.bbRecBtn.recording{color:var(--red)}
.bbLapCard{background:var(--surface);border-radius:16px;margin-bottom:8px;overflow:hidden}
.bbLapCard.inactive{padding:14px 18px}
.bbLapCard.active{padding:0}
.bbLapSetup{display:flex;justify-content:space-between;align-items:center}
.bbLapSetupText{font-size:13px;color:var(--text-tertiary)}
.bbLapSetupBtn{background:var(--bg);border:none;border-radius:8px;padding:8px 16px;font-size:13px;font-weight:600;color:var(--text);cursor:pointer;font-family:inherit}
.bbLapSetupBtn:active{opacity:0.7}
.bbLapActive{display:none;cursor:pointer;position:relative}
.bbLapCard.active .bbLapActive{display:block}
.bbLapStop{position:absolute;top:8px;right:8px;width:28px;height:28px;border-radius:50%;border:none;background:rgba(128,128,128,0.1);color:var(--text-tertiary);font-size:18px;font-weight:300;cursor:pointer;display:flex;align-items:center;justify-content:center;z-index:10;transition:all 0.15s}
.bbLapStop:hover{background:rgba(255,59,48,0.12);color:#ff3b30}
.bbLapStop:active{transform:scale(0.95);background:rgba(255,59,48,0.2);color:#ff3b30}
.bbLapCard.active .bbLapSetup{display:none}
.bbLapMain{display:flex;flex-direction:column;align-items:center;padding:16px 18px 12px}
.bbLapTime{font-size:44px;font-weight:600;line-height:1}
.bbLapMeta{display:flex;align-items:center;gap:12px;margin-top:6px}
.bbLapTrackName{font-size:12px;font-weight:500;opacity:0.45;margin-top:4px;letter-spacing:0.02em;max-width:200px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.bbLapCount{font-size:17px;font-weight:600;color:var(--text);opacity:0.6}
.bbLapState{font-size:11px;text-transform:uppercase;letter-spacing:0.04em;color:var(--text-tertiary);opacity:0.5}
.bbLapState.timing{color:var(--ok);opacity:0.9}
.bbLapState.finished{color:var(--ok);opacity:1;animation:finishedPulse 0.5s ease-in-out 2}
@keyframes finishedPulse{0%,100%{transform:scale(1)}50%{transform:scale(1.1)}}
.bbLapCard.first-run .bbLapMeta::after{content:'NEW';display:inline-block;margin-left:8px;padding:2px 6px;background:var(--amber);color:#fff;font-size:9px;font-weight:700;border-radius:4px;letter-spacing:0.05em}
.bbLapHistory{display:flex;justify-content:center;gap:0;padding:12px 18px;border-top:1px solid var(--divider)}
.bbLapHistItem{flex:1;display:flex;flex-direction:column;align-items:center}
.bbLapHistDivider{width:1px;height:36px;background:var(--divider)}
.bbLapHistLabel{font-size:11px;text-transform:uppercase;letter-spacing:0.02em;color:var(--text-tertiary);opacity:0.55}
.bbLapHistValue{font-size:20px;font-weight:600;margin-top:4px}
.bbLapHistValue.best{color:var(--ok)}
.bbLapHistValue.delta{font-size:18px}
.bbLapHistValue.delta.faster{color:var(--ok)}
.bbLapHistValue.delta.slower{color:var(--red)}
.bbLapFlash{animation:lapFlash 0.8s ease-out}
@keyframes lapFlash{0%{background:rgba(52,199,89,0.35);transform:scale(1.01)}15%{background:rgba(52,199,89,0.45);transform:scale(1.02)}100%{background:var(--surface);transform:scale(1)}}
.bbLapBestFlash{animation:bestFlash 0.8s ease-out}
@keyframes bestFlash{0%,30%{transform:scale(1.05)}100%{transform:scale(1)}}
.bbStartLineIndicator{display:none;padding:10px 16px;border-top:1px solid var(--divider);text-align:center}
.bbLapCard.active .bbStartLineIndicator{display:block}
.bbLapCard.active.timing .bbStartLineIndicator{display:none}
.bbStartLineText{font-size:14px;color:var(--text-secondary);font-weight:500}
.bbStartLineText.approaching{color:var(--amber)}
.bbStartLineText.close{color:var(--ok)}
.bbStartLineText.at-line{color:var(--ok);font-weight:600;animation:pulseText 1s ease-in-out infinite}
@keyframes pulseText{0%,100%{opacity:1}50%{opacity:0.5}}
.bbFinishLineIndicator{display:none;padding:10px 16px;border-top:1px solid var(--divider);text-align:center}
.bbLapCard.active.timing.p2p .bbFinishLineIndicator{display:block}
.bbFinishLineText{font-size:14px;color:var(--text-secondary);font-weight:500}
.bbFinishLineText.approaching{color:var(--amber)}
.bbFinishLineText.close{color:var(--ok)}
.bbFinishLineText.at-line{color:var(--ok);font-weight:600}
.bbModal{position:fixed;inset:0;background:rgba(0,0,0,0.4);display:flex;align-items:center;justify-content:center;opacity:0;visibility:hidden;transition:opacity 0.2s ease;z-index:200;padding:16px}
.dark .bbModal{background:rgba(0,0,0,0.6)}
.bbModal.open{opacity:1;visibility:visible}
.bbModalContent{background:var(--surface);border-radius:16px;width:100%;max-width:400px;max-height:calc(100vh - 32px);overflow:hidden;display:flex;flex-direction:column;transform:scale(0.95);transition:transform 0.2s ease}
.bbModal.open .bbModalContent{transform:scale(1)}
.bbModalHeader{display:flex;justify-content:space-between;align-items:center;padding:16px 18px;border-bottom:1px solid var(--divider)}
.bbModalTitle{font-size:17px;font-weight:600}
.bbModalClose{background:none;border:none;font-size:24px;color:var(--text-tertiary);cursor:pointer;padding:0;line-height:1}
.bbModalClose:active{opacity:0.5}
.bbModalBody{padding:16px 18px;overflow-y:auto;flex:1}
.bbModalSection{margin-bottom:20px}
.bbModalSection:last-child{margin-bottom:0}
.bbSectionTitle{font-size:11px;font-weight:600;color:var(--text-tertiary);text-transform:uppercase;letter-spacing:0.08em;margin-bottom:10px}
.bbPosDisplay{background:var(--bg);border-radius:12px;padding:14px 16px}
.bbPosRow{display:flex;justify-content:space-between;align-items:baseline;padding:4px 0}
.bbPosLabel{font-size:12px;color:var(--text-tertiary)}
.bbPosValue{font-size:14px;font-weight:600;font-variant-numeric:tabular-nums}
.bbPosValue.waiting{color:var(--text-tertiary);font-weight:400}
.bbActionBtn{display:block;width:100%;padding:14px 18px;border:none;border-radius:12px;font-size:15px;font-weight:600;cursor:pointer;font-family:inherit;margin-bottom:10px;transition:opacity 0.15s}
.bbActionBtn:last-child{margin-bottom:0}
.bbActionBtn:active{opacity:0.7}
.bbActionBtn.primary{background:var(--ok);color:#fff}
.bbActionBtn.secondary{background:var(--bg);color:var(--text)}
.bbActionBtn:disabled{opacity:0.4;cursor:not-allowed}
.bbTrackList{border-radius:12px;overflow:hidden;background:var(--bg)}
.bbTrackEmpty{padding:20px;text-align:center;color:var(--text-tertiary);font-size:13px}
.bbTrackItem{display:flex;justify-content:space-between;align-items:center;padding:12px 14px;border-bottom:1px solid var(--divider)}
.bbTrackItem:last-child{border-bottom:none}
.bbTrackInfo{flex:1;min-width:0}
.bbTrackName{font-size:14px;font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.bbTrackMeta{font-size:11px;color:var(--text-tertiary);margin-top:2px}
.bbTrackMeta span{margin-right:10px}
.bbTrackActions{display:flex;gap:8px;margin-left:10px}
.bbTrackBtn{background:var(--surface);border:none;border-radius:6px;padding:6px 12px;font-size:12px;font-weight:600;color:var(--text);cursor:pointer;font-family:inherit}
.bbTrackBtn:active{opacity:0.7}
.bbTrackBtn.primary{background:var(--ok);color:#fff}
.bbTrackBtn.danger{color:var(--red)}
.bbActiveTrack{display:flex;align-items:center;gap:12px;background:rgba(52,199,89,0.1);border:1px solid rgba(52,199,89,0.2);border-radius:12px;padding:12px 14px;margin-bottom:16px}
.bbActiveTrackInfo{flex:1;min-width:0}
.bbActiveTrackName{font-size:15px;font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.bbActiveTrackMeta{font-size:12px;color:var(--text-secondary);margin-top:2px}
.bbActiveTrackClear{width:28px;height:28px;border-radius:50%;border:none;background:rgba(128,128,128,0.12);color:var(--text-tertiary);font-size:16px;cursor:pointer;display:flex;align-items:center;justify-content:center;flex-shrink:0}
.bbActiveTrackClear:active{background:rgba(255,59,48,0.15);color:#ff3b30}
.bbCreateTrackHint{font-size:13px;color:var(--text-secondary);margin:0 0 16px 0;line-height:1.5;text-align:center}
.bbCreateTrackBtns{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.bbRecordStartBtn{display:flex;flex-direction:column;align-items:center;padding:20px 14px 18px;border:2px solid var(--divider);border-radius:16px;background:linear-gradient(180deg, var(--surface) 0%, var(--bg) 100%);cursor:pointer;transition:all 0.2s ease;font-family:inherit;position:relative;overflow:hidden}
.bbRecordStartBtn::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:linear-gradient(90deg, transparent, #ff3b30, transparent);opacity:0;transition:opacity 0.2s}
.bbRecordStartBtn:active{transform:scale(0.97);border-color:#ff3b30}
.bbRecordStartIconWrap{width:48px;height:48px;border-radius:50%;background:rgba(255,59,48,0.1);display:flex;align-items:center;justify-content:center;margin-bottom:12px}
.bbRecordStartIcon{color:#ff3b30}
.bbRecordStartLabel{font-size:14px;font-weight:600;color:var(--text);margin-bottom:4px}
.bbRecordStartDesc{font-size:11px;color:var(--text-tertiary);text-align:center;line-height:1.35}
.bbRecordOverlay{position:fixed;bottom:0;left:0;right:0;background:var(--surface);padding:24px 20px;padding-bottom:calc(24px + env(safe-area-inset-bottom, 0px));border-radius:24px 24px 0 0;box-shadow:0 -8px 32px rgba(0,0,0,0.18);z-index:250;transform:translateY(100%);transition:transform 0.35s cubic-bezier(0.32, 0.72, 0, 1);border-top:3px solid #ff3b30}
.bbRecordOverlay.active{transform:translateY(0)}
.bbRecordState{animation:fadeIn 0.25s ease-out}
@keyframes fadeIn{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
.bbRecordHeader{display:flex;align-items:center;justify-content:center;gap:10px;margin-bottom:16px}
.bbRecordPulse{width:12px;height:12px;background:#ff3b30;border-radius:50%;animation:recPulse 1.2s ease-in-out infinite}
@keyframes recPulse{0%,100%{transform:scale(1);opacity:1;box-shadow:0 0 0 0 rgba(255,59,48,0.5)}50%{transform:scale(0.85);opacity:0.7;box-shadow:0 0 0 8px rgba(255,59,48,0)}}
.bbRecordTitle{font-size:14px;font-weight:600;color:var(--text-secondary);letter-spacing:0.02em;text-transform:uppercase}
.bbRecordHero{text-align:center;padding:16px 0 20px}
.bbRecordHeroValue{font-size:72px;font-weight:700;letter-spacing:-0.04em;line-height:1;font-variant-numeric:tabular-nums;color:var(--text)}
.bbRecordHeroUnit{font-size:12px;font-weight:600;color:var(--text-tertiary);margin-top:6px;text-transform:uppercase;letter-spacing:0.1em}
.bbRecordMeta{display:flex;justify-content:center;align-items:stretch;gap:0;margin-bottom:16px;padding:12px 16px;background:var(--bg);border-radius:12px}
.bbRecordMetaItem{flex:1;text-align:center;display:flex;flex-direction:column;justify-content:center}
.bbRecordMetaValue{font-size:18px;font-weight:700;font-variant-numeric:tabular-nums;color:var(--text);display:block;line-height:1.2}
.bbRecordMetaLabel{font-size:9px;color:var(--text-tertiary);text-transform:uppercase;letter-spacing:0.06em;margin-top:3px;font-weight:500}
.bbRecordMetaDivider{width:1px;height:auto;background:var(--divider);margin:0 10px;align-self:stretch}
.bbRecordQuality{display:flex;align-items:center;gap:10px;margin-bottom:14px;padding:10px 14px;background:var(--bg);border-radius:10px}
.bbRecordQualityBar{flex:1;height:6px;background:rgba(128,128,128,0.15);border-radius:3px;overflow:hidden}
.bbRecordQualityFill{height:100%;background:var(--ok);transition:width 0.3s ease-out, background 0.3s;border-radius:3px}
.bbRecordQualityFill.fair{background:var(--amber)}
.bbRecordQualityFill.poor{background:var(--red)}
.bbRecordQualityText{font-size:11px;font-weight:600;color:var(--text-secondary);min-width:60px;text-align:right;text-transform:uppercase;letter-spacing:0.02em}
.bbRecordHint{text-align:center;font-size:13px;color:var(--text-tertiary);margin:0 0 18px 0;line-height:1.45}
.bbRecordActions{display:flex;gap:10px}
.bbRecordActionBtn{flex:1;padding:14px 20px;border:none;border-radius:12px;font-size:15px;font-weight:600;cursor:pointer;font-family:inherit;text-align:center;justify-content:center;align-items:center;transition:transform 0.1s, opacity 0.15s}
.bbRecordActionBtn:active{transform:scale(0.97)}
.bbRecordActionBtn.secondary{background:rgba(128,128,128,0.12);color:var(--text)}
.bbRecordActionBtn.primary{background:var(--ok);color:#fff}
.bbRecordActionBtn.finish{background:#ff3b30;color:#fff}
.bbRecordSuccess{text-align:center;padding:24px 16px 20px}
.bbRecordSuccessIcon{width:72px;height:72px;margin:0 auto 16px;background:linear-gradient(135deg, rgba(52,199,89,0.15) 0%, rgba(52,199,89,0.08) 100%);border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:36px;color:var(--ok);animation:successPop 0.4s cubic-bezier(0.175, 0.885, 0.32, 1.275)}
@keyframes successPop{0%{transform:scale(0.5);opacity:0}100%{transform:scale(1);opacity:1}}
.bbRecordSuccessTitle{font-size:24px;font-weight:700;color:var(--text);margin-bottom:8px;letter-spacing:-0.01em}
.bbRecordSuccessStats{font-size:14px;color:var(--text-secondary);line-height:1.4}
</style></head>
<body>

<!-- DASHBOARD VIEW -->
<div id="view-dashboard" class="view active">
<header class="bbTopbar">
    <button class="bbBrandToggle" id="brandToggle" aria-label="Toggle theme">
        <span class="bbBrand">Blackbox</span>
        <span class="bbModeLabel" id="modeLabel">Bright</span>
        <span class="bbModeIcon" aria-hidden="true">◐</span>
    </button>
    <div class="bbTopRight">
        <div class="bbStatusCluster">
            <span class="bbDiag" id="link-diag">DIAG</span>
            <span class="bbStatusItem"><span class="bbDot" id="recDot"></span><span id="recTxt"></span></span>
            <span class="bbStatusItem"><span class="bbDot" id="gpsDot"></span><span id="gpsHz">GPS --</span></span>
            <span class="bbStatusItem"><span class="bbDot" id="liveDot"></span><span id="stxt">--</span></span>
        </div>
        <button class="bbKebab" id="menu-btn" aria-label="Menu">⋮</button>
    </div>
</header>
<main class="bbApp">
    <section class="bbCard bbLapCard inactive" id="lap-section">
        <div class="bbLapSetup" id="lap-setup">
            <span class="bbLapSetupText" id="lap-setup-text">Tap to configure</span>
            <button class="bbLapSetupBtn" id="btn-tracks">Tracks</button>
        </div>
        <div class="bbLapActive" id="lap-active" onclick="openTrackModal()">
            <button class="bbLapStop" id="btn-lap-stop" onclick="event.stopPropagation();confirmStopTiming();" title="Stop timing">×</button>
            <div class="bbLapMain">
                <div class="bbLapTime bbNum" id="lap-time">0:00.000</div>
                <div class="bbLapTrackName" id="lap-track-name"></div>
                <div class="bbLapMeta">
                    <span class="bbLapCount" id="lap-count">Lap 0</span>
                    <span class="bbLapState" id="lap-state">Armed</span>
                </div>
            </div>
            <div class="bbLapHistory">
                <div class="bbLapHistItem">
                    <span class="bbLapHistLabel">Best</span>
                    <span class="bbLapHistValue bbNum" id="best-lap">—:——</span>
                </div>
                <div class="bbLapHistDivider"></div>
                <div class="bbLapHistItem">
                    <span class="bbLapHistLabel">Last</span>
                    <span class="bbLapHistValue bbNum" id="last-lap">—:——</span>
                </div>
                <div class="bbLapHistDivider"></div>
                <div class="bbLapHistItem">
                    <span class="bbLapHistLabel">Delta</span>
                    <span class="bbLapHistValue bbNum delta" id="lap-delta">—</span>
                </div>
            </div>
            <div class="bbStartLineIndicator" id="start-line-indicator">
                <span class="bbStartLineText" id="start-line-text">Calculating...</span>
            </div>
            <div class="bbFinishLineIndicator" id="finish-line-indicator">
                <span class="bbFinishLineText" id="finish-line-text">Finish ahead...</span>
            </div>
        </div>
    </section>
    <section class="bbCard bbGCard">
        <div class="bbGTop">
            <div class="bbGTopLeft">
                <div class="bbSpeedLine"><span class="bbSpeed bbNum" id="speed">0</span><span class="bbUnit">km/h</span></div>
                <div class="bbManeuver" id="maneuver">IDLE</div>
            </div>
            <div class="bbGTopRight"><span class="bbNum" id="session-time">0:00</span></div>
        </div>
        <div class="bbGPlotWrap">
            <div class="bbGPlotFrame">
                <div class="bbAxis bbAxis--top">Accel</div>
                <div class="bbAxis bbAxis--bottom">Brake</div>
                <div class="bbAxis bbAxis--left">Left</div>
                <div class="bbAxis bbAxis--right">Right</div>
                <div class="bbPlotMeta" id="range-val">Range ±0.5g</div>
                <canvas class="bbGPlot" id="gcanvas"></canvas>
            </div>
        </div>
        <div class="bbGReadoutRow">
            <div class="bbGReadoutCol">
                <div class="bbGReadoutLbl">Lat</div>
                <div class="bbGReadoutVal bbNum" id="lat-g"><span class="bbSign">+</span><span class="bbVal">0.00</span><span class="bbGUnit">g</span></div>
            </div>
            <div class="bbGDivider"></div>
            <div class="bbGReadoutCol">
                <div class="bbGReadoutLbl">Long</div>
                <div class="bbGReadoutVal bbNum" id="lon-g"><span class="bbSign">+</span><span class="bbVal">0.00</span><span class="bbGUnit">g</span></div>
            </div>
        </div>
        <div class="bbGMax">
            <div class="bbMini"><span class="bbMiniLbl">Max Lat L</span><span class="bbMiniVal bbNum" id="max-l">0.00g</span></div>
            <div class="bbMini"><span class="bbMiniLbl">Max Lat R</span><span class="bbMiniVal bbNum" id="max-r">0.00g</span></div>
            <div class="bbMini"><span class="bbMiniLbl">Max Acc</span><span class="bbMiniVal bbNum" id="max-a">0.00g</span></div>
            <div class="bbMini"><span class="bbMiniLbl">Max Brk</span><span class="bbMiniVal bbNum" id="max-b">0.00g</span></div>
        </div>
    </section>
    <div class="bbTelemetryStrip">
        <span class="bbTItem"><span class="bbNum bbNum--hz" id="hz">0</span> Hz</span>
        <span class="bbTItem">GPS <span class="bbNum bbNum--gps" id="gps-acc">--</span>m</span>
        <span class="bbTItem">Yaw <span class="bbNum" id="yaw">0</span>°</span>
    </div>
</main>
</div>

<!-- DIAGNOSTICS VIEW -->
<div id="view-diagnostics" class="view">
<header class="bbTopbar">
    <button class="bbBrandToggle" id="brandToggle2" aria-label="Toggle theme">
        <span class="bbBrand">Diagnostics</span>
        <span class="bbModeLabel" id="modeLabel2">Bright</span>
        <span class="bbModeIcon" aria-hidden="true">◐</span>
    </button>
    <span class="bbBack" id="link-dashboard">← Dashboard</span>
</header>
<main class="bbDiagApp">
    <div class="bbGrid">
    <section class="bbDiagCard">
        <div class="bbCardHead">Recording</div>
        <div class="bbRow"><span class="bbLbl">Status</span><span class="bbVal" id="rec-status">Inactive</span></div>
        <div class="bbRow"><span class="bbLbl">Duration</span><span class="bbVal bbNum" id="rec-duration">--</span></div>
        <div class="bbRow"><span class="bbLbl">Chunks Saved</span><span class="bbVal bbNum" id="rec-chunks">0</span></div>
        <div class="bbRow"><span class="bbLbl">Last Save</span><span class="bbVal bbNum" id="rec-lastsave">--</span></div>
        <div style="padding-top:12px;display:flex;gap:8px">
            <button class="bbRecBtn" id="diag-rec">Start Recording</button>
            <button class="bbRecBtn" id="diag-export">Export CSV</button>
        </div>
    </section>
    <section class="bbDiagCard">
        <div class="bbCardHead">Sensors</div>
        <div class="bbRow"><span class="bbLbl">IMU Rate</span><span class="bbVal bbNum" id="imu-rate">--</span></div>
        <div class="bbRow"><span class="bbLbl">GPS Rate</span><span class="bbVal bbNum" id="gps-rate">--</span></div>
        <div class="bbRow"><span class="bbLbl">Loop Rate</span><span class="bbVal bbNum" id="loop-rate">--</span></div>
        <div class="bbRow"><span class="bbLbl">ZUPT Rate</span><span class="bbVal bbNum" id="zupt-rate">--</span></div>
        <div class="bbRow"><span class="bbLbl">EKF/GPS</span><span class="bbVal bbNum" id="ekf-per-gps">--</span></div>
    </section>
    </div>
    <div class="bbGrid">
    <section class="bbDiagCard">
        <div class="bbCardHead">GPS Status</div>
        <div class="bbRow"><span class="bbLbl">Fix</span><span class="bbVal" id="gps-fix">--</span></div>
        <div class="bbRow"><span class="bbLbl">Satellites</span><span class="bbVal bbNum" id="gps-sats">--</span></div>
        <div class="bbRow"><span class="bbLbl">HDOP</span><span class="bbVal bbNum" id="gps-hdop">--</span></div>
        <div class="bbRow"><span class="bbLbl">Warmup</span><span class="bbVal" id="gps-warmup">--</span></div>
    </section>
    <section class="bbDiagCard">
        <div class="bbCardHead">EKF Health</div>
        <div class="bbRow"><span class="bbLbl">Position σ</span><span class="bbVal bbNum" id="pos-sigma">--</span></div>
        <div class="bbRow"><span class="bbLbl">Velocity σ</span><span class="bbVal bbNum" id="vel-sigma">--</span></div>
        <div class="bbRow"><span class="bbLbl">Yaw σ</span><span class="bbVal bbNum" id="yaw-sigma">--</span></div>
        <div class="bbRow"><span class="bbLbl">Bias X</span><span class="bbVal bbNum" id="bias-x">--</span></div>
        <div class="bbRow"><span class="bbLbl">Bias Y</span><span class="bbVal bbNum" id="bias-y">--</span></div>
    </section>
    </div>
    <div class="bbGrid">
    <section class="bbDiagCard">
        <div class="bbCardHead">Sensor Fusion</div>
        <div class="bbRow"><span class="bbLbl">Lon Raw</span><span class="bbVal bbNum" id="lon-raw">--</span></div>
        <div class="bbRow"><span class="bbLbl">Lon Filtered</span><span class="bbVal bbNum" id="lon-filt">--</span></div>
        <div class="bbRow"><span class="bbLbl">Lon Blended</span><span class="bbVal bbNum" id="lon-blend">--</span></div>
        <div class="bbRow"><span class="bbLbl">GPS Weight</span><span class="bbVal bbNum" id="gps-wt">--</span></div>
        <div class="bbRow"><span class="bbLbl">GPS Accel</span><span class="bbVal bbNum" id="d-gps-acc">--</span></div>
        <div class="bbRow"><span class="bbLbl">GPS Rejected</span><span class="bbVal" id="gps-rej">--</span></div>
    </section>
    <section class="bbDiagCard">
        <div class="bbCardHead">Orientation</div>
        <div class="bbRow"><span class="bbLbl">Pitch Corr</span><span class="bbVal bbNum" id="pitch-corr">--</span></div>
        <div class="bbRow"><span class="bbLbl">Roll Corr</span><span class="bbVal bbNum" id="roll-corr">--</span></div>
        <div class="bbRow"><span class="bbLbl">Confidence</span><span class="bbVal bbNum" id="orient-conf">--</span></div>
        <div class="bbRow"><span class="bbLbl">Yaw Bias</span><span class="bbVal bbNum" id="yaw-bias">--</span></div>
        <div class="bbRow"><span class="bbLbl">Yaw Cal</span><span class="bbVal" id="yaw-cal">--</span></div>
        <div class="bbRow"><span class="bbLbl">Tilt X/Y</span><span class="bbVal bbNum" id="tilt-xy">--</span></div>
    </section>
    </div>
    <div class="bbGrid">
    <section class="bbDiagCard">
        <div class="bbCardHead">Configuration</div>
        <div class="bbRow"><span class="bbLbl">WiFi Mode</span><span class="bbVal" id="wifi-mode">--</span></div>
        <div class="bbRow"><span class="bbLbl">SSID</span><span class="bbVal" id="wifi-ssid">--</span></div>
        <div class="bbRow"><span class="bbLbl">Telemetry Rate</span><span class="bbVal" id="telem-hz">--</span></div>
        <div class="bbRow"><span class="bbLbl">GPS Model</span><span class="bbVal" id="gps-model">--</span></div>
    </section>
    <section class="bbDiagCard">
        <div class="bbCardHead">System</div>
        <div class="bbRow"><span class="bbLbl">Heap Free</span><span class="bbVal bbNum" id="heap">--</span></div>
        <div class="bbRow"><span class="bbLbl">TX Success</span><span class="bbVal bbNum" id="tx-ok">--</span></div>
        <div class="bbRow"><span class="bbLbl">TX Failed</span><span class="bbVal bbNum" id="tx-fail">--</span></div>
    </section>
    </div>
    <div class="bbUptime" id="uptime">Uptime: --</div>
</main>
</div>

<div class="bbMenuOverlay" id="menu-overlay">
    <div class="bbMenuPanel">
        <button class="bbMenuItem" id="menu-tracks">Tracks</button>
        <button class="bbMenuItem" id="menu-rec">Start Recording</button>
        <button class="bbMenuItem" id="menu-export">Export CSV</button>
        <button class="bbMenuItem destructive" id="menu-clear">Clear Session</button>
    </div>
</div>

<div class="bbModal" id="track-modal">
    <div class="bbModalContent">
        <div class="bbModalHeader">
            <span class="bbModalTitle">Tracks</span>
            <button class="bbModalClose" id="track-modal-close">&times;</button>
        </div>
        <div class="bbModalBody">
            <div id="active-track-section" class="bbActiveTrack" style="display:none">
                <div class="bbActiveTrackInfo">
                    <div class="bbActiveTrackName" id="active-track-name">—</div>
                    <div class="bbActiveTrackMeta" id="active-track-meta">—</div>
                </div>
                <button class="bbActiveTrackClear" id="btn-clear-track" title="Clear active track">×</button>
            </div>
            <div class="bbModalSection" id="create-track-section">
                <div class="bbSectionTitle">Record New Track</div>
                <p class="bbCreateTrackHint">Drive to start position, then tap to begin</p>
                <div class="bbCreateTrackBtns">
                    <button class="bbRecordStartBtn" id="btn-record-loop" onclick="startTrackRecording('loop')">
                        <div class="bbRecordStartIconWrap">
                            <svg class="bbRecordStartIcon" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
                                <circle cx="12" cy="12" r="9"/>
                                <polyline points="16,12 12,8 8,12"/>
                                <line x1="12" y1="8" x2="12" y2="16"/>
                            </svg>
                        </div>
                        <span class="bbRecordStartLabel">Circuit</span>
                        <span class="bbRecordStartDesc">Auto-detects return to start</span>
                    </button>
                    <button class="bbRecordStartBtn" id="btn-record-p2p" onclick="startTrackRecording('point_to_point')">
                        <div class="bbRecordStartIconWrap">
                            <svg class="bbRecordStartIcon" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
                                <circle cx="5" cy="12" r="3"/>
                                <circle cx="19" cy="12" r="3"/>
                                <line x1="8" y1="12" x2="16" y2="12"/>
                                <polyline points="13,9 16,12 13,15"/>
                            </svg>
                        </div>
                        <span class="bbRecordStartLabel">Stage</span>
                        <span class="bbRecordStartDesc">Mark finish when you're done</span>
                    </button>
                </div>
            </div>
            <div class="bbModalSection">
                <div class="bbSectionTitle">Saved Tracks</div>
                <div class="bbTrackList" id="track-list"><div class="bbTrackEmpty">No saved tracks</div></div>
            </div>
        </div>
    </div>
</div>

<div class="bbRecordOverlay" id="record-overlay">
    <div class="bbRecordState" id="rec-state-recording">
        <div class="bbRecordHeader">
            <div class="bbRecordPulse"></div>
            <span class="bbRecordTitle" id="rec-title">Recording Circuit</span>
        </div>
        <div class="bbRecordHero">
            <div class="bbRecordHeroValue" id="rec-distance">0</div>
            <div class="bbRecordHeroUnit">meters</div>
        </div>
        <div class="bbRecordMeta">
            <div class="bbRecordMetaItem">
                <span class="bbRecordMetaValue" id="rec-corners">0</span>
                <span class="bbRecordMetaLabel">corners</span>
            </div>
            <div class="bbRecordMetaDivider"></div>
            <div class="bbRecordMetaItem">
                <span class="bbRecordMetaValue" id="rec-points">0</span>
                <span class="bbRecordMetaLabel">points</span>
            </div>
            <div class="bbRecordMetaDivider"></div>
            <div class="bbRecordMetaItem">
                <span class="bbRecordMetaValue" id="rec-elapsed">0:00</span>
                <span class="bbRecordMetaLabel">elapsed</span>
            </div>
        </div>
        <div class="bbRecordQuality">
            <div class="bbRecordQualityBar">
                <div class="bbRecordQualityFill" id="rec-quality-fill" style="width:80%"></div>
            </div>
            <span class="bbRecordQualityText" id="rec-quality-text">GPS Good</span>
        </div>
        <p class="bbRecordHint" id="rec-hint">Complete one lap. We'll detect when you return.</p>
        <div class="bbRecordActions" id="rec-actions-normal">
            <button class="bbRecordActionBtn secondary" onclick="cancelTrackRecording()">Cancel</button>
            <button class="bbRecordActionBtn finish" id="btn-mark-finish" style="display:none" onclick="markRecordingFinish()">Mark Finish</button>
        </div>
    </div>
    <div class="bbRecordState" id="rec-state-complete" style="display:none">
        <div class="bbRecordSuccess">
            <div class="bbRecordSuccessIcon">✓</div>
            <div class="bbRecordSuccessTitle" id="rec-success-title">Track Complete!</div>
            <div class="bbRecordSuccessStats" id="rec-success-stats">—</div>
        </div>
        <div class="bbRecordActions">
            <button class="bbRecordActionBtn secondary" onclick="continueTrackRecording()">Keep Driving</button>
            <button class="bbRecordActionBtn primary" onclick="finishTrackRecording()">Save Track</button>
        </div>
    </div>
</div>

<script>
const $=id=>document.getElementById(id);
const MODES={0:'IDLE',1:'ACCEL',2:'BRAKE',4:'CORNER',5:'ACCEL',6:'BRAKE'};
const MODE_COLORS={0:'var(--mode-idle)',1:'var(--mode-accel)',2:'var(--mode-brake)',4:'var(--mode-idle)',5:'var(--mode-accel)',6:'var(--mode-brake)'};
const LAP_FLAG_CROSSED_START=1,LAP_FLAG_CROSSED_FINISH=2,LAP_FLAG_NEW_LAP=4,LAP_FLAG_NEW_BEST=8,LAP_FLAG_INVALID=16;
const DB_NAME='blackbox-rec',DB_VERSION=1,CHUNK_INTERVAL=60000;

let isDark=window.matchMedia('(prefers-color-scheme:dark)').matches;
function applyTheme(){
    document.body.classList.toggle('dark',isDark);
    $('modeLabel').textContent=$('modeLabel2').textContent=isDark?'Dark':'Bright';
}
applyTheme();
$('brandToggle').onclick=$('brandToggle2').onclick=()=>{isDark=!isDark;applyTheme()};

// View switching
$('link-diag').onclick=()=>{$('view-dashboard').classList.remove('active');$('view-diagnostics').classList.add('active')};
$('link-dashboard').onclick=()=>{$('view-diagnostics').classList.remove('active');$('view-dashboard').classList.add('active')};

function getModeColorHex(mo){
    const h={0:isDark?'#636366':'#8e8e93',1:isDark?'#ffffff':'#1c1c1e',2:'#ff3b30',4:isDark?'#636366':'#8e8e93',5:isDark?'#ffffff':'#1c1c1e',6:'#ff3b30'};
    return h[mo]||h[0];
}
let currentModeColor='#8e8e93';

// IndexedDB
let db=null;
function openDB(){
    return new Promise((resolve,reject)=>{
        const req=indexedDB.open(DB_NAME,DB_VERSION);
        req.onerror=()=>reject(req.error);
        req.onsuccess=()=>{db=req.result;resolve(db)};
        req.onupgradeneeded=(e)=>{
            const d=e.target.result;
            if(!d.objectStoreNames.contains('chunks')){
                d.createObjectStore('chunks',{keyPath:['sessionId','chunkIndex']});
            }
            if(!d.objectStoreNames.contains('sessions')){
                d.createObjectStore('sessions',{keyPath:'sessionId'});
            }
        };
    });
}

// Recording state
let rec=false,sessionId=null,chunkBuffer=[],chunkIndex=0,lastSaveTime=0,sessionStart=0;

async function startRecording(){
    if(!db)await openDB();
    sessionId=Date.now();sessionStart=sessionId;chunkBuffer=[];chunkIndex=0;lastSaveTime=Date.now();rec=true;
    const tx=db.transaction('sessions','readwrite');
    tx.objectStore('sessions').put({sessionId,startTime:sessionId,status:'active'});
    updateRecUI();
}

async function saveChunk(){
    if(!db||chunkBuffer.length===0)return;
    const data=[...chunkBuffer];
    try{
        const tx=db.transaction('chunks','readwrite');
        await new Promise((resolve,reject)=>{
            const req=tx.objectStore('chunks').put({sessionId,chunkIndex,data,timestamp:Date.now()});
            req.onsuccess=resolve;req.onerror=()=>reject(req.error);
        });
        chunkIndex++;chunkBuffer=[];lastSaveTime=Date.now();
        updateRecUI();
    }catch(e){console.error('Chunk save failed:',e);/* Keep buffer intact for retry */}
}

async function stopRecording(){
    if(chunkBuffer.length>0)await saveChunk();
    if(db&&sessionId){
        const tx=db.transaction('sessions','readwrite');
        tx.objectStore('sessions').put({sessionId,startTime:sessionStart,status:'complete',chunks:chunkIndex});
    }
    rec=false;updateRecUI();
}

function updateRecUI(){
    const dot=$('recDot'),txt=$('recTxt'),btn=$('menu-rec'),dbtn=$('diag-rec'),stat=$('rec-status');
    if(rec){
        dot.classList.add('rec');txt.textContent='REC';btn.textContent='Stop Recording';dbtn.textContent='Stop Recording';dbtn.classList.add('recording');
        stat.textContent='● Recording';stat.className='bbVal ok';
    }else{
        dot.classList.remove('rec');txt.textContent='';btn.textContent='Start Recording';dbtn.textContent='Start Recording';dbtn.classList.remove('recording');
        stat.textContent='Inactive';stat.className='bbVal';
    }
    $('rec-chunks').textContent=chunkIndex;
    $('rec-lastsave').textContent=lastSaveTime?(Math.floor((Date.now()-lastSaveTime)/1000)+'s ago'):'--';
    $('rec-duration').textContent=sessionStart?fmtTime(Date.now()-sessionStart):'--';
}

async function exportCSV(){
    if(!db){alert('No data');return}
    // Flush any buffered data if recording
    if(rec&&chunkBuffer.length>0)await saveChunk();
    const tx=db.transaction(['sessions','chunks'],'readonly');
    const sessions=await new Promise(r=>{const req=tx.objectStore('sessions').getAll();req.onsuccess=()=>r(req.result)});
    if(!sessions.length){alert('No recordings found');return}
    const latest=sessions.sort((a,b)=>b.sessionId-a.sessionId)[0];
    const chunks=await new Promise(r=>{const req=tx.objectStore('chunks').getAll();req.onsuccess=()=>r(req.result)});
    const sessionChunks=chunks.filter(c=>c.sessionId===latest.sessionId).sort((a,b)=>a.chunkIndex-b.chunkIndex);
    if(!sessionChunks.length){alert('No data in recording');return}
    let allData=[];
    sessionChunks.forEach(c=>allData=allData.concat(c.data));
    let csv='time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid,lon_imu,lon_gps,gps_weight,pitch_corr,pitch_conf,roll_corr,roll_conf,tilt_x,tilt_y\n';
    allData.forEach(r=>{csv+=r.t+','+r.sp+','+r.ax+','+r.ay+','+r.wz+','+r.mo+','+r.latg+','+r.lng+','+(r.lat||0)+','+(r.lon||0)+','+(r.gpsOk||0)+','+(r.lon_imu||0).toFixed(4)+','+(r.lon_gps||0).toFixed(4)+','+(r.gps_wt||0).toFixed(2)+','+(r.pitch_c||0).toFixed(2)+','+(r.pitch_cf||0).toFixed(1)+','+(r.roll_c||0).toFixed(2)+','+(r.roll_cf||0).toFixed(1)+','+(r.tilt_x||0).toFixed(4)+','+(r.tilt_y||0).toFixed(4)+'\n'});
    const b=new Blob([csv],{type:'text/csv'}),u=URL.createObjectURL(b),a=document.createElement('a');
    a.href=u;a.download='blackbox_'+new Date(latest.sessionId).toISOString().slice(0,19).replace(/[T:]/g,'-')+'.csv';a.click();
}

// TrackRecorder - Adaptive sampling for track recording with corner state machine
class TrackRecorder{
    constructor(){this.config={minDistance:2,maxDistance:30,headingThreshold:0.15,cornerEntryThreshold:0.025,cornerExitThreshold:0.012,minCornerLength:3,minLoopDistance:150,closeProximity:25,headingTolerance:0.5};this.reset()}
    reset(){this.recording=false;this.trackType='loop';this.startPos=null;this.startHeading=0;this.startTime=0;this.keyPoints=[];this.rawSamples=[];this.lastPos=null;this.lastSigma=3.0;this.totalDistance=0;this.loopDetected=false;this.inCorner=false;this.cornerCount=0;this.cornerEntryDist=0}
    start(pos,trackType='loop'){if(this.recording)return false;this.reset();this.recording=true;this.trackType=trackType;this.startPos={x:pos.x,y:pos.y};this.startHeading=pos.heading||0;this.startTime=Date.now();this.keyPoints.push({x:pos.x,y:pos.y,heading:pos.heading||0,speed:pos.speed||0,sigma:pos.sigma||3.0,t:Date.now(),curvature:0,isCorner:false});this.lastPos={x:pos.x,y:pos.y,heading:pos.heading||0};return true}
    addSample(pos){if(!this.recording||!this.lastPos)return{stored:false};const sigma=pos.sigma||3.0;this.lastSigma=sigma;const dx=pos.x-this.lastPos.x,dy=pos.y-this.lastPos.y,dist=Math.sqrt(dx*dx+dy*dy);if(dist<0.5)return{stored:false,stats:this.getStats()};const speedBasedMinDist=Math.max(this.config.minDistance,pos.speed*0.1);const headingChange=Math.abs(this._wrapAngle(pos.heading-this.lastPos.heading));const shouldStore=dist>=speedBasedMinDist||headingChange>=this.config.headingThreshold||dist>=this.config.maxDistance;if(!shouldStore)return{stored:false,stats:this.getStats()};const curvature=dist>0.1?headingChange/dist:0;let isCorner=false;if(!this.inCorner){if(curvature>this.config.cornerEntryThreshold){this.inCorner=true;this.cornerEntryDist=this.totalDistance;isCorner=true}}else{isCorner=true;if(curvature<this.config.cornerExitThreshold){const cornerLength=this.totalDistance-this.cornerEntryDist;if(cornerLength>=this.config.minCornerLength)this.cornerCount++;this.inCorner=false}}this.keyPoints.push({x:pos.x,y:pos.y,heading:pos.heading||0,speed:pos.speed||0,sigma:sigma,t:Date.now(),curvature:curvature,isCorner:isCorner});this.totalDistance+=dist;this.lastPos={x:pos.x,y:pos.y,heading:pos.heading||0};if(this.trackType==='loop'){const lr=this._checkLoopClosure(pos);if(lr.detected){this.loopDetected=true;return{stored:true,loopDetected:true,stats:this.getStats()}}}return{stored:true,curvature:curvature,isCorner:isCorner,stats:this.getStats()}}
    _checkLoopClosure(pos){if(this.totalDistance<this.config.minLoopDistance)return{detected:false};const dts=Math.sqrt((pos.x-this.startPos.x)**2+(pos.y-this.startPos.y)**2);if(dts>=this.config.closeProximity)return{detected:false};const hd=Math.abs(this._wrapAngle(pos.heading-this.startHeading));return hd<this.config.headingTolerance?{detected:true}:{detected:false}}
    _wrapAngle(a){while(a>Math.PI)a-=2*Math.PI;while(a<-Math.PI)a+=2*Math.PI;return a}
    getStats(){const displayCorners=this.inCorner?this.cornerCount+1:this.cornerCount;return{recording:this.recording,trackType:this.trackType,keyPointCount:this.keyPoints.length,totalDistance:this.totalDistance,loopDetected:this.loopDetected,elapsedMs:this.recording?Date.now()-this.startTime:0,gpsQuality:this._gpsQualityRating(this.lastSigma),corners:displayCorners,inCorner:this.inCorner}}
    _gpsQualityRating(sigma){if(sigma<2.0)return'excellent';if(sigma<3.0)return'good';if(sigma<4.0)return'fair';return'poor'}
    cancel(){this.recording=false}
    finish(trackName,gpsOrigin=null){if(!this.recording)return null;this.recording=false;if(this.keyPoints.length<10)return null;const centerline=this._smoothPath(this.keyPoints,3);const startLine=this._calculateTimingLine(centerline,'start');let finishLine=null;if(this.trackType==='point_to_point')finishLine=this._calculateTimingLine(centerline,'finish');const bounds=this._calculateBounds(centerline);const totalDist=this._calculatePathLength(centerline);const quality=this._assessQuality();return{id:'track_'+Date.now()+'_'+Math.random().toString(36).substr(2,9),name:trackName,type:this.trackType,created:Date.now(),startLine:startLine,finishLine:finishLine,centerline:centerline,bounds:bounds,totalDistance:totalDist,origin:{x:this.startPos.x,y:this.startPos.y},gpsOrigin:gpsOrigin,quality:quality}}
    _smoothPath(pts,ws){if(!pts||pts.length===0)return[];if(pts.length<=ws)return pts.map(p=>({...p}));const result=[];const hw=Math.floor(ws/2);for(let i=0;i<pts.length;i++){const s=Math.max(0,i-hw),e=Math.min(pts.length-1,i+hw),w=pts.slice(s,e+1);if(w.length===0){result.push({...pts[i]});continue}let sx=0,sy=0,sw=0;for(let j=0;j<w.length;j++){const df=Math.abs(j-(i-s)),pw=1/(1+df*0.5),sg=w[j].sigma||3.0,qw=1/(sg+0.5),wt=pw*qw;sx+=w[j].x*wt;sy+=w[j].y*wt;sw+=wt}const x=sw>0?sx/sw:pts[i].x,y=sw>0?sy/sw:pts[i].y;result.push({x:isNaN(x)?pts[i].x:x,y:isNaN(y)?pts[i].y:y,heading:pts[i].heading,speed:pts[i].speed,sigma:pts[i].sigma,t:pts[i].t,curvature:pts[i].curvature,isCorner:pts[i].isCorner})}return result}
    _calculateTimingLine(cl,which='start'){if(!cl||cl.length===0)return{p1:[0,12],p2:[0,-12],direction:0};const idx=which==='start'?Math.min(2,cl.length-1):Math.max(0,cl.length-3);const ns=5,s=Math.max(0,idx-ns),e=Math.min(cl.length-1,idx+ns),pts=cl.slice(s,e+1);let sx=0,sy=0,sw=0;for(const p of pts){const wt=1/(p.sigma||3.0+0.5);sx+=p.x*wt;sy+=p.y*wt;sw+=wt}const cx=sw>0?sx/sw:cl[idx].x,cy=sw>0?sy/sw:cl[idx].y;const safeCx=isNaN(cx)?0:cx,safeCy=isNaN(cy)?0:cy;let dir=0;if(pts.length>=2){const f=pts[0],l=pts[pts.length-1];dir=Math.atan2(l.y-f.y,l.x-f.x)}const w=12,perp=dir+Math.PI/2;return{p1:[safeCx+Math.cos(perp)*w,safeCy+Math.sin(perp)*w],p2:[safeCx-Math.cos(perp)*w,safeCy-Math.sin(perp)*w],direction:dir}}
    _calculateBounds(cl){if(!cl||cl.length===0)return{minX:0,minY:0,maxX:100,maxY:100};let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;for(const p of cl){minX=Math.min(minX,p.x);minY=Math.min(minY,p.y);maxX=Math.max(maxX,p.x);maxY=Math.max(maxY,p.y)}return{minX,minY,maxX,maxY}}
    _calculatePathLength(cl){if(!cl||cl.length<2)return 0;let len=0;for(let i=1;i<cl.length;i++){const dx=cl[i].x-cl[i-1].x,dy=cl[i].y-cl[i-1].y;len+=Math.sqrt(dx*dx+dy*dy)}return len}
    _assessQuality(){if(this.keyPoints.length===0)return{rating:'poor',avgUncertainty:5.0,goodSampleRatio:0,corners:0,totalPoints:0};const vp=this.keyPoints.filter(p=>p.sigma<3.0);const ts=this.keyPoints.reduce((s,p)=>s+(p.sigma||3.0),0);const as=ts/this.keyPoints.length;const displayCorners=this.inCorner?this.cornerCount+1:this.cornerCount;const gr=vp.length/this.keyPoints.length;let rating='good';if(isNaN(as)||as>3.5||gr<0.7)rating='fair';if(isNaN(as)||as>4.5||gr<0.5)rating='poor';return{rating:rating,avgUncertainty:isNaN(as)?3.0:as,goodSampleRatio:isNaN(gr)?0.5:gr,corners:displayCorners,totalPoints:this.keyPoints.length}}
}

// Track Manager IndexedDB
const TRACK_DB='blackbox-tracks',TRACK_DB_VER=1;
let trackDb=null,activeTrack=null,currentPos=null,suppressStartLineIndicator=false;
let p2pNeedsWarmup=false,trackRecorder=null;

function openTrackDB(){
    return new Promise((res,rej)=>{
        const r=indexedDB.open(TRACK_DB,TRACK_DB_VER);
        r.onerror=()=>rej(r.error);
        r.onsuccess=()=>{trackDb=r.result;res(trackDb)};
        r.onupgradeneeded=e=>{
            const d=e.target.result;
            if(!d.objectStoreNames.contains('tracks')){
                const s=d.createObjectStore('tracks',{keyPath:'id'});
                s.createIndex('modified','modified',{unique:false});
            }
        };
    });
}

async function saveTrack(track){
    if(!trackDb)await openTrackDB();
    track.modified=Date.now();
    return new Promise((res,rej)=>{
        const tx=trackDb.transaction('tracks','readwrite');
        const r=tx.objectStore('tracks').put(track);
        r.onsuccess=()=>res(track);r.onerror=()=>rej(r.error);
    });
}

async function getTrack(id){
    if(!trackDb)await openTrackDB();
    return new Promise((res,rej)=>{
        const tx=trackDb.transaction('tracks','readonly');
        const r=tx.objectStore('tracks').get(id);
        r.onsuccess=()=>res(r.result);r.onerror=()=>rej(r.error);
    });
}

async function getAllTracks(){
    if(!trackDb)await openTrackDB();
    return new Promise((res,rej)=>{
        const tx=trackDb.transaction('tracks','readonly');
        const r=tx.objectStore('tracks').getAll();
        r.onsuccess=()=>res(r.result||[]);r.onerror=()=>rej(r.error);
    });
}

async function deleteTrackFromDB(id){
    if(!trackDb)await openTrackDB();
    return new Promise((res,rej)=>{
        const tx=trackDb.transaction('tracks','readwrite');
        const r=tx.objectStore('tracks').delete(id);
        r.onsuccess=()=>res();r.onerror=()=>rej(r.error);
    });
}

function genTrackId(){return 'track_'+Date.now()+'_'+Math.random().toString(36).substr(2,9)}
function escHtml(t){const d=document.createElement('div');d.textContent=t;return d.innerHTML}

// Track Manager UI
function openTrackModal(){
    $('track-modal').classList.add('open');
    renderTrackList();
    updateActiveTrackDisplay();
}
function closeTrackModal(){$('track-modal').classList.remove('open')}

function updateActiveTrackDisplay(){
    const sec=$('active-track-section');
    if(activeTrack){
        sec.style.display='flex';
        $('active-track-name').textContent=activeTrack.name;
        const best=activeTrack.bestLapMs?fmtLapTime(activeTrack.bestLapMs):'—';
        const unit=activeTrack.type==='point_to_point'?'runs':'laps';
        $('active-track-meta').textContent='Best: '+best+' · '+(activeTrack.lapCount||0)+' '+unit;
    }else{sec.style.display='none'}
}

async function renderTrackList(){
    const el=$('track-list');
    try{
        const tracks=await getAllTracks();
        if(tracks.length===0){el.innerHTML='<div class=\"bbTrackEmpty\">No saved tracks</div>';return}
        tracks.sort((a,b)=>b.modified-a.modified);
        el.innerHTML=tracks.map(t=>{
            const best=t.bestLapMs?fmtLapTime(t.bestLapMs):'—';
            const isActive=activeTrack&&activeTrack.id===t.id;
            const unit=t.type==='point_to_point'?'runs':'laps';
            return '<div class=\"bbTrackItem\" data-id=\"'+t.id+'\">'+
                '<div class=\"bbTrackInfo\">'+
                    '<div class=\"bbTrackName\">'+escHtml(t.name)+(isActive?' ✓':'')+'</div>'+
                    '<div class=\"bbTrackMeta\"><span>Best: '+best+'</span><span>'+(t.lapCount||0)+' '+unit+'</span></div>'+
                '</div>'+
                '<div class=\"bbTrackActions\">'+
                    '<button class=\"bbTrackBtn primary\" data-action=\"use\">Use</button>'+
                    '<button class=\"bbTrackBtn danger\" data-action=\"delete\">×</button>'+
                '</div></div>';
        }).join('');
        el.querySelectorAll('.bbTrackBtn').forEach(b=>{
            b.onclick=()=>handleTrackAction(b.dataset.action,b.closest('.bbTrackItem').dataset.id);
        });
    }catch(e){console.error('Track list error:',e);el.innerHTML='<div class=\"bbTrackEmpty\">Error loading</div>'}
}

async function handleTrackAction(action,id){
    if(action==='use'){
        const t=await getTrack(id);
        if(t)await activateTrack(t);
        closeTrackModal();
    }else if(action==='delete'){
        if(confirm('Delete this track?')){
            await deleteTrackFromDB(id);
            if(activeTrack&&activeTrack.id===id)await deactivateTrack();
            renderTrackList();
        }
    }
}

async function activateTrack(track){
    const isP2P=track.type==='point_to_point';
    const line=track.startLine;
    let url;
    if(isP2P&&track.finishLine){
        const f=track.finishLine;
        url='/api/laptimer/configure?type=point_to_point'+
            '&p1_x='+line.p1[0].toFixed(2)+'&p1_y='+line.p1[1].toFixed(2)+
            '&p2_x='+line.p2[0].toFixed(2)+'&p2_y='+line.p2[1].toFixed(2)+
            '&dir='+line.direction.toFixed(4)+
            '&f_p1_x='+f.p1[0].toFixed(2)+'&f_p1_y='+f.p1[1].toFixed(2)+
            '&f_p2_x='+f.p2[0].toFixed(2)+'&f_p2_y='+f.p2[1].toFixed(2)+
            '&f_dir='+f.direction.toFixed(4);
    }else{
        url='/api/laptimer/configure?type=loop'+
            '&p1_x='+line.p1[0].toFixed(2)+'&p1_y='+line.p1[1].toFixed(2)+
            '&p2_x='+line.p2[0].toFixed(2)+'&p2_y='+line.p2[1].toFixed(2)+
            '&dir='+line.direction.toFixed(4);
    }
    try{
        const r=await fetch(url);
        const j=await r.json();
        if(j.error){alert('Failed to configure lap timer: '+j.error);return}
        activeTrack=track;
        suppressStartLineIndicator=false;
        p2pNeedsWarmup=isP2P&&track.isNew;
        const sec=$('lap-section');
        sec.classList.toggle('p2p',isP2P);
        sec.classList.toggle('first-run',!track.lapCount);
        $('lap-setup-text').textContent=track.name;
        $('lap-track-name').textContent=track.name;
        const unit=isP2P?'Run':'Lap';
        const countEl=sec.querySelector('.bbLapCount');
        if(countEl)countEl.textContent=unit+' 0';
        updateActiveTrackDisplay();
        console.log('Track activated:',track.name,'Type:',track.type);
    }catch(e){alert('Failed to configure lap timer: '+e.message)}
}

async function deactivateTrack(){
    try{await fetch('/api/laptimer/configure?type=clear')}catch(e){}
    activeTrack=null;
    p2pNeedsWarmup=false;
    $('lap-setup-text').textContent='Tap to configure';
    $('lap-track-name').textContent='';
    updateActiveTrackDisplay();
}

// Track Recording Functions
function startTrackRecording(trackType='loop'){
    if(!currentPos||!currentPos.valid){alert('Position not available');return false}
    if(!trackRecorder)trackRecorder=new TrackRecorder();
    const pos={x:currentPos.x,y:currentPos.y,heading:currentPos.yaw,speed:currentPos.speed,valid:true,sigma:currentPos.sigma||3.0};
    if(!trackRecorder.start(pos,trackType)){console.warn('Failed to start recording');return false}
    const overlay=$('record-overlay');overlay.classList.add('active');
    $('rec-title').textContent=trackType==='loop'?'Recording Circuit':'Recording Stage';
    $('rec-distance').textContent='0';$('rec-corners').textContent='0';$('rec-points').textContent='0';$('rec-elapsed').textContent='0:00';
    $('rec-hint').textContent=trackType==='loop'?"Complete one lap. We'll detect when you return.":'Drive the stage, then tap Mark Finish at the end.';
    $('rec-state-recording').style.display='block';$('rec-state-complete').style.display='none';
    const markFinishBtn=$('btn-mark-finish');if(markFinishBtn)markFinishBtn.style.display=trackType==='point_to_point'?'flex':'none';
    closeTrackModal();return true;
}
function updateTrackRecording(){
    if(!trackRecorder||!trackRecorder.recording||!currentPos||!currentPos.valid)return;
    const pos={x:currentPos.x,y:currentPos.y,heading:currentPos.yaw,speed:currentPos.speed,valid:true,sigma:currentPos.sigma||3.0};
    const result=trackRecorder.addSample(pos);
    const stats=trackRecorder.getStats();
    const distVal=Math.round(stats.totalDistance);
    $('rec-distance').textContent=isNaN(distVal)?'0':distVal;
    $('rec-corners').textContent=stats.corners||0;
    $('rec-points').textContent=stats.keyPointCount||0;
    const elapsed=stats.elapsedMs||0,mins=Math.floor(elapsed/60000),secs=Math.floor((elapsed%60000)/1000);
    $('rec-elapsed').textContent=mins+':'+secs.toString().padStart(2,'0');
    updateRecordingQualityDisplay(stats.gpsQuality);
    if(result&&result.loopDetected&&$('rec-state-complete').style.display==='none')onLoopDetected();
}
function cancelTrackRecording(){
    if(trackRecorder){trackRecorder.cancel();trackRecorder=null}
    $('record-overlay').classList.remove('active');
}
function markRecordingFinish(){
    if(!trackRecorder||!trackRecorder.recording||trackRecorder.trackType!=='point_to_point')return;
    const stats=trackRecorder.getStats();
    if(stats.totalDistance<50){alert('Track is too short. Drive at least 50 meters.');return}
    onLoopDetected();
}
function onLoopDetected(){
    const stats=trackRecorder.getStats();
    const quality=(stats.gpsQuality||'good').charAt(0).toUpperCase()+(stats.gpsQuality||'good').slice(1);
    const distance=Math.round(stats.totalDistance)||0,corners=stats.corners||0;
    const isStage=trackRecorder&&trackRecorder.trackType==='point_to_point';
    $('rec-success-title').textContent=isStage?'Stage Complete!':'Circuit Complete!';
    $('rec-success-stats').textContent=distance+'m · '+corners+' corners · '+quality+' quality';
    $('rec-state-recording').style.display='none';$('rec-state-complete').style.display='block';
}
function continueTrackRecording(){
    if(!trackRecorder)return;
    trackRecorder.loopDetected=false;
    $('rec-state-recording').style.display='block';$('rec-state-complete').style.display='none';
    const isStage=trackRecorder.trackType==='point_to_point';
    $('rec-hint').textContent=isStage?'Continue driving. Tap Mark Finish when done.':"Continue driving. We'll detect the next loop.";
}
async function finishTrackRecording(){
    if(!trackRecorder||!trackRecorder.recording)return;
    const defaultName=trackRecorder.trackType==='loop'?'New Circuit':'New Stage';
    const trackName=prompt('Enter track name:',defaultName);if(!trackName)return;
    const trackData=trackRecorder.finish(trackName,null);
    if(!trackData){alert('Failed to create track: not enough data or poor quality');return}
    $('record-overlay').classList.remove('active');
    const track={id:trackData.id,name:trackData.name,type:trackData.type,startLine:trackData.startLine,finishLine:trackData.finishLine,origin:trackData.origin,centerline:trackData.centerline,bounds:trackData.bounds,pathLength:trackData.totalDistance||0,quality:trackData.quality,bestLapMs:null,lapCount:0,createdAt:trackData.created,isNew:true};
    await saveTrack(track);await activateTrack(track);renderTrackList();
    alert('Track "'+track.name+'" saved!\\n'+track.centerline.length+' points · '+Math.round(track.pathLength||0)+'m');
}
function updateRecordingQualityDisplay(quality){
    const fill=$('rec-quality-fill'),text=$('rec-quality-text');
    const pct={excellent:100,good:80,fair:60,poor:40}[quality]||50;
    fill.style.width=pct+'%';fill.className='bbRecordQualityFill';
    if(quality==='fair')fill.classList.add('fair');else if(quality==='poor')fill.classList.add('poor');
    text.textContent='GPS '+(quality||'Good').charAt(0).toUpperCase()+(quality||'good').slice(1);
}

async function clearActiveTrack(){
    if(!activeTrack)return;
    await deactivateTrack();
    updateActiveTrackDisplay();
    renderTrackList();
}
function confirmStopTiming(){
    if(!activeTrack)return;
    if(confirm('Stop timing "'+activeTrack.name+'"?'))clearActiveTrack();
}

async function updateTrackBestLap(lapTimeMs){
    if(!activeTrack)return;
    if(!activeTrack.bestLapMs||lapTimeMs<activeTrack.bestLapMs)activeTrack.bestLapMs=lapTimeMs;
    activeTrack.lapCount=(activeTrack.lapCount||0)+1;
    await saveTrack(activeTrack);
}

// Start line indicator
function getDistanceToStartLine(){
    if(!activeTrack||!activeTrack.startLine||!currentPos)return null;
    const line=activeTrack.startLine;
    const cx=(line.p1[0]+line.p2[0])/2,cy=(line.p1[1]+line.p2[1])/2;
    const dx=cx-currentPos.x,dy=cy-currentPos.y;
    const dist=Math.sqrt(dx*dx+dy*dy);
    const bearing=Math.atan2(dy,dx);
    return{distance:dist,bearing};
}

function formatDistance(m){return m>=1000?(m/1000).toFixed(1)+'km':Math.round(m)+'m'}

function bearingToArrow(bearing,heading){
    let rel=bearing-heading;
    while(rel>Math.PI)rel-=2*Math.PI;
    while(rel<-Math.PI)rel+=2*Math.PI;
    const deg=rel*180/Math.PI;
    if(deg>-22.5&&deg<=22.5)return'↑';
    if(deg>22.5&&deg<=67.5)return'↗';
    if(deg>67.5&&deg<=112.5)return'→';
    if(deg>112.5&&deg<=157.5)return'↘';
    if(deg>157.5||deg<=-157.5)return'↓';
    if(deg>-157.5&&deg<=-112.5)return'↙';
    if(deg>-112.5&&deg<=-67.5)return'←';
    if(deg>-67.5&&deg<=-22.5)return'↖';
    return'•';
}

function updateStartLineIndicator(){
    const el=$('start-line-indicator'),txt=$('start-line-text');
    if(!el||!txt)return;
    const sec=$('lap-section');
    if(!lapTimerActive||!activeTrack||sec.classList.contains('timing')){el.style.display='none';return}
    el.style.display='block';
    const isP2P=activeTrack&&activeTrack.type==='point_to_point';
    if(suppressStartLineIndicator){
        if(isP2P){
            const r=getDistanceToStartLine();
            if(r){
                const arrow=bearingToArrow(r.bearing,currentPos.yaw);
                txt.textContent='Return to start: '+formatDistance(r.distance)+' '+arrow;
                txt.className=r.distance<50?'bbStartLineText approaching':'bbStartLineText';
            }else{txt.textContent='Return to start';txt.className='bbStartLineText'}
        }else{txt.textContent='Drive a lap, then cross start to begin';txt.className='bbStartLineText'}
        return;
    }
    if(!currentPos||!currentPos.valid){txt.textContent='Position unavailable';txt.className='bbStartLineText';return}
    const r=getDistanceToStartLine();
    if(!r){el.style.display='none';return}
    const arrow=bearingToArrow(r.bearing,currentPos.yaw);
    if(r.distance>100){txt.textContent='Start line: '+formatDistance(r.distance)+' '+arrow;txt.className='bbStartLineText'}
    else if(r.distance>50){txt.textContent='Approaching start: '+formatDistance(r.distance)+' '+arrow;txt.className='bbStartLineText approaching'}
    else if(r.distance>15){txt.textContent='Almost there: '+formatDistance(r.distance)+' '+arrow;txt.className='bbStartLineText close'}
    else{txt.textContent='Cross to begin! '+arrow;txt.className='bbStartLineText at-line'}
}

function getDistanceToFinishLine(){
    if(!activeTrack||!activeTrack.finishLine||!currentPos)return null;
    const line=activeTrack.finishLine;
    const cx=(line.p1[0]+line.p2[0])/2,cy=(line.p1[1]+line.p2[1])/2;
    const dx=cx-currentPos.x,dy=cy-currentPos.y;
    return{distance:Math.sqrt(dx*dx+dy*dy),bearing:Math.atan2(dy,dx)};
}

function updateFinishLineIndicator(){
    const el=$('finish-line-indicator'),txt=$('finish-line-text');
    if(!el||!txt)return;
    const isP2P=activeTrack&&activeTrack.type==='point_to_point';
    const sec=$('lap-section');
    if(!isP2P||!sec.classList.contains('timing')||!activeTrack.finishLine||!currentPos||!currentPos.valid){el.style.display='none';return}
    el.style.display='block';
    const r=getDistanceToFinishLine();
    if(!r){txt.textContent='Position unavailable';txt.className='bbFinishLineText';return}
    const arrow=bearingToArrow(r.bearing,currentPos.yaw);
    if(r.distance>100){txt.textContent='Distance to finish: '+formatDistance(r.distance)+' '+arrow;txt.className='bbFinishLineText'}
    else if(r.distance>50){txt.textContent='Approaching finish: '+formatDistance(r.distance)+' '+arrow;txt.className='bbFinishLineText approaching'}
    else if(r.distance>15){txt.textContent='Almost there: '+formatDistance(r.distance)+' '+arrow;txt.className='bbFinishLineText close'}
    else{txt.textContent='Cross to finish! '+arrow;txt.className='bbFinishLineText at-line'}
}

// G-meter
const cv=$('gcanvas'),ctx=cv.getContext('2d');
const SCALE_STEPS=[0.3,0.5,0.8,1.0,1.5,2.0];
let currentScale=0.5,magHist=[];
const TRAIL_DURATION_MS=2500,TRAIL_MAX_POINTS=60,TRAIL_JITTER_THRESHOLD=0.008;

function resize(){
    const frame=cv.parentElement,rect=frame.getBoundingClientRect();
    const size=Math.min(rect.width,rect.height);
    cv.width=size*devicePixelRatio;cv.height=size*devicePixelRatio;
    cv.style.width=size+'px';cv.style.height=size+'px';
    ctx.setTransform(1,0,0,1,0,0);ctx.scale(devicePixelRatio,devicePixelRatio);
}
resize();window.addEventListener('resize',resize);

function updateScale(gx,gy){
    const now=Date.now(),mag=Math.sqrt(gx*gx+gy*gy);
    magHist.push({t:now,m:mag});magHist=magHist.filter(h=>now-h.t<1000);
    const peakMag=Math.max(...magHist.map(h=>h.m),0.1);
    const needed=peakMag*1.4;
    let targetScale=SCALE_STEPS[0];
    for(const step of SCALE_STEPS){if(step>=needed){targetScale=step;break}targetScale=step}
    if(targetScale>currentScale)currentScale+=(targetScale-currentScale)*0.15;
    else if(targetScale<currentScale)currentScale+=(targetScale-currentScale)*0.03;
    for(const step of SCALE_STEPS){if(Math.abs(currentScale-step)<0.02){currentScale=step;break}}
    $('range-val').textContent='Range ±'+currentScale.toFixed(1)+'g';
}

function hexToRgba(hex,a){const r=parseInt(hex.slice(1,3),16),g=parseInt(hex.slice(3,5),16),b=parseInt(hex.slice(5,7),16);return`rgba(${r},${g},${b},${a})`}
function downsampleTrail(points,maxPoints){
    if(points.length<=maxPoints)return points;
    const recentCount=Math.min(25,Math.floor(maxPoints*0.4));
    const recent=points.slice(-recentCount),older=points.slice(0,-recentCount);
    const step=Math.ceil(older.length/(maxPoints-recentCount));
    return[...older.filter((_,i)=>i%step===0),...recent];
}

let trail=[];
function drawG(){
    const size=cv.width/devicePixelRatio,cx=size/2,cy=size/2,r=size*0.38;
    ctx.clearRect(0,0,size,size);
    const ringColor=isDark?'rgba(255,255,255,0.08)':'rgba(0,0,0,0.06)';
    const axisColor=isDark?'rgba(255,255,255,0.25)':'rgba(0,0,0,0.18)';
    ctx.lineWidth=1;ctx.strokeStyle=ringColor;
    [0.2,0.4,0.6,0.8].forEach(f=>{ctx.beginPath();ctx.arc(cx,cy,r*f,0,Math.PI*2);ctx.stroke()});
    ctx.strokeStyle=axisColor;ctx.lineWidth=2.5;
    ctx.beginPath();ctx.moveTo(cx-r,cy);ctx.lineTo(cx+r,cy);ctx.moveTo(cx,cy-r);ctx.lineTo(cx,cy+r);ctx.stroke();
    ctx.fillStyle=isDark?'rgba(255,255,255,0.08)':'rgba(0,0,0,0.06)';
    ctx.beginPath();ctx.arc(cx,cy,2,0,Math.PI*2);ctx.fill();
    const now=Date.now(),recentTrail=trail.filter(p=>now-p.t<TRAIL_DURATION_MS);
    const displayTrail=downsampleTrail(recentTrail,TRAIL_MAX_POINTS);
    if(displayTrail.length>1){
        ctx.lineCap='round';ctx.lineJoin='round';
        for(let i=1;i<displayTrail.length;i++){
            const p0=displayTrail[i-1],p1=displayTrail[i];
            const age=(now-p1.t)/TRAIL_DURATION_MS,alpha=Math.max(0.05,0.35*(1-age*0.85));
            ctx.strokeStyle=hexToRgba(currentModeColor,alpha);ctx.lineWidth=1.5-age*0.6;
            ctx.beginPath();
            ctx.moveTo(cx+(p0.x/currentScale)*r,cy-(p0.y/currentScale)*r);
            ctx.lineTo(cx+(p1.x/currentScale)*r,cy-(p1.y/currentScale)*r);
            ctx.stroke();
        }
    }
    if(trail.length>0){
        const cur=trail[trail.length-1],x=cx+(cur.x/currentScale)*r,y=cy-(cur.y/currentScale)*r;
        ctx.strokeStyle='#ffffff';ctx.lineWidth=2;ctx.lineCap='round';
        ctx.beginPath();ctx.moveTo(x-8,y);ctx.lineTo(x+8,y);ctx.moveTo(x,y-8);ctx.lineTo(x,y+8);ctx.stroke();
    }
}

// State
let cnt=0,lastSeq=0;
let maxL=0,maxR=0,maxA=0,maxB=0,peak=0;
let speed_ema=0,displaySessionStart=Date.now();
let emaGx=0,emaGy=0,lastT=0;
const EMA_TAU=0.10;
let lastGpsState=0;
let fusion={lon_imu:0,lon_gps:0,gps_wt:0,gps_rate:0,pitch_c:0,pitch_cf:0,roll_c:0,roll_cf:0,tilt_x:0,tilt_y:0};
let lapTimerActive=false,lapCount=0,bestLapMs=0,lastLapMs=0,prevLapFlags=0,prevLapTimeMs=0;

function fmtGSigned(v){const sign=v>=0?'+':'−';return{sign,num:Math.abs(v).toFixed(2)}}
function fmtTime(ms){const s=Math.floor(ms/1000),m=Math.floor(s/60),h=Math.floor(m/60);if(h>0)return h+':'+String(m%60).padStart(2,'0')+':'+String(s%60).padStart(2,'0');return m+':'+String(s%60).padStart(2,'0')}

function fmtLapTime(ms){if(ms===0)return'0:00.000';const mins=Math.floor(ms/60000),secs=Math.floor((ms%60000)/1000),millis=ms%1000;return mins+':'+String(secs).padStart(2,'0')+'.'+String(millis).padStart(3,'0')}

function updateLapTimer(lapTimeMs,lapCnt,lapFlags){
    const sec=$('lap-section'),active=(lapFlags&LAP_FLAG_INVALID)===0&&(lapTimeMs>0||lapCnt>0||(lapFlags&(LAP_FLAG_CROSSED_START|LAP_FLAG_CROSSED_FINISH)));
    if(active&&!lapTimerActive){sec.classList.remove('inactive');sec.classList.add('active');lapTimerActive=true}
    else if(!active&&lapTimerActive){sec.classList.add('inactive');sec.classList.remove('active','timing');lapTimerActive=false}
    if(!lapTimerActive)return;
    // Update current lap time
    $('lap-time').textContent=fmtLapTime(lapTimeMs);
    $('lap-count').textContent='Lap '+(lapCnt+(lapTimeMs>0?1:0));
    // Update state indicator
    const stateEl=$('lap-state');
    if(lapTimeMs>0){stateEl.textContent='Timing';stateEl.classList.add('timing');sec.classList.add('timing');suppressStartLineIndicator=false}
    else{stateEl.textContent='Armed';stateEl.classList.remove('timing');sec.classList.remove('timing')}
    // Handle new lap flag - save completed lap time to track
    if((lapFlags&LAP_FLAG_NEW_LAP)&&!(prevLapFlags&LAP_FLAG_NEW_LAP)){
        sec.classList.add('bbLapFlash');setTimeout(()=>sec.classList.remove('bbLapFlash'),600);
        // prevLapTimeMs contains the completed lap time (before reset)
        if(prevLapTimeMs>0)updateTrackBestLap(prevLapTimeMs);
    }
    prevLapTimeMs=lapTimeMs;
    // Handle new best flag
    if((lapFlags&LAP_FLAG_NEW_BEST)&&!(prevLapFlags&LAP_FLAG_NEW_BEST)){
        const bestEl=$('best-lap');bestEl.classList.add('bbLapBestFlash');setTimeout(()=>bestEl.classList.remove('bbLapBestFlash'),800);
    }
    prevLapFlags=lapFlags;
}

function updateReadouts(lat,lon){
    const latFmt=fmtGSigned(lat),lonFmt=fmtGSigned(lon);
    $('lat-g').innerHTML=`<span class="bbSign">${latFmt.sign}</span><span class="bbVal">${latFmt.num}</span><span class="bbGUnit">g</span>`;
    $('lon-g').innerHTML=`<span class="bbSign">${lonFmt.sign}</span><span class="bbVal">${lonFmt.num}</span><span class="bbGUnit">g</span>`;
}

function resetState(){
    maxL=maxR=maxA=maxB=peak=0;
    $('max-l').textContent=$('max-r').textContent=$('max-a').textContent=$('max-b').textContent='0.00g';
    $('speed').classList.remove('peak');
    trail=[];magHist=[];currentScale=SCALE_STEPS[0];
    $('range-val').textContent='Range ±'+currentScale.toFixed(1)+'g';
    emaGx=emaGy=0;displaySessionStart=Date.now();
}

let peakHighlightTimeout=null;

function process(buf){
    const d=new DataView(buf);
    const ax=d.getFloat32(7,1),ay=d.getFloat32(11,1),wz=d.getFloat32(19,1),sp=d.getFloat32(51,1),mo=d.getUint8(55);
    const lat=d.getFloat32(56,1),lon=d.getFloat32(60,1),gpsOk=d.getUint8(64);
    // EKF state for track manager
    const ekfYaw=d.getFloat32(31,1),ekfX=d.getFloat32(35,1),ekfY=d.getFloat32(39,1);
    // Lap timer fields (74-byte protocol v2)
    const lapTimeMs=buf.byteLength>=72?d.getUint32(65,1):0;
    const lapCnt=buf.byteLength>=72?d.getUint16(69,1):0;
    const lapFlags=buf.byteLength>=72?d.getUint8(71):0;
    const latg=ay/9.81,lng=-ax/9.81,yawDeg=Math.abs(wz*57.3);
    // Update position for track manager and recording
    currentPos={x:ekfX,y:ekfY,yaw:ekfYaw,speed:sp,valid:gpsOk===1,sigma:3.0};
    updateTrackRecording();
    updateStartLineIndicator();
    updateFinishLineIndicator();

    const now=Date.now();
    const dt=lastT?Math.min((now-lastT)/1000,0.2):0.033;
    lastT=now;
    const alpha=1-Math.exp(-dt/EMA_TAU);
    emaGx=alpha*latg+(1-alpha)*emaGx;
    emaGy=alpha*lng+(1-alpha)*emaGy;

    speed_ema=0.7*sp+0.3*speed_ema;
    const dspd=speed_ema<1?0:Math.round(speed_ema);

    const mag=Math.sqrt(emaGx*emaGx+emaGy*emaGy);
    const noise=speed_ema<5?0.04:0.015;
    const gx=mag<noise?0:emaGx,gy=mag<noise?0:emaGy;

    const speedEl=$('speed');
    if(dspd>peak){
        peak=dspd;speedEl.classList.add('peak');
        if(peakHighlightTimeout)clearTimeout(peakHighlightTimeout);
        peakHighlightTimeout=setTimeout(()=>speedEl.classList.remove('peak'),900);
    }
    speedEl.textContent=dspd;

    currentModeColor=getModeColorHex(mo);
    const el=$('maneuver');
    el.textContent=MODES[mo]||'IDLE';
    el.style.color=MODE_COLORS[mo]||MODE_COLORS[0];

    updateReadouts(gx,gy);
    $('yaw').textContent=Math.round(yawDeg);

    if(latg<0&&Math.abs(latg)>maxL){maxL=Math.abs(latg);$('max-l').textContent=maxL.toFixed(2)+'g'}
    if(latg>0&&latg>maxR){maxR=latg;$('max-r').textContent=maxR.toFixed(2)+'g'}
    if(lng>0&&lng>maxA){maxA=lng;$('max-a').textContent=maxA.toFixed(2)+'g'}
    if(lng<0&&Math.abs(lng)>maxB){maxB=Math.abs(lng);$('max-b').textContent=maxB.toFixed(2)+'g'}

    if(speed_ema>=2){updateScale(gx,gy)}else{magHist=[];currentScale=SCALE_STEPS[0];$('range-val').textContent='Range ±'+currentScale.toFixed(1)+'g'}

    const lastPt=trail.length>0?trail[trail.length-1]:null;
    const dx=lastPt?Math.abs(gx-lastPt.x):1,dy=lastPt?Math.abs(gy-lastPt.y):1;
    if(!lastPt||dx>TRAIL_JITTER_THRESHOLD||dy>TRAIL_JITTER_THRESHOLD){trail.push({x:gx,y:gy,t:now})}
    trail=trail.filter(p=>now-p.t<TRAIL_DURATION_MS+200);

    lastGpsState=gpsOk;
    updateLapTimer(lapTimeMs,lapCnt,lapFlags);
    drawG();
    cnt++;

    // Recording
    if(rec){
        chunkBuffer.push({t:now,sp,ax,ay,wz,mo,latg,lng,lat,lon,gpsOk,lon_imu:fusion.lon_imu,lon_gps:fusion.lon_gps,gps_wt:fusion.gps_wt,pitch_c:fusion.pitch_c,pitch_cf:fusion.pitch_cf,roll_c:fusion.roll_c,roll_cf:fusion.roll_cf,tilt_x:fusion.tilt_x,tilt_y:fusion.tilt_y});
        if(now-lastSaveTime>=CHUNK_INTERVAL)saveChunk();
    }
}

async function poll(){
    try{
        const r=await fetch('/api/telemetry');
        const j=await r.json();
        if(j.data&&j.seq!==lastSeq){
            lastSeq=j.seq;
            if(j.f){
                fusion.lon_imu=j.f.li||0;fusion.lon_gps=j.f.lg||0;fusion.gps_wt=j.f.gw||0;fusion.gps_rate=j.f.gr||0;
                fusion.pitch_c=j.f.pc||0;fusion.pitch_cf=j.f.pf||0;fusion.roll_c=j.f.rc||0;fusion.roll_cf=j.f.rf||0;
                fusion.tilt_x=j.f.tx||0;fusion.tilt_y=j.f.ty||0;
            }
            const b=atob(j.data),a=new Uint8Array(b.length);
            for(let i=0;i<b.length;i++)a[i]=b.charCodeAt(i);
            process(a.buffer);
            $('liveDot').classList.add('on');$('stxt').textContent='LIVE';
        }
        setTimeout(poll,33);
    }catch(e){$('liveDot').classList.remove('on');$('stxt').textContent='--';setTimeout(poll,500)}
}

// Diagnostics polling
function rateClass(actual,expected){const pct=actual/expected;if(pct>=0.9)return'ok';if(pct>=0.5)return'warn';return'err'}
async function updateDiag(){
    try{
        const r=await fetch('/api/diagnostics');
        const d=await r.json();
        if(d.error)return;
        $('wifi-mode').textContent=d.wifi.mode;
        $('wifi-ssid').textContent=d.wifi.ssid;
        $('telem-hz').textContent=d.config.telemetry_hz+' Hz';
        $('gps-model').textContent=d.config.gps_model;
        const imuEl=$('imu-rate');
        imuEl.textContent=d.sensor_rates.imu_hz.toFixed(0)+'/'+d.sensor_rates.imu_expected.toFixed(0)+' Hz';
        imuEl.className='bbVal bbNum '+rateClass(d.sensor_rates.imu_hz,d.sensor_rates.imu_expected);
        const gpsEl=$('gps-rate');
        gpsEl.textContent=d.sensor_rates.gps_hz.toFixed(1)+'/'+d.sensor_rates.gps_expected.toFixed(0)+' Hz';
        gpsEl.className='bbVal bbNum '+rateClass(d.sensor_rates.gps_hz,d.sensor_rates.gps_expected);
        const lr=$('loop-rate');
        lr.textContent=d.sensor_rates.loop_hz.toFixed(0)+' Hz';
        lr.className='bbVal bbNum '+(d.sensor_rates.loop_hz>500?'ok':(d.sensor_rates.loop_hz>200?'warn':'err'));
        $('zupt-rate').textContent=d.sensor_rates.zupt_per_min.toFixed(1)+'/min';
        $('ekf-per-gps').textContent=d.sensor_rates.ekf_per_gps.toFixed(1);
        $('gps-fix').textContent=d.gps.fix?'Valid':'No Fix';
        $('gps-fix').className='bbVal '+(d.gps.fix?'ok':'warn');
        $('gps-sats').textContent=d.gps.satellites;
        $('gps-sats').className='bbVal bbNum '+(d.gps.satellites>=4?'ok':(d.gps.satellites>=1?'warn':'err'));
        $('gps-hdop').textContent=d.gps.hdop.toFixed(1);
        $('gps-hdop').className='bbVal bbNum '+(d.gps.hdop<2?'ok':(d.gps.hdop<5?'warn':'err'));
        $('gps-warmup').textContent=d.gps.warmup?'Complete':'Warming...';
        $('gps-warmup').className='bbVal '+(d.gps.warmup?'ok':'warn');
        const ps=$('pos-sigma');ps.textContent=d.ekf.pos_sigma.toFixed(2)+' m';
        ps.className='bbVal bbNum '+(d.ekf.pos_sigma<5?'ok':(d.ekf.pos_sigma<10?'warn':'err'));
        const vs=$('vel-sigma');vs.textContent=d.ekf.vel_sigma.toFixed(2)+' m/s';
        vs.className='bbVal bbNum '+(d.ekf.vel_sigma<0.5?'ok':(d.ekf.vel_sigma<1.0?'warn':'err'));
        const ys=$('yaw-sigma');ys.textContent=d.ekf.yaw_sigma_deg.toFixed(1)+'°';
        ys.className='bbVal bbNum '+(d.ekf.yaw_sigma_deg<5?'ok':(d.ekf.yaw_sigma_deg<10?'warn':'err'));
        const bx=$('bias-x');bx.textContent=d.ekf.bias_x.toFixed(3)+' m/s²';
        bx.className='bbVal bbNum '+(Math.abs(d.ekf.bias_x)<0.3?'ok':(Math.abs(d.ekf.bias_x)<0.5?'warn':'err'));
        const by=$('bias-y');by.textContent=d.ekf.bias_y.toFixed(3)+' m/s²';
        by.className='bbVal bbNum '+(Math.abs(d.ekf.bias_y)<0.3?'ok':(Math.abs(d.ekf.bias_y)<0.5?'warn':'err'));
        const hp=$('heap');hp.textContent=(d.system.heap_free/1024).toFixed(0)+' KB';
        hp.className='bbVal bbNum '+(d.system.heap_free>40000?'ok':(d.system.heap_free>20000?'warn':'err'));
        $('tx-ok').textContent=d.system.tx_ok.toLocaleString();
        const txf=$('tx-fail');txf.textContent=d.system.tx_fail;
        txf.className='bbVal bbNum '+(d.system.tx_fail>0?'warn':'ok');
        const s=d.system.uptime_s;
        const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
        $('uptime').textContent='Uptime: '+h+'h '+m+'m '+sec+'s';
        if(d.fusion){
            $('lon-raw').textContent=d.fusion.lon_raw.toFixed(3)+' m/s²';
            $('lon-filt').textContent=d.fusion.lon_filtered.toFixed(3)+' m/s²';
            $('lon-blend').textContent=d.fusion.lon_blended.toFixed(3)+' m/s²';
            const wt=$('gps-wt');wt.textContent=(d.fusion.gps_weight*100).toFixed(0)+'%';
            wt.className='bbVal bbNum '+(d.fusion.gps_weight>0?'ok':'warn');
            const ga=$('d-gps-acc');ga.textContent=isNaN(d.fusion.gps_accel)?'N/A':d.fusion.gps_accel.toFixed(3)+' m/s²';
            const rej=$('gps-rej');rej.textContent=d.fusion.gps_rejected?'Yes':'No';
            rej.className='bbVal '+(d.fusion.gps_rejected?'warn':'ok');
            const pc=$('pitch-corr');pc.textContent=d.fusion.pitch_corr.toFixed(1)+'°';
            pc.className='bbVal bbNum '+(Math.abs(d.fusion.pitch_corr)<10?'ok':'warn');
            const rc=$('roll-corr');rc.textContent=d.fusion.roll_corr.toFixed(1)+'°';
            rc.className='bbVal bbNum '+(Math.abs(d.fusion.roll_corr)<10?'ok':'warn');
            const oc=$('orient-conf');oc.textContent=d.fusion.pitch_conf.toFixed(0)+'%/'+d.fusion.roll_conf.toFixed(0)+'%';
            oc.className='bbVal bbNum '+(d.fusion.pitch_conf>50?'ok':(d.fusion.pitch_conf>10?'warn':'err'));
            const yb=$('yaw-bias');const ybVal=Math.abs(d.fusion.yaw_bias*1000);
            yb.textContent=ybVal.toFixed(1)+' mrad/s';
            yb.className='bbVal bbNum '+(ybVal<10?'ok':(ybVal<50?'warn':'err'));
            const yc=$('yaw-cal');yc.textContent=d.fusion.yaw_calibrated?'Yes':'No';
            yc.className='bbVal '+(d.fusion.yaw_calibrated?'ok':'warn');
            const txy=$('tilt-xy');txy.textContent=d.fusion.tilt_x.toFixed(3)+'/'+d.fusion.tilt_y.toFixed(3);
            const tiltMax=Math.max(Math.abs(d.fusion.tilt_x),Math.abs(d.fusion.tilt_y));
            txy.className='bbVal bbNum '+(tiltMax<0.3?'ok':(tiltMax<0.5?'warn':'err'));
        }
    }catch(e){}
}

// Menu
$('menu-btn').onclick=()=>$('menu-overlay').classList.add('open');
$('menu-overlay').onclick=e=>{if(e.target===$('menu-overlay'))$('menu-overlay').classList.remove('open')};

$('menu-rec').onclick=$('diag-rec').onclick=async()=>{
    $('menu-overlay').classList.remove('open');
    if(rec)await stopRecording();else await startRecording();
};
$('menu-export').onclick=$('diag-export').onclick=()=>{$('menu-overlay').classList.remove('open');exportCSV()};
$('menu-tracks').onclick=$('btn-tracks').onclick=()=>{$('menu-overlay').classList.remove('open');openTrackModal()};
$('track-modal-close').onclick=closeTrackModal;
$('track-modal').onclick=e=>{if(e.target===$('track-modal'))closeTrackModal()};
$('btn-clear-track').onclick=clearActiveTrack;
$('menu-clear').onclick=()=>{$('menu-overlay').classList.remove('open');resetState()};

// Init
setInterval(()=>{
    $('hz').textContent=cnt;cnt=0;
    $('session-time').textContent=fmtTime(Date.now()-displaySessionStart);
    if(lastGpsState){$('gpsDot').classList.add('on');$('gpsHz').textContent='GPS '+Math.round(fusion.gps_rate)}
    else{$('gpsDot').classList.remove('on');$('gpsHz').textContent='GPS --'}
    $('gps-acc').textContent=lastGpsState?'2':'--';
    updateRecUI();
},1000);

setInterval(updateDiag,1000);

openDB().then(()=>{
    // Check for incomplete sessions on load
    const tx=db.transaction('sessions','readonly');
    tx.objectStore('sessions').getAll().onsuccess=(e)=>{
        const active=e.target.result.filter(s=>s.status==='active');
        if(active.length>0){
            const latest=active.sort((a,b)=>b.sessionId-a.sessionId)[0];
            sessionId=latest.sessionId;sessionStart=latest.startTime;
            // Count existing chunks
            const ctx=db.transaction('chunks','readonly');
            ctx.objectStore('chunks').getAll().onsuccess=(e2)=>{
                chunkIndex=e2.target.result.filter(c=>c.sessionId===sessionId).length;
                $('rec-status').textContent='⚠ Recovered';$('rec-status').className='bbVal warn';
                updateRecUI();
            };
        }
    };
}).catch(e=>console.log('IndexedDB error:',e));

openTrackDB().catch(e=>console.log('Track DB error:',e));

resize();setTimeout(()=>{resize();drawG()},50);
poll();
updateDiag();
</script></body></html>"#;

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
            max_uri_handlers: 8,
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

        // HTTP polling endpoint - includes fusion data for synchronized CSV export
        let state_poll = state.clone();
        server.fn_handler(
            "/api/telemetry",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let json = if let Some(data) = state_poll.get_telemetry_base64() {
                    // Include fusion diagnostics inline for perfect synchronization
                    if let Some(diag_state) = state_poll.diagnostics() {
                        let f = diag_state.snapshot().fusion;
                        format!(
                            concat!(
                                r#"{{"seq":{},"data":"{}","#,
                                r#""f":{{"li":{:.4},"lg":{:.4},"gw":{:.2},"gr":{:.1},"pc":{:.1},"pf":{:.0},"rc":{:.1},"rf":{:.0},"tx":{:.4},"ty":{:.4}}}}}"#
                            ),
                            state_poll.packet_count(),
                            data,
                            f.lon_imu_filtered,
                            f.gps_accel,
                            f.gps_weight,
                            f.gps_rate,
                            f.pitch_correction_deg,
                            f.pitch_confidence * 100.0,
                            f.roll_correction_deg,
                            f.roll_confidence * 100.0,
                            f.tilt_offset_x,
                            f.tilt_offset_y,
                        )
                    } else {
                        format!(
                            r#"{{"seq":{},"data":"{}"}}"#,
                            state_poll.packet_count(),
                            data
                        )
                    }
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
                    let d = diag_state.snapshot();
                    format!(
                        concat!(
                            r#"{{"sensor_rates":{{"imu_hz":{:.1},"gps_hz":{:.1},"loop_hz":{:.0},"imu_expected":{:.0},"gps_expected":{:.0},"zupt_per_min":{:.1},"ekf_per_gps":{:.1}}},"#,
                            r#""ekf":{{"pos_sigma":{:.2},"vel_sigma":{:.2},"yaw_sigma_deg":{:.1},"bias_x":{:.4},"bias_y":{:.4}}},"#,
                            r#""system":{{"heap_free":{},"uptime_s":{},"tx_ok":{},"tx_fail":{}}},"#,
                            r#""gps":{{"model":"{}","rate_hz":{},"fix":{},"warmup":{},"satellites":{},"hdop":{:.1}}},"#,
                            r#""wifi":{{"mode":"{}","ssid":"{}"}},"#,
                            r#""config":{{"telemetry_hz":{},"gps_model":"{}","warmup_fixes":{}}},"#,
                            r#""fusion":{{"lon_raw":{:.3},"lon_filtered":{:.3},"lon_blended":{:.3},"gps_weight":{:.2},"gps_accel":{:.3},"gps_rate":{:.1},"gps_rejected":{},"pitch_corr":{:.1},"roll_corr":{:.1},"pitch_conf":{:.0},"roll_conf":{:.0},"yaw_bias":{:.4},"yaw_calibrated":{},"tilt_x":{:.3},"tilt_y":{:.3},"tilt_valid":{}}}}}"#
                        ),
                        d.sensor_rates.imu_hz, d.sensor_rates.gps_hz, d.sensor_rates.loop_hz,
                        d.sensor_rates.imu_expected_hz, d.sensor_rates.gps_expected_hz,
                        d.sensor_rates.zupt_per_min, d.sensor_rates.ekf_predictions_per_gps,
                        d.ekf_health.position_sigma, d.ekf_health.velocity_sigma,
                        d.ekf_health.yaw_sigma_deg, d.ekf_health.bias_x, d.ekf_health.bias_y,
                        d.system_health.free_heap_bytes, d.system_health.uptime_seconds,
                        d.system_health.telemetry_sent, d.system_health.telemetry_failed,
                        d.gps_health.model_name, d.gps_health.configured_rate_hz,
                        d.gps_health.fix_valid, d.gps_health.warmup_complete,
                        d.gps_health.satellites, d.gps_health.hdop,
                        d.wifi_status.mode, d.wifi_status.ssid,
                        d.config.telemetry_rate_hz, d.config.gps_model, d.config.gps_warmup_fixes,
                        // Fusion diagnostics
                        d.fusion.lon_imu_raw, d.fusion.lon_imu_filtered, d.fusion.lon_blended,
                        d.fusion.gps_weight, d.fusion.gps_accel, d.fusion.gps_rate, d.fusion.gps_rejected,
                        d.fusion.pitch_correction_deg, d.fusion.roll_correction_deg,
                        d.fusion.pitch_confidence * 100.0, d.fusion.roll_confidence * 100.0,
                        d.fusion.yaw_bias, d.fusion.yaw_calibrated,
                        d.fusion.tilt_offset_x, d.fusion.tilt_offset_y, d.fusion.tilt_valid
                    )
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
        // GET /api/laptimer/configure?type=loop&p1_x=...&p1_y=...&p2_x=...&p2_y=...&dir=...
        // GET /api/laptimer/configure?type=clear
        // GET /api/laptimer/configure?type=point_to_point&p1_x=...&p1_y=...&p2_x=...&p2_y=...&dir=...&f_p1_x=...&f_p1_y=...&f_p2_x=...&f_p2_y=...&f_dir=...
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
