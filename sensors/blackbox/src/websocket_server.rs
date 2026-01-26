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
    --mono:ui-monospace,SFMono-Regular,SF Mono,Menlo,Consolas,Liberation Mono,monospace;
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
.bbDeltaBar{display:none;flex-direction:column;align-items:center;gap:4px;margin:6px 0 8px}
.bbLapCard.timing .bbDeltaBar{display:flex}
.bbDeltaText{font-size:28px;font-weight:800;font-variant-numeric:tabular-nums;letter-spacing:-0.02em;color:var(--text);text-shadow:0 2px 6px rgba(0,0,0,0.25);transition:color 0.12s ease;line-height:1}
.bbDeltaText.ahead{color:var(--ok);text-shadow:0 0 16px rgba(52,199,89,0.5),0 2px 6px rgba(0,0,0,0.2)}
.bbDeltaText.behind{color:var(--red);text-shadow:0 0 16px rgba(255,59,48,0.5),0 2px 6px rgba(0,0,0,0.2)}
.bbDeltaTrack{position:relative;width:84%;height:10px;background:linear-gradient(180deg,rgba(255,255,255,0.04) 0%,rgba(0,0,0,0.12) 100%);border:1px solid rgba(255,255,255,0.06);border-radius:5px;overflow:hidden}
.bbDeltaCenter{position:absolute;left:50%;top:-1px;bottom:-1px;width:2px;background:rgba(255,255,255,0.3);transform:translateX(-50%);border-radius:1px;z-index:1}
.bbDeltaFill{position:absolute;top:0;height:100%;width:0%;left:50%;background:var(--ok);transition:width 0.08s cubic-bezier(0.25,0.46,0.45,0.94),left 0.08s cubic-bezier(0.25,0.46,0.45,0.94),background 0.12s ease,box-shadow 0.15s ease;border-radius:5px}
.bbDeltaFill.ahead{background:linear-gradient(90deg,var(--ok) 0%,rgba(52,199,89,0.8) 100%)}
.bbDeltaFill.behind{background:linear-gradient(270deg,var(--red) 0%,rgba(255,59,48,0.8) 100%)}
.bbDeltaBar.no-ref .bbDeltaTrack{opacity:0.2}
.bbDeltaBar.no-ref .bbDeltaCenter{opacity:0.2}
.bbDeltaBar.no-ref .bbDeltaText{color:var(--text-tertiary);opacity:0.3;text-shadow:none}
.bbDeltaBar.no-ref .bbDeltaFill{display:none}
.bbDeltaFill.glow-ahead{box-shadow:0 0 16px rgba(52,199,89,0.7),0 0 32px rgba(52,199,89,0.3)}
.bbDeltaFill.glow-behind{box-shadow:0 0 16px rgba(255,59,48,0.7),0 0 32px rgba(255,59,48,0.3)}
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
.bbExportAllBtn{background:var(--card-bg);border:1px solid var(--border);border-radius:4px;color:var(--text-secondary);padding:4px 10px;font-size:12px;cursor:pointer;margin-right:12px}
.bbExportAllBtn:active{opacity:0.7}
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
.bbTrackBtn.secondary{background:var(--surface);color:var(--text-secondary);padding:6px 8px}
.bbTrackItem{flex-wrap:wrap}
.bbTrackHistory{width:100%;margin-top:12px;padding-top:12px;border-top:1px solid var(--divider)}
.bbHistoryEmpty{display:flex;flex-direction:column;align-items:center;padding:16px;color:var(--text-tertiary);font-size:13px;text-align:center}
.bbHistoryHint{font-size:11px;margin-top:4px;opacity:0.7}
.bbHistorySummary{display:flex;justify-content:space-between;font-size:12px;color:var(--text-secondary);margin-bottom:10px}
.bbHistoryTotal{font-weight:600}
.bbHistorySession{background:var(--bg);border-radius:8px;padding:10px 12px;margin-bottom:8px}
.bbHistorySession:last-child{margin-bottom:0}
.bbHistorySessionHeader{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}
.bbHistoryDate{font-size:12px;font-weight:600;color:var(--text)}
.bbHistorySessionStats{font-size:11px;color:var(--text-tertiary)}
.bbHistoryLaps{display:flex;flex-wrap:wrap;gap:6px}
.bbHistoryLap{font-size:11px;font-family:var(--mono);color:var(--text-secondary);background:var(--surface);padding:3px 6px;border-radius:4px}
.bbHistoryLap.best{color:var(--ok);font-weight:600}
.bbHistoryMore{font-size:11px;color:var(--text-tertiary);text-align:center;padding:8px;font-style:italic}
.bbHistoryActions{margin-top:12px;text-align:center}
.bbHistoryClearBtn{background:transparent;border:1px solid var(--red);color:var(--red);font-size:11px;padding:6px 12px;border-radius:6px;cursor:pointer;font-family:inherit}
.bbHistoryClearBtn:active{opacity:0.7}
.bbActiveTrack{display:flex;align-items:center;gap:12px;background:rgba(52,199,89,0.1);border:1px solid rgba(52,199,89,0.2);border-radius:12px;padding:12px 14px;margin-bottom:16px}
.bbActiveTrackInfo{flex:1;min-width:0}
.bbActiveTrackName{font-size:15px;font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.bbActiveTrackMeta{font-size:12px;color:var(--text-secondary);margin-top:2px}
.bbActiveTrackClear{width:28px;height:28px;border-radius:50%;border:none;background:rgba(128,128,128,0.12);color:var(--text-tertiary);font-size:16px;cursor:pointer;display:flex;align-items:center;justify-content:center;flex-shrink:0}
.bbStorageOverview{background:var(--bg);border-radius:12px;padding:16px;margin-bottom:16px}
.bbStorageTotal{text-align:center;margin-bottom:12px;padding-bottom:12px;border-bottom:1px solid var(--divider)}
.bbStorageLabel{font-size:11px;color:var(--text-tertiary);text-transform:uppercase;letter-spacing:0.05em}
.bbStorageValue{font-size:24px;font-weight:700;margin-top:4px}
.bbStorageBreakdown{display:flex;justify-content:space-around}
.bbStorageItem{text-align:center}
.bbStorageItemLabel{font-size:11px;color:var(--text-tertiary);display:block}
.bbStorageItemValue{font-size:14px;font-weight:600;margin-top:2px}
.bbSectionCount{font-size:12px;font-weight:normal;color:var(--text-tertiary)}
.bbTrackDataList{border-radius:12px;overflow:hidden;background:var(--bg);max-height:250px;overflow-y:auto}
.bbTrackDataItem{padding:14px 16px;border-bottom:1px solid var(--divider);display:flex;justify-content:space-between;align-items:center;gap:12px}
.bbTrackDataItem:last-child{border-bottom:none}
.bbTrackDataInfo{flex:1;min-width:0}
.bbTrackDataName{font-size:14px;font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.bbTrackDataMeta{font-size:11px;color:var(--text-tertiary);margin-top:3px;display:flex;gap:8px}
.bbTrackDataRef{margin-top:6px;font-size:12px}
.bbTrackDataRef.has-ref{color:var(--ok);font-weight:500}
.bbTrackDataRef.no-ref{color:var(--text-tertiary);font-style:italic}
.bbTrackDataActions{display:flex;gap:8px;flex-shrink:0}
.bbSessionList{border-radius:12px;overflow:hidden;background:var(--bg);max-height:calc(100vh - 350px);overflow-y:auto}
.bbSessionEmpty{padding:30px 20px;text-align:center;color:var(--text-tertiary);font-size:13px}
.bbSessionEmpty .icon{font-size:32px;margin-bottom:10px;opacity:0.4}
.bbSessionItem{padding:14px 16px;border-bottom:1px solid var(--divider);display:flex;justify-content:space-between;align-items:center;gap:12px}
.bbSessionItem:last-child{border-bottom:none}
.bbSessionItem.active{background:rgba(52,199,89,0.08)}
.bbSessionInfo{flex:1;min-width:0}
.bbSessionDate{font-size:14px;font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.bbSessionMeta{font-size:11px;color:var(--text-tertiary);margin-top:3px;display:flex;gap:8px;flex-wrap:wrap}
.bbSessionStatus{font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:0.05em;margin-top:4px}
.bbSessionStatus.complete{color:var(--ok)}
.bbSessionStatus.active,.bbSessionStatus.recovered{color:var(--amber)}
.bbSessionActions{display:flex;gap:8px;flex-shrink:0}
.bbSessionBtn{background:var(--surface);border:none;border-radius:6px;padding:8px 14px;font-size:12px;font-weight:600;color:var(--text);cursor:pointer;font-family:inherit;transition:opacity 0.15s}
.bbSessionBtn:active{opacity:0.7}
.bbSessionBtn:disabled{opacity:0.4;cursor:not-allowed}
.bbSessionBtn.danger{color:var(--red)}
.bbClearAllBtn{display:block;width:100%;padding:14px 18px;margin-top:16px;border:none;border-radius:12px;font-size:14px;font-weight:600;cursor:pointer;font-family:inherit;background:var(--bg);color:var(--red);transition:opacity 0.15s}
.bbClearAllBtn:active{opacity:0.7}
.bbClearAllBtn:disabled{opacity:0.4;cursor:not-allowed}
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
.bbToast{position:fixed;bottom:80px;left:50%;transform:translateX(-50%);background:var(--surface);border-radius:12px;padding:12px 16px;box-shadow:0 4px 20px rgba(0,0,0,.25);display:flex;align-items:center;gap:12px;z-index:10001;animation:bbToastSlideIn 0.3s ease-out;max-width:calc(100vw - 32px)}
@keyframes bbToastSlideIn{from{opacity:0;transform:translateX(-50%) translateY(20px)}to{opacity:1;transform:translateX(-50%) translateY(0)}}
.bbToast.dismissing{animation:bbToastSlideOut 0.2s ease-in forwards}
@keyframes bbToastSlideOut{from{opacity:1;transform:translateX(-50%) translateY(0)}to{opacity:0;transform:translateX(-50%) translateY(20px)}}
.bbToastIcon{font-size:24px;flex-shrink:0}
.bbToastText{flex:1;min-width:0}
.bbToastText strong{display:block;font-size:14px;font-weight:600;margin-bottom:2px}
.bbToastText span{font-size:12px;color:var(--text-secondary);display:block;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.bbToastActions{display:flex;gap:8px;flex-shrink:0}
.bbToastBtn{padding:8px 14px;border-radius:8px;border:none;font-size:13px;font-weight:600;cursor:pointer;font-family:inherit}
.bbToastBtn.primary{background:var(--ok);color:#fff}
.bbToastBtn.secondary{background:var(--divider);color:var(--text)}
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
                <div class="bbDeltaBar no-ref" id="delta-bar"><div class="bbDeltaText" id="delta-text">Set best lap</div><div class="bbDeltaTrack"><div class="bbDeltaCenter"></div><div class="bbDeltaFill" id="delta-fill"></div></div></div>
                <div class="bbLapTrackName" id="lap-track-name"></div>
                <div class="bbLapMeta">
                    <span class="bbLapCount" id="lap-count">Lap 1</span>
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
        <button class="bbMenuItem" id="menu-data">Data</button>
        <button class="bbMenuItem" id="menu-rec">Start Recording</button>
        <button class="bbMenuItem" id="menu-export">Export CSV</button>
        <button class="bbMenuItem destructive" id="menu-clear">Clear Session</button>
    </div>
</div>

<div class="bbModal" id="track-modal">
    <div class="bbModalContent">
        <div class="bbModalHeader">
            <span class="bbModalTitle">Tracks</span>
            <button class="bbExportAllBtn" id="btn-export-all" title="Export all tracks">Export All</button>
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

<div class="bbModal" id="data-modal">
    <div class="bbModalContent">
        <div class="bbModalHeader">
            <span class="bbModalTitle">Data</span>
            <button class="bbModalClose" id="data-modal-close">&times;</button>
        </div>
        <div class="bbModalBody">
            <div class="bbStorageOverview">
                <div class="bbStorageTotal"><div class="bbStorageLabel">Total Storage</div><div class="bbStorageValue" id="data-total-size">—</div></div>
                <div class="bbStorageBreakdown"><div class="bbStorageItem"><span class="bbStorageItemLabel">Recordings</span><span class="bbStorageItemValue" id="data-rec-size">—</span></div><div class="bbStorageItem"><span class="bbStorageItemLabel">Tracks</span><span class="bbStorageItemValue" id="data-track-size">—</span></div></div>
            </div>
            <div class="bbModalSection">
                <div class="bbSectionTitle">Recordings <span class="bbSectionCount" id="data-rec-count"></span></div>
                <div class="bbSessionList" id="session-list"><div class="bbSessionEmpty"><div class="icon">📊</div>Loading...</div></div>
                <button class="bbClearAllBtn" id="btn-clear-all-recordings">Clear All Recordings</button>
            </div>
            <div class="bbModalSection">
                <div class="bbSectionTitle">Tracks & Best Laps <span class="bbSectionCount" id="data-track-count"></span></div>
                <div class="bbTrackDataList" id="track-data-list"><div class="bbSessionEmpty"><div class="icon">🏁</div>Loading...</div></div>
                <button class="bbClearAllBtn" id="btn-clear-all-tracks">Clear All Tracks</button>
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
const DB_NAME='blackbox-rec',DB_VERSION=2,CHUNK_INTERVAL=60000;

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
            const tx=e.target.transaction;
            const oldVersion=e.oldVersion;
            if(!d.objectStoreNames.contains('chunks')){
                const cs=d.createObjectStore('chunks',{keyPath:['sessionId','chunkIndex']});
                cs.createIndex('sessionId','sessionId',{unique:false});
            }
            if(!d.objectStoreNames.contains('sessions')){
                d.createObjectStore('sessions',{keyPath:'sessionId'});
            }
            if(oldVersion>=1&&oldVersion<2){
                const cs=tx.objectStore('chunks');
                if(!cs.indexNames.contains('sessionId')){
                    cs.createIndex('sessionId','sessionId',{unique:false});
                }
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
    let csv='time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid,ekf_x,ekf_y,ekf_yaw,gps_course,lon_imu,lon_gps,gps_weight,pitch_corr,pitch_conf,roll_corr,roll_conf,tilt_x,tilt_y\n';
    allData.forEach(r=>{const gc=r.gpsCourse;const gcStr=isNaN(gc)?'':gc.toFixed(4);csv+=r.t+','+r.sp+','+r.ax+','+r.ay+','+r.wz+','+r.mo+','+r.latg+','+r.lng+','+(r.lat||0)+','+(r.lon||0)+','+(r.gpsOk||0)+','+(r.ekfX||0).toFixed(3)+','+(r.ekfY||0).toFixed(3)+','+(r.ekfYaw||0).toFixed(4)+','+gcStr+','+(r.lon_imu||0).toFixed(4)+','+(r.lon_gps||0).toFixed(4)+','+(r.gps_wt||0).toFixed(2)+','+(r.pitch_c||0).toFixed(2)+','+(r.pitch_cf||0).toFixed(1)+','+(r.roll_c||0).toFixed(2)+','+(r.roll_cf||0).toFixed(1)+','+(r.tilt_x||0).toFixed(4)+','+(r.tilt_y||0).toFixed(4)+'\n'});
    const b=new Blob([csv],{type:'text/csv'}),u=URL.createObjectURL(b),a=document.createElement('a');
    a.href=u;a.download='blackbox_'+new Date(latest.sessionId).toISOString().slice(0,19).replace(/[T:]/g,'-')+'.csv';a.click();
}

// Data Management Functions
function idbPromise(r){return new Promise((res,rej)=>{r.onsuccess=()=>res(r.result);r.onerror=()=>rej(r.error)})}

async function getAllSessions(){
    if(!db)await openDB();
    const tx=db.transaction(['sessions','chunks'],'readonly');
    const sessions=await idbPromise(tx.objectStore('sessions').getAll());
    const allChunks=await idbPromise(tx.objectStore('chunks').getAll());
    const chunksBySession=new Map();
    for(const c of allChunks){
        if(!chunksBySession.has(c.sessionId))chunksBySession.set(c.sessionId,{count:0,samples:0});
        const s=chunksBySession.get(c.sessionId);s.count++;s.samples+=(c.data&&c.data.length)||0;
    }
    const enriched=sessions.map(s=>{
        const cs=chunksBySession.get(s.sessionId)||{count:0,samples:0};
        return{sessionId:s.sessionId,startTime:s.startTime||s.sessionId,status:s.status,chunkCount:cs.count,sampleCount:cs.samples,durationMs:cs.samples*50,estimatedBytes:cs.samples*50};
    });
    enriched.sort((a,b)=>b.sessionId-a.sessionId);
    return enriched;
}

async function getStorageStats(){
    const sessions=await getAllSessions();
    const totalBytes=sessions.reduce((sum,s)=>sum+s.estimatedBytes,0);
    return{sessionCount:sessions.length,totalBytes,sessions};
}

async function deleteSession(targetId){
    if(!db)await openDB();
    if(rec&&targetId===sessionId)throw new Error('Cannot delete active recording');
    const tx=db.transaction(['sessions','chunks'],'readwrite');
    const cs=tx.objectStore('chunks'),idx=cs.index('sessionId'),range=IDBKeyRange.only(targetId);
    await new Promise((res,rej)=>{
        const cur=idx.openCursor(range);
        cur.onerror=()=>rej(cur.error);
        cur.onsuccess=e=>{const c=e.target.result;if(c){c.delete();c.continue()}else res()};
    });
    await idbPromise(tx.objectStore('sessions').delete(targetId));
    await new Promise((res,rej)=>{tx.oncomplete=res;tx.onerror=()=>rej(tx.error)});
}

async function exportSessionById(targetId){
    if(!db)await openDB();
    if(rec&&targetId===sessionId&&chunkBuffer.length>0)await saveChunk();
    const tx=db.transaction(['sessions','chunks'],'readonly');
    const session=await idbPromise(tx.objectStore('sessions').get(targetId));
    if(!session)throw new Error('Session not found');
    const idx=tx.objectStore('chunks').index('sessionId');
    const chunks=await idbPromise(idx.getAll(targetId));
    chunks.sort((a,b)=>a.chunkIndex-b.chunkIndex);
    if(!chunks.length)throw new Error('No data in recording');
    let allData=[];chunks.forEach(c=>allData=allData.concat(c.data||[]));
    let csv='time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid,ekf_x,ekf_y,ekf_yaw,gps_course,lon_imu,lon_gps,gps_weight,pitch_corr,pitch_conf,roll_corr,roll_conf,tilt_x,tilt_y\n';
    allData.forEach(r=>{const gc=r.gpsCourse;const gcStr=isNaN(gc)?'':gc.toFixed(4);csv+=r.t+','+r.sp+','+r.ax+','+r.ay+','+r.wz+','+r.mo+','+r.latg+','+r.lng+','+(r.lat||0)+','+(r.lon||0)+','+(r.gpsOk||0)+','+(r.ekfX||0).toFixed(3)+','+(r.ekfY||0).toFixed(3)+','+(r.ekfYaw||0).toFixed(4)+','+gcStr+','+(r.lon_imu||0).toFixed(4)+','+(r.lon_gps||0).toFixed(4)+','+(r.gps_wt||0).toFixed(2)+','+(r.pitch_c||0).toFixed(2)+','+(r.pitch_cf||0).toFixed(1)+','+(r.roll_c||0).toFixed(2)+','+(r.roll_cf||0).toFixed(1)+','+(r.tilt_x||0).toFixed(4)+','+(r.tilt_y||0).toFixed(4)+'\n'});
    const b=new Blob([csv],{type:'text/csv'}),u=URL.createObjectURL(b),a=document.createElement('a');
    a.href=u;a.download='blackbox_'+new Date(targetId).toISOString().slice(0,19).replace(/[T:]/g,'-')+'.csv';a.click();
    return{session,sampleCount:allData.length};
}

async function clearAllRecordings(){
    if(rec)throw new Error('Cannot clear while recording');
    if(!db)await openDB();
    const tx=db.transaction(['sessions','chunks'],'readwrite');
    await idbPromise(tx.objectStore('sessions').clear());
    await idbPromise(tx.objectStore('chunks').clear());
    await new Promise((res,rej)=>{tx.oncomplete=res;tx.onerror=()=>rej(tx.error)});
}

function formatLapTimeMs(ms){if(!ms||ms<=0)return'--:--.---';const m=Math.floor(ms/60000);const s=Math.floor((ms%60000)/1000);const msc=ms%1000;return m+':'+(s<10?'0':'')+s+'.'+(msc<100?'0':'')+(msc<10?'0':'')+msc}
function formatSessionDuration(ms){const totalSec=Math.floor(ms/1000),h=Math.floor(totalSec/3600),m=Math.floor((totalSec%3600)/60);if(h>0)return h+'h '+m+'m';return m+'m'}
function estimateObjectSize(obj){if(obj===null||obj===undefined)return 0;if(typeof obj==='string')return obj.length*2;if(typeof obj==='number')return 8;if(typeof obj==='boolean')return 4;if(obj instanceof ArrayBuffer)return obj.byteLength;if(Array.isArray(obj))return obj.reduce((s,i)=>s+estimateObjectSize(i),0);if(typeof obj==='object')return Object.keys(obj).reduce((s,k)=>s+k.length*2+estimateObjectSize(obj[k]),0);return 8}

async function getTrackDataStats(){
    if(!trackDb)await openTrackDB();
    const tx=trackDb.transaction(['tracks','reference_laps'],'readonly');
    const tracks=await idbPromise(tx.objectStore('tracks').getAll());
    const refs=await idbPromise(tx.objectStore('reference_laps').getAll());
    const refMap=new Map(refs.map(r=>[r.trackId,r]));
    let totalBytes=0;
    const result=tracks.map(t=>{
        const ref=refMap.get(t.id);
        const tBytes=estimateObjectSize(t);
        const rBytes=ref?estimateObjectSize(ref):0;
        totalBytes+=tBytes+rBytes;
        const corners=typeof t.corners==='number'?t.corners:(Array.isArray(t.corners)?t.corners.length:0);
        const points=typeof t.keyPoints==='number'?t.keyPoints:((t.centerline&&t.centerline.length)||0);
        return{id:t.id,name:t.name,corners:corners,points:points,totalBytes:tBytes+rBytes,hasReferenceLap:!!ref,referenceLapTime:(ref&&ref.lapTimeMs)||0,referenceLapSamples:(ref&&ref.samples&&ref.samples.length)||0}
    });
    return{tracks:result,totalBytes}
}

async function getCombinedStorageStats(){
    const recStats=await getStorageStats();
    const trackStats=await getTrackDataStats();
    return{totalBytes:recStats.totalBytes+trackStats.totalBytes,recordingsBytes:recStats.totalBytes,tracksBytes:trackStats.totalBytes,sessionCount:recStats.sessionCount,trackCount:trackStats.tracks.length}
}

async function deleteTrackWithRef(trackId){
    if(!trackDb)await openTrackDB();
    // First delete lap history using its index
    await deleteLapHistory(trackId);
    // Then delete track and reference lap in single transaction
    const tx=trackDb.transaction(['tracks','reference_laps'],'readwrite');
    await idbPromise(tx.objectStore('tracks').delete(trackId));
    await idbPromise(tx.objectStore('reference_laps').delete(trackId));
    await new Promise((res,rej)=>{tx.oncomplete=res;tx.onerror=()=>rej(tx.error)});
    if(activeTrack&&activeTrack.id===trackId)clearSession();
}

async function clearAllTracks(){
    if(!trackDb)await openTrackDB();
    const tx=trackDb.transaction(['tracks','reference_laps','lap_history'],'readwrite');
    await idbPromise(tx.objectStore('tracks').clear());
    await idbPromise(tx.objectStore('reference_laps').clear());
    await idbPromise(tx.objectStore('lap_history').clear());
    await new Promise((res,rej)=>{tx.oncomplete=res;tx.onerror=()=>rej(tx.error)});
    clearSession();
}

function formatBytes(bytes){
    if(bytes===0)return'0 B';
    const k=1024,sizes=['B','KB','MB','GB'],i=Math.floor(Math.log(bytes)/Math.log(k));
    return parseFloat((bytes/Math.pow(k,i)).toFixed(1))+' '+sizes[i];
}

function formatSessionDate(ts){
    const d=new Date(ts),now=new Date();
    const isToday=d.toDateString()===now.toDateString();
    const isYesterday=d.toDateString()===new Date(now-86400000).toDateString();
    const time=d.toLocaleTimeString([],{hour:'numeric',minute:'2-digit'});
    if(isToday)return'Today '+time;
    if(isYesterday)return'Yesterday '+time;
    return d.toLocaleDateString([],{month:'short',day:'numeric',year:'numeric'})+' '+time;
}

function formatDuration(ms){
    const totalSec=Math.floor(ms/1000),h=Math.floor(totalSec/3600),m=Math.floor((totalSec%3600)/60),s=totalSec%60;
    if(h>0)return h+'h '+m+'m';
    if(m>0)return m+'m '+s+'s';
    return s+'s';
}

// TrackRecorder - Adaptive sampling for track recording with corner state machine
class TrackRecorder{
    constructor(){this.config={minDistance:2,maxDistance:30,headingThreshold:0.15,cornerEntryThreshold:0.025,cornerExitThreshold:0.012,minCornerLength:3,minLoopDistance:150,closeProximity:12};this.reset()}
    reset(){this.recording=false;this.trackType='loop';this.startPos=null;this.startHeading=0;this.startTime=0;this.keyPoints=[];this.rawSamples=[];this.lastPos=null;this.lastSigma=3.0;this.totalDistance=0;this.loopDetected=false;this.inCorner=false;this.cornerCount=0;this.cornerEntryDist=0;this.speedGate=false}
    start(pos,trackType='loop'){if(this.recording)return false;this.reset();this.recording=true;this.trackType=trackType;this.startPos={x:pos.x,y:pos.y};this.startHeading=pos.heading||0;this.startTime=Date.now();this.keyPoints.push({x:pos.x,y:pos.y,lat:pos.lat||0,lon:pos.lon||0,heading:pos.heading||0,speed:pos.speed||0,sigma:pos.sigma||3.0,t:Date.now(),curvature:0,isCorner:false});this.lastPos={x:pos.x,y:pos.y,heading:pos.heading||0};return true}
    addSample(pos){if(!this.recording||!this.lastPos)return{stored:false};
    // Check loop closure FIRST - even when stopped at start position
    // Only check distance traveled + proximity to start (heading check removed - unreliable due to GPS/EKF reference frame mismatch)
    if(this.trackType==='loop'&&this.totalDistance>=this.config.minLoopDistance){const dts=Math.sqrt((pos.x-this.startPos.x)**2+(pos.y-this.startPos.y)**2);if(dts<this.config.closeProximity){this.loopDetected=true;return{stored:true,loopDetected:true,stats:this.getStats()}}}
    // Speed gate - skip samples while stationary (but loop check already ran above)
    const spd=pos.speed||0;if(!this.speedGate){if(spd<2.0)return{stored:false,stats:this.getStats()};this.speedGate=true;this.lastPos={x:pos.x,y:pos.y,heading:pos.heading||0};if(this.keyPoints.length<=1){this.startHeading=pos.heading||0}}else{if(spd<0.5){this.speedGate=false;return{stored:false,stats:this.getStats()}}}const sigma=pos.sigma||3.0;this.lastSigma=sigma;const dx=pos.x-this.lastPos.x,dy=pos.y-this.lastPos.y,dist=Math.sqrt(dx*dx+dy*dy);if(dist<0.5)return{stored:false,stats:this.getStats()};const speedBasedMinDist=Math.max(this.config.minDistance,pos.speed*0.1);const headingChange=Math.abs(this._wrapAngle(pos.heading-this.lastPos.heading));const shouldStore=dist>=speedBasedMinDist||headingChange>=this.config.headingThreshold||dist>=this.config.maxDistance;if(!shouldStore)return{stored:false,stats:this.getStats()};const curvature=dist>0.1?headingChange/dist:0;let isCorner=false;if(!this.inCorner){if(curvature>this.config.cornerEntryThreshold){this.inCorner=true;this.cornerEntryDist=this.totalDistance;isCorner=true}}else{isCorner=true;if(curvature<this.config.cornerExitThreshold){const cornerLength=this.totalDistance-this.cornerEntryDist;if(cornerLength>=this.config.minCornerLength)this.cornerCount++;this.inCorner=false}}this.keyPoints.push({x:pos.x,y:pos.y,lat:pos.lat||0,lon:pos.lon||0,heading:pos.heading||0,speed:pos.speed||0,sigma:sigma,t:Date.now(),curvature:curvature,isCorner:isCorner});this.totalDistance+=dist;this.lastPos={x:pos.x,y:pos.y,heading:pos.heading||0};if(this.trackType==='loop'){const lr=this._checkLoopClosure(pos);if(lr.detected){this.loopDetected=true;return{stored:true,loopDetected:true,stats:this.getStats()}}}return{stored:true,curvature:curvature,isCorner:isCorner,stats:this.getStats()}}
    _checkLoopClosure(pos){if(this.totalDistance<this.config.minLoopDistance)return{detected:false};const dts=Math.sqrt((pos.x-this.startPos.x)**2+(pos.y-this.startPos.y)**2);return dts<this.config.closeProximity?{detected:true}:{detected:false}}
    _wrapAngle(a){while(a>Math.PI)a-=2*Math.PI;while(a<-Math.PI)a+=2*Math.PI;return a}
    getStats(){const displayCorners=this.inCorner?this.cornerCount+1:this.cornerCount;return{recording:this.recording,trackType:this.trackType,keyPointCount:this.keyPoints.length,totalDistance:this.totalDistance,loopDetected:this.loopDetected,elapsedMs:this.recording?Date.now()-this.startTime:0,gpsQuality:this._gpsQualityRating(this.lastSigma),corners:displayCorners,inCorner:this.inCorner}}
    _gpsQualityRating(sigma){if(sigma<2.0)return'excellent';if(sigma<3.0)return'good';if(sigma<4.0)return'fair';return'poor'}
    cancel(){this.recording=false}
    finish(trackName,gpsOrigin=null){if(!this.recording)return null;this.recording=false;if(this.keyPoints.length<10)return null;const centerline=this._smoothPath(this.keyPoints,3);const startLine=this._calculateTimingLine(centerline,'start');let finishLine=null;if(this.trackType==='point_to_point')finishLine=this._calculateTimingLine(centerline,'finish');const bounds=this._calculateBounds(centerline);const totalDist=this._calculatePathLength(centerline);const quality=this._assessQuality();return{id:'track_'+Date.now()+'_'+Math.random().toString(36).substr(2,9),name:trackName,type:this.trackType,created:Date.now(),startLine:startLine,finishLine:finishLine,centerline:centerline,bounds:bounds,totalDistance:totalDist,origin:{x:this.startPos.x,y:this.startPos.y},gpsOrigin:gpsOrigin,quality:quality,corners:this.cornerCount,keyPoints:this.keyPoints.length}}
    _smoothPath(pts,ws){if(!pts||pts.length===0)return[];if(pts.length<=ws)return pts.map(p=>({...p}));const result=[];const hw=Math.floor(ws/2);for(let i=0;i<pts.length;i++){const s=Math.max(0,i-hw),e=Math.min(pts.length-1,i+hw),w=pts.slice(s,e+1);if(w.length===0){result.push({...pts[i]});continue}let sx=0,sy=0,sw=0;for(let j=0;j<w.length;j++){const df=Math.abs(j-(i-s)),pw=1/(1+df*0.5),sg=w[j].sigma||3.0,qw=1/(sg+0.5),wt=pw*qw;sx+=w[j].x*wt;sy+=w[j].y*wt;sw+=wt}const x=sw>0?sx/sw:pts[i].x,y=sw>0?sy/sw:pts[i].y;result.push({x:isNaN(x)?pts[i].x:x,y:isNaN(y)?pts[i].y:y,heading:pts[i].heading,speed:pts[i].speed,sigma:pts[i].sigma,t:pts[i].t,curvature:pts[i].curvature,isCorner:pts[i].isCorner})}return result}
    _calculateTimingLine(cl,which='start'){if(!cl||cl.length===0)return{p1:[0,12],p2:[0,-12],direction:0};const idx=which==='start'?Math.min(2,cl.length-1):Math.max(0,cl.length-3);const ns=5,s=Math.max(0,idx-ns),e=Math.min(cl.length-1,idx+ns),pts=cl.slice(s,e+1);let sx=0,sy=0,sw=0;for(const p of pts){const wt=1/(p.sigma||3.0+0.5);sx+=p.x*wt;sy+=p.y*wt;sw+=wt}const cx=sw>0?sx/sw:cl[idx].x,cy=sw>0?sy/sw:cl[idx].y;const safeCx=isNaN(cx)?0:cx,safeCy=isNaN(cy)?0:cy;let dir=0;if(pts.length>=2){const f=pts[0],l=pts[pts.length-1];dir=Math.atan2(l.y-f.y,l.x-f.x)}const w=12,perp=dir+Math.PI/2;return{p1:[safeCx+Math.cos(perp)*w,safeCy+Math.sin(perp)*w],p2:[safeCx-Math.cos(perp)*w,safeCy-Math.sin(perp)*w],direction:dir}}
    _calculateBounds(cl){if(!cl||cl.length===0)return{minX:0,minY:0,maxX:100,maxY:100};let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;for(const p of cl){minX=Math.min(minX,p.x);minY=Math.min(minY,p.y);maxX=Math.max(maxX,p.x);maxY=Math.max(maxY,p.y)}return{minX,minY,maxX,maxY}}
    _calculatePathLength(cl){if(!cl||cl.length<2)return 0;let len=0;for(let i=1;i<cl.length;i++){const dx=cl[i].x-cl[i-1].x,dy=cl[i].y-cl[i-1].y;len+=Math.sqrt(dx*dx+dy*dy)}return len}
    _assessQuality(){if(this.keyPoints.length===0)return{rating:'poor',avgUncertainty:5.0,goodSampleRatio:0,corners:0,totalPoints:0};const vp=this.keyPoints.filter(p=>p.sigma<3.0);const ts=this.keyPoints.reduce((s,p)=>s+(p.sigma||3.0),0);const as=ts/this.keyPoints.length;const displayCorners=this.inCorner?this.cornerCount+1:this.cornerCount;const gr=vp.length/this.keyPoints.length;let rating='good';if(isNaN(as)||as>3.5||gr<0.7)rating='fair';if(isNaN(as)||as>4.5||gr<0.5)rating='poor';return{rating:rating,avgUncertainty:isNaN(as)?3.0:as,goodSampleRatio:isNaN(gr)?0.5:gr,corners:displayCorners,totalPoints:this.keyPoints.length}}
}

// Track Manager IndexedDB (v3 adds lap_history for session history)
const TRACK_DB='blackbox-tracks',TRACK_DB_VER=3;
let trackDb=null,activeTrack=null,currentPos=null,suppressStartLineIndicator=false;
let p2pNeedsWarmup=false,trackRecorder=null;
let referenceLap=null,lapTracker=null,lastDeltaMs=0,deltaTrend=0;
let currentSessionId=null,currentSessionStart=null;

// GPS Reference Tracking (mirrors firmware's algorithm: average of first 5 GPS fixes)
let jsGpsRef=null,jsGpsRefSamples=[];
function updateJsGpsRef(lat,lon){if(jsGpsRef||lat===0||lon===0)return;jsGpsRefSamples.push({lat,lon});if(jsGpsRefSamples.length>=5){const avgLat=jsGpsRefSamples.reduce((s,p)=>s+p.lat,0)/5;const avgLon=jsGpsRefSamples.reduce((s,p)=>s+p.lon,0)/5;jsGpsRef={lat:avgLat,lon:avgLon}}}
function gpsToLocal(lat,lon){if(!jsGpsRef)return null;const latRad=jsGpsRef.lat*Math.PI/180;return{x:(lon-jsGpsRef.lon)*Math.cos(latRad)*111320,y:(lat-jsGpsRef.lat)*111320}}
function localToGps(x,y){if(!jsGpsRef)return null;const latRad=jsGpsRef.lat*Math.PI/180;return{lat:jsGpsRef.lat+y/111320,lon:jsGpsRef.lon+x/(111320*Math.cos(latRad))}}
function transformCoord(x,y,fromRef,toRef){const fLatRad=fromRef.lat*Math.PI/180;const lon=fromRef.lon+x/(111320*Math.cos(fLatRad));const lat=fromRef.lat+y/111320;const tLatRad=toRef.lat*Math.PI/180;return{x:(lon-toRef.lon)*Math.cos(tLatRad)*111320,y:(lat-toRef.lat)*111320}}
function transformLine(line,fromRef,toRef){const p1=transformCoord(line.p1[0],line.p1[1],fromRef,toRef);const p2=transformCoord(line.p2[0],line.p2[1],fromRef,toRef);return{p1:[p1.x,p1.y],p2:[p2.x,p2.y],direction:line.direction}}

// Track Auto-Detection
class TrackAutoDetector{
    constructor(){this.lastCheck=0;this.checkInterval=3000;this.candidateTrack=null;this.matchCount=0;this.requiredMatches=2;this.activeToast=null}
    async check(x,y,heading){
        const now=Date.now();if(now-this.lastCheck<this.checkInterval)return null;this.lastCheck=now;
        if(activeTrack){this.reset();return null}
        if(this.activeToast)return null;
        const tracks=await getAllTracks();let bestMatch=null,bestScore=0;
        for(const t of tracks){if(t.isDemo)continue;const sc=this.scoreMatch(x,y,heading,t);if(sc>bestScore&&sc>0.5){bestScore=sc;bestMatch=t}}
        if(bestMatch){if(this.candidateTrack&&this.candidateTrack.id===bestMatch.id)this.matchCount++;else{this.candidateTrack=bestMatch;this.matchCount=1}if(this.matchCount>=this.requiredMatches)return{track:bestMatch,confidence:bestScore}}else this.reset();
        return null
    }
    scoreMatch(x,y,heading,track){
        // Skip tracks from different sessions if we can't transform
        if(track.gpsOrigin&&!jsGpsRef)return 0;
        // Transform bounds check if needed
        let bx=x,by=y;
        if(track.gpsOrigin&&jsGpsRef){const t=transformCoord(x,y,jsGpsRef,track.gpsOrigin);bx=t.x;by=t.y}
        if(track.bounds){const m=100;if(bx<track.bounds.minX-m||bx>track.bounds.maxX+m||by<track.bounds.minY-m||by>track.bounds.maxY+m)return 0}
        const cl=track.centerline||[];if(cl.length===0)return 0;
        let minD=Infinity,nearP=null;for(const p of cl){const d=Math.sqrt((bx-p.x)**2+(by-p.y)**2);if(d<minD){minD=d;nearP=p}}
        if(!nearP||minD>50)return 0;
        const distScore=Math.max(0,1-minD/50);
        const hdiff=Math.abs(wrapAngle(heading-(nearP.heading||0)));
        const headScore=Math.max(0,1-hdiff/Math.PI);
        return distScore*0.6+headScore*0.4
    }
    reset(){this.candidateTrack=null;this.matchCount=0}
    setActiveToast(t){this.activeToast=t}
    clearActiveToast(){this.activeToast=null}
}
const trackAutoDetector=new TrackAutoDetector();

function showTrackDetectedToast(track){
    dismissActiveToast();
    const t=document.createElement('div');t.className='bbToast';
    t.innerHTML='<div class=\"bbToastIcon\">📍</div><div class=\"bbToastText\"><strong>Track detected</strong><span>'+escHtml(track.name)+'</span></div><div class=\"bbToastActions\"><button class=\"bbToastBtn primary\" onclick=\"activateDetectedTrack()\">Activate</button><button class=\"bbToastBtn secondary\" onclick=\"dismissActiveToast()\">Dismiss</button></div>';
    document.body.appendChild(t);trackAutoDetector.setActiveToast(t);t._detectedTrack=track;
    t._dismissTimeout=setTimeout(()=>dismissActiveToast(),10000)
}
function dismissActiveToast(){
    const t=trackAutoDetector.activeToast;if(!t)return;
    if(t._dismissTimeout)clearTimeout(t._dismissTimeout);
    t.classList.add('dismissing');setTimeout(()=>{if(t.parentNode)t.parentNode.removeChild(t)},200);
    trackAutoDetector.clearActiveToast();trackAutoDetector.reset()
}
async function activateDetectedTrack(){
    const t=trackAutoDetector.activeToast;if(!t||!t._detectedTrack)return;
    const track=t._detectedTrack;dismissActiveToast();await activateTrack(track)
}

function openTrackDB(){
    return new Promise((res,rej)=>{
        const r=indexedDB.open(TRACK_DB,TRACK_DB_VER);
        r.onerror=()=>rej(r.error);
        r.onsuccess=()=>{trackDb=r.result;res(trackDb)};
        r.onupgradeneeded=e=>{
            const d=e.target.result,oldVer=e.oldVersion;
            if(!d.objectStoreNames.contains('tracks')){
                const s=d.createObjectStore('tracks',{keyPath:'id'});
                s.createIndex('modified','modified',{unique:false});
            }
            if(oldVer<2&&!d.objectStoreNames.contains('reference_laps')){
                const rs=d.createObjectStore('reference_laps',{keyPath:'id'});
                rs.createIndex('trackId','trackId',{unique:true});
            }
            if(oldVer<3&&!d.objectStoreNames.contains('lap_history')){
                const hs=d.createObjectStore('lap_history',{keyPath:'id'});
                hs.createIndex('trackId','trackId',{unique:false});
                hs.createIndex('sessionId','sessionId',{unique:false});
                hs.createIndex('timestamp','timestamp',{unique:false});
            }
        };
    });
}

// Reference lap CRUD
async function saveRefLap(ref){if(!trackDb)await openTrackDB();return new Promise((res,rej)=>{const tx=trackDb.transaction('reference_laps','readwrite');const r=tx.objectStore('reference_laps').put(ref);r.onsuccess=()=>res(ref);r.onerror=()=>rej(r.error)})}
async function getRefLap(trackId){if(!trackDb)await openTrackDB();return new Promise((res,rej)=>{const tx=trackDb.transaction('reference_laps','readonly');const r=tx.objectStore('reference_laps').index('trackId').get(trackId);r.onsuccess=()=>res(r.result||null);r.onerror=()=>rej(r.error)})}
async function deleteRefLap(trackId){if(!trackDb)await openTrackDB();return new Promise((res,rej)=>{const tx=trackDb.transaction('reference_laps','readwrite');const r=tx.objectStore('reference_laps').index('trackId').openCursor(IDBKeyRange.only(trackId));r.onsuccess=()=>{const c=r.result;if(c){c.delete();c.continue()}else res()};r.onerror=()=>rej(r.error)})}

// Session management for lap history
function genSessionId(){return 'session_'+Date.now()+'_'+Math.random().toString(36).substr(2,9)}
function genUUID(){if(crypto.randomUUID)return crypto.randomUUID();return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g,c=>{const r=Math.random()*16|0;return(c==='x'?r:(r&0x3|0x8)).toString(16)})}
function startNewSession(){currentSessionId=genSessionId();currentSessionStart=Date.now()}
function clearSession(){currentSessionId=null;currentSessionStart=null}

// Lap history CRUD
async function recordLapHistory(trackId,lapTimeMs,lapNum){
    if(!trackDb)await openTrackDB();if(!currentSessionId)return;
    const lap={id:genUUID(),trackId,timestamp:Date.now(),lapTimeMs,lapNumber:lapNum,sessionId:currentSessionId,sessionStart:currentSessionStart};
    return new Promise((res,rej)=>{const tx=trackDb.transaction('lap_history','readwrite');const r=tx.objectStore('lap_history').put(lap);r.onsuccess=()=>res(lap);r.onerror=()=>rej(r.error)})
}
async function getLapHistory(trackId,limit=200){
    if(!trackDb)await openTrackDB();
    return new Promise((res,rej)=>{const tx=trackDb.transaction('lap_history','readonly');const r=tx.objectStore('lap_history').index('trackId').getAll(IDBKeyRange.only(trackId));
        r.onsuccess=()=>{const laps=r.result||[];laps.sort((a,b)=>b.timestamp-a.timestamp);const ltd=laps.slice(0,limit);
            const sm=new Map();for(const l of ltd){if(!sm.has(l.sessionId))sm.set(l.sessionId,{sessionId:l.sessionId,sessionStart:l.sessionStart,laps:[],bestMs:Infinity});const s=sm.get(l.sessionId);s.laps.push(l);if(l.lapTimeMs<s.bestMs)s.bestMs=l.lapTimeMs}
            const sessions=Array.from(sm.values());sessions.sort((a,b)=>b.sessionStart-a.sessionStart);for(const s of sessions)s.laps.sort((a,b)=>a.lapNumber-b.lapNumber);
            res({sessions,totalLaps:laps.length})};r.onerror=()=>rej(r.error)})
}
async function deleteLapHistory(trackId){if(!trackDb)await openTrackDB();return new Promise((res,rej)=>{const tx=trackDb.transaction('lap_history','readwrite');const r=tx.objectStore('lap_history').index('trackId').openCursor(IDBKeyRange.only(trackId));r.onsuccess=()=>{const c=r.result;if(c){c.delete();c.continue()}else res()};r.onerror=()=>rej(r.error)})}
async function clearAllLapHistory(){if(!trackDb)await openTrackDB();return new Promise((res,rej)=>{const tx=trackDb.transaction('lap_history','readwrite');const r=tx.objectStore('lap_history').clear();r.onsuccess=()=>res();r.onerror=()=>rej(r.error)})}

// LapTracker - tracks distance and samples during lap for delta calculation
class LapTracker{
    constructor(){this.reset()}
    reset(){this.dist=0;this.lastPos=null;this.samples=[];this.startTime=null;this.lastSampleT=0}
    update(x,y,lapTimeMs){
        if(this.startTime===null)this.startTime=lapTimeMs;
        if(this.lastPos){const dx=x-this.lastPos.x,dy=y-this.lastPos.y,d=Math.sqrt(dx*dx+dy*dy);if(d<50&&d>0.01)this.dist+=d}
        this.lastPos={x,y};
        if(lapTimeMs-this.lastSampleT>=100){this.samples.push({d:this.dist,t:lapTimeMs});this.lastSampleT=lapTimeMs}
        return this.dist
    }
    toBinary(){const buf=new ArrayBuffer(this.samples.length*8),v=new Float32Array(buf);for(let i=0;i<this.samples.length;i++){v[i*2]=this.samples[i].d;v[i*2+1]=this.samples[i].t}return buf}
    isValid(){return this.samples.length>=10&&this.dist>50}
}

function calcDelta(dist,time,ref){
    if(!ref||!ref.samples)return{deltaMs:0,valid:false};
    const s=new Float32Array(ref.samples),n=s.length/2;
    if(n<2||dist>s[(n-1)*2]*1.1||dist<s[0])return{deltaMs:0,valid:false};
    let lo=0,hi=n-1;while(hi-lo>1){const mid=Math.floor((lo+hi)/2);if(s[mid*2]<dist)lo=mid;else hi=mid}
    const d0=s[lo*2],t0=s[lo*2+1],d1=s[hi*2],t1=s[hi*2+1];
    if(d1===d0)return{deltaMs:time-t0,valid:true};
    const frac=(dist-d0)/(d1-d0),refT=t0+frac*(t1-t0);
    return{deltaMs:time-refT,valid:true}
}

function fmtDelta(ms){const sign=ms>0?'+':'',sec=ms/1000;return sign+(Math.abs(sec)<10?sec.toFixed(2):sec.toFixed(1))}

function updateDeltaBar(opts){const{deltaMs,trend=0,hasRef=false,isP2P=false}=opts;const bar=$('delta-bar'),fill=$('delta-fill'),txt=$('delta-text');if(!bar||!fill||!txt)return;if(!hasRef){bar.classList.add('no-ref');fill.className='bbDeltaFill';fill.style.width='0%';fill.style.left='50%';txt.textContent=isP2P?'Set best run':'Set best lap';txt.className='bbDeltaText';return}if(deltaMs===0){bar.classList.remove('no-ref');fill.className='bbDeltaFill';fill.style.width='0%';fill.style.left='50%';txt.textContent='±0.00';txt.className='bbDeltaText';return}bar.classList.remove('no-ref');const MAX=2000,abs=Math.abs(deltaMs),pct=Math.min(50,(abs/MAX)*50),ahead=deltaMs<0;if(ahead){fill.style.left=(50-pct)+'%';fill.style.width=pct+'%'}else{fill.style.left='50%';fill.style.width=pct+'%'}let fc='bbDeltaFill';if(abs>30)fc+=ahead?' ahead':' behind';if(abs>1000)fc+=ahead?' glow-ahead':' glow-behind';fill.className=fc;let ds=fmtDelta(deltaMs);if(trend<-5)ds+=' ▲';else if(trend>5)ds+=' ▼';txt.textContent=ds;txt.className='bbDeltaText'+(abs>30?(ahead?' ahead':' behind'):'')}

async function saveNewRefLap(trackId,lapTimeMs,tracker){
    try{
        const ref={id:'ref_'+Date.now()+'_'+Math.random().toString(36).substr(2,9),trackId,lapTimeMs,created:Date.now(),samples:tracker.toBinary(),sampleCount:tracker.samples.length,totalDistance:tracker.dist};
        referenceLap=ref;const sec=$('lap-section');if(sec)sec.classList.add('has-ref');console.log('Reference lap created:',fmtLapTime(lapTimeMs),'('+ref.sampleCount+' samples)');
        await deleteRefLap(trackId);await saveRefLap(ref);console.log('Reference lap saved to IndexedDB')
    }catch(e){console.error('Failed to save ref lap:',e)}
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
function wrapAngle(a){while(a>Math.PI)a-=2*Math.PI;while(a<-Math.PI)a+=2*Math.PI;return a}

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
            const lc=t.lapCount||0;
            return '<div class=\"bbTrackItem\" data-id=\"'+t.id+'\">'+
                '<div class=\"bbTrackInfo\">'+
                    '<div class=\"bbTrackName\">'+escHtml(t.name)+(isActive?' ✓':'')+'</div>'+
                    '<div class=\"bbTrackMeta\"><span>Best: '+best+'</span><span>'+lc+' '+unit+'</span></div>'+
                '</div>'+
                '<div class=\"bbTrackActions\">'+
                    '<button class=\"bbTrackBtn primary\" data-action=\"use\">Use</button>'+
                    (lc>0?'<button class=\"bbTrackBtn secondary\" data-action=\"history\" title=\"View history\">H</button>':'')+
                    '<button class=\"bbTrackBtn secondary\" data-action=\"export\" title=\"Export track data\">↓</button>'+
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
        if(confirm('Delete this track and all lap history?')){
            await deleteLapHistory(id);
            await deleteTrackFromDB(id);
            if(activeTrack&&activeTrack.id===id)await deactivateTrack();
            renderTrackList();
        }
    }else if(action==='history'){
        await showTrackHistory(id);
    }else if(action==='export'){
        await exportTrackData(id);
    }
}

async function exportTrackData(id){
    try{
        const track=await getTrack(id);
        if(!track){alert('Track not found');return}
        const history=await getLapHistory(id);
        const refLap=await getRefLap(id);
        const exportData={
            version:1,
            exportedAt:new Date().toISOString(),
            track:track,
            lapHistory:history||[],
            referenceLap:refLap||null
        };
        const json=JSON.stringify(exportData,null,2);
        const blob=new Blob([json],{type:'application/json'});
        const url=URL.createObjectURL(blob);
        const a=document.createElement('a');
        a.href=url;
        const safeName=track.name.replace(/[^a-z0-9]/gi,'_').toLowerCase();
        a.download='track_'+safeName+'_'+new Date().toISOString().slice(0,10)+'.json';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }catch(e){
        console.error('Export error:',e);
        alert('Export failed: '+e.message);
    }
}

async function exportAllTracks(){
    try{
        const tracks=await getAllTracks();
        if(tracks.length===0){alert('No tracks to export');return}
        const allData={
            version:1,
            exportedAt:new Date().toISOString(),
            deviceInfo:{gpsRef:jsGpsRef},
            tracks:[]
        };
        for(const t of tracks){
            const history=await getLapHistory(t.id);
            const refLap=await getRefLap(t.id);
            allData.tracks.push({
                track:t,
                lapHistory:history||[],
                referenceLap:refLap||null
            });
        }
        const json=JSON.stringify(allData,null,2);
        const blob=new Blob([json],{type:'application/json'});
        const url=URL.createObjectURL(blob);
        const a=document.createElement('a');
        a.href=url;
        a.download='blackbox_tracks_'+new Date().toISOString().slice(0,10)+'.json';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        alert('Exported '+tracks.length+' track(s)');
    }catch(e){
        console.error('Export all error:',e);
        alert('Export failed: '+e.message);
    }
}

// Session history display
function fmtSessionDate(ts){
    const d=new Date(ts),now=new Date(),today=new Date(now.getFullYear(),now.getMonth(),now.getDate()),yest=new Date(today.getTime()-86400000),sd=new Date(d.getFullYear(),d.getMonth(),d.getDate());
    if(sd.getTime()===today.getTime())return 'Today';if(sd.getTime()===yest.getTime())return 'Yesterday';return d.toLocaleDateString('en-US',{weekday:'short',month:'short',day:'numeric'})
}
function fmtSessionTime(ts){return new Date(ts).toLocaleTimeString('en-US',{hour:'numeric',minute:'2-digit',hour12:true})}
async function showTrackHistory(id){
    const t=await getTrack(id);if(!t)return;
    const hist=await getLapHistory(id);
    const isP2P=t.type==='point_to_point';
    const unit=isP2P?'run':'lap',units=isP2P?'runs':'laps';
    const item=document.querySelector('.bbTrackItem[data-id=\"'+id+'\"]');if(!item)return;
    const ex=item.querySelector('.bbTrackHistory');if(ex){ex.remove();return}
    const sec=document.createElement('div');sec.className='bbTrackHistory';
    if(hist.sessions.length===0){sec.innerHTML='<div class=\"bbHistoryEmpty\"><span>No '+unit+' history</span><span class=\"bbHistoryHint\">Complete some '+units+' to start tracking</span></div>'}
    else{
        let html='<div class=\"bbHistorySummary\"><span class=\"bbHistoryTotal\">'+hist.totalLaps+' '+(hist.totalLaps===1?unit:units)+' total</span><span>'+hist.sessions.length+' session'+(hist.sessions.length===1?'':'s')+'</span></div><div class=\"bbHistorySessions\">';
        const recent=hist.sessions.slice(0,5);
        for(const s of recent){
            html+='<div class=\"bbHistorySession\"><div class=\"bbHistorySessionHeader\"><span class=\"bbHistoryDate\">'+fmtSessionDate(s.sessionStart)+' '+fmtSessionTime(s.sessionStart)+'</span><span class=\"bbHistorySessionStats\">Best: '+fmtLapTime(s.bestMs)+' · '+s.laps.length+' '+(s.laps.length===1?unit:units)+'</span></div><div class=\"bbHistoryLaps\">';
            for(const l of s.laps){const isBest=l.lapTimeMs===s.bestMs;html+='<span class=\"bbHistoryLap'+(isBest?' best':'')+'\">'+fmtLapTime(l.lapTimeMs)+'</span>'}
            html+='</div></div>';
        }
        if(hist.sessions.length>5){const more=hist.sessions.length-5;html+='<div class=\"bbHistoryMore\">+'+more+' more session'+(more===1?'':'s')+'</div>'}
        html+='</div><div class=\"bbHistoryActions\"><button class=\"bbHistoryClearBtn\" onclick=\"clearTrackHistoryConfirm(\''+id+'\');\">Clear History</button></div>';
        sec.innerHTML=html;
    }
    item.appendChild(sec);
}
async function clearTrackHistoryConfirm(id){
    if(confirm('Clear all lap history for this track?')){
        await deleteLapHistory(id);
        const t=await getTrack(id);if(t){t.lapCount=0;await saveTrack(t)}
        renderTrackList();
    }
}

async function activateTrack(track){
    const isP2P=track.type==='point_to_point';
    // Warn if GPS reference not ready but track has origin (cross-session use)
    if(track.gpsOrigin&&!jsGpsRef){alert('GPS not ready. Wait for GPS lock before activating saved tracks.');return}
    // Transform coordinates if track was recorded with different GPS reference
    let line=track.startLine,finLine=track.finishLine;
    if(track.gpsOrigin&&jsGpsRef){
        const dLat=(track.gpsOrigin.lat-jsGpsRef.lat)*111320;
        const dLon=(track.gpsOrigin.lon-jsGpsRef.lon)*Math.cos(jsGpsRef.lat*Math.PI/180)*111320;
        if(Math.sqrt(dLat*dLat+dLon*dLon)>1.0){
            line=transformLine(track.startLine,track.gpsOrigin,jsGpsRef);
            if(track.finishLine)finLine=transformLine(track.finishLine,track.gpsOrigin,jsGpsRef);
        }
    }
    let url;
    if(isP2P&&finLine){
        url='/api/laptimer/configure?type=point_to_point'+
            '&p1_x='+line.p1[0].toFixed(2)+'&p1_y='+line.p1[1].toFixed(2)+
            '&p2_x='+line.p2[0].toFixed(2)+'&p2_y='+line.p2[1].toFixed(2)+
            '&dir='+line.direction.toFixed(4)+
            '&f_p1_x='+finLine.p1[0].toFixed(2)+'&f_p1_y='+finLine.p1[1].toFixed(2)+
            '&f_p2_x='+finLine.p2[0].toFixed(2)+'&f_p2_y='+finLine.p2[1].toFixed(2)+
            '&f_dir='+finLine.direction.toFixed(4);
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
        activeTrack={...track,startLine:line,finishLine:finLine};
        suppressStartLineIndicator=false;
        p2pNeedsWarmup=isP2P&&track.isNew;
        // Start new session for lap history
        startNewSession();
        // Load reference lap for delta calculation
        try{referenceLap=await getRefLap(track.id);if(referenceLap)console.log('Reference lap loaded:',fmtLapTime(referenceLap.lapTimeMs))}catch(e){referenceLap=null}
        // Initialize lap tracker
        if(!lapTracker)lapTracker=new LapTracker();lapTracker.reset();lastDeltaMs=0;deltaTrend=0;
        // Update delta bar display
        updateDeltaBar({deltaMs:null,hasRef:!!referenceLap,isP2P})
        const sec=$('lap-section');
        sec.classList.toggle('p2p',isP2P);
        sec.classList.toggle('first-run',!track.lapCount);
        sec.classList.toggle('has-ref',!!referenceLap);
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
    activeTrack=null;referenceLap=null;clearSession();
    if(lapTracker)lapTracker.reset();
    p2pNeedsWarmup=false;
    $('lap-setup-text').textContent='Tap to configure';
    $('lap-track-name').textContent='';
    updateActiveTrackDisplay();
}

// Track Recording Functions
function startTrackRecording(trackType='loop'){
    if(!currentPos||!currentPos.valid){alert('Position not available');return false}
    if(!trackRecorder)trackRecorder=new TrackRecorder();
    // Use GPS course if valid, else EKF yaw (heading used for corner detection, not loop closure)
    const heading=currentPos.validGpsCourse?currentPos.gpsCourse:currentPos.yaw;
    const pos={x:currentPos.x,y:currentPos.y,lat:currentPos.lat,lon:currentPos.lon,heading:heading,speed:currentPos.speed,valid:true,sigma:currentPos.sigma||3.0};
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
    // Use GPS course when valid, otherwise EKF yaw (heading used for corner detection, not loop closure)
    const heading=currentPos.validGpsCourse?currentPos.gpsCourse:currentPos.yaw;
    const pos={x:currentPos.x,y:currentPos.y,lat:currentPos.lat,lon:currentPos.lon,heading:heading,speed:currentPos.speed,valid:true,sigma:currentPos.sigma||3.0};
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
    const trackData=trackRecorder.finish(trackName,jsGpsRef);
    if(!trackData){alert('Failed to create track: not enough data or poor quality');return}
    $('record-overlay').classList.remove('active');
    const track={id:trackData.id,name:trackData.name,type:trackData.type,startLine:trackData.startLine,finishLine:trackData.finishLine,origin:trackData.origin,gpsOrigin:trackData.gpsOrigin,centerline:trackData.centerline,bounds:trackData.bounds,pathLength:trackData.totalDistance||0,quality:trackData.quality,bestLapMs:null,lapCount:0,corners:trackData.corners||0,keyPoints:(trackData.centerline&&trackData.centerline.length)||trackData.keyPoints||0,createdAt:trackData.created,isNew:true};
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
    // Record lap to history
    recordLapHistory(activeTrack.id,lapTimeMs,activeTrack.lapCount).catch(e=>console.error('Failed to record lap:',e));
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
    else if(!active&&lapTimerActive){sec.classList.add('inactive');sec.classList.remove('active','timing','has-ref');lapTimerActive=false}
    if(!lapTimerActive)return;
    // Update current lap time
    $('lap-time').textContent=fmtLapTime(lapTimeMs);
    const isP2P=activeTrack&&activeTrack.type==='point_to_point';
    const unit=isP2P?'Run':'Lap';
    $('lap-count').textContent=unit+' '+Math.max(1,lapCnt+(lapTimeMs>0?1:0));
    // Update state indicator
    const stateEl=$('lap-state');
    if(lapTimeMs>0){
        stateEl.textContent=isP2P?'Running':'Timing';stateEl.classList.add('timing');sec.classList.add('timing');suppressStartLineIndicator=false;
        // Track distance and calculate delta
        if(lapTracker&&currentPos&&currentPos.valid){
            const dist=lapTracker.update(currentPos.x,currentPos.y,lapTimeMs);
            const isP2P=activeTrack&&activeTrack.type==='point_to_point';
            if(referenceLap){
                const delta=calcDelta(dist,lapTimeMs,referenceLap);
                if(delta.valid){
                    deltaTrend=0.15*(delta.deltaMs-lastDeltaMs)+0.85*deltaTrend;lastDeltaMs=delta.deltaMs;
                    updateDeltaBar({deltaMs:delta.deltaMs,trend:deltaTrend,hasRef:true,isP2P});
                }
            }else{updateDeltaBar({deltaMs:null,hasRef:false,isP2P})}
        }
    }else{stateEl.textContent='Armed';stateEl.classList.remove('timing');sec.classList.remove('timing')}
    // Handle new best flag FIRST - save as reference lap before reset
    if((lapFlags&LAP_FLAG_NEW_BEST)&&!(prevLapFlags&LAP_FLAG_NEW_BEST)){
        const bestEl=$('best-lap');bestEl.classList.add('bbLapBestFlash');setTimeout(()=>bestEl.classList.remove('bbLapBestFlash'),800);
        // Save reference lap for delta (use prevLapTimeMs which is the completed lap time)
        if(activeTrack&&lapTracker&&lapTracker.isValid()&&prevLapTimeMs>0){saveNewRefLap(activeTrack.id,prevLapTimeMs,lapTracker)}
    }
    // Handle new lap flag - save completed lap time to track and reset lap tracker
    if((lapFlags&LAP_FLAG_NEW_LAP)&&!(prevLapFlags&LAP_FLAG_NEW_LAP)){
        sec.classList.add('bbLapFlash');setTimeout(()=>sec.classList.remove('bbLapFlash'),600);
        // prevLapTimeMs contains the completed lap time (before reset)
        if(prevLapTimeMs>0)updateTrackBestLap(prevLapTimeMs);
        // Reset lap tracker for next lap
        if(lapTracker){lapTracker.reset();lastDeltaMs=0;deltaTrend=0}
        updateDeltaBar({deltaMs:0,trend:0,hasRef:!!referenceLap,isP2P:activeTrack&&activeTrack.type==='point_to_point'})
        // P2P: Show "Finished!" briefly then return to "Armed"
        if(isP2P){stateEl.textContent='Finished!';stateEl.classList.add('finished');setTimeout(()=>{stateEl.textContent='Armed';stateEl.classList.remove('finished','timing');sec.classList.remove('timing')},1500)}
    }
    prevLapTimeMs=lapTimeMs;
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
    const ax=d.getFloat32(7,1),ay=d.getFloat32(11,1),wz=d.getFloat32(19,1),sp=d.getFloat32(51,1);
    const posSigma=d.getFloat32(55,1),mo=d.getUint8(59);
    const lat=d.getFloat32(60,1),lon=d.getFloat32(64,1),gpsOk=d.getUint8(68);
    // GPS course (radians) - only valid when moving, NaN when stationary
    const gpsCourse=d.getFloat32(69,1);
    // EKF state for track manager
    const ekfYaw=d.getFloat32(31,1),ekfX=d.getFloat32(35,1),ekfY=d.getFloat32(39,1);
    // Lap timer fields (82-byte protocol v4)
    const lapTimeMs=buf.byteLength>=80?d.getUint32(73,1):0;
    const lapCnt=buf.byteLength>=80?d.getUint16(77,1):0;
    const lapFlags=buf.byteLength>=80?d.getUint8(79):0;
    const latg=ay/9.81,lng=-ax/9.81,yawDeg=Math.abs(wz*57.3);
    // Track GPS reference (average of first 5 fixes, same as firmware)
    if(gpsOk===1)updateJsGpsRef(lat,lon);
    // Update position for track manager and recording
    // gpsCourse is the preferred heading source when valid (not NaN and speed > 2 km/h)
    const validGpsCourse=!isNaN(gpsCourse)&&sp>2.0;
    currentPos={x:ekfX,y:ekfY,yaw:ekfYaw,gpsCourse:gpsCourse,validGpsCourse:validGpsCourse,speed:sp,valid:gpsOk===1,sigma:posSigma,lat:lat,lon:lon};
    updateTrackRecording();
    updateStartLineIndicator();
    updateFinishLineIndicator();

    // Track auto-detection (only when no active track and not recording)
    // Use GPS course for heading when valid (more reliable than EKF yaw)
    if(!activeTrack&&!trackRecorder&&currentPos.valid){
        const detHeading=currentPos.validGpsCourse?currentPos.gpsCourse:currentPos.yaw;
        trackAutoDetector.check(currentPos.x,currentPos.y,detHeading).then(detected=>{
            if(detected)showTrackDetectedToast(detected.track)
        }).catch(()=>{})
    }

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
        chunkBuffer.push({t:now,sp,ax,ay,wz,mo,latg,lng,lat,lon,gpsOk,ekfX,ekfY,ekfYaw,gpsCourse,lon_imu:fusion.lon_imu,lon_gps:fusion.lon_gps,gps_wt:fusion.gps_wt,pitch_c:fusion.pitch_c,pitch_cf:fusion.pitch_cf,roll_c:fusion.roll_c,roll_cf:fusion.roll_cf,tilt_x:fusion.tilt_x,tilt_y:fusion.tilt_y});
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
$('btn-tracks').onclick=()=>openTrackModal();
$('track-modal-close').onclick=closeTrackModal;
$('track-modal').onclick=e=>{if(e.target===$('track-modal'))closeTrackModal()};
$('btn-clear-track').onclick=clearActiveTrack;
$('btn-export-all').onclick=exportAllTracks;
$('menu-clear').onclick=()=>{$('menu-overlay').classList.remove('open');resetState()};

// Data Modal
function openDataModal(){$('data-modal').classList.add('open');renderDataModal()}
function closeDataModal(){$('data-modal').classList.remove('open')}

async function renderDataModal(){
    const combined=await getCombinedStorageStats();
    $('data-total-size').textContent=formatBytes(combined.totalBytes);
    $('data-rec-size').textContent=formatBytes(combined.recordingsBytes);
    $('data-track-size').textContent=formatBytes(combined.tracksBytes);
    $('data-rec-count').textContent='('+combined.sessionCount+')';
    $('data-track-count').textContent='('+combined.trackCount+')';
    await renderSessionList();
    await renderTrackDataList()
}

async function renderSessionList(){
    const listEl=$('session-list'),clearBtn=$('btn-clear-all-recordings');
    try{
        const stats=await getStorageStats();
        clearBtn.disabled=stats.sessionCount===0||rec;
        if(stats.sessionCount===0){
            listEl.innerHTML='<div class=\"bbSessionEmpty\"><div class=\"icon\">📊</div>No recordings yet.<br>Start recording to capture telemetry.</div>';
            return;
        }
        listEl.innerHTML=stats.sessions.map(s=>{
            const isCur=rec&&s.sessionId===sessionId;
            const statusCls=isCur?'active':(s.status==='active'?'recovered':'complete');
            const statusLbl=isCur?'● Recording':(s.status==='active'?'⚠ Recovered':'● Complete');
            return'<div class=\"bbSessionItem'+(isCur?' active':'')+'\" data-session-id=\"'+s.sessionId+'\">'+
                '<div class=\"bbSessionInfo\">'+
                    '<div class=\"bbSessionDate\">'+formatSessionDate(s.startTime)+'</div>'+
                    '<div class=\"bbSessionMeta\"><span>'+s.chunkCount+' chunk'+(s.chunkCount!==1?'s':'')+'</span><span>'+formatBytes(s.estimatedBytes)+'</span><span>'+formatSessionDuration(s.durationMs)+'</span></div>'+
                    '<div class=\"bbSessionStatus '+statusCls+'\">'+statusLbl+'</div>'+
                '</div>'+
                '<div class=\"bbSessionActions\">'+
                    '<button class=\"bbSessionBtn\" data-action=\"export\">Export</button>'+
                    '<button class=\"bbSessionBtn danger\" data-action=\"delete\"'+(isCur?' disabled':'')+'>Delete</button>'+
                '</div>'+
            '</div>';
        }).join('');
        listEl.querySelectorAll('.bbSessionBtn').forEach(btn=>{btn.onclick=()=>handleSessionAction(btn)});
    }catch(e){console.error('Failed to load sessions:',e);listEl.innerHTML='<div class=\"bbSessionEmpty\"><div class=\"icon\">⚠️</div>Error loading recordings</div>'}
}

async function renderTrackDataList(){
    const listEl=$('track-data-list'),clearBtn=$('btn-clear-all-tracks');
    try{
        const stats=await getTrackDataStats();
        clearBtn.disabled=stats.tracks.length===0;
        if(stats.tracks.length===0){
            listEl.innerHTML='<div class=\"bbSessionEmpty\"><div class=\"icon\">🏁</div>No tracks saved.<br>Record a track to get started.</div>';
            return;
        }
        listEl.innerHTML=stats.tracks.map(t=>{
            const hasRef=t.hasReferenceLap;
            const refInfo=hasRef?'Best: '+formatLapTimeMs(t.referenceLapTime)+' ('+t.referenceLapSamples+' pts)':'No best lap';
            return'<div class=\"bbTrackDataItem\" data-track-id=\"'+t.id+'\">'+
                '<div class=\"bbTrackDataInfo\">'+
                    '<div class=\"bbTrackDataName\">'+t.name+'</div>'+
                    '<div class=\"bbTrackDataMeta\"><span>'+t.corners+' corners</span><span>'+t.points+' points</span><span>'+formatBytes(t.totalBytes)+'</span></div>'+
                    '<div class=\"bbTrackDataRef '+(hasRef?'has-ref':'no-ref')+'\">'+refInfo+'</div>'+
                '</div>'+
                '<div class=\"bbTrackDataActions\">'+
                    '<button class=\"bbSessionBtn\" data-action=\"clear-best\"'+(hasRef?'':' disabled')+'>Clear Best</button>'+
                    '<button class=\"bbSessionBtn danger\" data-action=\"delete-track\">Delete</button>'+
                '</div>'+
            '</div>';
        }).join('');
        listEl.querySelectorAll('.bbSessionBtn').forEach(btn=>{btn.onclick=()=>handleTrackDataAction(btn)});
    }catch(e){console.error('Failed to load tracks:',e);listEl.innerHTML='<div class=\"bbSessionEmpty\"><div class=\"icon\">⚠️</div>Error loading tracks</div>'}
}

async function handleSessionAction(btn){
    const action=btn.dataset.action,item=btn.closest('.bbSessionItem'),targetId=parseInt(item.dataset.sessionId,10);
    if(action==='export'){
        try{btn.textContent='Exporting...';await exportSessionById(targetId);btn.textContent='Export'}
        catch(e){alert('Export failed: '+e.message);btn.textContent='Export'}
    }else if(action==='delete'){
        const dateStr=item.querySelector('.bbSessionDate').textContent;
        if(!confirm('Delete recording from '+dateStr+'?\\n\\nThis cannot be undone.'))return;
        try{btn.textContent='Deleting...';await deleteSession(targetId);await renderDataModal()}
        catch(e){alert('Delete failed: '+e.message);btn.textContent='Delete'}
    }
}

async function handleTrackDataAction(btn){
    const action=btn.dataset.action,item=btn.closest('.bbTrackDataItem'),trackId=item.dataset.trackId,trackName=item.querySelector('.bbTrackDataName').textContent;
    if(action==='clear-best'){
        if(!confirm('Clear best lap for \"'+trackName+'\"?\\n\\nThis cannot be undone.'))return;
        try{btn.textContent='Clearing...';btn.disabled=true;await deleteRefLap(trackId);await renderDataModal()}
        catch(e){alert('Clear failed: '+e.message);btn.textContent='Clear Best';btn.disabled=false}
    }else if(action==='delete-track'){
        if(!confirm('Delete track \"'+trackName+'\" and its best lap?\\n\\nThis cannot be undone.'))return;
        try{btn.textContent='Deleting...';await deleteTrackWithRef(trackId);if(activeTrack&&activeTrack.id===trackId){activeTrack=null;updateActiveTrackDisplay()}await renderDataModal()}
        catch(e){alert('Delete failed: '+e.message);btn.textContent='Delete'}
    }
}

$('menu-data').onclick=()=>{$('menu-overlay').classList.remove('open');openDataModal()};
$('data-modal-close').onclick=closeDataModal;
$('data-modal').onclick=e=>{if(e.target===$('data-modal'))closeDataModal()};
$('btn-clear-all-recordings').onclick=async()=>{
    if(rec){alert('Cannot clear while recording. Stop recording first.');return}
    const stats=await getStorageStats();
    if(stats.sessionCount===0){alert('No recordings to clear.');return}
    if(!confirm('Delete all '+stats.sessionCount+' recording'+(stats.sessionCount!==1?'s':'')+'?\\n\\nThis will free '+formatBytes(stats.totalBytes)+' of storage.\\n\\nThis cannot be undone.'))return;
    try{$('btn-clear-all-recordings').textContent='Clearing...';$('btn-clear-all-recordings').disabled=true;await clearAllRecordings();await renderDataModal();$('btn-clear-all-recordings').textContent='Clear All Recordings'}
    catch(e){alert('Clear failed: '+e.message);$('btn-clear-all-recordings').textContent='Clear All Recordings';$('btn-clear-all-recordings').disabled=false}
};
$('btn-clear-all-tracks').onclick=async()=>{
    const stats=await getTrackDataStats();
    if(stats.tracks.length===0){alert('No tracks to clear.');return}
    if(!confirm('Delete all '+stats.tracks.length+' track'+(stats.tracks.length!==1?'s':'')+'?\\n\\nThis will free '+formatBytes(stats.totalBytes)+' of storage.\\n\\nThis cannot be undone.'))return;
    try{$('btn-clear-all-tracks').textContent='Clearing...';$('btn-clear-all-tracks').disabled=true;await clearAllTracks();activeTrack=null;updateActiveTrackDisplay();await renderDataModal();$('btn-clear-all-tracks').textContent='Clear All Tracks'}
    catch(e){alert('Clear failed: '+e.message);$('btn-clear-all-tracks').textContent='Clear All Tracks';$('btn-clear-all-tracks').disabled=false}
};

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
