use std::sync::{
    atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering},
    Arc, Mutex,
};

/// HTTP telemetry server for dashboard and settings
/// Uses HTTP polling for reliable updates without thread blocking
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::Write;
use log::info;

use crate::{diagnostics::DiagnosticsState, mode::ModeConfig, settings::DrivingMode};

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

/// Current driving mode (for API display)
#[derive(Clone, Copy, PartialEq)]
pub enum CurrentMode {
    Track,
    Canyon,
    City,
    Highway,
    Custom,
}

impl CurrentMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            CurrentMode::Track => "track",
            CurrentMode::Canyon => "canyon",
            CurrentMode::City => "city",
            CurrentMode::Highway => "highway",
            CurrentMode::Custom => "custom",
        }
    }

    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "track" => Some(CurrentMode::Track),
            "canyon" => Some(CurrentMode::Canyon),
            "city" => Some(CurrentMode::City),
            "highway" => Some(CurrentMode::Highway),
            "custom" => Some(CurrentMode::Custom),
            _ => None,
        }
    }
}

impl From<DrivingMode> for CurrentMode {
    fn from(mode: DrivingMode) -> Self {
        match mode {
            DrivingMode::Track => CurrentMode::Track,
            DrivingMode::Canyon => CurrentMode::Canyon,
            DrivingMode::City => CurrentMode::City,
            DrivingMode::Highway => CurrentMode::Highway,
            DrivingMode::Custom => CurrentMode::Custom,
        }
    }
}

impl From<CurrentMode> for DrivingMode {
    fn from(mode: CurrentMode) -> Self {
        match mode {
            CurrentMode::Track => DrivingMode::Track,
            CurrentMode::Canyon => DrivingMode::Canyon,
            CurrentMode::City => DrivingMode::City,
            CurrentMode::Highway => DrivingMode::Highway,
            CurrentMode::Custom => DrivingMode::Custom,
        }
    }
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

impl From<ModeConfig> for ModeSettings {
    fn from(cfg: ModeConfig) -> Self {
        Self {
            acc_thr: cfg.acc_thr,
            acc_exit: cfg.acc_exit,
            brake_thr: cfg.brake_thr,
            brake_exit: cfg.brake_exit,
            lat_thr: cfg.lat_thr,
            lat_exit: cfg.lat_exit,
            yaw_thr: cfg.yaw_thr,
            min_speed: cfg.min_speed,
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
    /// Current driving mode (0=track, 1=canyon, 2=city, 3=highway, 4=custom)
    current_mode: AtomicU8,
    /// Mode change requested (new mode value, 255 = no change)
    mode_change_requested: AtomicU8,
    /// Optional diagnostics state (shared with main loop)
    diagnostics_state: Option<Arc<DiagnosticsState>>,
}

impl TelemetryServerState {
    pub fn new() -> Self {
        Self {
            telemetry_data: Mutex::new(Vec::with_capacity(128)),
            packet_counter: AtomicU32::new(0),
            calibration_requested: AtomicBool::new(false),
            mode_settings: Mutex::new(ModeSettings::default()),
            settings_changed: AtomicBool::new(false),
            current_mode: AtomicU8::new(2), // Default to City
            mode_change_requested: AtomicU8::new(255), // No change
            diagnostics_state: None,
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
            current_mode: AtomicU8::new(2), // Default to City
            mode_change_requested: AtomicU8::new(255), // No change
            diagnostics_state: Some(diagnostics),
        }
    }

    /// Get current mode
    pub fn get_current_mode(&self) -> CurrentMode {
        match self.current_mode.load(Ordering::SeqCst) {
            0 => CurrentMode::Track,
            1 => CurrentMode::Canyon,
            2 => CurrentMode::City,
            3 => CurrentMode::Highway,
            4 => CurrentMode::Custom,
            _ => CurrentMode::City,
        }
    }

    /// Set current mode (called when mode is loaded from NVS)
    pub fn set_current_mode(&self, mode: CurrentMode) {
        let val = match mode {
            CurrentMode::Track => 0,
            CurrentMode::Canyon => 1,
            CurrentMode::City => 2,
            CurrentMode::Highway => 3,
            CurrentMode::Custom => 4,
        };
        self.current_mode.store(val, Ordering::SeqCst);
    }

    /// Request a mode change (called from API)
    pub fn request_mode_change(&self, mode: CurrentMode) {
        let val = match mode {
            CurrentMode::Track => 0,
            CurrentMode::Canyon => 1,
            CurrentMode::City => 2,
            CurrentMode::Highway => 3,
            CurrentMode::Custom => 4,
        };
        self.mode_change_requested.store(val, Ordering::SeqCst);
    }

    /// Take mode change request (called from main loop)
    pub fn take_mode_change(&self) -> Option<CurrentMode> {
        let val = self.mode_change_requested.swap(255, Ordering::SeqCst);
        match val {
            0 => Some(CurrentMode::Track),
            1 => Some(CurrentMode::Canyon),
            2 => Some(CurrentMode::City),
            3 => Some(CurrentMode::Highway),
            4 => Some(CurrentMode::Custom),
            _ => None,
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

/// Telemetry HTTP server
pub struct TelemetryServer {
    _server: EspHttpServer<'static>,
    state: Arc<TelemetryServerState>,
}

/// Mobile-optimized dashboard with HTTP polling, G-meter trail and max values
const DASHBOARD_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes"><title>Blackbox</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,system-ui,sans-serif;background:#0a0a0f;color:#f0f0f0;min-height:100vh;min-height:100dvh;display:flex;flex-direction:column;-webkit-user-select:none;user-select:none}
.hdr{background:#111;padding:10px 16px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #222}
.logo{font-weight:700;font-size:16px;letter-spacing:2px}
.hdr-r{display:flex;align-items:center;gap:10px}
.timer{font-size:11px;color:#444;font-variant-numeric:tabular-nums}
.st{display:flex;align-items:center;gap:6px;font-size:12px;color:#888}
.dot{width:8px;height:8px;border-radius:50%;background:#444}
.dot.on{background:#22c55e}
.dot.ws{background:#3b82f6}
.dot.rec{background:#ef4444;animation:pulse 1s infinite}
@keyframes pulse{50%{opacity:.4}}
@keyframes recPulse{0%,100%{border-color:#252530}50%{border-color:#ef4444}}
.main{flex:1;padding:12px;display:flex;flex-direction:column;gap:10px;overflow-y:auto}
.mode-box{background:linear-gradient(135deg,#12121a,#1a1a24);border-radius:14px;padding:16px;text-align:center;border:1px solid #252530;transition:border-color .3s}
.mode-box.rec{animation:recPulse 1s infinite}
.mode-lbl{font-size:10px;color:#555;text-transform:uppercase;letter-spacing:2px}
.mode-val{font-size:38px;font-weight:700;letter-spacing:2px;margin:4px 0}
.mode-icon{font-size:20px}
.m-idle{color:#555}.m-accel{color:#22c55e}.m-brake{color:#ef4444}.m-corner{color:#3b82f6}
.spd-row{display:flex;gap:10px}
.spd-box{flex:1;background:linear-gradient(135deg,#12121a,#1a1a24);border-radius:14px;padding:14px;text-align:center;border:1px solid #252530}
.spd-val{font-size:56px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1}
.spd-unit{font-size:12px;color:#555;margin-top:2px}
.max-spd{width:70px;background:#111;border-radius:14px;padding:10px 8px;text-align:center;border:1px solid #1a1a24;display:flex;flex-direction:column;justify-content:center}
.max-spd-val{font-size:24px;font-weight:700;color:#f59e0b;font-variant-numeric:tabular-nums}
.max-spd-lbl{font-size:8px;color:#555;text-transform:uppercase;letter-spacing:1px}
.gf-box{background:#111;border-radius:12px;padding:16px 12px;display:flex;flex-direction:column;align-items:center;border:1px solid #1a1a24}
.gf-container{position:relative;width:140px;height:140px}
.gf-canvas{position:absolute;top:0;left:0}
.gf-max-row{display:flex;justify-content:space-between;align-items:center;width:100%;margin-top:8px}
.gf-max{display:flex;gap:4px;font-size:11px;color:#444;font-variant-numeric:tabular-nums;font-weight:600}
.gf-max span{padding:3px 8px;background:#0a0a0f;border-radius:6px;border:1px solid #1a1a24}
.gf-max .val{color:#60a5fa;font-weight:700}
.rst-btn{background:#1a1a24;border:1px solid #252530;border-radius:4px;color:#555;font-size:8px;padding:4px 8px;cursor:pointer;text-transform:uppercase;letter-spacing:1px}
.rst-btn:active{background:#252530;color:#888}
.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.met{background:#111;border-radius:10px;padding:10px 6px;text-align:center;border:1px solid #1a1a24}
.met-val{font-size:18px;font-weight:600;font-variant-numeric:tabular-nums}
.met-lbl{font-size:9px;color:#444;text-transform:uppercase;margin-top:2px}
.ctrl{display:flex;gap:8px;padding:10px 12px;background:#111;border-top:1px solid #222}
.btn{flex:1;padding:12px;border:none;border-radius:8px;font-size:13px;font-weight:600;cursor:pointer;display:flex;align-items:center;justify-content:center;gap:6px}
.btn-rec{background:#1e3a5f;color:#60a5fa}.btn-rec.on{background:#5f1e1e;color:#f87171}
.btn-exp{background:#1a1a24;color:#666}
.btn:active{opacity:.8}
.gps-box{background:#111;border-radius:8px;padding:8px 12px;text-align:center;font-size:11px;color:#555;border:1px solid #1a1a24}
.gps-box.ok{color:#22c55e}
.cfg-section{background:#111;border-radius:10px;padding:12px;margin-top:10px;border:1px solid #1a1a24}
.cfg-title{font-size:11px;color:#555;text-transform:uppercase;letter-spacing:1px;margin-bottom:12px;text-align:center}
.preset-row{display:flex;gap:6px;margin-bottom:12px}
.preset-btn{flex:1;padding:10px 4px;border:1px solid #252530;border-radius:8px;background:#0a0a0f;color:#666;font-size:10px;font-weight:600;text-transform:uppercase;letter-spacing:0.5px;cursor:pointer;transition:all .2s}
.preset-btn:active{transform:scale(0.97)}
.preset-btn.active{background:linear-gradient(135deg,#1e3a5f,#1a2d4a);border-color:#3b82f6;color:#60a5fa;box-shadow:0 0 12px rgba(59,130,246,0.2)}
.preset-btn.custom{border-style:dashed}
.preset-btn.custom.active{border-style:solid}
.preset-summary{background:#0a0a0f;border-radius:8px;padding:12px;border:1px solid #1a1a24;display:grid;grid-template-columns:1fr 1fr;gap:8px 16px;font-size:11px}
.preset-summary.hidden{display:none}
.ps-row{display:flex;justify-content:space-between;align-items:center}
.ps-label{color:#555;text-transform:uppercase;font-size:9px;letter-spacing:0.5px}
.ps-value{color:#60a5fa;font-weight:600;font-variant-numeric:tabular-nums}
.ps-value span{color:#444;font-weight:400}
.cfg-sliders{display:none}
.cfg-sliders.show{display:block}
.cfg-row{display:flex;align-items:center;gap:8px;margin-bottom:8px}
.cfg-lbl{font-size:10px;color:#666;width:50px}
.cfg-slider{flex:1;-webkit-appearance:none;background:#1a1a24;height:6px;border-radius:3px}
.cfg-slider::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;background:#3b82f6;border-radius:50%}
.cfg-val{font-size:12px;color:#60a5fa;width:35px;text-align:right;font-variant-numeric:tabular-nums}
.cfg-unit{font-size:9px;color:#444;width:22px}
.cfg-btns{display:flex;gap:8px;margin-top:12px}
.cfg-btn{flex:1;padding:10px;border:none;border-radius:6px;font-size:11px;font-weight:600;background:#1a1a24;color:#666;cursor:pointer;transition:all .2s}
.cfg-btn:active{transform:scale(0.97)}
.cfg-btn.cfg-save{background:linear-gradient(135deg,#1e3a5f,#1a2d4a);color:#60a5fa}
</style></head>
<body>
<div class="hdr"><div class="logo">BLACKBOX <span style="font-size:9px;color:#333">v4</span></div><div class="hdr-r"><a href="/autotune" style="color:#60a5fa;text-decoration:none;font-size:10px;margin-right:12px">⚙ Tune</a><a href="/diagnostics" style="color:#333;text-decoration:none;font-size:9px;margin-right:10px;opacity:0.6">DIAG</a><span class="timer" id="timer">00:00</span><div class="st"><span class="dot" id="dot"></span><span id="stxt">--</span></div></div></div>
<div class="main">
<div class="mode-box" id="modebox"><div class="mode-lbl">Mode</div><div class="mode-val m-idle" id="mode">IDLE</div><div class="mode-icon" id="icon">●</div></div>
<div class="spd-row">
<div class="spd-box"><div class="spd-val" id="spd">0</div><div class="spd-unit">km/h</div></div>
<div class="max-spd" id="maxspdbox"><div class="max-spd-val" id="maxspd">0</div><div class="max-spd-lbl">MAX</div></div>
</div>
<div class="gf-box">
<div class="gf-container"><canvas id="gfc" class="gf-canvas" width="140" height="140"></canvas></div>
<div class="gf-max-row">
<div class="gf-max"><span>L <b class="val" id="maxL">0.0</b></span><span>B <b class="val" id="maxB">0.0</b></span><span>A <b class="val" id="maxA">0.0</b></span><span>R <b class="val" id="maxR">0.0</b></span></div>
<button class="rst-btn" id="rstg">CLR</button>
</div>
</div>
<div class="metrics">
<div class="met"><div class="met-val" id="latg">0.0</div><div class="met-lbl">Lat G</div></div>
<div class="met"><div class="met-val" id="lng">0.0</div><div class="met-lbl">Lon G</div></div>
<div class="met"><div class="met-val" id="yaw">0</div><div class="met-lbl">Yaw°/s</div></div>
<div class="met"><div class="met-val" id="hz">0</div><div class="met-lbl">Hz</div></div>
</div>
<div class="gps-box" id="gpsbox"><span id="gps">GPS: --</span></div>
<div class="cfg-section">
<div class="cfg-title">Driving Preset</div>
<div class="preset-row">
<button class="preset-btn" data-preset="track">Track</button>
<button class="preset-btn" data-preset="canyon">Canyon</button>
<button class="preset-btn active" data-preset="city">City</button>
<button class="preset-btn" data-preset="highway">Hwy</button>
<button class="preset-btn custom" data-preset="custom">Custom</button>
</div>
<div class="preset-summary" id="preset-summary">
<div class="ps-row"><span class="ps-label">Accel</span><span class="ps-value" id="ps-acc">0.10<span>/0.05g</span></span></div>
<div class="ps-row"><span class="ps-label">Brake</span><span class="ps-value" id="ps-brake">0.18<span>/0.09g</span></span></div>
<div class="ps-row"><span class="ps-label">Lateral</span><span class="ps-value" id="ps-lat">0.12<span>/0.06g</span></span></div>
<div class="ps-row"><span class="ps-label">Yaw</span><span class="ps-value" id="ps-yaw">0.05<span> r/s</span></span></div>
<div class="ps-row" style="grid-column:span 2;justify-content:center;margin-top:4px;padding-top:8px;border-top:1px solid #1a1a24"><span class="ps-label">Min Speed</span><span class="ps-value" id="ps-minspd" style="margin-left:8px">2.0<span> m/s</span></span></div>
</div>
<div class="cfg-sliders" id="cfg-sliders">
<div class="cfg-row"><span class="cfg-lbl">Accel</span><input type="range" min="0.05" max="0.80" step="0.01" class="cfg-slider" id="s-acc" value="0.10" oninput="updS('acc')"><span class="cfg-val" id="v-acc">0.10</span><span class="cfg-unit">g</span></div>
<div class="cfg-row"><span class="cfg-lbl">Acc Exit</span><input type="range" min="0.02" max="0.50" step="0.01" class="cfg-slider" id="s-accexit" value="0.05" oninput="updS('accexit')"><span class="cfg-val" id="v-accexit">0.05</span><span class="cfg-unit">g</span></div>
<div class="cfg-row"><span class="cfg-lbl">Brake</span><input type="range" min="0.10" max="1.20" step="0.01" class="cfg-slider" id="s-brake" value="0.18" oninput="updS('brake')"><span class="cfg-val" id="v-brake">0.18</span><span class="cfg-unit">g</span></div>
<div class="cfg-row"><span class="cfg-lbl">Brk Exit</span><input type="range" min="0.05" max="0.60" step="0.01" class="cfg-slider" id="s-brakeexit" value="0.09" oninput="updS('brakeexit')"><span class="cfg-val" id="v-brakeexit">0.09</span><span class="cfg-unit">g</span></div>
<div class="cfg-row"><span class="cfg-lbl">Lateral</span><input type="range" min="0.05" max="1.20" step="0.01" class="cfg-slider" id="s-lat" value="0.12" oninput="updS('lat')"><span class="cfg-val" id="v-lat">0.12</span><span class="cfg-unit">g</span></div>
<div class="cfg-row"><span class="cfg-lbl">Lat Exit</span><input type="range" min="0.02" max="0.60" step="0.01" class="cfg-slider" id="s-latexit" value="0.06" oninput="updS('latexit')"><span class="cfg-val" id="v-latexit">0.06</span><span class="cfg-unit">g</span></div>
<div class="cfg-row"><span class="cfg-lbl">Yaw</span><input type="range" min="0.02" max="0.35" step="0.005" class="cfg-slider" id="s-yaw" value="0.05" oninput="updS('yaw')"><span class="cfg-val" id="v-yaw">0.050</span><span class="cfg-unit">r/s</span></div>
<div class="cfg-row"><span class="cfg-lbl">Min Spd</span><input type="range" min="1.0" max="10.0" step="0.5" class="cfg-slider" id="s-minspd" value="2.0" oninput="updS('minspd')"><span class="cfg-val" id="v-minspd">2.0</span><span class="cfg-unit">m/s</span></div>
<div class="cfg-btns"><button class="cfg-btn" onclick="resetToPreset()">Reset</button><button class="cfg-btn cfg-save" onclick="saveCfg()">Apply</button></div>
</div>
</div>
</div>
<div class="ctrl">
<button class="btn btn-rec" id="rec">● REC</button>
<button class="btn btn-exp" id="exp">Export</button>
</div>
<script>
const M={0:'IDLE',1:'ACCEL',2:'BRAKE',4:'CORNER',5:'ACCEL+CORNER',6:'BRAKE+CORNER'},C={0:'m-idle',1:'m-accel',2:'m-brake',4:'m-corner',5:'m-accel',6:'m-brake'},I={0:'●',1:'▲',2:'▼',4:'◆',5:'⬈',6:'⬊'};
let rec=0,data=[],cnt=0,lastSeq=0;
let trail=[],maxL=0,maxR=0,maxA=0,maxB=0,maxSpd=0;
let speed_ema=0;
let sessionStart=Date.now();
let currentPreset='city';
const $=id=>document.getElementById(id);
const cv=$('gfc'),ctx=cv.getContext('2d');
const CX=70,CY=70,R=55,SCL=R/2;

// Default presets (must match settings.rs DrivingMode::default_config())
// Each mode has independent thresholds - calibrate via Autotune page
const DEFAULT_PRESETS={
track:{acc:0.30,acc_exit:0.15,brake:0.50,brake_exit:0.25,lat:0.50,lat_exit:0.25,yaw:0.15,min_speed:3.0,desc:'Racing/track days'},
canyon:{acc:0.20,acc_exit:0.10,brake:0.35,brake_exit:0.17,lat:0.30,lat_exit:0.15,yaw:0.10,min_speed:2.5,desc:'Spirited mountain roads'},
city:{acc:0.10,acc_exit:0.05,brake:0.18,brake_exit:0.09,lat:0.12,lat_exit:0.06,yaw:0.05,min_speed:2.0,desc:'Daily street driving'},
highway:{acc:0.08,acc_exit:0.04,brake:0.15,brake_exit:0.07,lat:0.10,lat_exit:0.05,yaw:0.04,min_speed:4.0,desc:'Highway cruising'}
};

// Get presets - prefer calibrated profiles from localStorage, fall back to defaults
function getPresets(){
const calib=localStorage.getItem('bb_profiles');
if(calib){
try{return JSON.parse(calib).profiles}catch(e){}
}
return DEFAULT_PRESETS;
}
const PRESETS=getPresets();

function fmtTime(ms){const s=Math.floor(ms/1000),m=Math.floor(s/60);return String(m).padStart(2,'0')+':'+String(s%60).padStart(2,'0')}

function resetGMax(){maxL=maxR=maxA=maxB=0;$('maxL').textContent=$('maxR').textContent=$('maxA').textContent=$('maxB').textContent='0.0'}
function resetAll(){
// Just clear max values and reset timer - no calibration trigger
resetGMax();maxSpd=0;$('maxspd').textContent='0';sessionStart=Date.now();$('timer').textContent='00:00';trail=[];
}

function drawG(){
ctx.clearRect(0,0,140,140);
// Outer ring with gradient
const grd=ctx.createRadialGradient(CX,CY,0,CX,CY,R);
grd.addColorStop(0,'#0a0a0f');grd.addColorStop(1,'#1a1a24');
ctx.strokeStyle=grd;ctx.lineWidth=1.5;
ctx.beginPath();ctx.arc(CX,CY,R,0,Math.PI*2);ctx.stroke();
// Inner rings with subtle gradient
ctx.strokeStyle='#252530';ctx.lineWidth=1;
ctx.beginPath();ctx.arc(CX,CY,R*0.66,0,Math.PI*2);ctx.stroke();
ctx.strokeStyle='#1a1a24';
ctx.beginPath();ctx.arc(CX,CY,R*0.33,0,Math.PI*2);ctx.stroke();
// Cross with tick marks
ctx.strokeStyle='#252530';ctx.lineWidth=1;
ctx.beginPath();ctx.moveTo(CX-R,CY);ctx.lineTo(CX+R,CY);ctx.moveTo(CX,CY-R);ctx.lineTo(CX,CY+R);ctx.stroke();
// Tick marks at 0.5g intervals
ctx.strokeStyle='#1a1a24';ctx.lineWidth=1;
for(let g of[0.25,0.5,0.75,1.0]){
const r=g*R/2;
ctx.beginPath();ctx.moveTo(CX-3,CY-r);ctx.lineTo(CX+3,CY-r);ctx.stroke();
ctx.beginPath();ctx.moveTo(CX-3,CY+r);ctx.lineTo(CX+3,CY+r);ctx.stroke();
ctx.beginPath();ctx.moveTo(CX-r,CY-3);ctx.lineTo(CX-r,CY+3);ctx.stroke();
ctx.beginPath();ctx.moveTo(CX+r,CY-3);ctx.lineTo(CX+r,CY+3);ctx.stroke();
}
// Labels - larger and bolder
ctx.fillStyle='#60a5fa';ctx.font='bold 12px system-ui';ctx.textAlign='center';
ctx.fillText('BRK',CX,11);ctx.fillText('ACC',CX,135);
ctx.textAlign='left';ctx.fillText('L',2,CY+4);
ctx.textAlign='right';ctx.fillText('R',138,CY+4);
// Trail with connecting line (the tail)
if(trail.length>1){
ctx.strokeStyle='rgba(59,130,246,0.15)';ctx.lineWidth=2;
ctx.beginPath();
ctx.moveTo(CX+trail[0].x*SCL,CY-trail[0].y*SCL);
for(let i=1;i<trail.length;i++){
ctx.lineTo(CX+trail[i].x*SCL,CY-trail[i].y*SCL);
}
ctx.stroke();
}
// Trail dots with gradient fade
for(let i=0;i<trail.length;i++){
const t=trail[i],a=(i+1)/trail.length;
const hue=200+a*20;
ctx.fillStyle=`hsla(${hue},70%,60%,${a*0.6})`;
ctx.shadowColor=`hsla(${hue},70%,60%,${a*0.3})`;ctx.shadowBlur=4;
ctx.beginPath();ctx.arc(CX+t.x*SCL,CY-t.y*SCL,2+a*2,0,Math.PI*2);ctx.fill();
}
ctx.shadowBlur=0;
// Current dot with enhanced glow
if(trail.length>0){
const c=trail[trail.length-1];
ctx.shadowColor='#3b82f6';ctx.shadowBlur=12;
ctx.fillStyle='#60a5fa';
ctx.beginPath();ctx.arc(CX+c.x*SCL,CY-c.y*SCL,7,0,Math.PI*2);ctx.fill();
ctx.shadowBlur=6;
ctx.fillStyle='#fff';
ctx.beginPath();ctx.arc(CX+c.x*SCL,CY-c.y*SCL,3,0,Math.PI*2);ctx.fill();
ctx.shadowBlur=0;
}
}

function process(buf){
const d=new DataView(buf);
const ax=d.getFloat32(7,1),ay=d.getFloat32(11,1),wz=d.getFloat32(19,1),sp=d.getFloat32(51,1),mo=d.getUint8(55);
const lat=d.getFloat32(56,1),lon=d.getFloat32(60,1),gpsOk=d.getUint8(64);
const latg=ay/9.81,lng=-ax/9.81,yaw=Math.abs(wz*57.3);
// EMA filter - alpha=0.7 for responsive yet smooth display
speed_ema=0.7*sp+(1-0.7)*speed_ema;
const dspd=speed_ema<1?0:Math.round(speed_ema);
$('mode').textContent=M[mo]||'IDLE';$('mode').className='mode-val '+(C[mo]||'m-idle');$('icon').textContent=I[mo]||'●';
$('spd').textContent=dspd;$('latg').textContent=latg.toFixed(2);$('lng').textContent=lng.toFixed(2);
$('yaw').textContent=yaw.toFixed(0);
// Max speed
if(dspd>maxSpd){maxSpd=dspd;$('maxspd').textContent=maxSpd}
if(gpsOk){$('gps').textContent=lat.toFixed(6)+', '+lon.toFixed(6);$('gpsbox').className='gps-box ok'}
else{$('gps').textContent='GPS: No Fix';$('gpsbox').className='gps-box'}
// G-meter: x=lateral(+right), y=longitudinal(+accel)
const gx=Math.max(-2,Math.min(2,latg)),gy=Math.max(-2,Math.min(2,lng));
trail.push({x:gx,y:gy});if(trail.length>30)trail.shift();
// Max G values
if(latg<0&&Math.abs(latg)>maxL){maxL=Math.abs(latg);$('maxL').textContent=maxL.toFixed(2)}
if(latg>0&&latg>maxR){maxR=latg;$('maxR').textContent=maxR.toFixed(2)}
if(lng>0&&lng>maxA){maxA=lng;$('maxA').textContent=maxA.toFixed(2)}
if(lng<0&&Math.abs(lng)>maxB){maxB=Math.abs(lng);$('maxB').textContent=maxB.toFixed(2)}
drawG();
cnt++;
if(rec)data.push({t:Date.now(),sp,ax,ay,wz,mo,latg,lng,lat,lon,gpsOk});
}

// HTTP polling - self-scheduling for maximum throughput
async function poll(){
try{
const r=await fetch('/api/telemetry');
const j=await r.json();
if(j.data&&j.seq!==lastSeq){
lastSeq=j.seq;
const b=atob(j.data),a=new Uint8Array(b.length);
for(let i=0;i<b.length;i++)a[i]=b.charCodeAt(i);
process(a.buffer);
$('dot').className='dot on';$('stxt').textContent='HTTP';
}
setTimeout(poll,33); // Poll at ~30Hz to match ESP32 telemetry rate
}catch(e){$('dot').className='dot';$('stxt').textContent='Offline';setTimeout(poll,500)} // Retry slower on error
}

// Reset button: resets G max, speed max, timer, and triggers calibration
$('rstg').onclick=resetAll;
$('gfc').ondblclick=resetGMax;

// Reset max speed on tap
$('maxspdbox').onclick=()=>{maxSpd=0;$('maxspd').textContent='0'};

// Update timer and Hz display
setInterval(()=>{$('hz').textContent=cnt;cnt=0;$('timer').textContent=fmtTime(Date.now()-sessionStart)},1000);
drawG();

$('rec').onclick=()=>{rec=!rec;if(rec){data=[];$('rec').className='btn btn-rec on';$('rec').textContent='■ STOP';$('dot').classList.add('rec');$('modebox').classList.add('rec')}
else{$('rec').className='btn btn-rec';$('rec').textContent='● REC';$('dot').classList.remove('rec');$('modebox').classList.remove('rec');
if(data.length){const s=JSON.parse(localStorage.getItem('bb')||'[]');s.unshift({id:Date.now(),n:data.length,d:data});
localStorage.setItem('bb',JSON.stringify(s.slice(0,10)));alert('Saved '+data.length+' pts')}}};

$('exp').onclick=()=>{const s=JSON.parse(localStorage.getItem('bb')||'[]');if(!s.length)return alert('No data');
let c='time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid\n';s[0].d.forEach(r=>{c+=r.t+','+r.sp+','+r.ax+','+r.ay+','+r.wz+','+r.mo+','+r.latg+','+r.lng+','+(r.lat||0)+','+(r.lon||0)+','+(r.gpsOk||0)+'\n'});
const b=new Blob([c],{type:'text/csv'}),u=URL.createObjectURL(b),a=document.createElement('a');a.href=u;a.download='blackbox.csv';a.click()};

// Settings functions
function updS(id){var s=$('s-'+id),v=$('v-'+id);if(s&&v)v.textContent=parseFloat(s.value).toFixed(id==='minspd'?1:(id==='yaw'?3:2))}

// Update preset summary display
function updateSummary(p){
$('ps-acc').innerHTML=p.acc.toFixed(2)+'<span>/'+p.acc_exit.toFixed(2)+'g</span>';
$('ps-brake').innerHTML=p.brake.toFixed(2)+'<span>/'+p.brake_exit.toFixed(2)+'g</span>';
$('ps-lat').innerHTML=p.lat.toFixed(2)+'<span>/'+p.lat_exit.toFixed(2)+'g</span>';
$('ps-yaw').innerHTML=p.yaw.toFixed(2)+'<span> r/s</span>';
$('ps-minspd').innerHTML=p.min_speed.toFixed(1)+'<span> m/s</span>';
}

// Apply preset values to sliders
function applyPresetToSliders(p){
$('s-acc').value=p.acc;$('v-acc').textContent=p.acc.toFixed(2);
$('s-accexit').value=p.acc_exit;$('v-accexit').textContent=p.acc_exit.toFixed(2);
$('s-brake').value=p.brake;$('v-brake').textContent=p.brake.toFixed(2);
$('s-brakeexit').value=p.brake_exit;$('v-brakeexit').textContent=p.brake_exit.toFixed(2);
$('s-lat').value=p.lat;$('v-lat').textContent=p.lat.toFixed(2);
$('s-latexit').value=p.lat_exit;$('v-latexit').textContent=p.lat_exit.toFixed(2);
$('s-yaw').value=p.yaw;$('v-yaw').textContent=p.yaw.toFixed(3);
$('s-minspd').value=p.min_speed;$('v-minspd').textContent=p.min_speed.toFixed(1);
}

// Get calibration data from localStorage
// Schema: { profiles: { track: {...}, city: {...}, ... }, lastMode: 'track', date: '...' }
function getCalibrationData(){
try{
const data=JSON.parse(localStorage.getItem('bb_profiles')||'null');
return data;
}catch(e){return null}
}
function isCalibrated(){
const data=getCalibrationData();
return data&&data.profiles&&Object.keys(data.profiles).length>0;
}
// Get autotune profile for current preset (used by Custom mode)
function getAutotuneProfile(){
const data=getCalibrationData();
if(data&&data.profiles){
// Return last calibrated mode's profile, or first available
const mode=data.lastMode||Object.keys(data.profiles)[0];
return data.profiles[mode]||null;
}
return null;
}

// Select preset and apply
function selectPreset(name){
currentPreset=name;
// Update button states
document.querySelectorAll('.preset-btn').forEach(b=>{
b.classList.toggle('active',b.dataset.preset===name);
});
// Show/hide sliders vs summary
const isCustom=name==='custom';
$('preset-summary').classList.toggle('hidden',isCustom);
$('cfg-sliders').classList.toggle('show',isCustom);
// If not custom, apply preset and send to ESP32 (includes mode switch)
if(!isCustom&&PRESETS[name]){
const p=PRESETS[name];
updateSummary(p);
applyPresetToSliders(p);
sendSettings(p,name);
}else if(isCustom){
// For custom, try to load from autotune profile
const at=getAutotuneProfile();
if(at){
const p={acc:at.acc,acc_exit:at.acc_exit,brake:at.brake,brake_exit:at.brake_exit,lat:at.lat,lat_exit:at.lat_exit,yaw:at.yaw,min_speed:at.min_speed};
applyPresetToSliders(p);
sendSettings(p,'custom');
}
}
}

// Reset to current preset (for custom mode)
function resetToPreset(){
if(currentPreset!=='custom'&&PRESETS[currentPreset]){
applyPresetToSliders(PRESETS[currentPreset]);
}else{
// Default to city if in custom
applyPresetToSliders(PRESETS.city);
}
}

// Send settings to ESP32 (includes mode change for correct NVS slot)
async function sendSettings(p,mode){
const brake=-Math.abs(p.brake),brake_exit=-Math.abs(p.brake_exit);
try{
// First switch to target mode if specified
if(mode){await fetch('/api/settings/set?mode='+mode);await new Promise(r=>setTimeout(r,150))}
// Then send settings
const url='/api/settings/set?acc='+p.acc+'&acc_exit='+p.acc_exit+'&brake='+brake+'&brake_exit='+brake_exit+'&lat='+p.lat+'&lat_exit='+p.lat_exit+'&yaw='+p.yaw+'&min_speed='+p.min_speed;
await fetch(url);
}catch(e){console.log('Settings send failed:',e)}
}

async function saveCfg(){
var acc=parseFloat($('s-acc').value),accexit=parseFloat($('s-accexit').value),brake=parseFloat($('s-brake').value),brakeexit=parseFloat($('s-brakeexit').value),lat=parseFloat($('s-lat').value),latexit=parseFloat($('s-latexit').value),yaw=parseFloat($('s-yaw').value),minspd=parseFloat($('s-minspd').value);
// Validation
if(accexit>=acc){alert('Accel Exit must be < Accel Entry');return}
if(brakeexit>=brake){alert('Brake Exit must be < Brake Entry');return}
if(latexit>=lat){alert('Lateral Exit must be < Lateral Entry');return}
// Send (with current preset mode for NVS storage)
const p={acc,acc_exit:accexit,brake,brake_exit:brakeexit,lat,lat_exit:latexit,yaw,min_speed:minspd};
try{
await sendSettings(p,currentPreset);
var btn=document.querySelector('.cfg-save');
btn.textContent='Applied';btn.style.background='#10b981';
setTimeout(()=>{btn.textContent='Apply';btn.style.background=''},1500);
}catch(e){alert('Save failed: '+e.message)}
}

// Initialize preset buttons
document.querySelectorAll('.preset-btn').forEach(btn=>{
btn.onclick=()=>selectPreset(btn.dataset.preset);
});

// Load current settings from ESP32 and detect matching preset
async function loadCfg(){
try{
const r=await fetch('/api/settings');
const s=await r.json();
if(s.acc!==undefined){
// Check if current settings match a preset
let matched='custom';
for(const[name,p]of Object.entries(PRESETS)){
if(Math.abs(s.acc-p.acc)<0.01&&Math.abs(s.lat-p.lat)<0.01&&Math.abs(s.min_speed-p.min_speed)<0.1){
matched=name;break;
}
}
// Update preset buttons to show calibration status
const calib=getCalibrationData();
document.querySelectorAll('.preset-btn:not(.custom)').forEach(btn=>{
const name=btn.dataset.preset;
if(calib&&calib.profiles){
// Show calibrated indicator
btn.innerHTML=name.charAt(0).toUpperCase()+name.slice(1)+' <span style="color:#22c55e;font-size:8px">●</span>';
btn.title='Calibrated '+new Date(calib.date).toLocaleDateString();
}
});
// Update custom button
const customBtn=document.querySelector('.preset-btn.custom');
if(customBtn){
customBtn.textContent='Custom';
}
currentPreset=matched;
// Update UI
document.querySelectorAll('.preset-btn').forEach(b=>{
b.classList.toggle('active',b.dataset.preset===matched);
});
const isCustom=matched==='custom';
$('preset-summary').classList.toggle('hidden',isCustom);
$('cfg-sliders').classList.toggle('show',isCustom);
// Load values
const current={acc:s.acc,acc_exit:s.acc_exit,brake:Math.abs(s.brake),brake_exit:Math.abs(s.brake_exit),lat:s.lat,lat_exit:s.lat_exit,yaw:s.yaw,min_speed:s.min_speed};
updateSummary(current);
applyPresetToSliders(current);
}
}catch(e){
// Default to city preset on error
selectPreset('city');
}
}

// Load settings, then start polling (self-scheduling for max throughput)
loadCfg().then(()=>poll());
</script></body></html>"#;

/// Diagnostics page HTML - auto-refreshes every second
const DIAGNOSTICS_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Blackbox Diagnostics</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,system-ui,monospace;background:#0a0a0f;color:#e0e0e0;padding:16px;font-size:14px}
h1{font-size:18px;margin-bottom:16px;color:#60a5fa;letter-spacing:2px}
.section{background:#111;border:1px solid #252530;border-radius:8px;padding:12px;margin-bottom:12px}
.section h2{font-size:11px;color:#555;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #1a1a24}
.row:last-child{border-bottom:none}
.label{color:#888}
.value{color:#60a5fa;font-weight:600}
.ok{color:#22c55e}
.warn{color:#f59e0b}
.err{color:#ef4444}
.back{display:inline-block;margin-bottom:16px;color:#60a5fa;text-decoration:none;font-size:12px}
.back:hover{text-decoration:underline}
.uptime{color:#555;font-size:11px;margin-top:12px;text-align:center}
</style></head>
<body>
<a href="/" class="back">&larr; Back to Dashboard</a>
<h1>BLACKBOX DIAGNOSTICS</h1>
<div class="grid">
<div class="section">
<h2>Configuration</h2>
<div class="row"><span class="label">WiFi Mode</span><span class="value" id="wifi-mode">--</span></div>
<div class="row"><span class="label">SSID</span><span class="value" id="wifi-ssid">--</span></div>
<div class="row"><span class="label">Telemetry Rate</span><span class="value" id="telem-hz">--</span></div>
<div class="row"><span class="label">GPS Model</span><span class="value" id="gps-model">--</span></div>
<div class="row"><span class="label">Warmup Fixes</span><span class="value" id="warmup-fixes">--</span></div>
</div>
<div class="section">
<h2>Sensors</h2>
<div class="row"><span class="label">IMU Rate</span><span class="value" id="imu-rate">--</span></div>
<div class="row"><span class="label">GPS Rate</span><span class="value" id="gps-rate">--</span></div>
<div class="row"><span class="label">Loop Rate</span><span class="value" id="loop-rate">--</span></div>
<div class="row"><span class="label">ZUPT Rate</span><span class="value" id="zupt-rate">--</span></div>
<div class="row"><span class="label">EKF/GPS</span><span class="value" id="ekf-per-gps">--</span></div>
<div class="row"><span class="label">GPS Fix</span><span class="value" id="gps-fix">--</span></div>
<div class="row"><span class="label">Satellites</span><span class="value" id="gps-sats">--</span></div>
<div class="row"><span class="label">HDOP</span><span class="value" id="gps-hdop">--</span></div>
<div class="row"><span class="label">Warmup</span><span class="value" id="gps-warmup">--</span></div>
</div>
</div>
<div class="grid">
<div class="section">
<h2>EKF Health</h2>
<div class="row"><span class="label">Position sigma</span><span class="value" id="pos-sigma">--</span></div>
<div class="row"><span class="label">Velocity sigma</span><span class="value" id="vel-sigma">--</span></div>
<div class="row"><span class="label">Yaw sigma</span><span class="value" id="yaw-sigma">--</span></div>
<div class="row"><span class="label">Bias X</span><span class="value" id="bias-x">--</span></div>
<div class="row"><span class="label">Bias Y</span><span class="value" id="bias-y">--</span></div>
</div>
<div class="section">
<h2>System</h2>
<div class="row"><span class="label">Heap Free</span><span class="value" id="heap">--</span></div>
<div class="row"><span class="label">TX Success</span><span class="value" id="tx-ok">--</span></div>
<div class="row"><span class="label">TX Failed</span><span class="value" id="tx-fail">--</span></div>
</div>
</div>
<div class="uptime" id="uptime">Uptime: --</div>
<script>
const $=id=>document.getElementById(id);
function rateClass(actual,expected){
  const pct=actual/expected;
  if(pct>=0.9)return'ok';
  if(pct>=0.5)return'warn';
  return'err';
}
async function update(){
  try{
    const r=await fetch('/api/diagnostics');
    const d=await r.json();
    if(d.error)return;
    $('wifi-mode').textContent=d.wifi.mode;
    $('wifi-ssid').textContent=d.wifi.ssid;
    $('telem-hz').textContent=d.config.telemetry_hz+' Hz';
    $('gps-model').textContent=d.config.gps_model;
    $('warmup-fixes').textContent=d.config.warmup_fixes;
    const imuEl=$('imu-rate');
    imuEl.textContent=d.sensor_rates.imu_hz.toFixed(0)+' / '+d.sensor_rates.imu_expected.toFixed(0)+' Hz';
    imuEl.className='value '+rateClass(d.sensor_rates.imu_hz,d.sensor_rates.imu_expected);
    const gpsEl=$('gps-rate');
    gpsEl.textContent=d.sensor_rates.gps_hz.toFixed(1)+' / '+d.sensor_rates.gps_expected.toFixed(0)+' Hz';
    gpsEl.className='value '+rateClass(d.sensor_rates.gps_hz,d.sensor_rates.gps_expected);
    $('loop-rate').textContent=d.sensor_rates.loop_hz.toFixed(0)+' Hz';
    $('zupt-rate').textContent=d.sensor_rates.zupt_per_min.toFixed(1)+'/min';
    $('ekf-per-gps').textContent=d.sensor_rates.ekf_per_gps.toFixed(1);
    $('gps-fix').textContent=d.gps.fix?'Valid':'No Fix';
    $('gps-fix').className='value '+(d.gps.fix?'ok':'warn');
    $('gps-sats').textContent=d.gps.satellites;
    $('gps-sats').className='value '+(d.gps.satellites>=4?'ok':(d.gps.satellites>=1?'warn':'err'));
    $('gps-hdop').textContent=d.gps.hdop.toFixed(1);
    $('gps-hdop').className='value '+(d.gps.hdop<2?'ok':(d.gps.hdop<5?'warn':'err'));
    $('gps-warmup').textContent=d.gps.warmup?'Complete':'Warming up...';
    $('gps-warmup').className='value '+(d.gps.warmup?'ok':'warn');
    $('pos-sigma').textContent=d.ekf.pos_sigma.toFixed(2)+' m';
    $('vel-sigma').textContent=d.ekf.vel_sigma.toFixed(2)+' m/s';
    $('yaw-sigma').textContent=d.ekf.yaw_sigma_deg.toFixed(1)+'deg';
    $('bias-x').textContent=d.ekf.bias_x.toFixed(4)+' m/s2';
    $('bias-y').textContent=d.ekf.bias_y.toFixed(4)+' m/s2';
    $('heap').textContent=(d.system.heap_free/1024).toFixed(0)+' KB';
    $('tx-ok').textContent=d.system.tx_ok.toLocaleString();
    const txf=$('tx-fail');
    txf.textContent=d.system.tx_fail;
    txf.className='value '+(d.system.tx_fail>0?'warn':'ok');
    const s=d.system.uptime_s;
    const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
    $('uptime').textContent='Uptime: '+h+'h '+m+'m '+sec+'s';
  }catch(e){console.log('Diag fetch error:',e)}
}
setInterval(update,1000);
update();
</script>
</body></html>"#;

/// Auto-tune page for calibration assistance
/// Uses EVENT DETECTION with full physics validation including:
/// - GPS heading vs gyro yaw rate
/// - Centripetal acceleration check (a_lat = v × ω)
/// - Impulse validation (ΔV = G × t)
/// - Event duration tracking
/// - Report export and localStorage save
const AUTOTUNE_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes"><title>Blackbox AutoTune</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,system-ui,sans-serif;background:#0a0a0f;color:#f0f0f0;min-height:100vh;display:flex;flex-direction:column}
.hdr{background:#111;padding:12px 16px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid #222}
.logo{font-weight:700;font-size:14px;letter-spacing:2px}
.back{color:#60a5fa;text-decoration:none;font-size:12px}
.main{flex:1;padding:12px;display:flex;flex-direction:column;gap:10px;overflow-y:auto}
.card{background:#111;border-radius:10px;padding:14px;border:1px solid #1a1a24}
.card-title{font-size:11px;color:#555;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;display:flex;justify-content:space-between;align-items:center}
.instructions{background:#0f1a2e;border:1px solid #1e3a5f;border-radius:8px;padding:12px;margin-top:10px}
.instructions-title{font-size:10px;color:#60a5fa;font-weight:600;margin-bottom:8px}
.instructions-text{font-size:11px;color:#888;line-height:1.5}
.instructions-text li{margin-bottom:4px}
.status-bar{display:flex;justify-content:space-between;align-items:center;font-size:12px;margin-top:10px}
.status{display:flex;align-items:center;gap:6px}
.dot{width:8px;height:8px;border-radius:50%;background:#444}
.dot.on{background:#22c55e}
.dot.rec{background:#ef4444;animation:pulse 1s infinite}
@keyframes pulse{50%{opacity:.4}}
.events{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}
.event{background:#0a0a0f;border-radius:6px;padding:10px;text-align:center;border:1px solid #1a1a24}
.event-count{font-size:24px;font-weight:700;color:#60a5fa;font-variant-numeric:tabular-nums}
.event-peak{font-size:10px;color:#22c55e;margin-top:2px}
.event-lbl{font-size:9px;color:#555;text-transform:uppercase;margin-top:4px}
.event.needs-more .event-count{color:#f59e0b}
.event.complete .event-count{color:#22c55e}
.live-bar{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin-top:10px}
.live{background:#0a0a0f;border-radius:6px;padding:8px;text-align:center;border:1px solid #1a1a24}
.live-val{font-size:14px;font-weight:600;font-variant-numeric:tabular-nums}
.live-lbl{font-size:8px;color:#555;text-transform:uppercase}
.live.active-accel{border-color:#22c55e}.live.active-accel .live-val{color:#22c55e}
.live.active-brake{border-color:#ef4444}.live.active-brake .live-val{color:#ef4444}
.live.active-turn{border-color:#3b82f6}.live.active-turn .live-val{color:#3b82f6}
.progress-bar{width:100%;height:6px;background:#1a1a24;border-radius:3px;margin-top:10px;overflow:hidden}
.progress-fill{height:100%;background:linear-gradient(90deg,#3b82f6,#22c55e);border-radius:3px;transition:width 0.3s}
.progress-text{font-size:10px;color:#666;margin-top:6px;text-align:center}
.progress-text.excellent{color:#22c55e}
.progress-text.good{color:#60a5fa}
.confidence-badge{display:inline-block;font-size:9px;padding:2px 8px;border-radius:10px;margin-left:6px}
.confidence-badge.low{background:#2a1a1a;color:#f87171}
.confidence-badge.med{background:#2a2a1a;color:#fbbf24}
.confidence-badge.high{background:#1a2a1a;color:#22c55e}
.results{margin-top:10px}
.result-row{display:flex;justify-content:space-between;align-items:center;padding:10px;background:#0a0a0f;border-radius:6px;margin-bottom:6px;border:1px solid #1a1a24}
.result-name{font-size:11px;color:#888}
.result-vals{display:flex;align-items:center;gap:6px;font-size:12px}
.result-current{color:#555}
.result-arrow{color:#444}
.result-new{color:#22c55e;font-weight:600}
.result-method{font-size:8px;color:#3b82f6;background:#0f1a2e;padding:2px 6px;border-radius:3px}
.btns{display:flex;gap:8px;margin-top:12px}
.btn{flex:1;padding:14px;border:none;border-radius:8px;font-size:13px;font-weight:600;cursor:pointer}
.btn-start{background:#1e3a5f;color:#60a5fa}
.btn-start.recording{background:#5f1e1e;color:#f87171}
.btn-apply{background:#1a1a24;color:#555}
.btn-apply.ready{background:linear-gradient(135deg,#1e3a5f,#1a2d4a);color:#60a5fa}
.btn-sm{font-size:10px;padding:6px 10px;border-radius:4px;background:#1a1a24;color:#60a5fa;border:none;cursor:pointer}
.btn:active,.btn-sm:active{opacity:.8}
.btn:disabled{opacity:.5;cursor:not-allowed}
.report{margin-top:10px;padding:10px;background:#0a0a0f;border-radius:6px;font-size:9px;color:#888;max-height:200px;overflow-y:auto;white-space:pre-wrap;font-family:monospace;line-height:1.4}
.baseline-row{display:flex;justify-content:space-between;background:#0a0a0f;border-radius:6px;padding:8px 12px;margin-top:8px;font-size:11px}
.baseline-row span:nth-child(odd){color:#666}
.baseline-row span:nth-child(even){color:#22c55e;font-weight:600}
.profile-table{width:100%;border-collapse:collapse;margin-top:8px;font-size:10px}
.profile-table th{background:#1a1a24;color:#888;padding:6px 4px;text-align:center;font-weight:500}
.profile-table td{padding:6px 4px;text-align:center;border-bottom:1px solid #1a1a24}
.profile-table tr:hover{background:#0f0f14}
.mode-select{display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
.mode-btn{padding:10px 14px;border:1px solid #333;border-radius:8px;background:#1a1a2e;color:#888;font-size:12px;cursor:pointer;transition:all 0.2s}
.mode-btn:hover{border-color:#60a5fa;color:#60a5fa}
.mode-btn.selected{border-color:#22c55e;background:#0a1a0a;color:#22c55e;font-weight:600}
</style></head>
<body>
<div class="hdr"><a href="/" class="back">← Dashboard</a><div class="logo">AUTOTUNE</div></div>
<div class="main">

<div class="card">
<div class="card-title">Select Mode to Calibrate</div>
<div class="mode-select">
<button class="mode-btn selected" data-mode="city">🏙️ City</button>
<button class="mode-btn" data-mode="canyon">🏔️ Canyon</button>
<button class="mode-btn" data-mode="track">🏁 Track</button>
<button class="mode-btn" data-mode="highway">🛣️ Highway</button>
</div>
<div id="mode-desc" style="font-size:11px;color:#666;text-align:center;margin-top:8px">Daily street driving - calibrate during normal commute</div>
<div id="current-settings" style="display:none;font-size:10px;color:#888;text-align:center;margin-top:6px;padding:6px;background:#1a1a24;border-radius:4px"></div>
<div class="instructions">
<div class="instructions-title">📋 How It Works</div>
<div class="instructions-text">
<ol>
<li>Select the mode you want to calibrate above</li>
<li>Press <b>Start Calibration</b> below</li>
<li>Drive in that style for 10-20 minutes</li>
<li>The system detects your accel/brake/turn events</li>
<li>Press <b>Apply</b> to save thresholds for this mode</li>
</ol>
<div style="margin-top:8px;padding:8px;background:#0a1a0a;border-radius:6px;font-size:10px;color:#22c55e">
<b>Tip:</b> Calibrate each mode separately by driving in that style. City = commute, Track = aggressive, Highway = gentle cruising.
</div>
</div>
</div>
</div>

<div class="card">
<div class="card-title">Event Detection<span id="time" style="font-size:12px;color:#60a5fa">0:00</span></div>
<div class="status-bar">
<div class="status"><span class="dot" id="dot"></span><span id="stxt">Ready</span></div>
<span id="gps-status" style="font-size:10px;color:#555">GPS: --</span>
</div>
<div class="events">
<div class="event" id="ev-accel"><div class="event-count" id="ev-accel-n">0</div><div class="event-peak" id="ev-accel-peak">peak --</div><div class="event-lbl">Accels</div></div>
<div class="event" id="ev-brake"><div class="event-count" id="ev-brake-n">0</div><div class="event-peak" id="ev-brake-peak">peak --</div><div class="event-lbl">Brakes</div></div>
<div class="event" id="ev-turn"><div class="event-count" id="ev-turn-n">0</div><div class="event-peak" id="ev-turn-peak">peak --</div><div class="event-lbl">Turns</div></div>
</div>
<div class="live-bar">
<div class="live" id="live-lon"><div class="live-val" id="lv-lon">0.00</div><div class="live-lbl">Lon G</div></div>
<div class="live" id="live-lat"><div class="live-val" id="lv-lat">0.00</div><div class="live-lbl">Lat G</div></div>
<div class="live" id="live-cent"><div class="live-val" id="lv-cent">0.00</div><div class="live-lbl">v×ω G</div></div>
<div class="live" id="live-spd"><div class="live-val" id="lv-spd">0</div><div class="live-lbl">km/h</div></div>
</div>
<div class="progress-bar"><div class="progress-fill" id="progress" style="width:0%"></div></div>
<div class="progress-text" id="progress-text">Drive to collect events</div>
</div>

<div class="card" id="results-card" style="display:none">
<div class="card-title">Calibrated Thresholds <span id="result-mode" style="color:#22c55e">City</span></div>
<div class="baseline-row"><span>Accel</span><span id="base-acc">--</span><span>Brake</span><span id="base-brake">--</span><span>Lateral</span><span id="base-lat">--</span><span>Yaw</span><span id="base-yaw">--</span></div>
<div style="font-size:10px;color:#666;text-align:center;margin-top:8px">Thresholds learned from your driving. Entry values shown (exit = 50%).</div>
</div>

<div class="btns">
<button class="btn btn-start" id="btn-start">Start Calibration</button>
<button class="btn btn-apply" id="btn-apply" disabled>Apply Settings</button>
</div>

<div class="card" id="report-card" style="display:none">
<div class="card-title">Analysis Report<button class="btn-sm" id="btn-export">Export</button></div>
<div class="report" id="report"></div>
</div>
</div>

<script>
const M={0:'IDLE',1:'ACCEL',2:'BRAKE',4:'CORNER',5:'A+C',6:'B+C'};
const $=id=>document.getElementById(id);
const G=9.80665;
const DEG=180/Math.PI; // rad to deg conversion

// Noise floor and event parameters
const NOISE_FLOOR=0.08; // 0.08g - reject road bumps, only detect intentional inputs
const EVENT_MIN_DURATION=200; // ms - must sustain for 200ms
const EVENT_COOLDOWN=400; // ms - between events
const MIN_SPEED_KMH=7.2; // Must match mode.rs min_speed (2.0 m/s = 7.2 km/h)
// EMA alpha adjusted for 25Hz polling to match 200Hz mode.rs dynamics
// At 200Hz with α=0.35, reaches 97% in 40ms. At 25Hz, need α=0.85 for similar response.
const EMA_ALPHA=0.85;

// Mode selection
let selectedMode='city';
const MODE_DESC={
    city:'Daily street driving - calibrate during normal commute',
    canyon:'Spirited mountain roads - calibrate on twisty roads',
    track:'Racing/track days - calibrate during aggressive driving',
    highway:'Highway cruising - calibrate during gentle lane changes'
};
const MODE_LABELS={city:'🏙️ City',canyon:'🏔️ Canyon',track:'🏁 Track',highway:'🛣️ Highway'};

// State
let recording=false;
let startTime=0;
let lastSeq=0;
let hasGpsLock=false;

// EMA for display
let emaLonG=0,emaLatG=0;

// EMA state matching mode.rs (for accurate threshold simulation)
let emaLonSim=0,emaLatSim=0,emaYawSim=0;

// Event detection state
let accelEv={on:false,t0:0,peak:0,spd0:0};
let brakeEv={on:false,t0:0,peak:0,spd0:0};
let turnEv={on:false,t0:0,peakLat:0,peakYaw:0};
let lastEvEnd=0;

// Collected events
let accelEvents=[];  // {peak, dur, dv}
let brakeEvents=[];  // {peak, dur, dv}
let turnEvents=[];   // {peakLat, peakYaw, dur}
let calibratedProfile=null;

// Session persistence - survives page navigation within same browser tab
const SESSION_KEY='bb_autotune_session';
function saveSession(){
    if(!recording&&accelEvents.length===0)return; // Don't save empty sessions
    const session={
        recording,startTime,selectedMode,
        accelEvents,brakeEvents,turnEvents,
        calibratedProfile,
        savedAt:Date.now()
    };
    sessionStorage.setItem(SESSION_KEY,JSON.stringify(session));
}
function clearSession(){
    sessionStorage.removeItem(SESSION_KEY);
}
function restoreSession(){
    try{
        const data=sessionStorage.getItem(SESSION_KEY);
        if(!data)return false;
        const s=JSON.parse(data);
        // Expire sessions older than 1 hour
        if(Date.now()-s.savedAt>3600000){clearSession();return false}
        // Restore state
        recording=s.recording;
        startTime=s.startTime;
        selectedMode=s.selectedMode;
        accelEvents=s.accelEvents||[];
        brakeEvents=s.brakeEvents||[];
        turnEvents=s.turnEvents||[];
        calibratedProfile=s.calibratedProfile;
        // Update UI
        document.querySelectorAll('.mode-btn').forEach(b=>{
            b.classList.toggle('selected',b.dataset.mode===selectedMode);
        });
        $('mode-desc').textContent=MODE_DESC[selectedMode];
        updateEvents();
        if(recording){
            $('btn-start').textContent='Stop Calibration';
            $('btn-start').className='btn btn-start recording';
        }
        if(calibratedProfile){
            $('results-card').style.display='block';
            $('btn-apply').disabled=false;
            $('btn-apply').className='btn btn-apply ready';
        }
        return true;
    }catch(e){return false}
}

function parsePacket(buf){
    const d=new DataView(buf);
    return{
        ts:d.getUint32(3,true),
        ax:d.getFloat32(7,true),  // Raw IMU accel (m/s²)
        ay:d.getFloat32(11,true),
        vx:d.getFloat32(43,true),
        vy:d.getFloat32(47,true),
        speed:d.getFloat32(51,true),
        wz:d.getFloat32(19,true),
        yaw:d.getFloat32(31,true), // EKF yaw for body→vehicle transform
        mode:d.getUint8(55),
        lat:d.getFloat32(56,true),
        lon:d.getFloat32(60,true),
        gpsOk:d.getUint8(64)
    };
}

// IMU-based acceleration matching mode.rs (earth_to_car transformation)
function earthToCar(ax,ay,yaw){
    const c=Math.cos(yaw),s=Math.sin(yaw);
    const lonG=(ax*c+ay*s)/G;  // Forward in vehicle frame
    const latG=(-ax*s+ay*c)/G; // Left in vehicle frame
    return{lonG,latG};
}

function process(p){
    const accImu=earthToCar(p.ax,p.ay,p.yaw);
    const now=Date.now();
    const spdMs=p.speed/3.6;

    // Update EMA simulation matching mode.rs
    emaLonSim=(1-EMA_ALPHA)*emaLonSim+EMA_ALPHA*accImu.lonG;
    emaLatSim=(1-EMA_ALPHA)*emaLatSim+EMA_ALPHA*accImu.latG;
    emaYawSim=(1-EMA_ALPHA)*emaYawSim+EMA_ALPHA*p.wz;

    // Centripetal: v × ω (for display only)
    const centG=(spdMs*Math.abs(p.wz))/G;

    // Raw IMU in g's for stationary display
    const rawLonG=p.ax/G, rawLatG=p.ay/G;
    const isMoving=spdMs>0.5;
    const displayLonG=isMoving?accImu.lonG:rawLonG;
    const displayLatG=isMoving?accImu.latG:rawLatG;

    // EMA for display
    emaLonG=0.2*displayLonG+0.8*emaLonG;
    emaLatG=0.2*displayLatG+0.8*emaLatG;

    // Track GPS lock status
    hasGpsLock=p.gpsOk===1;

    // Live display
    $('lv-lon').textContent=emaLonG.toFixed(2);
    $('lv-lat').textContent=emaLatG.toFixed(2);
    $('lv-cent').textContent=centG.toFixed(2);
    $('lv-spd').textContent=Math.round(p.speed);
    $('gps-status').textContent=p.gpsOk?'GPS: '+p.lat.toFixed(4)+','+p.lon.toFixed(4):(isMoving?'GPS: No Fix':'IMU RAW');
    $('gps-status').style.color=p.gpsOk?'#22c55e':isMoving?'#555':'#f59e0b';

    // Highlight active - use EMA-simulated values (what mode.rs sees)
    $('live-lon').className='live'+(emaLonSim>NOISE_FLOOR?' active-accel':emaLonSim<-NOISE_FLOOR?' active-brake':'');
    $('live-lat').className='live'+(Math.abs(emaLatSim)>NOISE_FLOOR?' active-turn':'');
    $('live-cent').className='live'+(centG>NOISE_FLOOR?' active-turn':'');

    if(!recording)return;

    // === EVENT DETECTION ===
    // Detect accel/brake/turn events using EMA-filtered values matching mode.rs

    // ACCELERATION
    if(emaLonSim>NOISE_FLOOR&&p.speed>MIN_SPEED_KMH){
        if(!accelEv.on&&now-lastEvEnd>EVENT_COOLDOWN){
            accelEv={on:true,t0:now,peak:emaLonSim,spd0:p.speed};
        }else if(accelEv.on){
            accelEv.peak=Math.max(accelEv.peak,emaLonSim);
        }
    }else if(accelEv.on){
        const dur=now-accelEv.t0;
        if(dur>EVENT_MIN_DURATION){
            accelEvents.push({peak:accelEv.peak,dur,dv:p.speed-accelEv.spd0});
            updateEvents();
        }
        accelEv.on=false;lastEvEnd=now;
    }

    // BRAKING
    if(emaLonSim<-NOISE_FLOOR&&p.speed>MIN_SPEED_KMH){
        if(!brakeEv.on&&now-lastEvEnd>EVENT_COOLDOWN){
            brakeEv={on:true,t0:now,peak:Math.abs(emaLonSim),spd0:p.speed};
        }else if(brakeEv.on){
            brakeEv.peak=Math.max(brakeEv.peak,Math.abs(emaLonSim));
        }
    }else if(brakeEv.on){
        const dur=now-brakeEv.t0;
        if(dur>EVENT_MIN_DURATION){
            brakeEvents.push({peak:brakeEv.peak,dur,dv:p.speed-brakeEv.spd0});
            updateEvents();
        }
        brakeEv.on=false;lastEvEnd=now;
    }

    // TURN - use EMA values with sign consistency check (matching mode.rs)
    const latEmaAbs=Math.abs(emaLatSim),yawEmaAbs=Math.abs(emaYawSim);
    const signConsistent=(emaLatSim*emaYawSim)>0;
    if(latEmaAbs>NOISE_FLOOR&&yawEmaAbs>0.02&&p.speed>MIN_SPEED_KMH&&signConsistent){
        if(!turnEv.on&&now-lastEvEnd>EVENT_COOLDOWN){
            turnEv={on:true,t0:now,peakLat:latEmaAbs,peakYaw:yawEmaAbs};
        }else if(turnEv.on){
            turnEv.peakLat=Math.max(turnEv.peakLat,latEmaAbs);
            turnEv.peakYaw=Math.max(turnEv.peakYaw,yawEmaAbs);
        }
    }else if(turnEv.on){
        const dur=now-turnEv.t0;
        if(dur>EVENT_MIN_DURATION){
            turnEvents.push({peakLat:turnEv.peakLat,peakYaw:turnEv.peakYaw,dur});
            updateEvents();
        }
        turnEv.on=false;lastEvEnd=now;
    }

    // Progress & Confidence
    // Base: 5 accel, 5 brake, 10 turns = 100%
    // Excellent: 15+ accel, 15+ brake, 25+ turns = high confidence
    const rawPct=((accelEvents.length/5+brakeEvents.length/5+turnEvents.length/10)/3)*100;
    const displayPct=Math.min(100,rawPct);
    $('progress').style.width=displayPct+'%';

    // Confidence: based on minimum event count (weakest link)
    const minEvents=Math.min(accelEvents.length,brakeEvents.length,turnEvents.length/2);
    const ptxt=$('progress-text');
    if(minEvents<3){
        ptxt.textContent='Drive to collect events';
        ptxt.className='progress-text';
    }else if(minEvents<8){
        ptxt.textContent='Good - keep driving for better accuracy';
        ptxt.className='progress-text good';
    }else if(minEvents<15){
        ptxt.textContent='Great data! More driving still helps';
        ptxt.className='progress-text good';
    }else{
        ptxt.textContent='Excellent! High confidence thresholds';
        ptxt.className='progress-text excellent';
    }

    if(accelEvents.length>=3&&brakeEvents.length>=3&&turnEvents.length>=5)computeSuggestions();
}

function updateEvents(){
    $('ev-accel-n').textContent=accelEvents.length;
    $('ev-brake-n').textContent=brakeEvents.length;
    $('ev-turn-n').textContent=turnEvents.length;
    if(accelEvents.length)$('ev-accel-peak').textContent='peak '+Math.max(...accelEvents.map(e=>e.peak)).toFixed(2)+'g';
    if(brakeEvents.length)$('ev-brake-peak').textContent='peak '+Math.max(...brakeEvents.map(e=>e.peak)).toFixed(2)+'g';
    if(turnEvents.length)$('ev-turn-peak').textContent='peak '+Math.max(...turnEvents.map(e=>e.peakLat)).toFixed(2)+'g';
    $('ev-accel').className='event'+(accelEvents.length>=5?' complete':accelEvents.length>=2?' needs-more':'');
    $('ev-brake').className='event'+(brakeEvents.length>=5?' complete':brakeEvents.length>=2?' needs-more':'');
    $('ev-turn').className='event'+(turnEvents.length>=10?' complete':turnEvents.length>=4?' needs-more':'');
    saveSession(); // Persist state for page navigation
}

function percentile(arr,p){
    if(!arr.length)return 0;
    const s=[...arr].sort((a,b)=>a-b);
    return s[Math.min(Math.floor(s.length*p/100),s.length-1)];
}

function median(arr){return percentile(arr,50)}

// IQR outlier rejection - removes values outside 1.5×IQR from Q1/Q3
function filterOutliers(arr){
    if(arr.length<4)return arr; // Need enough data for IQR
    const q1=percentile(arr,25),q3=percentile(arr,75);
    const iqr=q3-q1;
    if(iqr<0.01)return arr; // All values similar, keep all
    const lo=q1-1.5*iqr,hi=q3+1.5*iqr;
    return arr.filter(v=>v>=lo&&v<=hi);
}

function confBadge(n,min,good,high){
    if(n>=high)return'<span class="confidence-badge high">n='+n+'</span>';
    if(n>=good)return'<span class="confidence-badge med">n='+n+'</span>';
    if(n>=min)return'<span class="confidence-badge low">n='+n+'</span>';
    return'';
}

function computeSuggestions(){
    $('results-card').style.display='block';
    $('result-mode').textContent=MODE_LABELS[selectedMode];

    // Compute thresholds from collected events
    // Use MEDIAN (P50) × 0.7 for entry threshold
    calibratedProfile={acc:0.10,acc_exit:0.05,brake:0.18,brake_exit:0.09,lat:0.12,lat_exit:0.06,yaw:0.05,min_speed:2.0};

    if(accelEvents.length>=3){
        const peaks=filterOutliers(accelEvents.map(e=>e.peak));
        const p50=median(peaks);
        calibratedProfile.acc=Math.max(0.05,p50*0.7);
        calibratedProfile.acc_exit=Math.max(0.02,calibratedProfile.acc*0.5);
    }

    if(brakeEvents.length>=3){
        const peaks=filterOutliers(brakeEvents.map(e=>e.peak));
        const p50=median(peaks);
        calibratedProfile.brake=Math.max(0.08,p50*0.7);
        calibratedProfile.brake_exit=Math.max(0.04,calibratedProfile.brake*0.5);
    }

    if(turnEvents.length>=5){
        const latPeaks=filterOutliers(turnEvents.map(e=>e.peakLat));
        const latP50=median(latPeaks);
        calibratedProfile.lat=Math.max(0.05,latP50*0.7);
        calibratedProfile.lat_exit=Math.max(0.02,calibratedProfile.lat*0.5);

        const yawPeaks=filterOutliers(turnEvents.map(e=>e.peakYaw));
        const yawP50=median(yawPeaks);
        calibratedProfile.yaw=Math.max(0.02,yawP50*0.7);
    }

    // Display calibrated values (use innerHTML for badge HTML)
    $('base-acc').innerHTML=calibratedProfile.acc.toFixed(2)+'g'+confBadge(accelEvents.length,3,8,15);
    $('base-brake').innerHTML=calibratedProfile.brake.toFixed(2)+'g'+confBadge(brakeEvents.length,3,8,15);
    $('base-lat').innerHTML=calibratedProfile.lat.toFixed(2)+'g'+confBadge(turnEvents.length,5,12,20);
    $('base-yaw').innerHTML=(calibratedProfile.yaw*DEG).toFixed(1)+'°/s';

    // Enable apply when we have enough data
    if(accelEvents.length>=3&&brakeEvents.length>=3&&turnEvents.length>=5){
        $('btn-apply').disabled=false;
        $('btn-apply').className='btn btn-apply ready';
    }
}

function generateReport(){
    const L=[];
    const dur=startTime?Math.floor((Date.now()-startTime)/1000):0;
    L.push('BLACKBOX AUTOTUNE CALIBRATION REPORT');
    L.push('====================================');
    L.push('Date: '+new Date().toISOString());
    L.push('Duration: '+Math.floor(dur/60)+'m '+dur%60+'s');
    L.push('');

    // Confidence assessment
    const minEv=Math.min(accelEvents.length,brakeEvents.length,turnEvents.length/2);
    const conf=minEv>=15?'HIGH':minEv>=8?'MEDIUM':'LOW';
    L.push('CALIBRATION QUALITY: '+conf);
    L.push('  Accel events: '+accelEvents.length+(accelEvents.length>=15?' ✓':accelEvents.length>=8?' ○':' ⚠'));
    L.push('  Brake events: '+brakeEvents.length+(brakeEvents.length>=15?' ✓':brakeEvents.length>=8?' ○':' ⚠'));
    L.push('  Turn events:  '+turnEvents.length+(turnEvents.length>=20?' ✓':turnEvents.length>=12?' ○':' ⚠'));
    L.push('');

    // Event statistics
    if(accelEvents.length){
        const p=accelEvents.map(e=>e.peak);
        L.push('ACCELERATION EVENTS:');
        L.push('  G-force: min='+Math.min(...p).toFixed(3)+' med='+median(p).toFixed(3)+' max='+Math.max(...p).toFixed(3));
        L.push('');
    }
    if(brakeEvents.length){
        const p=brakeEvents.map(e=>e.peak);
        L.push('BRAKING EVENTS:');
        L.push('  G-force: min='+Math.min(...p).toFixed(3)+' med='+median(p).toFixed(3)+' max='+Math.max(...p).toFixed(3));
        L.push('');
    }
    if(turnEvents.length){
        const lat=turnEvents.map(e=>e.peakLat);
        const yaw=turnEvents.map(e=>e.peakYaw*DEG);
        L.push('TURN EVENTS:');
        L.push('  Lateral G: min='+Math.min(...lat).toFixed(3)+' med='+median(lat).toFixed(3)+' max='+Math.max(...lat).toFixed(3));
        L.push('  Yaw(°/s): min='+Math.min(...yaw).toFixed(1)+' med='+median(yaw).toFixed(1)+' max='+Math.max(...yaw).toFixed(1));
        L.push('');
    }

    // Calibrated thresholds for selected mode
    if(calibratedProfile){
        L.push('CALIBRATED THRESHOLDS ('+MODE_LABELS[selectedMode]+'):');
        L.push('  Accel: '+calibratedProfile.acc.toFixed(3)+'g (exit: '+calibratedProfile.acc_exit.toFixed(3)+'g)');
        L.push('  Brake: '+calibratedProfile.brake.toFixed(3)+'g (exit: '+calibratedProfile.brake_exit.toFixed(3)+'g)');
        L.push('  Lateral: '+calibratedProfile.lat.toFixed(3)+'g (exit: '+calibratedProfile.lat_exit.toFixed(3)+'g)');
        L.push('  Yaw: '+(calibratedProfile.yaw*DEG).toFixed(1)+'°/s ('+calibratedProfile.yaw.toFixed(4)+' rad/s)');
        L.push('  Min Speed: '+calibratedProfile.min_speed.toFixed(1)+' m/s');
    }

    return L.join('\\n');
}

async function poll(){
    try{
        const r=await fetch('/api/telemetry');
        const j=await r.json();
        if(j.data&&j.seq!==lastSeq){
            lastSeq=j.seq;
            const b=atob(j.data),a=new Uint8Array(b.length);
            for(let i=0;i<b.length;i++)a[i]=b.charCodeAt(i);
            process(parsePacket(a.buffer));
            $('dot').className='dot'+(recording?' rec':' on');
            $('stxt').textContent=recording?'Recording':'Connected';
        }
        setTimeout(poll,40);
    }catch(e){$('dot').className='dot';$('stxt').textContent='Offline';setTimeout(poll,500)}
}

async function applySettings(){
    if(!calibratedProfile){
        alert('Collect more data first');
        return;
    }

    const p=calibratedProfile;
    try{
        // First switch ESP32 to the target mode
        await fetch('/api/settings/set?mode='+selectedMode);
        // Wait for mode change to propagate to main loop
        await new Promise(r=>setTimeout(r,150));
        // Now send settings (will be saved to the correct mode's NVS slot)
        await fetch('/api/settings/set?acc='+p.acc+'&acc_exit='+p.acc_exit+'&brake='+(0-p.brake)+'&brake_exit='+(0-p.brake_exit)+'&lat='+p.lat+'&lat_exit='+p.lat_exit+'&yaw='+p.yaw+'&min_speed='+p.min_speed);

        // Save to localStorage for this mode
        // Schema: { profiles: { track: {...}, city: {...}, ... }, lastMode: 'track', date: '...' }
        let stored=JSON.parse(localStorage.getItem('bb_profiles')||'{}');
        if(!stored.profiles)stored.profiles={};
        stored.profiles[selectedMode]={...p,calibrated:new Date().toISOString()};
        stored.lastMode=selectedMode;
        stored.date=new Date().toISOString();
        localStorage.setItem('bb_profiles',JSON.stringify(stored));

        clearSession(); // Clear calibration session after successful save

        const rep=generateReport();
        $('report').textContent=rep;
        $('report-card').style.display='block';

        if(confirm(MODE_LABELS[selectedMode]+' profile saved!\\n\\nThresholds applied to device and saved to NVS.\\nProfile saved to browser for future sessions.\\n\\nGo to Dashboard?')){
            window.location.href='/';
        }
    }catch(e){alert('Failed: '+e)}
}

function exportReport(){
    const minEvents=Math.min(accelEvents.length,brakeEvents.length,turnEvents.length/2);
    const exportData={
        version:2,
        exportDate:new Date().toISOString(),
        mode:selectedMode,
        duration:startTime?Math.floor((Date.now()-startTime)/1000):0,
        quality:{
            confidence:minEvents>=15?'HIGH':minEvents>=8?'MEDIUM':'LOW',
            eventCounts:{accel:accelEvents.length,brake:brakeEvents.length,turn:turnEvents.length}
        },
        calibrated:calibratedProfile,
        events:{
            accel:accelEvents.map(e=>({peak:+e.peak.toFixed(4),dur:e.dur,dv:+e.dv.toFixed(2)})),
            brake:brakeEvents.map(e=>({peak:+e.peak.toFixed(4),dur:e.dur,dv:+e.dv.toFixed(2)})),
            turn:turnEvents.map(e=>({peakLat:+e.peakLat.toFixed(4),peakYaw:+e.peakYaw.toFixed(4),dur:e.dur}))
        },
        textReport:generateReport()
    };

    const blob=new Blob([JSON.stringify(exportData,null,2)],{type:'application/json'});
    const url=URL.createObjectURL(blob);
    const a=document.createElement('a');
    a.href=url;a.download='blackbox-'+selectedMode+'-calibration-'+Date.now()+'.json';
    a.click();
    setTimeout(()=>URL.revokeObjectURL(url),100);
}

function toggleRecording(){
    const btn=$('btn-start');

    // Check for GPS lock before starting
    if(!recording&&!hasGpsLock){
        alert('GPS lock required!\\n\\nWait for GPS status to show coordinates (green) before starting calibration.\\n\\nThis ensures accurate speed and position data.');
        return;
    }

    recording=!recording;
    if(recording){
        accelEvents=[];brakeEvents=[];turnEvents=[];
        accelEv.on=false;brakeEv.on=false;turnEv.on=false;
        lastEvEnd=0;
        emaLonSim=0;emaLatSim=0;emaYawSim=0;
        startTime=Date.now();
        calibratedProfile=null; // Clear previous calibration

        updateEvents();
        $('results-card').style.display='none';
        $('report-card').style.display='none';
        $('btn-apply').disabled=true;
        $('btn-apply').className='btn btn-apply';
        $('progress').style.width='0%';

        btn.textContent='Stop Calibration';
        btn.className='btn btn-start recording';
        saveSession(); // Persist recording state
    }else{
        btn.textContent='Start Calibration';
        btn.className='btn btn-start';
        computeSuggestions();
        $('report').textContent=generateReport();
        $('report-card').style.display='block';
        saveSession(); // Persist completed calibration
    }
}

setInterval(()=>{
    if(recording){
        const s=Math.floor((Date.now()-startTime)/1000);
        $('time').textContent=Math.floor(s/60)+':'+String(s%60).padStart(2,'0');
    }
},1000);

$('btn-start').onclick=toggleRecording;
$('btn-apply').onclick=applySettings;
$('btn-export').onclick=exportReport;

// Mode selection handlers
document.querySelectorAll('.mode-btn').forEach(btn=>{
    btn.onclick=function(){
        if(recording){alert('Stop calibration first');return}
        document.querySelectorAll('.mode-btn').forEach(b=>b.classList.remove('selected'));
        this.classList.add('selected');
        selectedMode=this.dataset.mode;
        $('mode-desc').textContent=MODE_DESC[selectedMode];
        // Reset results when mode changes
        $('results-card').style.display='none';
        $('btn-apply').disabled=true;
        $('btn-apply').className='btn btn-apply';
        calibratedProfile=null;
        saveSession(); // Persist mode change
    };
});

// Load current NVS settings for comparison
async function loadCurrentSettings(){
    try{
        const r=await fetch('/api/settings');
        const s=await r.json();
        if(s.acc!==undefined){
            const el=$('current-settings');
            if(el){
                el.innerHTML='<strong>Current NVS:</strong> acc:'+s.acc.toFixed(2)+'g brake:'+Math.abs(s.brake).toFixed(2)+'g lat:'+s.lat.toFixed(2)+'g yaw:'+s.yaw.toFixed(2)+' min_spd:'+s.min_speed.toFixed(1)+'m/s';
                el.style.display='block';
            }
        }
    }catch(e){}
}

// Initialize: restore session if exists, load current settings
restoreSession();
loadCurrentSettings();
poll();
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
            max_uri_handlers: 10, /* Dashboard, autotune, telemetry, status, calibrate,
                                   * settings GET, settings SET, diagnostics page, diagnostics
                                   * API */
            max_open_sockets: 8, // HTTP only - no long-lived WebSocket connections
            stack_size: 10240,
            ..Default::default()
        };

        let mut server = EspHttpServer::new(&server_config)?;
        let state = Arc::new(match diagnostics {
            Some(diag) => TelemetryServerState::with_diagnostics(diag),
            None => TelemetryServerState::new(),
        });

        // Serve the dashboard HTML
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

        // Serve the auto-tune calibration page
        server.fn_handler(
            "/autotune",
            esp_idf_svc::http::Method::Get,
            |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let html_bytes = AUTOTUNE_HTML.as_bytes();
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

        // WebSocket removed - using HTTP polling for reliability and to eliminate
        // thread blocking

        // HTTP polling fallback endpoint
        let state_poll = state.clone();
        server.fn_handler(
            "/api/telemetry",
            esp_idf_svc::http::Method::Get,
            move |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let json = if let Some(data) = state_poll.get_telemetry_base64() {
                    format!(
                        r#"{{"seq":{},"data":"{}"}}"#,
                        state_poll.packet_count(),
                        data
                    )
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
            let mode = state_get.get_current_mode();
            let json = format!(
                r#"{{"mode":"{}","acc":{:.2},"acc_exit":{:.2},"brake":{:.2},"brake_exit":{:.2},"lat":{:.2},"lat_exit":{:.2},"yaw":{:.3},"min_speed":{:.1}}}"#,
                mode.as_str(), s.acc_thr, s.acc_exit, s.brake_thr, s.brake_exit, s.lat_thr, s.lat_exit, s.yaw_thr, s.min_speed
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
            let mut mode_change: Option<CurrentMode> = None;

            // Parse query parameters
            if let Some(query) = uri.split('?').nth(1) {
                for param in query.split('&') {
                    let mut parts = param.split('=');
                    if let (Some(key), Some(val)) = (parts.next(), parts.next()) {
                        match key {
                            "mode" => {
                                if let Some(m) = CurrentMode::from_str(val) {
                                    mode_change = Some(m);
                                }
                            }
                            _ => {
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
                }
            }

            // Handle mode change (main loop will load settings for new mode from NVS)
            if let Some(new_mode) = mode_change {
                state_set.request_mode_change(new_mode);
                info!("Mode change requested: {}", new_mode.as_str());
            } else {
                // Regular settings update for current mode
                state_set.set_settings(settings);
                info!("Settings updated: acc={:.2}, brake={:.2}, lat={:.2}, yaw={:.3}",
                    settings.acc_thr, settings.brake_thr, settings.lat_thr, settings.yaw_thr);
            }

            let current_mode = state_set.get_current_mode();
            let json = format!(
                r#"{{"status":"ok","mode":"{}","acc":{:.2},"acc_exit":{:.2},"brake":{:.2},"brake_exit":{:.2},"lat":{:.2},"lat_exit":{:.2},"yaw":{:.3},"min_speed":{:.1}}}"#,
                current_mode.as_str(), settings.acc_thr, settings.acc_exit, settings.brake_thr, settings.brake_exit, settings.lat_thr, settings.lat_exit, settings.yaw_thr, settings.min_speed
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
                            r#""config":{{"telemetry_hz":{},"gps_model":"{}","warmup_fixes":{}}}}}"#
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
                        d.config.telemetry_rate_hz, d.config.gps_model, d.config.gps_warmup_fixes
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

        // Diagnostics HTML page
        server.fn_handler(
            "/diagnostics",
            esp_idf_svc::http::Method::Get,
            |req| -> Result<(), esp_idf_svc::io::EspIOError> {
                let html_bytes = DIAGNOSTICS_HTML.as_bytes();
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Validates localStorage schema consistency between dashboard and autotune
    /// pages.
    ///
    /// Both pages MUST use the same schema for bb_profiles:
    /// ```json
    /// {
    ///   "profiles": { "track": {...}, "city": {...}, ... },
    ///   "lastMode": "track",
    ///   "date": "2025-01-11T..."
    /// }
    /// ```
    ///
    /// This test catches the bug where autotune saved flat profiles but
    /// dashboard expected a `.profiles` wrapper, breaking the main page.
    #[test]
    fn test_localstorage_schema_consistency() {
        // Dashboard page must read from .profiles property
        assert!(
            DASHBOARD_HTML.contains(".profiles"),
            "Dashboard must access .profiles property from localStorage"
        );
        assert!(
            DASHBOARD_HTML.contains("getCalibrationData()"),
            "Dashboard must use getCalibrationData() helper"
        );
        assert!(
            DASHBOARD_HTML.contains("getAutotuneProfile()"),
            "Dashboard must define getAutotuneProfile() function"
        );

        // Autotune page must save with .profiles wrapper
        assert!(
            AUTOTUNE_HTML.contains("stored.profiles[selectedMode]"),
            "Autotune must save to stored.profiles[mode], not stored[mode]"
        );
        assert!(
            AUTOTUNE_HTML.contains("if(!stored.profiles)stored.profiles={}"),
            "Autotune must initialize .profiles object if missing"
        );

        // Both pages must use the same localStorage key
        let dashboard_key_count = DASHBOARD_HTML.matches("bb_profiles").count();
        let autotune_key_count = AUTOTUNE_HTML.matches("bb_profiles").count();
        assert!(
            dashboard_key_count > 0,
            "Dashboard must use 'bb_profiles' localStorage key"
        );
        assert!(
            autotune_key_count > 0,
            "Autotune must use 'bb_profiles' localStorage key"
        );

        // Schema documentation must be present in both pages
        assert!(
            DASHBOARD_HTML.contains("Schema:"),
            "Dashboard must document localStorage schema in comments"
        );
        assert!(
            AUTOTUNE_HTML.contains("Schema:"),
            "Autotune must document localStorage schema in comments"
        );
    }

    /// Validates that required JavaScript functions are defined in dashboard
    #[test]
    fn test_required_js_functions_defined() {
        // Dashboard must define these functions
        assert!(
            DASHBOARD_HTML.contains("function getCalibrationData()"),
            "Dashboard must define getCalibrationData()"
        );
        assert!(
            DASHBOARD_HTML.contains("function getAutotuneProfile()"),
            "Dashboard must define getAutotuneProfile()"
        );
        assert!(
            DASHBOARD_HTML.contains("function isCalibrated()"),
            "Dashboard must define isCalibrated()"
        );
        assert!(
            DASHBOARD_HTML.contains("function getPresets()"),
            "Dashboard must define getPresets()"
        );
    }
}
