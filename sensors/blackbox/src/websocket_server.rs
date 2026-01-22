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

/// Apple-native iOS dashboard with light/dark mode and kebab menu
const DASHBOARD_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="default"><title>Blackbox</title>
<style>
:root{
    --bg:#f2f2f7;
    --surface:#ffffff;
    --text:#1c1c1e;
    --text-secondary:#48484a;
    --text-tertiary:#8e8e93;
    --divider:rgba(0,0,0,.08);
    --ok:#34c759;
    --amber:#ff9500;
    --red:#ff3b30;
    --mode-idle:#8e8e93;
    --mode-accel:#1c1c1e;
    --mode-brake:#ff3b30;
}
.dark{
    --bg:#000000;
    --surface:#1c1c1e;
    --text:#ffffff;
    --text-secondary:#aeaeb2;
    --text-tertiary:#636366;
    --divider:rgba(255,255,255,.08);
    --mode-idle:#636366;
    --mode-accel:#ffffff;
    --mode-brake:#ff3b30;
}
*{margin:0;padding:0;box-sizing:border-box}
body{
    font-family:-apple-system,BlinkMacSystemFont,'SF Pro Text','SF Pro Display',system-ui,sans-serif;
    background:var(--bg);
    color:var(--text);
    height:100vh;
    height:100dvh;
    display:flex;
    flex-direction:column;
    -webkit-user-select:none;
    user-select:none;
    overflow:hidden;
    transition:background 0.3s,color 0.3s;
}
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
.bbHigh{color:var(--ok);font-weight:600}
.bbDiag{color:var(--text-tertiary);text-decoration:none;font-size:10px;opacity:.5}
.bbKebab{background:none;border:0;padding:4px;font-size:24px;line-height:1;opacity:.6;cursor:pointer;color:inherit}
.bbApp{flex:1;display:flex;flex-direction:column;padding:0 12px 8px;min-height:0}
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
</style></head>
<body>

<header class="bbTopbar">
    <button class="bbBrandToggle" id="brandToggle" aria-label="Toggle theme">
        <span class="bbBrand">Blackbox</span>
        <span class="bbModeLabel" id="modeLabel">Bright</span>
        <span class="bbModeIcon" aria-hidden="true">◐</span>
    </button>
    <div class="bbTopRight">
        <div class="bbStatusCluster">
            <a href="/diagnostics" class="bbDiag">DIAG</a>
            <span class="bbStatusItem"><span class="bbDot" id="gpsDot"></span><span id="gpsHz">GPS --</span></span>
            <span class="bbStatusItem"><span class="bbDot" id="liveDot"></span><span id="stxt">--</span></span>
        </div>
        <button class="bbKebab" id="menu-btn" aria-label="Menu">⋮</button>
    </div>
</header>

<main class="bbApp">
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

<div class="bbMenuOverlay" id="menu-overlay">
    <div class="bbMenuPanel">
        <button class="bbMenuItem" id="menu-rec">Start Recording</button>
        <button class="bbMenuItem" id="menu-export">Export CSV</button>
        <button class="bbMenuItem destructive" id="menu-clear">Clear Session</button>
    </div>
</div>

<script>
const $=id=>document.getElementById(id);
const MODES={0:'IDLE',1:'ACCEL',2:'BRAKE',4:'CORNER',5:'ACCEL',6:'BRAKE'};
const MODE_COLORS={0:'var(--mode-idle)',1:'var(--mode-accel)',2:'var(--mode-brake)',4:'var(--mode-idle)',5:'var(--mode-accel)',6:'var(--mode-brake)'};

let isDark=window.matchMedia('(prefers-color-scheme:dark)').matches;
function applyTheme(){document.body.classList.toggle('dark',isDark);$('modeLabel').textContent=isDark?'Dark':'Bright'}
applyTheme();
$('brandToggle').onclick=()=>{isDark=!isDark;applyTheme()};

function getModeColorHex(mo){
    const h={0:isDark?'#636366':'#8e8e93',1:isDark?'#ffffff':'#1c1c1e',2:'#ff3b30',4:isDark?'#636366':'#8e8e93',5:isDark?'#ffffff':'#1c1c1e',6:'#ff3b30'};
    return h[mo]||h[0];
}
let currentModeColor='#8e8e93';

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
let rec=0,data=[],cnt=0,lastSeq=0;
let maxL=0,maxR=0,maxA=0,maxB=0,peak=0;
let speed_ema=0,sessionStart=Date.now();
let emaGx=0,emaGy=0,lastT=0;
const EMA_TAU=0.10;
let lastGpsState=0;
let fusion={lon_imu:0,lon_gps:0,gps_wt:0,gps_rate:0,pitch_c:0,pitch_cf:0,roll_c:0,roll_cf:0,tilt_x:0,tilt_y:0};

function fmtGSigned(v){const sign=v>=0?'+':'−';return{sign,num:Math.abs(v).toFixed(2)}}
function fmtTime(ms){const s=Math.floor(ms/1000),m=Math.floor(s/60);return m+':'+String(s%60).padStart(2,'0')}

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
    emaGx=emaGy=0;sessionStart=Date.now();
}

let peakHighlightTimeout=null;

function process(buf){
    const d=new DataView(buf);
    const ax=d.getFloat32(7,1),ay=d.getFloat32(11,1),wz=d.getFloat32(19,1),sp=d.getFloat32(51,1),mo=d.getUint8(55);
    const lat=d.getFloat32(56,1),lon=d.getFloat32(60,1),gpsOk=d.getUint8(64);
    const latg=ay/9.81,lng=-ax/9.81,yawDeg=Math.abs(wz*57.3);

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
    drawG();
    cnt++;
    if(rec)data.push({t:now,sp,ax,ay,wz,mo,latg,lng,lat,lon,gpsOk,...fusion});
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

$('menu-btn').onclick=()=>$('menu-overlay').classList.add('open');
$('menu-overlay').onclick=e=>{if(e.target===$('menu-overlay'))$('menu-overlay').classList.remove('open')};

$('menu-rec').onclick=()=>{
    rec=!rec;$('menu-overlay').classList.remove('open');
    if(rec){data=[];sessionStart=Date.now();$('menu-rec').textContent='Stop Recording'}
    else{$('menu-rec').textContent='Start Recording';
    if(data.length){const s=JSON.parse(localStorage.getItem('bb')||'[]');s.unshift({id:Date.now(),n:data.length,d:data});localStorage.setItem('bb',JSON.stringify(s.slice(0,10)));alert('Saved '+data.length+' pts')}}
};

$('menu-export').onclick=()=>{
    $('menu-overlay').classList.remove('open');
    const s=JSON.parse(localStorage.getItem('bb')||'[]');if(!s.length)return alert('No data');
    let c='time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid,lon_imu,lon_gps,gps_weight,pitch_corr,pitch_conf,roll_corr,roll_conf,tilt_x,tilt_y\n';
    s[0].d.forEach(r=>{c+=r.t+','+r.sp+','+r.ax+','+r.ay+','+r.wz+','+r.mo+','+r.latg+','+r.lng+','+(r.lat||0)+','+(r.lon||0)+','+(r.gpsOk||0)+','+(r.lon_imu||0).toFixed(4)+','+(r.lon_gps||0).toFixed(4)+','+(r.gps_wt||0).toFixed(2)+','+(r.pitch_c||0).toFixed(2)+','+(r.pitch_cf||0).toFixed(1)+','+(r.roll_c||0).toFixed(2)+','+(r.roll_cf||0).toFixed(1)+','+(r.tilt_x||0).toFixed(4)+','+(r.tilt_y||0).toFixed(4)+'\n'});
    const b=new Blob([c],{type:'text/csv'}),u=URL.createObjectURL(b),a=document.createElement('a');a.href=u;a.download='blackbox.csv';a.click();
};

$('menu-clear').onclick=()=>{$('menu-overlay').classList.remove('open');resetState()};

setInterval(()=>{
    $('hz').textContent=cnt;cnt=0;
    $('session-time').textContent=fmtTime(Date.now()-sessionStart);
    if(lastGpsState){$('gpsDot').classList.add('on');$('gpsHz').textContent='GPS '+Math.round(fusion.gps_rate)}
    else{$('gpsDot').classList.remove('on');$('gpsHz').textContent='GPS --'}
    $('gps-acc').textContent=lastGpsState?'2':'--';
},1000);

resize();setTimeout(()=>{resize();drawG()},50);
poll();
</script></body></html>"#;

/// Diagnostics page HTML - Apple-native iOS design matching dashboard
const DIAGNOSTICS_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="default"><title>Blackbox Diagnostics</title>
<style>
:root{
    --bg:#f2f2f7;
    --surface:#ffffff;
    --text:#1c1c1e;
    --text-secondary:#48484a;
    --text-tertiary:#8e8e93;
    --divider:rgba(0,0,0,.08);
    --ok:#34c759;
    --warn:#ff9500;
    --err:#ff3b30;
}
.dark{
    --bg:#000000;
    --surface:#1c1c1e;
    --text:#ffffff;
    --text-secondary:#aeaeb2;
    --text-tertiary:#636366;
    --divider:rgba(255,255,255,.08);
}
*{margin:0;padding:0;box-sizing:border-box}
body{
    font-family:-apple-system,BlinkMacSystemFont,'SF Pro Text','SF Pro Display',system-ui,sans-serif;
    background:var(--bg);
    color:var(--text);
    padding:0 12px 20px;
    font-size:13px;
    min-height:100vh;
    -webkit-user-select:none;
    user-select:none;
    transition:background 0.3s,color 0.3s;
}
.bbNum{font-variant-numeric:tabular-nums;font-feature-settings:"tnum" 1,"lnum" 1}
.bbTopbar{display:flex;justify-content:space-between;align-items:center;padding:14px 4px;flex-shrink:0}
.bbBrandToggle{background:none;border:0;padding:0;display:flex;align-items:baseline;gap:8px;cursor:pointer;color:inherit;font-family:inherit}
.bbBrand{font-size:11px;font-weight:600;opacity:.55;letter-spacing:.03em}
.bbModeLabel{font-size:11px;font-weight:400;opacity:.40;letter-spacing:.03em}
.bbModeIcon{font-size:10px;opacity:.40}
.bbBack{color:var(--text-tertiary);text-decoration:none;font-size:12px;font-weight:500;letter-spacing:.03em;padding:8px 14px;background:var(--surface);border-radius:16px;display:flex;align-items:center;gap:4px}
.bbBack:active{opacity:.7}
.bbCard{background:var(--surface);border-radius:16px;padding:14px 16px;margin-bottom:10px}
.bbCardHead{font-size:10px;font-weight:600;color:var(--text-tertiary);text-transform:uppercase;letter-spacing:.08em;margin-bottom:10px}
.bbRow{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid var(--divider)}
.bbRow:last-child{border-bottom:none}
.bbLbl{color:var(--text-secondary);font-size:12px;font-weight:500}
.bbVal{color:var(--text);font-weight:600;font-size:13px}
.ok{color:var(--ok)}
.warn{color:var(--warn)}
.err{color:var(--err)}
.bbGrid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:500px){.bbGrid{grid-template-columns:1fr}}
.bbUptime{text-align:center;margin-top:6px;padding:14px;font-size:12px;font-weight:500;color:var(--text-tertiary);letter-spacing:.03em;background:var(--surface);border-radius:16px}
</style></head>
<body>

<header class="bbTopbar">
    <button class="bbBrandToggle" id="brandToggle" aria-label="Toggle theme">
        <span class="bbBrand">Diagnostics</span>
        <span class="bbModeLabel" id="modeLabel">Bright</span>
        <span class="bbModeIcon" aria-hidden="true">◐</span>
    </button>
    <a href="/" class="bbBack">← Dashboard</a>
</header>

<div class="bbGrid">
<section class="bbCard">
    <div class="bbCardHead">Configuration</div>
    <div class="bbRow"><span class="bbLbl">WiFi Mode</span><span class="bbVal" id="wifi-mode">--</span></div>
    <div class="bbRow"><span class="bbLbl">SSID</span><span class="bbVal" id="wifi-ssid">--</span></div>
    <div class="bbRow"><span class="bbLbl">Telemetry Rate</span><span class="bbVal" id="telem-hz">--</span></div>
    <div class="bbRow"><span class="bbLbl">GPS Model</span><span class="bbVal" id="gps-model">--</span></div>
    <div class="bbRow"><span class="bbLbl">Warmup Fixes</span><span class="bbVal" id="warmup-fixes">--</span></div>
</section>
<section class="bbCard">
    <div class="bbCardHead">Sensors</div>
    <div class="bbRow"><span class="bbLbl">IMU Rate</span><span class="bbVal bbNum" id="imu-rate">--</span></div>
    <div class="bbRow"><span class="bbLbl">GPS Rate</span><span class="bbVal bbNum" id="gps-rate">--</span></div>
    <div class="bbRow"><span class="bbLbl">Loop Rate</span><span class="bbVal bbNum" id="loop-rate">--</span></div>
    <div class="bbRow"><span class="bbLbl">ZUPT Rate</span><span class="bbVal bbNum" id="zupt-rate">--</span></div>
    <div class="bbRow"><span class="bbLbl">EKF/GPS</span><span class="bbVal bbNum" id="ekf-per-gps">--</span></div>
</section>
</div>

<div class="bbGrid">
<section class="bbCard">
    <div class="bbCardHead">GPS Status</div>
    <div class="bbRow"><span class="bbLbl">Fix</span><span class="bbVal" id="gps-fix">--</span></div>
    <div class="bbRow"><span class="bbLbl">Satellites</span><span class="bbVal bbNum" id="gps-sats">--</span></div>
    <div class="bbRow"><span class="bbLbl">HDOP</span><span class="bbVal bbNum" id="gps-hdop">--</span></div>
    <div class="bbRow"><span class="bbLbl">Warmup</span><span class="bbVal" id="gps-warmup">--</span></div>
</section>
<section class="bbCard">
    <div class="bbCardHead">EKF Health</div>
    <div class="bbRow"><span class="bbLbl">Position σ</span><span class="bbVal bbNum" id="pos-sigma">--</span></div>
    <div class="bbRow"><span class="bbLbl">Velocity σ</span><span class="bbVal bbNum" id="vel-sigma">--</span></div>
    <div class="bbRow"><span class="bbLbl">Yaw σ</span><span class="bbVal bbNum" id="yaw-sigma">--</span></div>
    <div class="bbRow"><span class="bbLbl">Bias X</span><span class="bbVal bbNum" id="bias-x">--</span></div>
    <div class="bbRow"><span class="bbLbl">Bias Y</span><span class="bbVal bbNum" id="bias-y">--</span></div>
</section>
</div>

<div class="bbGrid">
<section class="bbCard">
    <div class="bbCardHead">Sensor Fusion</div>
    <div class="bbRow"><span class="bbLbl">Lon Raw</span><span class="bbVal bbNum" id="lon-raw">--</span></div>
    <div class="bbRow"><span class="bbLbl">Lon Filtered</span><span class="bbVal bbNum" id="lon-filt">--</span></div>
    <div class="bbRow"><span class="bbLbl">Lon Blended</span><span class="bbVal bbNum" id="lon-blend">--</span></div>
    <div class="bbRow"><span class="bbLbl">GPS Weight</span><span class="bbVal bbNum" id="gps-wt">--</span></div>
    <div class="bbRow"><span class="bbLbl">GPS Accel</span><span class="bbVal bbNum" id="gps-acc">--</span></div>
    <div class="bbRow"><span class="bbLbl">GPS Rejected</span><span class="bbVal" id="gps-rej">--</span></div>
</section>
<section class="bbCard">
    <div class="bbCardHead">Orientation</div>
    <div class="bbRow"><span class="bbLbl">Pitch Corr</span><span class="bbVal bbNum" id="pitch-corr">--</span></div>
    <div class="bbRow"><span class="bbLbl">Roll Corr</span><span class="bbVal bbNum" id="roll-corr">--</span></div>
    <div class="bbRow"><span class="bbLbl">Confidence</span><span class="bbVal bbNum" id="orient-conf">--</span></div>
    <div class="bbRow"><span class="bbLbl">Yaw Bias</span><span class="bbVal bbNum" id="yaw-bias">--</span></div>
    <div class="bbRow"><span class="bbLbl">Yaw Cal</span><span class="bbVal" id="yaw-cal">--</span></div>
    <div class="bbRow"><span class="bbLbl">Tilt X/Y</span><span class="bbVal bbNum" id="tilt-xy">--</span></div>
</section>
</div>

<section class="bbCard">
    <div class="bbCardHead">System</div>
    <div class="bbRow"><span class="bbLbl">Heap Free</span><span class="bbVal bbNum" id="heap">--</span></div>
    <div class="bbRow"><span class="bbLbl">TX Success</span><span class="bbVal bbNum" id="tx-ok">--</span></div>
    <div class="bbRow"><span class="bbLbl">TX Failed</span><span class="bbVal bbNum" id="tx-fail">--</span></div>
</section>

<div class="bbUptime" id="uptime">Uptime: --</div>

<script>
const $=id=>document.getElementById(id);

let isDark=window.matchMedia('(prefers-color-scheme:dark)').matches;
function applyTheme(){document.body.classList.toggle('dark',isDark);$('modeLabel').textContent=isDark?'Dark':'Bright'}
applyTheme();
$('brandToggle').onclick=()=>{isDark=!isDark;applyTheme()};

function rateClass(actual,expected){const pct=actual/expected;if(pct>=0.9)return'ok';if(pct>=0.5)return'warn';return'err'}
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
    const ps=$('pos-sigma');
    ps.textContent=d.ekf.pos_sigma.toFixed(2)+' m';
    ps.className='bbVal bbNum '+(d.ekf.pos_sigma<5?'ok':(d.ekf.pos_sigma<10?'warn':'err'));
    const vs=$('vel-sigma');
    vs.textContent=d.ekf.vel_sigma.toFixed(2)+' m/s';
    vs.className='bbVal bbNum '+(d.ekf.vel_sigma<0.5?'ok':(d.ekf.vel_sigma<1.0?'warn':'err'));
    const ys=$('yaw-sigma');
    ys.textContent=d.ekf.yaw_sigma_deg.toFixed(1)+'°';
    ys.className='bbVal bbNum '+(d.ekf.yaw_sigma_deg<5?'ok':(d.ekf.yaw_sigma_deg<10?'warn':'err'));
    const bx=$('bias-x');
    bx.textContent=d.ekf.bias_x.toFixed(3)+' m/s²';
    bx.className='bbVal bbNum '+(Math.abs(d.ekf.bias_x)<0.3?'ok':(Math.abs(d.ekf.bias_x)<0.5?'warn':'err'));
    const by=$('bias-y');
    by.textContent=d.ekf.bias_y.toFixed(3)+' m/s²';
    by.className='bbVal bbNum '+(Math.abs(d.ekf.bias_y)<0.3?'ok':(Math.abs(d.ekf.bias_y)<0.5?'warn':'err'));
    const hp=$('heap');
    hp.textContent=(d.system.heap_free/1024).toFixed(0)+' KB';
    hp.className='bbVal bbNum '+(d.system.heap_free>40000?'ok':(d.system.heap_free>20000?'warn':'err'));
    $('tx-ok').textContent=d.system.tx_ok.toLocaleString();
    const txf=$('tx-fail');
    txf.textContent=d.system.tx_fail;
    txf.className='bbVal bbNum '+(d.system.tx_fail>0?'warn':'ok');
    const s=d.system.uptime_s;
    const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
    $('uptime').textContent='Uptime: '+h+'h '+m+'m '+sec+'s';
    if(d.fusion){
      $('lon-raw').textContent=d.fusion.lon_raw.toFixed(3)+' m/s²';
      $('lon-filt').textContent=d.fusion.lon_filtered.toFixed(3)+' m/s²';
      $('lon-blend').textContent=d.fusion.lon_blended.toFixed(3)+' m/s²';
      const wt=$('gps-wt');
      wt.textContent=(d.fusion.gps_weight*100).toFixed(0)+'%';
      wt.className='bbVal bbNum '+(d.fusion.gps_weight>0?'ok':'warn');
      const ga=$('gps-acc');
      ga.textContent=isNaN(d.fusion.gps_accel)?'N/A':d.fusion.gps_accel.toFixed(3)+' m/s²';
      const rej=$('gps-rej');
      rej.textContent=d.fusion.gps_rejected?'Yes':'No';
      rej.className='bbVal '+(d.fusion.gps_rejected?'warn':'ok');
      const pc=$('pitch-corr');
      pc.textContent=d.fusion.pitch_corr.toFixed(1)+'°';
      pc.className='bbVal bbNum '+(Math.abs(d.fusion.pitch_corr)<10?'ok':'warn');
      const rc=$('roll-corr');
      rc.textContent=d.fusion.roll_corr.toFixed(1)+'°';
      rc.className='bbVal bbNum '+(Math.abs(d.fusion.roll_corr)<10?'ok':'warn');
      const oc=$('orient-conf');
      oc.textContent=d.fusion.pitch_conf.toFixed(0)+'%/'+d.fusion.roll_conf.toFixed(0)+'%';
      oc.className='bbVal bbNum '+(d.fusion.pitch_conf>50?'ok':(d.fusion.pitch_conf>10?'warn':'err'));
      const yb=$('yaw-bias');
      const ybVal=Math.abs(d.fusion.yaw_bias*1000);
      yb.textContent=ybVal.toFixed(1)+' mrad/s';
      yb.className='bbVal bbNum '+(ybVal<10?'ok':(ybVal<50?'warn':'err'));
      const yc=$('yaw-cal');
      yc.textContent=d.fusion.yaw_calibrated?'Yes':'No';
      yc.className='bbVal '+(d.fusion.yaw_calibrated?'ok':'warn');
      const txy=$('tilt-xy');
      txy.textContent=d.fusion.tilt_x.toFixed(3)+'/'+d.fusion.tilt_y.toFixed(3);
      const tiltMax=Math.max(Math.abs(d.fusion.tilt_x),Math.abs(d.fusion.tilt_y));
      txy.className='bbVal bbNum '+(tiltMax<0.3?'ok':(tiltMax<0.5?'warn':'err'));
    }
  }catch(e){console.log('Diag error:',e)}
}
setInterval(update,1000);
update();
</script>
</body></html>"#;

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
            max_uri_handlers: 9, /* Dashboard, telemetry, status, calibrate, settings GET,
                                  * settings SET, diagnostics page, diagnostics API */
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

        // WebSocket removed - using HTTP polling for reliability and to eliminate
        // thread blocking

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
