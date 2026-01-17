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

/// Tron-style dashboard with golden ratio layout, perspective G-meter, auto-zoom camera
const DASHBOARD_HTML: &str = r#"<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes"><title>Blackbox</title>
<style>
:root {
    --cyan: #00f0ff;
    --amber: #ff6a00;
    --bg: #030508;
    --hue: 190;
}
*{margin:0;padding:0;box-sizing:border-box}
body{
    font-family:'Courier New',monospace;
    background: var(--bg);
    color:#fff;
    height:100vh;
    height:100dvh;
    display:flex;
    flex-direction:column;
    -webkit-user-select:none;
    user-select:none;
    overflow:hidden;
}

/* Header */
.hdr{
    flex: 0 0 auto;
    background: linear-gradient(180deg, rgba(0,30,40,0.95) 0%, rgba(0,20,30,0.8) 100%);
    padding:10px 16px;
    display:flex;
    justify-content:space-between;
    align-items:center;
    border-bottom: 1px solid hsla(var(--hue), 100%, 50%, 0.4);
}
.logo{font-weight:400;font-size:14px;letter-spacing:4px;text-transform:uppercase;color:hsl(var(--hue),100%,60%)}
.hdr-r{display:flex;align-items:center;gap:12px}
.timer{font-size:11px;opacity:0.5;font-variant-numeric:tabular-nums;letter-spacing:2px}
.st{display:flex;align-items:center;gap:6px;font-size:10px;letter-spacing:1px}
.dot{width:6px;height:6px;background:#444;border-radius:50%}
.dot.on{background:hsl(var(--hue),100%,50%);box-shadow:0 0 10px hsl(var(--hue),100%,50%)}

/* Golden Ratio Layout */
#telemetryLayout{display:flex;flex-direction:column;width:100%;overflow:hidden}
.golden-box{width:100%;overflow:hidden;display:flex;flex-direction:column;border:1px solid hsla(var(--hue),100%,50%,0.35);box-shadow:0 0 20px hsla(var(--hue),100%,50%,0.12)}
#boxTop.at-peak{border-color:rgba(255,106,0,0.45);box-shadow:0 0 25px rgba(255,106,0,0.15)}
.box-inner{flex:1;display:flex;flex-direction:column;gap:12px;padding:16px;overflow:hidden}
.box-scroll{overflow-y:auto}

/* Top Box: Speed Focus */
.speed-focus{position:relative;display:flex;align-items:center;justify-content:center;padding:20px 16px;overflow:hidden;height:100%}
.mode-bg{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);font-size:clamp(260px,70vw,400px);opacity:0.14;color:hsl(var(--hue),100%,50%);pointer-events:none;transition:color 0.3s,opacity 0.3s;z-index:0;line-height:1}
.mode-bg.at-peak{color:#ff6a00;opacity:0.18}
.speed-stack{position:relative;z-index:1;display:flex;flex-direction:column;align-items:center;justify-content:center}
.peak-row{display:flex;flex-direction:column;align-items:center;gap:4px;height:52px;margin-bottom:12px;transition:opacity 0.25s}
.peak-row.hidden{opacity:0;pointer-events:none}
.peak-label{font-size:10px;font-weight:500;color:#556;letter-spacing:3px;text-transform:uppercase}
.peak-val{font-size:clamp(20px,5vw,24px);font-weight:600;color:#5a5a6a;font-variant-numeric:tabular-nums;letter-spacing:1px;line-height:1}
.current-speed{font-size:clamp(96px,28vw,160px);font-weight:600;line-height:0.85;font-variant-numeric:tabular-nums;letter-spacing:-1px;color:hsl(var(--hue),100%,60%);transition:color 0.15s,text-shadow 0.15s}
.current-speed.at-peak{color:#ff6a00;text-shadow:0 0 40px rgba(255,106,0,0.3)}
.speed-unit{font-size:clamp(20px,5vw,24px);font-weight:600;letter-spacing:2px;color:#5a5a6a;margin-top:8px;margin-bottom:16px;text-transform:lowercase;font-variant-numeric:tabular-nums}
.maneuver{font-size:clamp(20px,5vw,24px);font-weight:600;letter-spacing:3px;text-transform:uppercase;color:hsl(var(--hue),50%,50%);transition:color 0.3s}
.maneuver.braking{color:#ff6a00}

/* Middle Box: G-Force */
#boxMiddle{position:relative;overflow:hidden;background:var(--bg)}
.gf-canvas-full{position:absolute;top:0;left:0;width:100%;height:100%}
.gf-clear{position:absolute;top:8px;left:10px;padding:4px 10px;font-size:9px;font-weight:500;font-family:inherit;letter-spacing:2px;color:#556;background:rgba(0,0,0,0.4);border:1px solid #334;cursor:pointer;z-index:10}
.gf-axis{position:absolute;display:flex;flex-direction:column;align-items:center;gap:2px;z-index:5;pointer-events:none}
.gf-axis-label{font-size:10px;font-weight:500;letter-spacing:2px;color:#667}
.gf-axis-val{font-size:clamp(14px,3.5vw,18px);font-weight:600;font-variant-numeric:tabular-nums;color:#5a5a6a;transition:color 0.15s}
.gf-axis-val.peak-active{color:hsl(var(--hue),100%,60%)}
.gf-axis-top{top:8px;left:50%;transform:translateX(-50%)}
.gf-axis-bottom{bottom:8px;left:50%;transform:translateX(-50%)}
.gf-axis-left{left:10px;top:50%;transform:translateY(-50%);flex-direction:row;gap:6px}
.gf-axis-right{right:10px;top:50%;transform:translateY(-50%);flex-direction:row;gap:6px}

/* Bottom Box */
#boxBottom{background:var(--bg);border:1px solid #1a2a30;box-shadow:none}
#boxBottom .box-inner{gap:6px;padding:8px 10px}
#boxBottom .metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:4px}
#boxBottom .met{padding:6px 4px;background:transparent;border:1px solid #1a2a30;text-align:center}
#boxBottom .met-val{font-size:14px;font-weight:600;font-variant-numeric:tabular-nums;color:#7a8a8f}
#boxBottom .met-lbl{font-weight:500;font-size:8px;color:#556}
#boxBottom .gps-box{padding:5px;font-size:9px;font-weight:500;background:transparent;border:1px solid #1a2a30;color:#667;text-align:center}
#boxBottom .gps-box.ok{color:#0a8}
#boxBottom .ctrl{display:flex;padding:6px 0 0 0;gap:6px;border-top:1px solid #1a2a30}
#boxBottom .btn{flex:1;padding:8px;font-size:9px;font-weight:600;background:transparent;border:1px solid #1a2a30;color:#667;font-family:inherit;cursor:pointer}
#boxBottom .btn-rec.on{background:rgba(255,50,50,0.1);border-color:#4a2020;color:#c66}
</style></head>
<body>
<div class="hdr">
    <div class="logo">BLACKBOX</div>
    <div class="hdr-r">
        <a href="/diagnostics" style="color:#334;text-decoration:none;font-size:9px;letter-spacing:1px;opacity:0.5">DIAG</a>
        <span class="timer" id="timer">00:00</span>
        <div class="st"><span class="dot" id="dot"></span><span id="stxt">--</span></div>
    </div>
</div>
<div id="telemetryLayout">
    <div id="boxTop" class="golden-box">
        <div class="box-inner speed-focus">
            <div class="mode-bg" id="mode-bg">&#9671;</div>
            <div class="speed-stack">
                <div class="peak-row" id="peak-row">
                    <span class="peak-label">PEAK</span>
                    <span class="peak-val" id="peak-val">0</span>
                </div>
                <div class="current-speed" id="current-speed">0</div>
                <div class="speed-unit">km/h</div>
                <div class="maneuver" id="maneuver">Idle</div>
            </div>
        </div>
    </div>
    <div id="boxMiddle" class="golden-box">
        <button class="gf-clear" id="rstg">CLEAR</button>
        <div class="gf-axis gf-axis-top"><span class="gf-axis-label">BRK</span><span class="gf-axis-val" id="maxB">0.00</span></div>
        <div class="gf-axis gf-axis-bottom"><span class="gf-axis-val" id="maxA">0.00</span><span class="gf-axis-label">ACC</span></div>
        <div class="gf-axis gf-axis-left"><span class="gf-axis-label">L</span><span class="gf-axis-val" id="maxL">0.00</span></div>
        <div class="gf-axis gf-axis-right"><span class="gf-axis-val" id="maxR">0.00</span><span class="gf-axis-label">R</span></div>
        <canvas id="gfc" class="gf-canvas-full"></canvas>
    </div>
    <div id="boxBottom" class="golden-box">
        <div class="box-inner box-scroll">
            <div class="metrics">
                <div class="met"><div class="met-val" id="latg">0.00</div><div class="met-lbl">LAT G</div></div>
                <div class="met"><div class="met-val" id="lng">0.00</div><div class="met-lbl">LON G</div></div>
                <div class="met"><div class="met-val" id="yaw">0</div><div class="met-lbl">YAW</div></div>
                <div class="met"><div class="met-val" id="hz">0</div><div class="met-lbl">HZ</div></div>
            </div>
            <div class="gps-box" id="gpsbox"><span id="gps">GPS: --</span></div>
            <div class="ctrl">
                <button class="btn btn-rec" id="rec">REC</button>
                <button class="btn" id="exp">EXPORT</button>
            </div>
        </div>
    </div>
</div>
<script>
const PHI=1.6180339887;
const $=id=>document.getElementById(id);

// Layout
function applyGoldenHeights(){
    const hdr=document.querySelector('.hdr');
    const hdrH=hdr?hdr.offsetHeight:50;
    const H=window.innerHeight-hdrH;
    const top=H/(PHI*PHI),middle=H/(PHI*PHI),bottom=H/(PHI*PHI*PHI);
    $('telemetryLayout').style.height=H+'px';
    $('boxTop').style.height=top+'px';
    $('boxMiddle').style.height=middle+'px';
    $('boxBottom').style.height=bottom+'px';
}
window.addEventListener('resize',()=>{applyGoldenHeights();resizeCanvas()});

// Canvas
const cv=$('gfc'),ctx=cv.getContext('2d');
const GRID_COLS=28,GRID_ROWS=18,PERSPECTIVE_STRENGTH=0.55;

function resizeCanvas(){
    const box=$('boxMiddle');
    if(!box)return;
    const rect=box.getBoundingClientRect();
    cv.width=rect.width*window.devicePixelRatio;
    cv.height=rect.height*window.devicePixelRatio;
    ctx.setTransform(1,0,0,1,0,0);
    ctx.scale(window.devicePixelRatio,window.devicePixelRatio);
}

// Perspective grid
function gridToScreen(col,row,w,h){
    const nx=(col/GRID_COLS)*2-1,ny=(row/GRID_ROWS)*2-1;
    const depthFactor=(ny+1)*0.5;
    const perspectiveFactor=0.4+depthFactor*0.6;
    const px=nx*perspectiveFactor,py=ny*0.85+(1-depthFactor)*0.1;
    const padX=w*0.06,padY=h*0.08;
    return{x:padX+(px+1)*0.5*(w-2*padX),y:padY+(py+1)*0.5*(h-2*padY)};
}
function getCellCorners(col,row,w,h){
    return[gridToScreen(col,row,w,h),gridToScreen(col+1,row,w,h),gridToScreen(col+1,row+1,w,h),gridToScreen(col,row+1,w,h)];
}

// Auto-zoom
let magnitudeHistory=[],currentMaxG=0.5;
const PEAK_WINDOW_MS=800,MIN_MAX_G=0.2,MAX_MAX_G=2.0,ZOOM_OUT_RATE=0.12,ZOOM_IN_RATE=0.025,DEADBAND=0.04,SAFE_RADIUS_RATIO=2/3;

function updateAutoZoom(gx,gy){
    const now=Date.now(),magnitude=Math.sqrt(gx*gx+gy*gy);
    magnitudeHistory.push({time:now,mag:magnitude});
    magnitudeHistory=magnitudeHistory.filter(h=>now-h.time<PEAK_WINDOW_MS);
    const m_peak=Math.max(...magnitudeHistory.map(h=>h.mag),0.05);
    const targetMaxG=Math.max(m_peak/SAFE_RADIUS_RATIO,MIN_MAX_G);
    const ratio=targetMaxG/currentMaxG;
    if(ratio>1+DEADBAND)currentMaxG+=(targetMaxG-currentMaxG)*ZOOM_OUT_RATE;
    else if(ratio<1-DEADBAND)currentMaxG+=(targetMaxG-currentMaxG)*ZOOM_IN_RATE;
    currentMaxG=Math.max(MIN_MAX_G,Math.min(MAX_MAX_G,currentMaxG));
}

function gForceToGrid(gx,gy){
    const col=Math.floor((gx/currentMaxG+1)*0.5*GRID_COLS);
    const row=Math.floor((-gy/currentMaxG+1)*0.5*GRID_ROWS);
    return{col:Math.max(0,Math.min(GRID_COLS-1,col)),row:Math.max(0,Math.min(GRID_ROWS-1,row))};
}

// Trail & drawing
let gridTrail=[];
const HUE_CYAN=190,HUE_AMBER=25;
let currentHue=190;

function drawG(){
    const w=cv.width/window.devicePixelRatio,h=cv.height/window.devicePixelRatio;
    ctx.clearRect(0,0,w,h);
    ctx.strokeStyle=`hsla(${currentHue},50%,45%,0.2)`;ctx.lineWidth=1;
    for(let col=0;col<=GRID_COLS;col++){ctx.beginPath();const t=gridToScreen(col,0,w,h),b=gridToScreen(col,GRID_ROWS,w,h);ctx.moveTo(t.x,t.y);ctx.lineTo(b.x,b.y);ctx.stroke()}
    for(let row=0;row<=GRID_ROWS;row++){ctx.beginPath();const l=gridToScreen(0,row,w,h),r=gridToScreen(GRID_COLS,row,w,h);ctx.moveTo(l.x,l.y);ctx.lineTo(r.x,r.y);ctx.stroke()}
    ctx.strokeStyle=`hsla(${currentHue},50%,50%,0.35)`;ctx.lineWidth=2;
    const cCol=GRID_COLS/2,cRow=GRID_ROWS/2;
    ctx.beginPath();const vT=gridToScreen(cCol,0,w,h),vB=gridToScreen(cCol,GRID_ROWS,w,h);ctx.moveTo(vT.x,vT.y);ctx.lineTo(vB.x,vB.y);ctx.stroke();
    ctx.beginPath();const hL=gridToScreen(0,cRow,w,h),hR=gridToScreen(GRID_COLS,cRow,w,h);ctx.moveTo(hL.x,hL.y);ctx.lineTo(hR.x,hR.y);ctx.stroke();
    for(let i=0;i<gridTrail.length;i++){
        const cell=gridTrail[i],corners=getCellCorners(cell.col,cell.row,w,h),alpha=cell.intensity*0.6;
        ctx.fillStyle=`hsla(${cell.hue},100%,55%,${alpha})`;
        ctx.beginPath();ctx.moveTo(corners[0].x,corners[0].y);ctx.lineTo(corners[1].x,corners[1].y);ctx.lineTo(corners[2].x,corners[2].y);ctx.lineTo(corners[3].x,corners[3].y);ctx.closePath();ctx.fill();
    }
}

// State
const M={0:'Idle',1:'Accelerating',2:'Braking',4:'Cornering',5:'Corner Exit',6:'Trail Braking'};
const MODE_ICONS={0:'\u25C7',1:'\u25B3',2:'\u25BD',4:'\u25C8',5:'\u2B21',6:'\u2B22'};
let rec=0,data=[],cnt=0,lastSeq=0;
let trail=[],maxL=0,maxR=0,maxA=0,maxB=0;
let speed_ema=0,sessionStart=Date.now();
let displayedPeak=0,sessionPeak=0,lastAccelTime=0,wasAccelerating=false;
const PEAK_UPDATE_DELAY=3000;

function fmtTime(ms){const s=Math.floor(ms/1000),m=Math.floor(s/60);return String(m).padStart(2,'0')+':'+String(s%60).padStart(2,'0')}

function resetGMax(){
    maxL=maxR=maxA=maxB=0;
    $('maxL').textContent=$('maxR').textContent=$('maxA').textContent=$('maxB').textContent='0.00';
    trail=[];gridTrail=[];magnitudeHistory=[];currentMaxG=0.5;
    displayedPeak=sessionPeak=0;$('peak-val').textContent='0';$('peak-row').classList.add('hidden');
    sessionStart=Date.now();
}

function process(buf){
    const d=new DataView(buf);
    const ax=d.getFloat32(7,1),ay=d.getFloat32(11,1),wz=d.getFloat32(19,1),sp=d.getFloat32(51,1),mo=d.getUint8(55);
    const lat=d.getFloat32(56,1),lon=d.getFloat32(60,1),gpsOk=d.getUint8(64);
    const latg=ay/9.81,lng=-ax/9.81,yawDeg=Math.abs(wz*57.3);

    speed_ema=0.7*sp+(1-0.7)*speed_ema;
    const dspd=speed_ema<1?0:Math.round(speed_ema);
    const now=Date.now();
    const isAccelerating=lng>0.05,isBraking=lng<-0.1;

    // Peak tracking
    if(isAccelerating){lastAccelTime=now;wasAccelerating=true}
    if(dspd>sessionPeak)sessionPeak=dspd;
    const accelStoppedLongEnough=wasAccelerating&&!isAccelerating&&(now-lastAccelTime>=PEAK_UPDATE_DELAY);
    if(accelStoppedLongEnough||isBraking){if(sessionPeak>displayedPeak)displayedPeak=sessionPeak;wasAccelerating=false}
    const atPeak=dspd>=displayedPeak&&dspd>0;

    // Update hue based on acceleration state
    currentHue=isAccelerating?HUE_CYAN:HUE_AMBER;
    document.documentElement.style.setProperty('--hue',currentHue);

    // Top box
    $('current-speed').textContent=dspd;
    $('current-speed').classList.toggle('at-peak',atPeak);
    $('boxTop').classList.toggle('at-peak',atPeak);
    if(displayedPeak>0&&dspd<displayedPeak){$('peak-row').classList.remove('hidden');$('peak-val').textContent=displayedPeak}else{$('peak-row').classList.add('hidden')}
    $('maneuver').textContent=M[mo]||'Idle';
    $('maneuver').classList.toggle('braking',isBraking);
    $('mode-bg').textContent=MODE_ICONS[mo]||'\u25C7';
    $('mode-bg').classList.toggle('at-peak',atPeak);

    // Bottom metrics
    $('latg').textContent=latg.toFixed(2);
    $('lng').textContent=lng.toFixed(2);
    $('yaw').textContent=yawDeg.toFixed(0);

    // GPS
    if(gpsOk){$('gps').textContent=lat.toFixed(5)+', '+lon.toFixed(5);$('gpsbox').className='gps-box ok'}
    else{$('gps').textContent='GPS: No Fix';$('gpsbox').className='gps-box'}

    // G-force tracking
    const gx=latg,gy=lng;
    updateAutoZoom(gx,gy);
    trail.push({x:gx,y:gy});if(trail.length>30)trail.shift();

    // Max G with active highlighting
    const maxLEl=$('maxL'),maxREl=$('maxR'),maxAEl=$('maxA'),maxBEl=$('maxB');
    maxLEl.classList.remove('peak-active');maxREl.classList.remove('peak-active');maxAEl.classList.remove('peak-active');maxBEl.classList.remove('peak-active');
    if(latg<0&&Math.abs(latg)>maxL){maxL=Math.abs(latg);maxLEl.textContent=maxL.toFixed(2);maxLEl.classList.add('peak-active')}
    if(latg>0&&latg>maxR){maxR=latg;maxREl.textContent=maxR.toFixed(2);maxREl.classList.add('peak-active')}
    if(lng>0&&lng>maxA){maxA=lng;maxAEl.textContent=maxA.toFixed(2);maxAEl.classList.add('peak-active')}
    if(lng<0&&Math.abs(lng)>maxB){maxB=Math.abs(lng);maxBEl.textContent=maxB.toFixed(2);maxBEl.classList.add('peak-active')}

    // G-meter trail
    if(trail.length>0){
        const current=trail[trail.length-1];
        const gridPos=gForceToGrid(current.x,current.y);
        const indicatorHue=isAccelerating?HUE_CYAN:HUE_AMBER;
        const lastTrail=gridTrail[gridTrail.length-1];
        if(!lastTrail||lastTrail.col!==gridPos.col||lastTrail.row!==gridPos.row){
            gridTrail.push({col:gridPos.col,row:gridPos.row,intensity:1.0,hue:indicatorHue});
        }
    }
    for(let i=gridTrail.length-1;i>=0;i--){gridTrail[i].intensity*=0.96;if(gridTrail[i].intensity<0.03)gridTrail.splice(i,1)}
    if(gridTrail.length>80)gridTrail.shift();

    drawG();
    cnt++;
    if(rec)data.push({t:Date.now(),sp,ax,ay,wz,mo,latg,lng,lat,lon,gpsOk});
}

// HTTP polling
async function poll(){
    try{
        const r=await fetch('/api/telemetry');
        const j=await r.json();
        if(j.data&&j.seq!==lastSeq){
            lastSeq=j.seq;
            const b=atob(j.data),a=new Uint8Array(b.length);
            for(let i=0;i<b.length;i++)a[i]=b.charCodeAt(i);
            process(a.buffer);
            $('dot').className='dot on';$('stxt').textContent='LIVE';
        }
        setTimeout(poll,33);
    }catch(e){$('dot').className='dot';$('stxt').textContent='--';setTimeout(poll,500)}
}

// Event handlers
$('rstg').onclick=resetGMax;
$('rec').onclick=()=>{
    rec=!rec;
    if(rec){data=[];$('rec').className='btn btn-rec on';$('rec').textContent='STOP'}
    else{$('rec').className='btn btn-rec';$('rec').textContent='REC';
    if(data.length){const s=JSON.parse(localStorage.getItem('bb')||'[]');s.unshift({id:Date.now(),n:data.length,d:data});localStorage.setItem('bb',JSON.stringify(s.slice(0,10)));alert('Saved '+data.length+' pts')}}
};
$('exp').onclick=()=>{
    const s=JSON.parse(localStorage.getItem('bb')||'[]');if(!s.length)return alert('No data');
    let c='time,speed,ax,ay,wz,mode,lat_g,lon_g,gps_lat,gps_lon,gps_valid\n';
    s[0].d.forEach(r=>{c+=r.t+','+r.sp+','+r.ax+','+r.ay+','+r.wz+','+r.mo+','+r.latg+','+r.lng+','+(r.lat||0)+','+(r.lon||0)+','+(r.gpsOk||0)+'\n'});
    const b=new Blob([c],{type:'text/csv'}),u=URL.createObjectURL(b),a=document.createElement('a');a.href=u;a.download='blackbox.csv';a.click();
};

setInterval(()=>{$('hz').textContent=cnt;cnt=0;$('timer').textContent=fmtTime(Date.now()-sessionStart)},1000);

// Init
applyGoldenHeights();
setTimeout(()=>{resizeCanvas();drawG()},50);
poll();
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
