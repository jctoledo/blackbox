const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function fixture(extra={}){const elements=new Map();const c={console,Date,Math,Number,setTimeout(){},$:id=>{if(!elements.has(id)){const classes=new Set(['timing']);elements.set(id,{textContent:'old value',hidden:false,classList:{add(...names){names.forEach(n=>classes.add(n))},remove(...names){names.forEach(n=>classes.delete(n))},contains:n=>classes.has(n)}})}return elements.get(id)},...extra};vm.createContext(c);return {c,elements}}
for(const state of ['idle','mismatch'])test('unconfirmed timing clears matched delta, result and active styling: '+state,()=>{
 let invalidated=0,delta;const {c}=fixture({expectedLapRevision:2,lapTimerActive:true,activeTrack:{type:'loop'},referenceLap:{spatial:{}},lapTracker:{startTime:100,invalidate(){invalidated++}},updateDeltaBar:value=>delta=value,lapResultGeneration:0});
 vm.runInContext(slice('function fmtLapTime(', 'function updateReadouts'),c);
 c.updateLapTimer(0,0,0,{state:state==='idle'?'idle':'timing',revision:1});
 assert.equal(delta?.deltaMs,null);assert.equal(invalidated,1);assert.equal(c.$('lap-result').hidden,true);assert.equal(c.$('lap-state').classList.contains('timing'),false);assert.equal(c.$('lap-section').classList.contains('timing'),false);
});
test('reset renders cleared speed and force statistics without telemetry',()=>{
 let drawn=0;const {c}=fixture({maxL:1,maxR:1,maxA:1,maxB:1,peak:1,maximumFreshSpeed:100,trail:[1],currentScale:2,emaGx:1,emaGy:1,displaySessionStart:1,effortPeaks:{reset(){}},drawG(){drawn++}});
 vm.runInContext(slice('function resetState()', 'let peakHighlightTimeout'),c);c.resetState();
 for(const id of ['speed-maximum','effort-accel','effort-brake','effort-left','effort-right'])assert.equal(c.$(id).textContent,'—',id);assert.equal(drawn,1);
});
test('recovered pending metadata does not claim a running request',()=>{
 const {c}=fixture({escHtml:String});vm.runInContext(slice('function recordingDetailsHTML(', 'async function exportRecordingDetails('),c);
 const out=c.recordingDetailsHTML({recordingMetadata:{status:'pending',requestedAt:1}});assert.match(out,/interrupted/i);assert.match(out,/no request is running/i);
});
test('empty interrupted recording has no saved samples promise',()=>{
 const {c}=fixture();vm.runInContext(slice('function recordingRecoveryLabel(', 'function recordingDetailsHTML('),c);
 assert.match(c.recordingRecoveryLabel({sampleCount:0}),/no saved samples/i);assert.match(c.recordingRecoveryLabel({sampleCount:1}),/saved data available/i);
});
test('live metadata refresh preserves the details element, expanded state, focus and modal scroll',()=>{
 let selected,focused=false,action;const button={focus(options){assert.equal(options.preventScroll,true);focused=true}},body={contains:()=>true,replaceChildren(...nodes){this.nodes=nodes},querySelector:()=>button};
 const details={open:true,querySelector:()=>body},row={querySelector:()=>details},scroller={scrollTop:123},modal={classList:{contains:()=>true},querySelector(selector){if(selector==='.bbModalBody')return scroller;selected=selector;return row}};
 const {c}=fixture({document:{activeElement:{},createElement:()=>({content:{querySelector:()=>({childNodes:['new metadata']})}})},$:()=>modal,recordingDetailsHTML:()=>'<details/>',handleSessionAction:value=>action=value});
 vm.runInContext(slice('function refreshRecordingDetails(', 'function recordingDetailsHTML('),c);c.refreshRecordingDetails({sessionId:42});
 assert.equal(selected,'[data-session-id="42"]');assert.equal(details.open,true);assert.equal(focused,true);assert.equal(scroller.scrollTop,123);assert.deepEqual(body.nodes,['new metadata']);button.onclick();assert.equal(action,button);
});
test('invalid IMU evidence hides maneuver while fresh GPS speed and timing still update',()=>{
 let now=0,timingUpdates=0;const {c}=fixture({performance:{now:()=>now},document:{body:{classList:{add(){}}}},lapResultGeneration:0,lapTimerActive:false,lapTracker:null,referenceLap:null,updateLapProgress(){},updateDeltaBar(){},DataView,Uint8Array,ArrayBuffer,firmwareGpsRef:{},updateJsGpsRef(){},getGpsRef:()=>null,updateTrackRecording(){},updateStartLineIndicator(){},updateFinishLineIndicator(){},activeTrack:{},trackRecorder:null,lastT:0,EMA_TAU:.07,emaGx:0,emaGy:0,speed_ema:0,speedForDisplay:Math.round,peak:100,maximumFreshSpeed:null,getModeColorHex:()=> 'red',MODES:{6:'BRAKING + CORNERING'},MODE_COLORS:{0:'gray',6:'red'},effortPeaks:{accel:null,brake:null,left:null,right:null,update(){}},updateReadouts(){},maxL:0,maxR:0,maxA:0,maxB:0,trail:[],TRAIL_DURATION_MS:3000,currentScale:1,lastGpsState:0,updateLapTimer(){timingUpdates++},drawG(){},cnt:0,rec:false});
 const original=c.$;c.$=id=>{const el=original(id);el.style??={};return el};
 vm.runInContext(slice('function forcePoint(', 'function clearForces(')+slice('function process(buf', '// Read the complete bounded body'),c);vm.runInContext(slice('function clearForces(', 'function hexToRgba(')+slice('// Receipt freshness watchdog.', '// End receipt freshness watchdog.')+slice('function markTelemetryUnavailable(', 'async function poll()'),c);
 const bytes=Uint8Array.from(fs.readFileSync('tests/fixtures/telemetry-v4.hex','utf8').replace(/\s/g,'').match(/../g),x=>parseInt(x,16)),d=new DataView(bytes.buffer);d.setUint8(59,6);d.setFloat32(51,80,true);d.setUint16(80,bytes.slice(0,80).reduce((sum,b)=>(sum+b)&65535,0),true);
 for(const imuAge of [null,undefined,201]){c.process(bytes.buffer,{gps_valid:true,gps_age_ms:0,imu_age_ms:imuAge,lat:37,lon:-122});assert.equal(c.$('maneuver').textContent,'—');assert.equal(c.$('maneuver').style.color,'gray');assert.ok(c.$('speed').textContent>0);assert.equal(c.currentPos.valid,true)}
 assert.equal(timingUpdates,3);c.process(bytes.buffer,{gps_valid:true,gps_age_ms:0,imu_age_ms:0,lat:37,lon:-122});assert.equal(c.$('maneuver').textContent,'BRAKING + CORNERING');assert.equal(timingUpdates,4);
 vm.runInContext('lastTelemetryAt=0',c);now=301;c.checkTelemetryFreshness();assert.equal(c.$('maneuver').textContent,'—');assert.equal(c.$('maneuver').style.color,'gray');
 c.$('maneuver').textContent='BRAKING + CORNERING';c.$('maneuver').style.color='red';now=1501;c.checkTelemetryFreshness();assert.equal(c.$('maneuver').textContent,'—');assert.equal(c.$('maneuver').style.color,'gray');
 c.process(bytes.buffer,{gps_valid:true,gps_age_ms:0,imu_age_ms:0,lat:37,lon:-122});assert.equal(c.$('maneuver').textContent,'BRAKING + CORNERING');assert.equal(c.$('maneuver').style.color,'red');assert.equal(timingUpdates,5);
});

function timingFixture(){
 const {c}=fixture({LAP_FLAG_NEW_LAP:4,LAP_FLAG_NEW_BEST:8,expectedLapRevision:2,lapTimerActive:true,activeTrack:{id:'new',type:'loop',gpsOrigin:{lat:37,lon:-122}},referenceLap:null,lastDeltaMs:0,deltaTrend:0,prevLapCnt:0,prevLapTimeMs:0,prevLapFlags:0,updateDeltaBar(){},referencePosition:p=>p?{x:p.x,y:p.y}:null,resize(){},currentPos:{valid:true,x:0,y:0,gpsAgeMs:0}});
 vm.runInContext(slice('class LapTracker{', 'function fmtDelta(')+';this.lapTracker=new LapTracker();'+slice('function fmtLapTime(', 'function updateReadouts'),c);return c;
}
test('late old revision before the first lap does not poison clean reference capture',()=>{
 const c=timingFixture();c.updateLapTimer(9000,3,0,{state:'timing',revision:1});
 assert.equal(c.lapTracker.reliable,true);
 for(let i=1;i<=11;i++){c.currentPos.x=i*6;c.updateLapTimer(i*100,0,0,{state:'timing',revision:2,gps_valid:true})}
 assert.equal(c.lapTracker.isValid(),true);
 let referenceSaved=false;c.saveNewRefLap=(id,ms,tracker)=>{assert.equal(id,'new');assert.equal(ms,1200);referenceSaved=tracker.isValid()};c.updateTrackBestLap=async()=>{};
 c.updateLapTimer(20,1,0,{state:'timing',revision:2,gps_valid:true,last_lap_ms:1200,best_lap_ms:1200});assert.equal(referenceSaved,true);
});
test('interrupted active lap stays invalid until a confirmed armed attempt resets it',()=>{
 const c=timingFixture();c.updateLapTimer(100,0,0,{state:'timing',revision:2,gps_valid:true});c.updateLapTimer(200,0,0,{state:'timing',revision:1});
 assert.equal(c.lapTracker.reliable,false);c.updateLapTimer(300,0,0,{state:'timing',revision:2,gps_valid:true});assert.equal(c.lapTracker.reliable,false);
 c.updateLapTimer(0,0,0,{state:'armed',revision:2,gps_valid:true});assert.equal(c.lapTracker.reliable,true);assert.equal(c.lapTracker.startTime,null);
 c.updateLapTimer(100,0,0,{state:'timing',revision:2,gps_valid:true});assert.equal(c.lapTracker.reliable,true);
});
