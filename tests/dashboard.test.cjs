const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function section(start,end){return html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));}
function context(){
  const elements=new Map(),saved=[],references=[];
  const c={console,Math,Number,DataView,Uint8Array,setTimeout:()=>{},resize:()=>{},
    $:id=>{if(!elements.has(id))elements.set(id,{textContent:'',className:'',classList:{add(){},remove(){},contains(){return false}}});return elements.get(id)},
    expectedLapRevision:null,lapTimerActive:true,activeTrack:{id:'track-test',type:'loop',bestLapMs:61000},
    prevLapFlags:0,prevLapCnt:0,prevLapTimeMs:59900,
    LAP_FLAG_NEW_LAP:4,LAP_FLAG_NEW_BEST:8,suppressStartLineIndicator:false,
    lapTracker:{isValid:()=>true,reset(){},update(){throw Error('New lap data contaminated old reference')}},
    referenceLap:null,currentPos:{valid:true,x:0,y:0},lastDeltaMs:0,deltaTrend:0,
    updateDeltaBar(){},updateTrackBestLap:ms=>saved.push(ms),saveNewRefLap:(id,ms)=>references.push(ms)};
  vm.createContext(c);
  vm.runInContext(section('function fmtLapTime(', 'function updateReadouts'),c);
  return {c,elements,saved,references};
}
test('first completed lap survives missed one-frame flags and uses exact device result',()=>{
  const {c,saved,references,elements}=context();
  c.updateLapTimer(75,1,0,{state:'timing',last_lap_ms:60023,best_lap_ms:60023,gps_valid:true});
  assert.deepEqual(saved,[60023]); assert.deepEqual(references,[60023]);
  assert.equal(elements.get('last-lap').textContent,'1:00.023');
  assert.equal(elements.get('lap-delta').textContent,'−0.977s');
  c.lapTracker=null;
  c.updateLapTimer(108,1,0,{state:'timing',last_lap_ms:60023,best_lap_ms:60023,gps_valid:true});
  assert.equal(saved.length,1);
});
test('firmware reset is visible instead of pretending timing remains armed',()=>{
  const {c,elements,saved}=context();
  c.updateLapTimer(0,0,0,{state:'idle'});
  assert.equal(elements.get('lap-state').textContent,'Select track to re-arm');
  assert.equal(saved.length,0);
});
test('armed timer distinguishes missing GPS from waiting for a crossing',()=>{
  const {c,elements}=context();
  c.updateLapTimer(0,0,0,{state:'armed',gps_valid:false});
  assert.equal(elements.get('lap-state').textContent,'Waiting for GPS');
});
test('saved-track guidance transforms its origin after a device restart',()=>{
  const c={Math,activeTrack:{gpsOrigin:{lat:37,lon:-122},startLine:{p1:[-12,0],p2:[12,0],direction:Math.PI/2}},currentPos:{x:0,y:0},getGpsRef:()=>({lat:37.001,lon:-122})};
  vm.createContext(c);
  for(const name of ['transformCoord','transformLine'])vm.runInContext(html.split('\n').find(l=>l.startsWith('function '+name+'(')),c);
  vm.runInContext(section('function lineInCurrentFrame(', 'function formatDistance'),c);
  assert.ok(Math.abs(c.getDistanceToStartLine().distance-111.32)<0.01);
  assert.ok(Math.abs(c.getDistanceToStartLine().bearing+Math.PI/2)<0.001);
});
test('decoder rejects malformed or stale protocol packets',()=>{
  const c={DataView,Uint8Array}; vm.createContext(c);
  vm.runInContext(section('function process(buf','async function poll()'),c);
  assert.throws(()=>c.process(new ArrayBuffer(67)),/packet size/);
  assert.throws(()=>c.process(new ArrayBuffer(82)),/protocol/);
  const b=new ArrayBuffer(82),v=new DataView(b);v.setUint8(0,4);v.setUint16(1,0xAA55,true);
  assert.throws(()=>c.process(b),/checksum/);
});

test('old track telemetry cannot be recorded against a newly selected track',()=>{
 const {c,saved,elements}=context();c.expectedLapRevision=2;
 c.updateLapTimer(50,8,4,{state:'timing',revision:1,last_lap_ms:45000,best_lap_ms:45000});
 assert.equal(saved.length,0);assert.equal(elements.get('lap-state').textContent,'Waiting for track confirmation');
});
