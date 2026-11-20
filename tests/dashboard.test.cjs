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
    fmtDelta:ms=>(ms/1000).toFixed(2),updateDeltaBar(){},updateTrackBestLap:async ms=>saved.push(ms),saveNewRefLap:(id,ms)=>references.push(ms)};
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

test('recording commits preserve samples arriving during a pending write',async()=>{
 const transactions=[],records=[];
 const c={console,Date,Math,db:{transaction(){
   const tx={objectStore:()=>({put:value=>records.push(value)})};transactions.push(tx);return tx;
 }}};
 vm.createContext(c);
 vm.runInContext(section('// Recording state.', 'function updateRecUI()')+'function updateRecUI(){};sessionId=10;chunkBuffer=[{t:1}];',c);
 const first=c.saveChunk();
 vm.runInContext('chunkBuffer.push({t:2})',c);
 const second=c.saveChunk();
 assert.equal(records.length,1);
 transactions[0].oncomplete();await first;
 await new Promise(resolve=>setImmediate(resolve));
 assert.equal(records.length,2);
 assert.equal(records[0].data[0].t,1);assert.equal(records[1].data[0].t,2);
 assert.equal(records[1].chunkIndex,1);
 transactions[1].oncomplete();await second;
 assert.equal(vm.runInContext('chunkBuffer.length',c),0);
});
test('aborted recording writes restore the batch before newer samples',async()=>{
 let tx;const c={console,Date,Math,db:{transaction(){tx={objectStore:()=>({put(){}})};return tx}}};
 vm.createContext(c);vm.runInContext(section('// Recording state.', 'function updateRecUI()')+'function updateRecUI(){};sessionId=10;chunkBuffer=[{t:1}];',c);
 const pending=c.saveChunk();vm.runInContext('chunkBuffer.push({t:2})',c);
 tx.onabort();await assert.rejects(pending,/storage failed/);
 assert.equal(vm.runInContext('chunkBuffer.map(x=>x.t).join(",")',c),'1,2');
 assert.equal(vm.runInContext('chunkIndex',c),0);
});

test('track save waits for transaction commit and rejects a later abort',async()=>{
 let tx;const c={Date,trackDb:{transaction(){tx={objectStore:()=>({put(){}})};return tx}}};
 vm.createContext(c);vm.runInContext(section('async function saveTrack(', 'async function getTrack('),c);
 let settled=false;const pending=c.saveTrack({id:'track'});pending.then(()=>{settled=true},()=>{});
 await Promise.resolve();assert.equal(settled,false);
 tx.onabort();await assert.rejects(pending,/Track storage failed/);
});
test('reference replacement deletes old key and writes new lap in one committed transaction',async()=>{
 const ops=[];let tx,lookup;const store={index:()=>({getKey:()=>lookup={}}),delete:key=>ops.push(['delete',key]),put:ref=>ops.push(['put',ref.id])};
 const c={trackDb:{transaction(){tx={objectStore:()=>store};return tx}}};vm.createContext(c);
 vm.runInContext(section('function commitTrackWrite(', 'function openTrackDB(')+section('async function saveRefLap(', 'async function getRefLap('),c);
 let settled=false;const p=c.saveRefLap({id:'new',trackId:'track'}).then(()=>{settled=true});
 lookup.result='old';lookup.onsuccess();await Promise.resolve();assert.equal(settled,false);
 assert.deepEqual(ops,[['delete','old'],['put','new']]);tx.oncomplete();await p;assert.equal(settled,true);
});
test('failed stop acknowledgement preserves the active track',async()=>{
 const active={id:'still-running'},messages=[];
 const c={activeTrack:active,requestDevice:async()=>({ok:false}),alert:s=>messages.push(s)};
 vm.createContext(c);vm.runInContext(section('async function deactivateTrack(', '// Track Recording Functions'),c);
 assert.equal(await c.deactivateTrack(),false);assert.equal(c.activeTrack,active);assert.match(messages[0],/Could not stop/);
});
test('a stalled request is aborted and its timer is cleaned up',async()=>{
 let timeout,cleared=false;
 const c={AbortController,setTimeout:fn=>{timeout=fn;return 9},clearTimeout:id=>{assert.equal(id,9);cleared=true},fetch:(_,options)=>new Promise((resolve,reject)=>options.signal.addEventListener('abort',()=>reject(new Error('aborted'))))};
 vm.createContext(c);vm.runInContext(section('async function requestDevice(', 'function markTelemetryUnavailable('),c);
 const p=c.requestDevice('/api/telemetry');timeout();await assert.rejects(p,/aborted/);assert.equal(cleared,true);
});

test('distance progress is explicitly estimated and never asserts a finish',()=>{
 const c={Number,Math};vm.createContext(c);vm.runInContext(section('function estimatedLapProgress(', 'function updateLapProgress('),c);
 assert.equal(c.estimatedLapProgress({reliable:true,dist:250},{totalDistance:1000}),25);
 assert.equal(c.estimatedLapProgress({reliable:true,dist:1000},{totalDistance:1000}),99);
 assert.equal(c.estimatedLapProgress({reliable:false,dist:250},{totalDistance:1000}),null);
 assert.equal(c.estimatedLapProgress({reliable:true,dist:250},null),null);
 assert.equal(c.estimatedLapProgress({reliable:true,dist:250},{totalDistance:NaN}),null);
 assert.equal(c.estimatedLapProgress({reliable:true,dist:1200},{totalDistance:1000}),null);
});
test('reference distance is invalid for late starts, gaps and position jumps',()=>{
 const c={Math,Number,ArrayBuffer,Float32Array};vm.createContext(c);
 vm.runInContext(section('class LapTracker{','function calcDelta(')+'\nthis.Tracker=LapTracker;',c);
 for(const values of [[[0,0,2000]],[[0,0,100],[2,0,1700]],[[0,0,100],[51,0,200]],[[NaN,0,100]]]){
  const tracker=new c.Tracker();for(const v of values)tracker.update(...v);assert.equal(tracker.reliable,false);assert.equal(tracker.isValid(),false);
  tracker.reset();assert.equal(tracker.reliable,true);
 }
 const tracker=new c.Tracker();for(let i=0;i<20;i++)tracker.update(i*4,0,i*100);assert.equal(tracker.isValid(),true);
});
test('delta never extrapolates beyond recorded reference distance',()=>{
 const c={Float32Array,Number};vm.createContext(c);vm.runInContext(section('function calcDelta(', 'function fmtDelta('),c);
 const ref={samples:new Float32Array([0,0,100,10000]).buffer};
 assert.equal(c.calcDelta(50,5500,ref).deltaMs,500);
 assert.equal(c.calcDelta(101,10000,ref).valid,false);
 assert.equal(c.calcDelta(NaN,10000,ref).valid,false);
});

test('a slower first lap after rearming cannot replace the saved best reference',()=>{
 const {c,references,elements}=context();c.activeTrack.bestLapMs=58000;c.referenceLap={lapTimeMs:58000};
 c.updateLapTimer(75,1,0,{state:'timing',last_lap_ms:60023,best_lap_ms:60023,gps_valid:true});
 assert.equal(references.length,0);assert.equal(elements.get('best-lap').textContent,'0:58.000');
});
test('CSV retains timing evidence and distinguishes missing values from zero',()=>{
 const c={Number};vm.createContext(c);vm.runInContext(section('function samplesToCSV(', 'async function exportCSV('),c);
 const lines=c.samplesToCSV([{t:123,sp:0,gpsOk:0,gpsCourse:NaN,lapTimeMs:1700,lapCnt:2,lastLapMs:61023,deviceTimeMs:4000}]).trim().split('\n');
 const names=lines[0].split(','),values=lines[1].split(',');
 assert.equal(names.length,values.length);const row=Object.fromEntries(names.map((key,i)=>[key,values[i]]));
 assert.equal(row.speed,'0');assert.equal(row.gps_course,'');assert.equal(row.gps_lat,'');assert.equal(row.last_lap_ms,'61023');assert.equal(row.device_time_ms,'4000');
});

test('completed-lap feedback uses the exact persistent device time',()=>{
 const {c,elements}=context();c.updateLapTimer(75,1,0,{state:'timing',last_lap_ms:60023,best_lap_ms:60023,gps_valid:true});
 assert.equal(elements.get('lap-result').hidden,false);assert.match(elements.get('lap-result').textContent,/BEST LAP.*1:00.023/);
});
test('track text and attribute values are escaped before rendering',()=>{
 const c={String};vm.createContext(c);vm.runInContext(html.split('\n').find(l=>l.startsWith('function escHtml(')),c);
 assert.equal(c.escHtml('<img src=x onerror="alert(1)">'), '&lt;img src=x onerror=&quot;alert(1)&quot;&gt;');
 assert.equal(c.escHtml('id" onclick="bad'), 'id&quot; onclick=&quot;bad');
});

test('recording health counts observed GPS validity and gaps across chunks',()=>{
 const c={Number,Math};vm.createContext(c);vm.runInContext(section('function recordingHealth(', 'async function getAllSessions('),c);
 const h=c.recordingHealth([{chunkIndex:1,data:[{t:3000,gpsOk:0},{t:2900,gpsOk:1}]},{chunkIndex:0,data:[{t:0,gpsOk:1},{t:100,gpsOk:1}]}]);
 assert.equal(h.samples,4);assert.equal(h.gpsPercent,75);assert.equal(h.gaps,1);assert.equal(h.maxGapMs,2900);assert.equal(h.clockReversals,1);assert.equal(h.durationMs,3000);
 assert.equal(c.recordingHealth([]).gpsPercent,null);
 const stats=c.lapConsistency([{lapTimeMs:61000},{lapTimeMs:60000},{lapTimeMs:90000},{lapTimeMs:NaN}]);
 assert.equal(stats.median,61000);assert.equal(stats.spread,30000);assert.equal(stats.count,3);assert.equal(stats.best,60000);
 assert.equal(c.lapConsistency([{lapTimeMs:60000},{lapTimeMs:61001}]).median,60500.5);
});

test('nonfinite motion is rejected before telemetry can update the UI',()=>{
 const c={DataView,Uint8Array,Number};vm.createContext(c);vm.runInContext(section('function process(buf','async function poll()'),c);
 const b=new ArrayBuffer(82),v=new DataView(b),bytes=new Uint8Array(b);v.setUint8(0,4);v.setUint16(1,0xAA55,true);v.setFloat32(7,NaN,true);v.setUint16(80,bytes.slice(0,80).reduce((a,b)=>a+b,0)&65535,true);
 assert.throws(()=>c.process(b),/Invalid numeric telemetry/);
});
test('storage failure stops collection and preserves unsaved rows for export',async()=>{
 let tx;const c={console,Date,Math,db:{transaction(){tx={objectStore:()=>({put(){}})};return tx}}};vm.createContext(c);
 vm.runInContext(section('// Recording state.', 'function updateRecUI()')+'function updateRecUI(){};rec=true;sessionId=10;chunkBuffer=[{t:1}];',c);
 const p=c.saveChunk();tx.onabort();await assert.rejects(p);assert.equal(vm.runInContext('rec',c),false);assert.equal(vm.runInContext('chunkBuffer.length',c),1);assert.match(vm.runInContext('recordingError',c),/Keep this tab open/);
});

test('stage completion preserves its exact result while the device re-arms',()=>{
 const {c,elements,saved}=context();c.activeTrack.type='point_to_point';
 c.updateLapTimer(0,1,0,{state:'armed',last_lap_ms:1287475,best_lap_ms:1287475,gps_valid:true});
 assert.equal(saved[0],1287475);assert.match(elements.get('lap-result').textContent,/RUN.*21:27.475/);assert.equal(elements.get('lap-state').textContent,'Finished · ready for next start');
 c.lapTracker=null;c.updateLapTimer(0,1,0,{state:'armed',last_lap_ms:1287475,best_lap_ms:1287475,gps_valid:true});assert.equal(saved.length,1);
});

test('emergency export includes committed and unsaved rows without another write',async()=>{
 let csv='',clicked=false;const c={console,Number,Date,setTimeout:()=>{},db:{transaction:()=>({objectStore:name=>name==='sessions'?{get:()=>({result:{sessionId:10}})}:{index:()=>({getAll:()=>({result:[{chunkIndex:0,data:[{t:1,sp:10}]}]})})}})},idbPromise:async r=>r.result,chunkWrite:null,chunkBuffer:[{t:2,sp:11}],sessionId:10,Blob:class{constructor(parts){csv=parts.join('')}},URL:{createObjectURL:()=>'/synthetic.csv'},document:{createElement:()=>({click(){clicked=true}})}};
 vm.createContext(c);vm.runInContext(section('function samplesToCSV(', 'async function exportCSV(')+section('async function exportSessionById(', 'function formatBytes('),c);
 const result=await c.exportSessionById(10);assert.equal(result.sampleCount,2);assert.equal(clicked,true);assert.equal(csv.trim().split('\n').length,3);assert.equal(c.chunkBuffer.length,1);
});


test('reference save completion cannot replace a newly selected track reference',async()=>{
 let finish;const original={lapTimeMs:50000};const c={console,Date,Math,activeTrack:{id:'old'},referenceLap:original,saveRefLap:()=>new Promise(resolve=>{finish=resolve})};vm.createContext(c);
 vm.runInContext(section('async function saveNewRefLap(', 'async function saveTrack('),c);
 const p=c.saveNewRefLap('old',60000,{toBinary:()=>new ArrayBuffer(0),samples:[],dist:100});c.activeTrack={id:'new'};finish();await p;assert.equal(c.referenceLap,original);
});


test('track JSON preserves binary reference samples across serialization',()=>{
 const c={Float32Array,Array};vm.createContext(c);vm.runInContext(section('function portableReference(', 'async function exportTrackData('),c);
 const ref={samples:new Float32Array([0,0,25.5,1250]).buffer,lapTimeMs:1250};const restored=JSON.parse(JSON.stringify(c.portableReference(ref)));
 assert.deepEqual(restored.samples,[0,0,25.5,1250]);assert.equal(restored.sampleEncoding,'distance_metres_elapsed_ms_f32_pairs');assert.equal(c.portableReference(null),null);
});


test('streamed recording summaries retain gaps and validity across chunk boundaries',()=>{
 const c={Number,Math};vm.createContext(c);vm.runInContext(section('function recordingHealth(', 'async function getAllSessions('),c);
 const chunks=[{chunkIndex:0,data:[{t:0,gpsOk:1},{t:100,gpsOk:1}]},{chunkIndex:1,data:[{t:2200,gpsOk:0}]}];
 const whole=c.recordingHealth(chunks),streamed=c.recordingHealth([chunks[1]],c.recordingHealth([chunks[0]]));
 assert.deepEqual(streamed,whole);assert.equal(streamed.gaps,1);assert.equal(streamed.gpsPercent,67);
});
