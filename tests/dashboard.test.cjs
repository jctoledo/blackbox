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
