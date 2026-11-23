const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const {indexedDB}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return{promise,resolve}};
let serial=0;
async function fixture(){
 const elements=new Map(),alerts=[],downloads=[],events=new Map();let arms=0;
 const $=id=>{if(!elements.has(id)){const classes=new Set();elements.set(id,{id,textContent:'',hidden:false,disabled:false,style:{},classList:{add(...v){v.forEach(x=>classes.add(x))},remove(...v){v.forEach(x=>classes.delete(x))},contains:v=>classes.has(v)},focus(){}})}return elements.get(id)};
 const database=await new Promise((resolve,reject)=>{const r=indexedDB.open('track-recovery-'+(++serial));r.onerror=()=>reject(r.error);r.onsuccess=()=>resolve(r.result);r.onupgradeneeded=()=>{r.result.createObjectStore('tracks',{keyPath:'id'});r.result.createObjectStore('reference_laps',{keyPath:'id'}).createIndex('trackId','trackId',{unique:true})}});
 const c={console,Math,Number,Date,performance,Promise,Blob,Float32Array,ArrayBuffer,$,alerts,downloads,trackDb:database,trackRecorder:null,activeTrack:null,referenceLap:null,lapPersistenceError:'',referencePersistenceError:'',referenceSaveGeneration:0,currentSessionId:'original-session',routeCapture:null,
  document:{addEventListener:(name,fn)=>events.set(name,fn),createElement:()=>({click(){downloads.push(this)},remove(){}}),body:{appendChild(){}}},window:{addEventListener:(name,fn)=>events.set(name,fn)},URL:{createObjectURL:blob=>{c.lastBlob=blob;return'blob:route'},revokeObjectURL(){}},setTimeout(){},
  notifyUser:message=>alerts.push(message),requestDecision:async(message,value)=>typeof value==='string'?'Recovery circuit':true,closeTrackModal(){},renderTrackList(){},updateRecUI(){},validSpatialReference:()=>true,
  trackAutoDetector:{withdraw(){},beginTrackChange(){},endTrackChange(){}},getGpsRef:()=>({lat:37,lon:-122}),currentPos:{valid:true,x:0,y:0,lat:37,lon:-122,speed:20,yaw:0,sigma:2},
  activateTrack:async()=>{arms++;return true}};
 vm.createContext(c);vm.runInContext(slice('function validGpsCoordinates(', '// Track Manager IndexedDB'),c);vm.runInContext(slice('async function saveTrack(', 'async function getTrack('),c);vm.runInContext(slice('// Track Recording Functions','async function updateTrackBestLap('),c);
 await c.startTrackRecording('point_to_point');
 return{c,$,database,alerts,events,arms:()=>arms,async rows(){return new Promise((resolve,reject)=>{const tx=database.transaction('tracks'),r=tx.objectStore('tracks').getAll();r.onsuccess=()=>resolve(r.result);r.onerror=()=>reject(r.error)})},points(count=12){for(let i=1;i<count;i++)c.trackRecorder.addSample({x:i*10,y:0,lat:37,lon:-122+i*.00012,speed:10,heading:0,sigma:2});c.onLoopDetected()}};
}
test('invalid finish retains route ownership and Continue can collect enough points',async()=>{
 const f=await fixture();f.points(5);await f.c.finishTrackRecording();assert.equal(f.c.trackRecorder.recording,true);
 await f.c.continueTrackRecording();f.points(15);await f.c.finishTrackRecording();assert.equal((await f.rows()).length,1);assert.equal(f.arms(),1);assert.equal(f.c.trackRecorder,null);
});
test('an aborted route transaction retains one frozen identity for retry and portable export',async()=>{
 const f=await fixture();f.points();const original=f.database.transaction.bind(f.database);let abort=true;
 f.database.transaction=(...args)=>{const tx=original(...args);if(abort&&args[1]==='readwrite')queueMicrotask(()=>tx.abort());return tx};
 await f.c.finishTrackRecording();assert.equal((await f.rows()).length,0);assert.equal(f.$('record-overlay').classList.contains('active'),true);assert.equal(f.arms(),0);
 assert.match(f.$('rec-save-status').textContent,/not saved|could not be saved/i);
 // Exercise the actual importer against the emitted file; export must not touch storage.
 const noExportWrites=f.database.transaction;f.database.transaction=()=>{throw Error('Recovery download must not access IndexedDB')};
 await f.c.exportCapturedTrack();const exported=JSON.parse(await f.c.lastBlob.text());f.database.transaction=noExportWrites;
 vm.runInContext(slice('// Versioned backup / restore.', 'function readBackupStores('),f.c);
 const normalized=f.c.normalizeBackup(exported),imported=f.c.validateBackup(normalized);
 assert.equal(imported.tracks,1);assert.equal(imported.recordings,0);assert.equal(imported.references,0);assert.equal(imported.laps,0);
 assert.deepEqual(JSON.parse(JSON.stringify(normalized.data.tracks[0])),exported.track);assert.equal(exported.track.centerline.length,11);
 abort=false;await f.c.finishTrackRecording();const rows=await f.rows();assert.equal(rows.length,1);assert.equal(rows[0].id,exported.track.id);assert.equal(f.arms(),1);assert.equal(f.c.trackRecorder,null);
});
test('a committed route with failed arming retries arming without another storage write',async()=>{
 const f=await fixture();f.points();let calls=0;f.c.activateTrack=async()=>++calls>1;
 await f.c.finishTrackRecording();const first=await f.rows();assert.equal(first.length,1);assert.match(f.$('rec-save-status').textContent,/saved.*arming.*not confirmed/i);assert.equal(f.$('record-overlay').classList.contains('active'),true);
 const original=f.database.transaction.bind(f.database);f.database.transaction=(...args)=>{assert.notEqual(args[1],'readwrite','retry arm must not save again');return original(...args)};
 await f.c.finishTrackRecording();assert.equal(calls,2);assert.equal((await f.rows())[0].id,first[0].id);assert.equal(f.c.trackRecorder,null);
});
test('duplicate save clicks and cancellation during commit cannot discard or arm a different owner',async()=>{
 const f=await fixture();f.points();const gate=deferred(),save=f.c.saveTrack;let writes=0;
 f.c.saveTrack=async(...args)=>{writes++;await gate.promise;return save(...args)};
 const pending=f.c.finishTrackRecording();await f.c.finishTrackRecording();await f.c.cancelTrackRecording();assert.ok(f.c.trackRecorder);assert.equal(writes,1);
 gate.resolve();await pending;assert.equal((await f.rows()).length,1);assert.equal(f.arms(),1);assert.equal(f.c.trackRecorder,null);
});
test('frozen geometry does not change after capture and explicit discard can be declined',async()=>{
 const f=await fixture();f.points();const points=f.c.trackRecorder.keyPoints.length;
 f.c.currentPos={...f.c.currentPos,lon:-121.9};f.c.updateTrackRecording();assert.equal(f.c.trackRecorder.keyPoints.length,points);
 f.c.requestDecision=async()=>false;assert.equal(await f.c.cancelTrackRecording(),false);assert.ok(f.c.trackRecorder);
 f.c.requestDecision=async()=>true;assert.equal(await f.c.cancelTrackRecording(),true);assert.equal(f.c.trackRecorder,null);assert.equal(f.c.routeCapture,null);assert.equal((await f.rows()).length,0);
});
test('replaced capture owner cannot be armed or have its controls overwritten by a late save',async()=>{
 const f=await fixture();f.points();const gate=deferred(),entered=deferred(),save=f.c.saveTrack;
 f.c.saveTrack=async(...args)=>{entered.resolve();await gate.promise;return save(...args)};
 const pending=f.c.finishTrackRecording();await entered.promise;const replacement={busy:false};f.c.routeCapture=replacement;f.$('rec-save-status').textContent='New owner';
 gate.resolve();await pending;assert.equal(f.arms(),0);assert.equal(f.$('rec-save-status').textContent,'New owner');assert.equal(f.c.routeCapture,replacement);assert.equal((await f.rows()).length,1);
});
test('new-route add collision aborts without changing an original track',async()=>{
 const f=await fixture();f.points();const save=f.c.saveTrack;let first=true;
 f.c.saveTrack=async(track,options)=>{if(first){first=false;await save({...track,name:'Original stored track'},{create:true})}return save(track,options)};
 await f.c.finishTrackRecording();const rows=await f.rows();assert.equal(rows.length,1);assert.equal(rows[0].name,'Original stored track');assert.equal(f.arms(),0);assert.match(f.$('rec-save-status').textContent,/not saved/i);
});
test('reference transaction abort visibly warns and preserves prior stored and displayed reference',async()=>{
 const f=await fixture(),c=f.c;vm.runInContext(slice('function commitTrackWrite(', 'function openTrackDB(')+slice('// Reference lap CRUD', 'async function getRefLap(')+slice('async function saveNewRefLap(', 'async function saveTrack('),c);
 const previous={id:'old-ref',trackId:'original',lapTimeMs:70000,samples:new ArrayBuffer(0)};
 await new Promise((resolve,reject)=>{const tx=f.database.transaction('tracks','readwrite');tx.objectStore('tracks').put({id:'original'});tx.oncomplete=resolve;tx.onabort=()=>reject(tx.error)});await c.saveRefLap(previous);c.referenceLap=previous;c.activeTrack={id:'original',type:'loop'};
 c.updateRecUI=()=>{f.$('recording-notice').textContent=c.referencePersistenceError;f.$('recording-notice').hidden=!c.referencePersistenceError};
 const transaction=f.database.transaction.bind(f.database);let abort=true;
 f.database.transaction=(...args)=>{const tx=transaction(...args);if(abort&&args[1]==='readwrite'){const put=tx.objectStore('reference_laps').put.bind(tx.objectStore('reference_laps'));const store=tx.objectStore('reference_laps');store.put=(...values)=>{const request=put(...values);request.onsuccess=()=>tx.abort();return request}}return tx};
 const tracker={toBinary:()=>new ArrayBuffer(0),samples:[],dist:100,spatialPoints:[],spatialOrigin:{lat:37,lon:-122}};
 await c.saveNewRefLap('original',60000,tracker);assert.equal(c.referenceLap,previous);assert.match(f.$('recording-notice').textContent,/Reference could not be saved/);
 const stored=await new Promise((resolve,reject)=>{const tx=f.database.transaction('reference_laps'),r=tx.objectStore('reference_laps').getAll();r.onsuccess=()=>resolve(r.result);r.onerror=()=>reject(r.error)});assert.equal(stored.length,1);assert.equal(stored[0].id,'old-ref');
 abort=false;c.fmtLapTime=String;await c.saveNewRefLap('original',60000,tracker);assert.equal(c.referenceLap.lapTimeMs,60000);assert.equal(f.$('recording-notice').hidden,true);
});
