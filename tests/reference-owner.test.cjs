const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const {indexedDB}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const section=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
const deferred=()=>{let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no});return{promise,resolve,reject}};
function fixture(){
 const stored=[],updates=[],pending=[],elements=new Map(),owner={id:'A',type:'loop'};
 const c={console:{log(){},error(){}},Date,Math,ArrayBuffer,activeTrack:owner,currentSessionId:'session-A',referenceSaveGeneration:0,referenceLap:{id:'previous',lapTimeMs:70000,spatial:{version:1}},referencePersistenceError:'',fmtLapTime:String,
  $:id=>{if(!elements.has(id))elements.set(id,{classList:{add(){}},textContent:''});return elements.get(id)},
  validSpatialReference:()=>true,updateRecUI(){updates.push(c.referencePersistenceError)},
  saveRefLap(ref){stored.push(ref);const gate=deferred();pending.push(gate);return gate.promise}};
 vm.createContext(c);vm.runInContext(section('async function saveNewRefLap(','async function saveTrack('),c);
 const tracker={toBinary:()=>new ArrayBuffer(0),samples:[],dist:100,spatialOrigin:{lat:37,lon:-122},spatialPoints:[]};
 return{c,owner,stored,pending,updates,save:(ms=60000)=>c.saveNewRefLap('A',ms,tracker)};
}
test('old-owner reference failure cannot replace the current warning after track switch',async()=>{
 const f=fixture(),p=f.save();f.c.activeTrack={id:'B'};f.c.currentSessionId='session-B';f.c.referencePersistenceError='Current warning';f.pending[0].reject(Error('quota'));await p;
 assert.equal(f.c.referencePersistenceError,'Current warning');assert.equal(f.updates.length,0);
});
test('same-track reactivation does not accept reference publication from the previous timing session',async()=>{
 const f=fixture(),p=f.save(),current={id:'new-session-reference'};f.c.activeTrack={id:'A',type:'loop'};f.c.currentSessionId='session-A-new';f.c.referenceLap=current;f.pending[0].resolve(f.stored[0]);await p;
 assert.equal(f.c.referenceLap,current);assert.equal(f.updates.length,0);
});
test('current-owner failure preserves the displayed reference and reports the save failure',async()=>{
 const f=fixture(),previous=f.c.referenceLap,p=f.save();f.pending[0].reject(Error('quota'));await p;
 assert.equal(f.c.referenceLap,previous);assert.match(f.c.referencePersistenceError,/Reference could not be saved/);assert.equal(f.updates.length,1);
});
test('old success cannot clear a newer owners reference warning',async()=>{
 const f=fixture(),p=f.save();f.c.activeTrack={id:'A',type:'loop'};f.c.currentSessionId='new';f.c.referencePersistenceError='New owner failure';f.pending[0].resolve(f.stored[0]);await p;
 assert.equal(f.c.referencePersistenceError,'New owner failure');assert.equal(f.updates.length,0);
});
let serial=0;
async function persistence(){
 const db=await new Promise((resolve,reject)=>{const r=indexedDB.open('reference-owner-'+ ++serial);r.onerror=()=>reject(r.error);r.onsuccess=()=>resolve(r.result);r.onupgradeneeded=()=>{r.result.createObjectStore('tracks',{keyPath:'id'});r.result.createObjectStore('reference_laps',{keyPath:'id'}).createIndex('trackId','trackId',{unique:true})}});
 const c={trackDb:db,validSpatialReference:spatial=>spatial?.version===1};vm.createContext(c);vm.runInContext(section('function commitTrackWrite(','function openTrackDB(')+section('// Reference lap CRUD','async function getRefLap('),c);
 const write=(name,value)=>new Promise((resolve,reject)=>{const tx=db.transaction(name,'readwrite');tx.objectStore(name).put(value);tx.oncomplete=resolve;tx.onabort=()=>reject(tx.error)});
 await write('tracks',{id:'A'});
 const rows=()=>new Promise((resolve,reject)=>{const r=db.transaction('reference_laps').objectStore('reference_laps').getAll();r.onsuccess=()=>resolve(r.result);r.onerror=()=>reject(r.error)});
 return{db,c,write,rows};
}
test('a slower late reference cannot replace a faster spatial reference committed first',async()=>{
 const f=await persistence(),fast={id:'fast',trackId:'A',lapTimeMs:50000,spatial:{version:1}},slow={...fast,id:'slow',lapTimeMs:60000};
 await f.c.saveRefLap(fast);const retained=await f.c.saveRefLap(slow);assert.equal((await f.rows())[0].id,'fast');assert.equal(retained.id,'fast');f.db.close();
});
test('a clean spatial trace can upgrade a legacy distance-only reference',async()=>{
 const f=await persistence();await f.c.saveRefLap({id:'legacy',trackId:'A',lapTimeMs:40000});await f.c.saveRefLap({id:'spatial',trackId:'A',lapTimeMs:60000,spatial:{version:1}});assert.equal((await f.rows())[0].id,'spatial');f.db.close();
});
test('a reference delayed by database opening cannot recreate data for a deleted track',async()=>{
 const f=await persistence(),gate=deferred();f.c.trackDb=null;f.c.openTrackDB=async()=>{await gate.promise;f.c.trackDb=f.db};
 const p=f.c.saveRefLap({id:'orphan',trackId:'A',lapTimeMs:60000,spatial:{version:1}});
 await new Promise((resolve,reject)=>{const tx=f.db.transaction('tracks','readwrite');tx.objectStore('tracks').delete('A');tx.oncomplete=resolve;tx.onabort=()=>reject(tx.error)});gate.resolve();
 await assert.rejects(p,/deleted/);assert.equal((await f.rows()).length,0);f.db.close();
});

test('clearing a reference waits for an earlier save delayed by database opening',async()=>{
 const f=await persistence(),gate=deferred();f.c.trackDb=null;f.c.openTrackDB=async()=>{await gate.promise;f.c.trackDb=f.db};
 f.c.activeTrack=null;f.c.IDBKeyRange=require('fake-indexeddb').IDBKeyRange;vm.runInContext(section('async function deleteRefLap(','// Session management for lap history'),f.c);
 const saved=f.c.saveRefLap({id:'pending',trackId:'A',lapTimeMs:60000,spatial:{version:1}});
 await Promise.resolve();f.c.trackDb=f.db;const cleared=f.c.deleteRefLap('A');
 // A readonly transaction finishes after any already-started clear, before the delayed save opens.
 await f.rows();gate.resolve();await Promise.all([saved,cleared]);assert.equal((await f.rows()).length,0);f.db.close();
});

test('an older same-owner failure cannot replace a later successful save warning state',async()=>{
 const f=fixture(),old=f.save(),latest=f.save(50000);f.pending[1].resolve(f.stored[1]);await latest;f.pending[0].reject(Error('old failure'));await old;
 assert.equal(f.c.referenceLap.lapTimeMs,50000);assert.equal(f.c.referencePersistenceError,'');assert.equal(f.updates.length,1);
});
test('same-owner successes keep the faster displayed reference under reversed completions',async()=>{
 const f=fixture(),fast=f.save(50000),slow=f.save(60000);f.pending[1].resolve(f.stored[1]);await slow;f.pending[0].resolve(f.stored[0]);await fast;
 assert.equal(f.c.referenceLap.lapTimeMs,50000);
});
test('a failed reference transaction does not block the next queued save',async()=>{
 const f=await persistence();await assert.rejects(f.c.saveRefLap({id:'missing',trackId:'deleted',lapTimeMs:60000,spatial:{version:1}}),/deleted/);
 await f.c.saveRefLap({id:'next',trackId:'A',lapTimeMs:60000,spatial:{version:1}});assert.equal((await f.rows())[0].id,'next');f.db.close();
});
