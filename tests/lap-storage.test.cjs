const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const {indexedDB,IDBKeyRange}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function section(start,end){const from=html.indexOf(start);return html.slice(from,html.indexOf(end,from))}
let sequence=0;
async function setup(){
 const db=await new Promise((resolve,reject)=>{
  const request=indexedDB.open('lap-storage-'+(++sequence),1);
  request.onupgradeneeded=()=>{request.result.createObjectStore('tracks',{keyPath:'id'});request.result.createObjectStore('lap_history',{keyPath:'id'}).createIndex('trackId','trackId')};
  request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);
 });
 const track={id:'original',bestLapMs:61000,lapCount:4};
 const c={Date,Math,Error,IDBKeyRange,trackDb:db,activeTrack:track,currentSessionId:'original-session',currentSessionStart:100,genUUID:()=>String(++sequence)};
 vm.createContext(c);
 vm.runInContext(section('async function updateTrackBestLap(','// Start line indicator')+section('async function clearLapHistory(','// LapTracker'),c);
 await write(db,{tracks:[track]});return {db,c,track};
}
function write(db,collections){return new Promise((resolve,reject)=>{const tx=db.transaction(Object.keys(collections),'readwrite');tx.oncomplete=resolve;tx.onabort=()=>reject(tx.error);for(const [name,rows] of Object.entries(collections))for(const row of rows)tx.objectStore(name).put(row)})}
function read(db,name){return new Promise((resolve,reject)=>{const request=db.transaction(name).objectStore(name).getAll();request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error)})}
function abortStoreWrite(db,target,method){
 return {transaction(names,mode){
  const tx=db.transaction(names,mode),objectStore=tx.objectStore.bind(tx);
  tx.objectStore=name=>{
   const store=objectStore(name);if(name!==target)return store;
   const original=store[method].bind(store);store[method]=(...args)=>{const request=original(...args);request.addEventListener('success',()=>tx.abort());return request};return store;
  };return tx;
 }};
}
for(const next of ['switch','deactivate'])test('pending lap keeps original track and session after '+next,async()=>{
 const {db,c,track}=await setup();
 const pending=c.updateTrackBestLap(60023);
 const replacement={id:'new-track',lapCount:19,bestLapMs:40000};
 c.activeTrack=next==='switch'?replacement:null;c.currentSessionId=next==='switch'?'new-session':null;c.currentSessionStart=200;
 await pending;
 const saved=await read(db,'tracks'),laps=await read(db,'lap_history');
 assert.equal(saved[0].lapCount,5);assert.equal(saved[0].bestLapMs,60023);
 assert.equal(laps.length,1);assert.equal(laps[0].trackId,'original');assert.equal(laps[0].sessionId,'original-session');assert.equal(laps[0].sessionStart,100);assert.equal(laps[0].lapNumber,5);
 assert.equal(replacement.lapCount,19);assert.equal(track.lapCount,4);db.close();
});
test('lap result snapshots its session while database opening waits',async()=>{
 const {db,c}=await setup();let open;c.trackDb=null;c.openTrackDB=()=>new Promise(resolve=>{open=()=>{c.trackDb=db;resolve()}});
 const pending=c.updateTrackBestLap(60023);c.activeTrack=null;c.currentSessionId=null;open();await pending;
 assert.equal((await read(db,'lap_history'))[0].sessionId,'original-session');db.close();
});
test('failed history write rolls back track best/count and rejects the completion',async()=>{
 const {db,c,track}=await setup();c.trackDb=abortStoreWrite(db,'lap_history','add');
 await assert.rejects(c.updateTrackBestLap(60023),/Lap result storage failed/);
 assert.equal((await read(db,'tracks'))[0].lapCount,4);assert.equal((await read(db,'tracks'))[0].bestLapMs,61000);assert.equal((await read(db,'lap_history')).length,0);
 assert.equal(track.lapCount,4);assert.equal(track.bestLapMs,61000);db.close();
});
test('overlapping completions use committed counts and preserve the fastest result',async()=>{
 const {db,c,track}=await setup();await Promise.all([c.updateTrackBestLap(60023),c.updateTrackBestLap(62000)]);
 const saved=(await read(db,'tracks'))[0],laps=await read(db,'lap_history');
 assert.equal(saved.lapCount,6);assert.equal(saved.bestLapMs,60023);assert.deepEqual(laps.map(l=>l.lapNumber).sort(),[5,6]);assert.equal(track.lapCount,6);assert.equal(track.bestLapMs,60023);db.close();
});
test('clearing one history atomically resets only that track count and retains bests',async()=>{
 const {db,c}=await setup();await write(db,{tracks:[{id:'other',lapCount:2,bestLapMs:55000}],lap_history:[{id:'lap',trackId:'original'},{id:'other-lap',trackId:'other'}]});
 await c.deleteLapHistory('original');
 const saved=await read(db,'tracks'),laps=await read(db,'lap_history');
 assert.equal(saved.find(t=>t.id==='original').lapCount,0);assert.equal(saved.find(t=>t.id==='original').bestLapMs,61000);assert.equal(saved.find(t=>t.id==='other').lapCount,2);assert.equal(laps.length,1);assert.equal(laps[0].id,'other-lap');assert.equal(c.activeTrack.lapCount,0);db.close();
});
test('clear-all waits for commit and updates all track counts',async()=>{
 const {db,c}=await setup();await write(db,{tracks:[{id:'other',lapCount:2,bestLapMs:55000}],lap_history:[{id:'lap',trackId:'original'}]});
 await c.clearAllLapHistory();assert.equal((await read(db,'lap_history')).length,0);assert.ok((await read(db,'tracks')).every(t=>t.lapCount===0));assert.equal(c.activeTrack.lapCount,0);db.close();
});
test('history clear rejects an abort after request success without changing counts',async()=>{
 const {db,c}=await setup();await write(db,{lap_history:[{id:'lap',trackId:'original'}]});c.trackDb=abortStoreWrite(db,'lap_history','clear');
 await assert.rejects(c.clearAllLapHistory(),/Lap history deletion failed/);assert.equal((await read(db,'lap_history')).length,1);assert.equal((await read(db,'tracks'))[0].lapCount,4);assert.equal(c.activeTrack.lapCount,4);db.close();
});

test('per-track clear rolls back deleted laps when the count write aborts',async()=>{
 const {db,c}=await setup();await write(db,{lap_history:[{id:'lap',trackId:'original'}]});c.trackDb=abortStoreWrite(db,'tracks','put');
 await assert.rejects(c.deleteLapHistory('original'),/Lap history deletion failed/);assert.equal((await read(db,'lap_history')).length,1);assert.equal((await read(db,'tracks'))[0].lapCount,4);assert.equal(c.activeTrack.lapCount,4);db.close();
});
test('a rejected completion cannot inflate the next saved result',async()=>{
 const {db,c}=await setup();c.trackDb=abortStoreWrite(db,'lap_history','add');await assert.rejects(c.updateTrackBestLap(60000));
 c.trackDb=db;await c.updateTrackBestLap(62000);const saved=(await read(db,'tracks'))[0],laps=await read(db,'lap_history');
 assert.equal(saved.lapCount,5);assert.equal(saved.bestLapMs,61000);assert.equal(laps.length,1);assert.equal(laps[0].lapNumber,5);assert.equal(laps[0].lapTimeMs,62000);db.close();
});

test('a track deleted while database opening waits is never recreated by its pending lap',async()=>{
 const {db,c}=await setup();let open;c.trackDb=null;c.openTrackDB=()=>new Promise(resolve=>{open=()=>{c.trackDb=db;resolve()}});
 const pending=c.updateTrackBestLap(60023);
 await new Promise((resolve,reject)=>{const tx=db.transaction('tracks','readwrite');tx.oncomplete=resolve;tx.onabort=()=>reject(tx.error);tx.objectStore('tracks').delete('original')});
 open();await assert.rejects(pending,/Track was deleted/);
 assert.equal((await read(db,'tracks')).length,0);assert.equal((await read(db,'lap_history')).length,0);assert.equal(c.activeTrack.lapCount,4);db.close();
});
