const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const {indexedDB}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(start,end)=>html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));
function context(){
 const c={console,Number,Math,Date,Map,Set,Float32Array,ArrayBuffer,Promise,rec:false,recordingBusy:false,chunkWrite:null,activeTrack:null,trackRecorder:null,chunkBuffer:[],sessionId:null,sessionStart:0,chunkIndex:0};
 vm.createContext(c);vm.runInContext(slice('// Versioned backup / restore.','// End backup / restore'),c);
 vm.runInContext(slice('function portableReference(', 'async function exportTrackData'),c);
 vm.runInContext(slice('function createRecordingSession(', 'async function startRecording'),c);return c;
}
let serial=0;
function database(tracks=false){return new Promise((resolve,reject)=>{const r=indexedDB.open('backup-test-'+(++serial));r.onerror=()=>reject(r.error);r.onsuccess=()=>resolve(r.result);r.onupgradeneeded=()=>{const d=r.result;if(tracks){d.createObjectStore('tracks',{keyPath:'id'});d.createObjectStore('reference_laps',{keyPath:'id'}).createIndex('trackId','trackId',{unique:true});d.createObjectStore('lap_history',{keyPath:'id'})}else{d.createObjectStore('sessions',{keyPath:'sessionId'});d.createObjectStore('chunks',{keyPath:['sessionId','chunkIndex']}).createIndex('sessionId','sessionId')}}})}
function fixture(c){
 const track={id:'track-a',name:'Circuit <one>',type:'loop',startLine:{p1:[-12,0],p2:[12,0],direction:Math.PI/2},gpsOrigin:{lat:37,lon:-122},centerline:[{x:0,y:0},{x:20,y:40}],bestLapMs:60000,lapCount:1};
 const data={sessions:[{sessionId:100,startTime:100,status:'complete'}],chunks:[{sessionId:100,chunkIndex:0,data:[{t:101,sp:20,gpsOk:1},{t:201,sp:21,gpsOk:0,ax:null}]}],tracks:[track],references:[{id:'ref-a',trackId:'track-a',lapTimeMs:60000,totalDistance:100,sampleCount:3,samples:[0,0,50,30000,100,60000],sampleEncoding:'distance_metres_elapsed_ms_f32_pairs'}],history:[{id:'lap-a',trackId:'track-a',sessionId:'lap-session-a',sessionStart:100,timestamp:200,lapTimeMs:60000,lapNumber:1}]};
 return{format:'blackbox-backup',version:1,data,checksum:c.backupChecksum(data)};
}
test('full backup round trip preserves all sample values and reference/history links without overwriting originals',async()=>{
 const c=context(),value=fixture(c);c.db=await database();c.trackDb=await database(true);
 await c.addBackupStores(c.db,{sessions:value.data.sessions,chunks:value.data.chunks});
 await c.addBackupStores(c.trackDb,{tracks:value.data.tracks,reference_laps:value.data.references.map(r=>({...r,samples:new Float32Array(r.samples).buffer})),lap_history:value.data.history});
 const backup=JSON.parse(JSON.stringify(await c.createBackup()));assert.equal(c.validateBackup(backup).sampleCount,2);
 const plan=c.makeRestorePlan(backup,value.data.sessions,'copy',100);await c.executeRestore(plan,c.db,c.trackDb);
 const recording=await c.readBackupStores(c.db,['sessions','chunks']),tracks=await c.readBackupStores(c.trackDb,['tracks','reference_laps','lap_history']);
 assert.equal(recording.sessions.length,2);assert.equal(tracks.tracks.length,2);assert.deepEqual(recording.chunks[1].data,value.data.chunks[0].data);
 const restored=tracks.tracks.find(t=>t.id!=='track-a');assert.equal(restored.name,'Circuit <one> (restored)');
 const ref=tracks.reference_laps.find(r=>r.trackId===restored.id);assert.deepEqual(Array.from(new Float32Array(ref.samples)),value.data.references[0].samples);
 assert.equal(tracks.lap_history.find(l=>l.id!=='lap-a').trackId,restored.id);
 assert.equal(tracks.tracks.find(t=>t.id==='track-a').name,value.data.tracks[0].name);
});
test('validation rejects damaged files, unsupported versions, orphan chunks, bad geometry and invalid references before writing',()=>{
 const c=context();const damaged=fixture(c);damaged.data.chunks[0].data[0].sp=999;assert.throws(()=>c.validateBackup(damaged),/checksum/);
 const version=fixture(c);version.version=99;assert.throws(()=>c.validateBackup(version),/version/);
 for(const mutate of [v=>v.data.chunks[0].sessionId=999,v=>v.data.tracks[0].gpsOrigin.lat=NaN,v=>v.data.references[0].samples=[0,0,50,100,25,200],v=>v.data.references[0].sampleCount=5,v=>v.data.history[0].trackId='missing',v=>v.data.sessions.push({...v.data.sessions[0]})]){const v=fixture(c);mutate(v);v.checksum=c.backupChecksum(v.data);assert.throws(()=>c.validateBackup(v),/Invalid backup/)}
});
test('aborted recording transaction leaves no partial imported chunks or sessions',async()=>{
 const c=context(),d=await database(),t=await database(true),plan=c.makeRestorePlan(fixture(c),[],'abort',1000);
 const aborting={transaction(...args){const tx=d.transaction(...args);queueMicrotask(()=>tx.abort());return tx}};
 await assert.rejects(c.executeRestore(plan,aborting,t));assert.equal(plan.recordingsDone,false);
 const result=await c.readBackupStores(d,['sessions','chunks']);assert.equal(result.sessions.length,0);assert.equal(result.chunks.length,0);
});
test('track-group abort rolls back references and history; retry never duplicates committed recordings',async()=>{
 const c=context(),d=await database(),t=await database(true),plan=c.makeRestorePlan(fixture(c),[],'retry',1000);
 const aborting={transaction(...args){const tx=t.transaction(...args);queueMicrotask(()=>tx.abort());return tx}};
 await assert.rejects(c.executeRestore(plan,d,aborting));assert.equal(plan.recordingsDone,true);assert.equal(plan.tracksDone,false);
 const failed=await c.readBackupStores(t,['tracks','reference_laps','lap_history']);for(const rows of Object.values(failed))assert.equal(rows.length,0);
 await c.executeRestore(plan,d,t);await c.executeRestore(plan,d,t);
 assert.equal((await c.readBackupStores(d,['sessions'])).sessions.length,1);assert.equal((await c.readBackupStores(t,['tracks'])).tracks.length,1);
});
test('key collision aborts the import rather than overwriting existing data',async()=>{
 const c=context(),d=await database(),t=await database(true),plan=c.makeRestorePlan(fixture(c),[],'collision',1000);
 await c.addBackupStores(d,{sessions:[{sessionId:plan.sessions[0].sessionId,startTime:1,status:'complete',marker:'keep'}]});
 await assert.rejects(c.executeRestore(plan,d,t));const result=await c.readBackupStores(d,['sessions','chunks']);assert.equal(result.sessions[0].marker,'keep');assert.equal(result.chunks.length,0);
});
test('new and concurrent recordings allocate IDs above restored sessions without replacing data',async()=>{
 const c=context();c.db=await database();await c.addBackupStores(c.db,{sessions:[{sessionId:5000,startTime:1,status:'complete'}]});
 const ids=await Promise.all([c.createRecordingSession(100),c.createRecordingSession(100)]);assert.deepEqual(ids,[5001,5002]);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,3);
});
test('backup includes unsaved tail without requiring another write or consuming its buffer',async()=>{
 const c=context();c.db=await database();c.trackDb=await database(true);c.sessionId=100;c.sessionStart=100;c.chunkBuffer=[{t:101,sp:20}];
 const value=await c.createBackup();assert.equal(value.data.chunks[0].data.length,1);assert.equal(c.chunkBuffer.length,1);assert.equal((await c.readBackupStores(c.db,['chunks'])).chunks.length,0);
 c.rec=true;await assert.rejects(c.createBackup(),/Stop session and route recording/);
});
test('version 2 single and bundled track exports restore reference samples and full history',()=>{
 const c=context(),v=fixture(c),entry={track:v.data.tracks[0],referenceLap:v.data.references[0],lapHistory:{sessions:[{laps:v.data.history}]}};
 for(const source of [{version:2,...entry},{version:2,tracks:[entry]}]){const normalized=c.normalizeBackup(source),summary=c.validateBackup(normalized);assert.equal(summary.tracks,1);assert.equal(summary.laps,1);assert.equal(summary.references,1);assert.equal(summary.recordings,0);assert.equal(normalized.legacyImport,true)}
 assert.throws(()=>c.normalizeBackup({version:1,track:entry.track}),/version 2/);
});

test('selective backup reads only the chosen recording and retains track data',async()=>{
 const c=context(),value=fixture(c);c.db=await database();c.trackDb=await database(true);
 await c.addBackupStores(c.db,{sessions:[...value.data.sessions,{sessionId:200,startTime:200,status:'complete'}],chunks:[...value.data.chunks,{sessionId:200,chunkIndex:0,data:[{t:201,sp:30}]}]});
 await c.addBackupStores(c.trackDb,{tracks:value.data.tracks});
 const chosen=await c.createBackup('200');assert.equal(chosen.data.sessions.length,1);assert.equal(chosen.data.sessions[0].sessionId,200);assert.equal(chosen.data.chunks[0].data[0].sp,30);assert.equal(chosen.data.tracks.length,1);
 const tracks=await c.createBackup('tracks');assert.equal(tracks.data.sessions.length,0);assert.equal(tracks.data.chunks.length,0);assert.equal(tracks.data.tracks.length,1);
});

test('backup remains available with a selected track when the device is disconnected',async()=>{
 const c=context();c.db=await database();c.trackDb=await database(true);c.activeTrack={id:'offline-track'};
 const value=await c.createBackup();assert.equal(value.format,'blackbox-backup');assert.equal(c.activeTrack.id,'offline-track');
});
function uiContext(){
 const c=context(),elements=new Map();c.$=id=>{if(!elements.has(id))elements.set(id,{textContent:'',disabled:id==='backup-restore',value:'all'});return elements.get(id)};
 c.document={querySelectorAll:()=>['backup-download','backup-choose','backup-restore','backup-scope'].map(c.$)};
 c.genUUID=()=> 'ui-test';c.renderDataModal=async()=>{};c.renderTrackList=async()=>{};
 return{c,elements};
}
test('file preview writes nothing, restore reports committed counts, and a bad replacement file disables restore',async()=>{
 const {c,elements}=uiContext(),value=fixture(c);c.db=await database();c.trackDb=await database(true);
 const text=JSON.stringify(value);await c.previewBackup({size:text.length,text:async()=>text});
 assert.match(elements.get('backup-status').textContent,/Checksum verified.*1 recordings.*2 samples.*1 tracks/);
 assert.equal(elements.get('backup-restore').disabled,false);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,0);
 await c.restoreBackup();assert.match(elements.get('backup-status').textContent,/Restore complete: 1 recordings and 1 tracks/);assert.equal(elements.get('backup-restore').disabled,true);
 assert.equal((await c.readBackupStores(c.trackDb,['tracks'])).tracks.length,1);
 await c.previewBackup({size:10,text:async()=>'{bad json'});assert.match(elements.get('backup-status').textContent,/not valid JSON/);assert.equal(elements.get('backup-restore').disabled,true);
});
test('restore UI reports partial commit and retries only the missing group',async()=>{
 const {c,elements}=uiContext(),value=fixture(c);c.db=await database();const actual=await database(true);
 c.trackDb={transaction(...args){const tx=actual.transaction(...args);queueMicrotask(()=>tx.abort());return tx}};
 const text=JSON.stringify(value);await c.previewBackup({size:text.length,text:async()=>text});await c.restoreBackup();
 assert.match(elements.get('backup-status').textContent,/Recordings committed; tracks were not restored/);assert.equal(elements.get('backup-restore').textContent,'Retry remaining data');assert.equal(elements.get('backup-restore').disabled,false);
 c.trackDb=actual;await c.restoreBackup();assert.match(elements.get('backup-status').textContent,/Restore complete/);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,1);assert.equal((await c.readBackupStores(actual,['tracks'])).tracks.length,1);
});
test('oversized backup is refused before reading the file or writing any data',async()=>{
 const {c,elements}=uiContext();let read=false;await c.previewBackup({size:100*1024*1024+1,text:async()=>{read=true;return '{}'}});
 assert.equal(read,false);assert.match(elements.get('backup-status').textContent,/100 MiB/);assert.equal(elements.get('backup-restore').disabled,true);
});
