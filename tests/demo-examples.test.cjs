const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const {indexedDB}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function fixture(){const events=new Map(),elements=new Map();const c={console:{...console,error(){}},Response,URL,performance:{now:()=>0},window:{},document:{addEventListener(name,fn){events.set(name,fn)},getElementById(id){if(!elements.has(id))elements.set(id,{});return elements.get(id)}},location:{href:'https://example.test/demo.html'}};c.events=events;c.elements=elements;vm.createContext(c);vm.runInContext(html.slice(html.indexOf('// Drive session core.'),html.indexOf('// End Drive session core.')),c);vm.runInContext(fs.readFileSync('docs/demo.js','utf8').replace(' window.fetch=',' window.examplesForTest=exampleRows;window.seedForTest=seedExampleRecording; window.fetch='),c);vm.runInContext(html.slice(html.indexOf('class RollingRuns{'),html.indexOf('const rollingRuns='))+';this.Runs=RollingRuns;',c);return c}
test('demo examples populate both cornering signs and produce auditable accepted/rejected runs',()=>{
 const c=fixture(),circuit=c.window.examplesForTest('circuit'),rolling=c.window.examplesForTest('rolling');
 for(const rows of [circuit,rolling])for(const row of rows){assert.equal(row.gpsSynthetic,1);for(const value of Object.values(row))assert.ok(Number.isFinite(value))}
 const forces=circuit.filter(r=>r.rowType!==1);assert.ok(Math.max(...forces.map(r=>r.latg))>.3);assert.ok(Math.min(...forces.map(r=>r.latg))<-.3);assert.ok(Math.max(...forces.map(r=>r.lng))>.15);assert.ok(Math.min(...forces.map(r=>r.lng))<-.3);
 vm.runInContext(html.slice(html.indexOf('function samplesToCSV('),html.indexOf('async function exportCSV(')),c);const csv=c.samplesToCSV(rolling).trim().split('\n');assert.equal(csv[0].split(',').at(-1),'synthetic');assert.equal(csv[1].split(',').at(-1),'1');assert.equal(Number(csv[1].split(',')[1]),36);
 const detector=new c.Runs();for(const row of rolling)detector.update(row);const accepted=detector.results.filter(r=>r.status==='accepted'),rejected=detector.results.filter(r=>r.status==='rejected');assert.equal(accepted.length,2);assert.equal(rejected.length,1);assert.equal(rejected[0].reason,'GPS gap or clock boundary');for(const r of accepted)assert.ok(Math.abs(r.seconds-5.588)<1e-8);assert.equal(rolling.reduce((sum,r)=>sum+(r.gpsMissing||0),0),6);
});
let serial=0;
function database(){return new Promise((res,rej)=>{const r=indexedDB.open('demo-fixtures-'+(++serial));r.onerror=()=>rej(r.error);r.onsuccess=()=>res(r.result);r.onupgradeneeded=()=>{r.result.createObjectStore('sessions',{keyPath:'sessionId'});r.result.createObjectStore('chunks',{keyPath:['sessionId','chunkIndex']})}})}
function all(db,name){return new Promise((res,rej)=>{const r=db.transaction(name).objectStore(name).getAll();r.onsuccess=()=>res(r.result);r.onerror=()=>rej(r.error)})}
test('example recording seeding is atomic, repeatable and never replaces an existing session',async()=>{
 const c=fixture(),db=await database(),rows=c.window.examplesForTest('rolling');
 const first=await c.window.seedForTest(db,'SIM example',rows,'rolling-v5');assert.equal(await c.window.seedForTest(db,'SIM example',rows,'rolling-v5'),first);
 const sessions=await all(db,'sessions'),chunks=await all(db,'chunks');assert.equal(sessions.length,1);assert.equal(sessions[0].status,'complete');assert.equal(chunks.flatMap(c=>c.data).length,rows.length);
 const aborted={transaction(...args){const tx=db.transaction(...args);queueMicrotask(()=>tx.abort());return tx}};await assert.rejects(c.window.seedForTest(aborted,'Failed',rows,'different'));assert.equal((await all(db,'sessions')).length,1);assert.equal((await all(db,'chunks')).length,chunks.length);
 vm.runInContext(html.slice(html.indexOf('const failedRecordingMetadata='),html.indexOf('// End recording details core.')),c);vm.runInContext(html.slice(html.indexOf('// Versioned backup / restore.'),html.indexOf('// End backup / restore')),c);const data={sessions,chunks,tracks:[],references:[],history:[]};assert.equal(c.validateBackup({format:'blackbox-backup',version:1,data,checksum:c.backupChecksum(data)}).recordings,1);
});

test('metadata endpoint and owned example migration stay SIM without replacing rows or user data',async()=>{
 const c=fixture(),db=await database(),rows=c.window.examplesForTest('rolling');
 const id=await c.window.seedForTest(db,'SIM example',rows,'rolling-v7'),before=(await all(db,'chunks'));
 await new Promise((res,rej)=>{const tx=db.transaction('sessions','readwrite'),store=tx.objectStore('sessions');store.put({sessionId:id,startTime:rows[0].t,status:'complete',demoFixture:'rolling-v7',marker:'keep'});store.put({sessionId:id+1,startTime:1,status:'complete',label:'user data'});tx.oncomplete=res;tx.onerror=()=>rej(tx.error)});
 assert.equal(await c.window.seedForTest(db,'SIM replacement',[], 'rolling-v7'),id);
 const sessions=await all(db,'sessions'),example=sessions.find(s=>s.sessionId===id),user=sessions.find(s=>s.sessionId===id+1);
 assert.equal(sessions.length,2);assert.equal(example.marker,'keep');assert.equal(example.recordingMetadata.snapshot.simulated,true);assert.equal(example.recordingMetadata.snapshot.build.app_version,'SIM demo');assert.equal(example.recordingMetadata.snapshot.calibration.mount_alignment,null);assert.equal(user.recordingMetadata,undefined);assert.deepEqual(await all(db,'chunks'),before);
 const response=await c.window.fetch('/api/recording-metadata');assert.equal(response.headers.get('cache-control'),'no-store');const value=await response.json();assert.equal(value.simulated,true);assert.equal(value.gps.configured_model,'SIMULATED');
});

test('live and seeded metadata reuse the corresponding synthetic GPS boot identity',async()=>{
 const c=fixture(),metadata=await(await c.window.fetch('/api/recording-metadata')).json(),gps=await(await c.window.fetch('/api/gps-log')).json();assert.equal(metadata.boot_id,gps.boot_id);await c.events.get('DOMContentLoaded')();await c.elements.get('demo-interval').onclick({target:{}});const after=await(await c.window.fetch('/api/recording-metadata')).json(),gpsAfter=await(await c.window.fetch('/api/gps-log')).json();assert.notEqual(after.boot_id,metadata.boot_id);assert.equal(after.boot_id,gpsAfter.boot_id);
 for(const kind of ['circuit','rolling']){const db=await database(),rows=c.window.examplesForTest(kind);await c.window.seedForTest(db,'SIM example',rows,kind==='circuit'?'circuit-v6':'rolling-v7');const session=(await all(db,'sessions'))[0];assert.equal(session.recordingMetadata.snapshot.boot_id,rows.find(r=>r.gpsBootId!==undefined).gpsBootId)}
});
test('explicit Drive preview suppresses synthetic detection suggestions without activating a track',async()=>{const c=fixture();let dismissed=0,activated=0;c.location.href='https://example.test/demo.html?view=drive';c.trackAutoDetector={check:async()=>({track:{id:'demo'}})};c.showTrackDetectedToast=()=>{throw Error('Drive preview showed track overlay')};c.dismissActiveToast=()=>dismissed++;c.activateTrack=async()=>activated++;await c.events.get('DOMContentLoaded')();assert.equal(await c.trackAutoDetector.check(),null);c.showTrackDetectedToast();assert.equal(dismissed,1);assert.equal(activated,0)});
