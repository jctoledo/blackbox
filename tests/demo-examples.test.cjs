const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const {indexedDB}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function fixture(){const c={console,Response,URL,performance:{now:()=>0},window:{},document:{addEventListener(){}},location:{href:'https://example.test/demo.html'}};vm.createContext(c);vm.runInContext(fs.readFileSync('docs/demo.js','utf8').replace(' window.fetch=',' window.examplesForTest=exampleRows;window.seedForTest=seedExampleRecording; window.fetch='),c);vm.runInContext(html.slice(html.indexOf('class RollingRuns{'),html.indexOf('const rollingRuns='))+';this.Runs=RollingRuns;',c);return c}
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
 vm.runInContext(html.slice(html.indexOf('// Versioned backup / restore.'),html.indexOf('// End backup / restore')),c);const data={sessions,chunks,tracks:[],references:[],history:[]};assert.equal(c.validateBackup({format:'blackbox-backup',version:1,data,checksum:c.backupChecksum(data)}).recordings,1);
});
