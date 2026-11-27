const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const {indexedDB}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function fixture(){const events=new Map(),elements=new Map();const c={console:{...console,error(){}},Response,URL,btoa,DataView,Uint8Array,performance:{now:()=>0},window:{},document:{addEventListener(name,fn){events.set(name,fn)},getElementById(id){if(!elements.has(id))elements.set(id,{});return elements.get(id)}},location:{href:'https://example.test/demo.html'}};c.events=events;c.elements=elements;vm.createContext(c);vm.runInContext(html.slice(html.indexOf('// Drive session core.'),html.indexOf('// End Drive session core.')),c);vm.runInContext(fs.readFileSync('docs/demo.js','utf8').replace(' window.fetch=',' window.examplesForTest=exampleRows;window.routeForTest=demoRouteData;window.seedForTest=seedExampleRecording;window.populateForTest=populateExamples; window.fetch='),c);vm.runInContext(html.slice(html.indexOf('class RollingRuns{'),html.indexOf('const rollingRuns='))+';this.Runs=RollingRuns;',c);return c}
test('demo examples populate both cornering signs and produce auditable accepted/rejected runs',()=>{
 const c=fixture(),circuit=c.window.examplesForTest('circuit'),rolling=c.window.examplesForTest('rolling');
 for(const rows of [circuit,rolling])for(const row of rows){assert.equal(row.gpsSynthetic,1);for(const value of Object.values(row))assert.ok(Number.isFinite(value))}
 const forces=circuit.filter(r=>r.rowType!==1);assert.ok(Math.max(...forces.map(r=>r.latg))>.3);assert.ok(Math.min(...forces.map(r=>r.latg))<-.3);assert.ok(Math.max(...forces.map(r=>r.lng))>.15);assert.ok(Math.min(...forces.map(r=>r.lng))<-.3);
 vm.runInContext(html.slice(html.indexOf('function samplesToCSV('),html.indexOf('async function exportCSV(')),c);const csv=c.samplesToCSV(rolling).trim().split('\n');assert.equal(csv[0].split(',').at(-1),'synthetic');assert.equal(csv[1].split(',').at(-1),'1');assert.equal(Number(csv[1].split(',')[1]),36);
 const detector=new c.Runs();for(const row of rolling)detector.update(row);const accepted=detector.results.filter(r=>r.status==='accepted'),rejected=detector.results.filter(r=>r.status==='rejected');assert.equal(accepted.length,2);assert.equal(rejected.length,1);assert.equal(rejected[0].reason,'GPS gap or clock boundary');for(const r of accepted)assert.ok(Math.abs(r.seconds-5.588)<1e-8);assert.equal(rolling.reduce((sum,r)=>sum+(r.gpsMissing||0),0),6);
});
let serial=0;
test('the SIM winding map uses the recording clock and separates sustained acceleration from braking',()=>{
 const c=fixture(),rows=c.window.examplesForTest('winding');
 const {points,sections,events}=c.window.routeForTest(rows);
 const samples=rows.filter(row=>row.rowType!==1);
 assert.equal(points.length,samples.length);
 assert.ok(points.every((point,i)=>point.timeMs===samples[i].deviceTimeMs&&point.x===samples[i].ekfX&&point.y===samples[i].ekfY));
 assert.equal(sections.reduce((edges,section)=>edges+section.points.length-1,0),points.length-1);
 for(const {kind,points:segment} of sections){
  for(let i=1;i<segment.length;i++){
   const force=(segment[i-1].longitudinalG+segment[i].longitudinalG)/2;
   assert.equal(kind,force>=.10?'accel':force<=-.10?'brake':'neutral');
  }
 }
 for(const [name,measure] of [['bend',point=>Math.abs(point.lateralG)],['brake',point=>-point.longitudinalG],['accel',point=>point.longitudinalG]]){
  const peak=points.find(point=>point.timeMs===events[name]);
  assert(measure(peak)>.1);
  assert(points.every(point=>measure(point)<=measure(peak)));
 }
});

test('the synthetic winding road is open and force directions follow its motion',()=>{
 const c=fixture(),samples=c.window.examplesForTest('winding').filter(row=>row.rowType!==1);
 const first=samples[0],last=samples.at(-1);
 assert(Math.hypot(last.ekfX-first.ekfX,last.ekfY-first.ekfY)>1000);
 assert(samples.every(row=>row.gpsSynthetic===1&&row.gpsBootId===104));
 let integratedSpeed=first.sp/3.6,matchedTurns=0,turns=0;
 for(let i=1;i<samples.length;i++){
  const a=samples[i-1],b=samples[i],dt=(b.deviceTimeMs-a.deviceTimeMs)/1000;
  integratedSpeed+=(a.lng+b.lng)*.5*dt*9.81;
  if(i<6||i+6>=samples.length||Math.abs(b.latg)<.15)continue;
  const p=samples[i-6],q=samples[i+6];
  const cross=(b.ekfX-p.ekfX)*(q.ekfY-b.ekfY)-(b.ekfY-p.ekfY)*(q.ekfX-b.ekfX);
  turns++;
  if(Math.sign(cross)===-Math.sign(b.latg))matchedTurns++;
 }
 assert(Math.abs(integratedSpeed-last.sp/3.6)<.5);
 assert(turns>100&&matchedTurns/turns>.95);
 assert(Math.max(...samples.map(row=>row.latg))>.2);
 assert(Math.min(...samples.map(row=>row.latg))<-.2);
});

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
test('unchanged rolling fixture yields auditable accepted and gap-rejected examples for the six rolling presets within its speed range',()=>{
 const c=fixture(),rows=c.window.examplesForTest('rolling'),intervals=vm.runInContext('runIntervals.filter(i=>!i.kind&&i.hi<=34)',c);assert.equal(intervals.length,6);
 for(const interval of intervals){const detector=new c.Runs(interval);for(const row of rows)detector.update(row);const accepted=detector.results.filter(r=>r.status==='accepted'),rejected=detector.results.filter(r=>r.status==='rejected');assert.equal(accepted.length,2,interval.label);assert.equal(rejected.length,1,interval.label);assert.equal(rejected[0].reason,'GPS gap or clock boundary');
  for(const result of accepted){assert.ok(Math.abs(result.seconds-(interval.hi-interval.lo)/2)<1e-8);assert.equal(result.interval,interval.label);assert.equal(result.synthetic,1);assert.equal(result.boot,102);assert.ok(rows.some(row=>row.rowType===1&&row.gpsSeq===result.startSeq));assert.ok(rows.some(row=>row.rowType===1&&row.gpsSeq===result.endSeq))}
 }
 vm.runInContext(html.slice(html.indexOf('function runsToCSV('),html.indexOf('// End receiver-time rolling intervals.')),c);const csv=c.runsToCSV(rows);for(const interval of intervals){assert.ok(csv.includes(interval.label+',accepted,'));assert.ok(csv.includes(interval.label+',rejected,'))}
});

function savedSummary(c,rows){vm.runInContext(html.slice(html.indexOf('function forcesUsable('),html.indexOf('function clearForces('))+html.slice(html.indexOf('class EffortPeaks{'),html.indexOf('const effortPeaks='))+';this.Summary=RecordingSummary;',c);const summary=new c.Summary();for(const row of rows)summary.update(row);return summary.snapshot()}
test('actual seeded drives qualify saved force windows with known boot identity and useful signs',()=>{
 for(const kind of ['circuit','rolling']){const c=fixture(),rows=c.window.examplesForTest(kind),summary=savedSummary(c,rows);assert.ok(summary.samples.forceWindows>0,kind);assert.equal(summary.excluded.clock,0);assert.equal(summary.provenance.unknown,0);assert.equal(summary.provenance.synthetic,summary.samples.telemetry);assert.ok(summary.forcePeaksG.accel>.15);assert.ok(summary.forcePeaksG.brake>.15);for(const row of rows)assert.equal(row.gpsBootId,kind==='circuit'?101:102);
  if(kind==='circuit'){assert.ok(summary.forcePeaksG.left>.3);assert.ok(summary.forcePeaksG.right>.3)}
  else{assert.ok(Math.abs(summary.forcePeaksG.accel-2/9.81)<1e-12);assert.ok(Math.abs(summary.forcePeaksG.brake-2/9.81)<1e-12);assert.equal(summary.forcePeaksG.left,0);assert.equal(summary.forcePeaksG.right,0)}
 }
});
test('new complete-evidence demo fixtures preserve old examples and user recordings',async()=>{
 const c=fixture(),db=await database(),oldRows=c.window.examplesForTest('rolling').map(row=>{const old={...row};if(old.rowType!==1)delete old.gpsBootId;return old}),oldId=await c.window.seedForTest(db,'Old SIM example',oldRows,'rolling-v7');
 const userId=oldId+1;await new Promise((resolve,reject)=>{const tx=db.transaction(['sessions','chunks'],'readwrite');tx.objectStore('sessions').put({sessionId:userId,startTime:123,status:'complete',label:'My private drive'});tx.objectStore('chunks').put({sessionId:userId,chunkIndex:0,data:[{t:123,sp:1}]});tx.oncomplete=resolve;tx.onerror=()=>reject(tx.error)});
 const beforeSessions=await all(db,'sessions'),beforeChunks=await all(db,'chunks');c.openDB=async()=>db;c.updatePerformanceRuns=()=>{};c.effortPeaks={update(){}};c.sustainedEffortUsable=()=>true;
 await c.window.populateForTest({id:'track'});await c.window.populateForTest({id:'track'});
 const sessions=await all(db,'sessions'),chunks=await all(db,'chunks');assert.equal(sessions.length,6);assert.ok(sessions.some(s=>s.demoFixture==='winding-v1'));assert.ok(sessions.some(s=>s.demoFixture==='circuit-v7'));assert.ok(sessions.some(s=>s.demoFixture==='rolling-v8'));assert.ok(sessions.some(s=>s.demoFixture==='launch-v1'));assert.deepEqual(sessions.filter(s=>s.sessionId===oldId||s.sessionId===userId),beforeSessions);assert.deepEqual(chunks.filter(s=>s.sessionId===oldId||s.sessionId===userId),beforeChunks);
 for(const session of sessions.filter(s=>['circuit-v7','rolling-v8','launch-v1','winding-v1'].includes(s.demoFixture))){assert.ok(session.label.startsWith('SIM'));const rows=chunks.filter(chunk=>chunk.sessionId===session.sessionId).sort((a,b)=>a.chunkIndex-b.chunkIndex).flatMap(chunk=>chunk.data);assert.ok(rows.filter(row=>row.rowType!==1).every(row=>Number.isInteger(row.gpsBootId)))}
});

test('live demo boot identity reaches actual stored telemetry rows and saved force windows across scenarios',async()=>{
 const c=fixture();let now=0;c.performance.now=()=>now;c.chunkBuffer=[];c.fusion={};
 const parse=html.slice(html.indexOf('function process(buf'),html.indexOf('    // Track GPS reference -'));
 const push=html.match(/chunkBuffer\.push\(\{t:now,gpsSynthetic:[^\n]+/)[0];
 vm.runInContext(html.slice(html.indexOf('function forcesUsable('),html.indexOf('function clearForces('))+html.slice(html.indexOf('class EffortPeaks{'),html.indexOf('const effortPeaks='))+';this.Summary=RecordingSummary;'+parse+'const now=Date.now();'+push+'return chunkBuffer.pop();};this.storedRowForTest=process;',c);
 let previousBoot;
 for(const scenario of ['circuit','rolling']){
  if(scenario==='rolling'){await c.events.get('DOMContentLoaded')();await c.elements.get('demo-interval').onclick({target:{}})}
  const metadata=await(await c.window.fetch('/api/recording-metadata')).json(),gps=await(await c.window.fetch('/api/gps-log')).json(),summary=new c.Summary();assert.equal(metadata.boot_id,gps.boot_id);if(previousBoot!==undefined)assert.notEqual(metadata.boot_id,previousBoot);previousBoot=metadata.boot_id;
  for(let i=0;i<20;i++){const telemetry=await(await c.window.fetch('/api/telemetry')).json(),bytes=Uint8Array.from(Buffer.from(telemetry.data,'base64')),row=c.storedRowForTest(bytes.buffer,telemetry.timing);assert.equal(telemetry.timing.gps_boot_id,gps.boot_id);assert.equal(row.gpsBootId,gps.boot_id);assert.equal(row.gpsSynthetic,1);summary.update(row);now+=40}
  assert.ok(summary.snapshot().samples.forceWindows>0,scenario);assert.equal(summary.snapshot().excluded.clock,0);
 }
});


test('standing example reconstructs launches and quarter-mile estimates with explicit GPS-gap rejection',()=>{
 const c=fixture(),rows=c.window.examplesForTest('launch');
 for(const interval of vm.runInContext('runIntervals.filter(i=>i.kind)',c)){
  const d=vm.runInContext('createRunDetector',c)(interval);for(const row of rows)d.update(row);
  const accepted=d.results.filter(r=>r.status==='accepted');assert.equal(accepted.length,2,interval.label);assert.ok(d.results.some(r=>r.status==='rejected'&&r.reason==='GPS gap or clock boundary'));assert.ok(accepted.every(r=>r.synthetic===1&&r.startConvention==='stationary-fix-v1'&&r.rolloutM===0));
 }
 for(const interval of vm.runInContext('runIntervals.filter(i=>!i.kind)',c)){const d=new c.Runs(interval);for(const row of rows)d.update(row);assert.ok(d.results.some(r=>r.status==='accepted'),interval.label)}
});
