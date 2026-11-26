const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const {indexedDB,IDBKeyRange}=require('fake-indexeddb');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function context(db){
 const downloads=[],blobs=[];
 const c={db,IDBKeyRange,Blob,Date,Number,Math,console,chunkWrite:null,chunkBuffer:[],sessionId:null,setTimeout(){},
  URL:{createObjectURL(blob){blobs.push(blob);return 'blob:csv'},revokeObjectURL(){}},
  document:{createElement(){return {click(){downloads.push(this.download)}}}},idbPromise:r=>new Promise((resolve,reject)=>{r.onsuccess=()=>resolve(r.result);r.onerror=()=>reject(r.error)})};
 vm.createContext(c);
 for(const [a,b] of [['function gpsSamplesToCSV(', 'function driveDuration('],['function samplesToCSV(', 'async function exportCSV('],['async function exportSessionById(', 'async function clearAllRecordings('],['class RollingRuns{','const rollingRuns='],['function runsToCSV(', '// End receiver-time rolling intervals.']])vm.runInContext(slice(a,b),c);
 return {c,downloads,blobs};
}
let serial=0;
async function database(chunks=[],sessions=[1]){
 const db=await new Promise((resolve,reject)=>{const r=indexedDB.open('csv-'+ ++serial);r.onerror=()=>reject(r.error);r.onsuccess=()=>resolve(r.result);r.onupgradeneeded=()=>{r.result.createObjectStore('sessions',{keyPath:'sessionId'});r.result.createObjectStore('chunks',{keyPath:['sessionId','chunkIndex']}).createIndex('sessionId','sessionId')}});
 await new Promise((resolve,reject)=>{const tx=db.transaction(['sessions','chunks'],'readwrite');for(const sessionId of sessions)tx.objectStore('sessions').put({sessionId});for(const chunk of chunks)tx.objectStore('chunks').put(chunk);tx.oncomplete=resolve;tx.onerror=()=>reject(tx.error)});
 return db;
}
const chunk=(chunkIndex,data,sessionId=1)=>({sessionId,chunkIndex,data});
function gps(i,sp=18+i*.08,extra={}){return {rowType:1,t:9999,gpsActive:1,gpsAccepted:1,sp,gpsCourse:1,gpsUtcUs:1000000+i*40000,gpsDate:211126,gpsFractionDigits:2,gpsBootId:7,gpsSeq:i+1,gpsMissing:0,gpsSynthetic:0,...extra}}
for(const gpsOnly of [false,true])test('real export preserves mixed CSV bytes, zeros, blanks and numeric chunk order: '+gpsOnly,async()=>{
 const rows=[{t:0,sp:0,ax:null,gpsOk:0,gpsSynthetic:0},gps(0,0),{t:2,sp:NaN,ax:Infinity},{rowType:9,t:3,sp:12,gpsOk:undefined},gps(1,1,{gpsAccepted:undefined,gpsCourse:NaN})],pending=[{t:4,sp:0},gps(2,0)];
 const db=await database([chunk(10,rows.slice(3)),chunk(0,rows.slice(0,2)),chunk(2,rows.slice(2,3)),chunk(0,[{t:999}],2)],[1,2]);
 const {c,downloads,blobs}=context(db);c.sessionId=1;c.chunkBuffer=pending;
 const result=await c.exportSessionById(1,gpsOnly);
 assert.equal(await blobs[0].text(),(gpsOnly?c.gpsSamplesToCSV:c.samplesToCSV)(rows.concat(pending)));
 assert.equal(result.sampleCount,7);assert.equal(result.session.sessionId,1);assert.equal(c.chunkBuffer,pending);
 assert.deepEqual(downloads,[(gpsOnly?'blackbox_gps_':'blackbox_')+'1970-01-01-00-00-00.csv']);db.close();
});
test('writer settles before capture and only current owner tail is included once',async()=>{
 const db=await database([chunk(0,[{t:1}])],[1,2]),{c,blobs}=context(db);c.sessionId=1;c.chunkBuffer=[{t:2}];
 c.chunkWrite=Promise.resolve().then(()=>{c.chunkBuffer=[{t:3}]});
 assert.equal((await c.exportSessionById(1)).sampleCount,2);assert.equal(await blobs[0].text(),c.samplesToCSV([{t:1},{t:3}]));
 c.sessionId=2;c.chunkBuffer=[{t:999}];await c.exportSessionById(1);assert.equal(await blobs[1].text(),c.samplesToCSV([{t:1}]));db.close();
});
test('pending-only after failed writer exports without saving or consuming tail',async()=>{
 const db=await database(),{c,blobs}=context(db);c.sessionId=1;c.chunkBuffer=[{t:0},gps(0)];c.console={warn(){}};c.chunkWrite=Promise.reject(Error('quota'));
 assert.equal((await c.exportSessionById(1,true)).sampleCount,2);assert.equal(await blobs[0].text(),c.gpsSamplesToCSV(c.chunkBuffer));assert.equal(c.chunkBuffer.length,2);db.close();
});
test('many chunks prohibit raw getAll and serialize at most one chunk per call',async()=>{
 const chunks=Array.from({length:1200},(_,i)=>chunk(i,[{t:i*2,sp:0},{t:i*2+1,sp:1}])),db=await database(chunks),{c,blobs}=context(db),original=db.transaction.bind(db);
 let visits=0,calls=0,maxRows=0;
 db.transaction=(...args)=>{const tx=original(...args),store=tx.objectStore.bind(tx);tx.objectStore=name=>{const s=store(name);s.getAll=()=>{throw Error('whole-store getAll forbidden')};const index=s.index.bind(s);s.index=(...args)=>{const idx=index(...args);idx.getAll=()=>{throw Error('whole-session getAll forbidden')};return idx};if(name==='chunks'){const cursor=s.openCursor.bind(s);s.openCursor=(...args)=>{const r=cursor(...args);r.addEventListener('success',()=>{if(r.result)visits++});return r}}return s};return tx};
 const serialize=c.samplesToCSV;c.samplesToCSV=(rows,...args)=>{calls++;maxRows=Math.max(maxRows,rows.length);return serialize(rows,...args)};
 assert.equal((await c.exportSessionById(1)).sampleCount,2400);assert.equal(visits,1200);assert.equal(calls,1201);assert.equal(maxRows,2);
 assert.equal(await blobs[0].text(),serialize(chunks.flatMap(c=>c.data)));db.close();
});
for(const mode of ['abort','late abort','serialization','read'])test(mode+' failure never downloads a partial CSV',async()=>{
 const db=await database([chunk(0,[{t:0}]),chunk(1,[{t:1}])]),{c,downloads,blobs}=context(db),original=db.transaction.bind(db);let completed=false;
 db.transaction=(...args)=>{const tx=original(...args);tx.addEventListener('complete',()=>{completed=true});if(mode==='abort')queueMicrotask(()=>tx.abort());if(mode==='late abort'){const store=tx.objectStore.bind(tx);tx.objectStore=name=>{const s=store(name);if(name==='chunks'){const open=s.openCursor.bind(s);s.openCursor=(...args)=>{const r=open(...args);r.addEventListener('success',()=>{if(!r.result)tx.abort()});return r}}return s}}return tx};
 if(mode==='serialization'){const serialize=c.samplesToCSV;c.samplesToCSV=(rows,...args)=>{if(rows.some(r=>r.t===1))throw Error('serialization failed');return serialize(rows,...args)}}
 if(mode==='read'){db.transaction=(...args)=>{const tx=original(...args),store=tx.objectStore.bind(tx);tx.objectStore=name=>{const s=store(name);if(name==='chunks')s.openCursor=()=>{throw Error('read failed')};return s};return tx}}
 await assert.rejects(c.exportSessionById(1));assert.equal(downloads.length,0);assert.equal(blobs.length,0);assert.equal(completed,false);db.close();
});
test('snapshot excludes writes queued while chunks are being read and download waits for completion',async()=>{
 const db=await database([chunk(0,[{t:1}]),chunk(1,[{t:2}])]),{c,blobs}=context(db),serialize=c.samplesToCSV;let write,complete=false;
 const original=db.transaction.bind(db);db.transaction=(...args)=>{const tx=original(...args);if(args[1]==='readonly')tx.addEventListener('complete',()=>{complete=true});return tx};
 c.samplesToCSV=(rows,...args)=>{if(rows.length&&!write){write=new Promise((resolve,reject)=>{const tx=db.transaction('chunks','readwrite');tx.objectStore('chunks').put(chunk(2,[{t:3}]));tx.oncomplete=resolve;tx.onerror=()=>reject(tx.error)})}return serialize(rows,...args)};
 const result=await c.exportSessionById(1);assert.equal(complete,true);assert.equal(result.sampleCount,2);assert.equal(await blobs[0].text(),serialize([{t:1},{t:2}]));await write;db.close();
});
for(const [chunks,gpsOnly,message] of [[[],false,/No data/],[[chunk(0,[])],false,/No dashboard/],[[chunk(0,[gps(0)])],false,/No dashboard/],[[chunk(0,[{t:1}])],true,/No GPS timestamp/]])test('preserves no-matching-data error '+message,async()=>{
 const db=await database(chunks),{c,downloads}=context(db);await assert.rejects(c.exportSessionById(1,gpsOnly),message);assert.equal(downloads.length,0);db.close();
});
test('missing session refuses pending data',async()=>{const db=await database(),{c,downloads}=context(db);c.sessionId=2;c.chunkBuffer=[{t:1}];await assert.rejects(c.exportSessionById(2),/Session not found/);assert.equal(downloads.length,0);db.close()});
test('Runs preserves detector bytes across chunks, pending, mixed rows and quality gates',async()=>{
 const rows=Array.from({length:400},(_,i)=>gps(i,10+i*.08,{gpsSynthetic:i===0?1:0}));rows.splice(180,0,{t:5,sp:100});
 for(const mutate of [()=>{},r=>r[100].gpsAccepted=0,r=>r[100].gpsUtcUs=1,r=>r[100].gpsCourse=2]){
  const data=structuredClone(rows);mutate(data);const db=await database([chunk(1,data.slice(100,350)),chunk(0,data.slice(0,100))]),{c,blobs}=context(db);c.sessionId=1;c.chunkBuffer=data.slice(350);
  assert.equal((await c.exportSessionById(1,true,true)).sampleCount,data.length);assert.equal(await blobs[0].text(),c.runsToCSV(data));db.close();
 }
});
test('Runs refuses no attempts and more than 1000 attempts without downloading',async()=>{
 for(const rows of [[gps(0)],Array.from({length:1001*220},(_,i)=>gps(i,18+(i%220)*.08))]){
  const chunks=Array.from({length:Math.ceil(rows.length/220)},(_,i)=>chunk(i,rows.slice(i*220,(i+1)*220))),db=await database(chunks),{c,downloads}=context(db);
  assert.throws(()=>c.runsToCSV(rows),rows.length===1?/No performance attempts/:/More than 1000 attempts/);
  await assert.rejects(c.exportSessionById(1,true,true),rows.length===1?/No performance attempts/:/More than 1000 attempts/);assert.equal(downloads.length,0);db.close();
 }
});
