#!/usr/bin/env node
// Run only on lat. fake-indexeddb retains source and restored data in JS memory:
// RSS includes those databases and is NOT the browser module's working-set size.
const fs=require('node:fs/promises');
const path=require('node:path');
const os=require('node:os');
const assert=require('node:assert/strict');
const crypto=require('node:crypto');
const {context,database}=require('../tests/helpers/large-backup.cjs');
const fixture=require('./large_backup_fixture.cjs');
const minutes=Number(process.argv[2]||20);
if(![20,60,120].includes(minutes))throw Error('Choose 20, 60 or 120 fixture minutes');
const measure={fixtureMinutes:minutes,synthetic:true,rates:{telemetry:30,gps:25},memoryNote:'Node fake-indexeddb retains source, staging and destination. Sampled process memory is not isolated export overhead or phone capacity.',phases:[]};
let rssPeak=0,heapPeak=0;
function sample(){const m=process.memoryUsage();rssPeak=Math.max(rssPeak,m.rss);heapPeak=Math.max(heapPeak,m.heapUsed);return m}
async function phase(name,action){const start=performance.now();sample();const result=await action();measure.phases.push({name,seconds:(performance.now()-start)/1000,memory:sample()});return result}
function walk(database,id,consume){return new Promise((resolve,reject)=>{const tx=database.transaction('chunks','readonly'),r=tx.objectStore('chunks').index('sessionId').openCursor(id);tx.oncomplete=resolve;tx.onabort=tx.onerror=()=>reject(tx.error||Error('Cursor failed'));r.onsuccess=()=>{const cursor=r.result;if(!cursor)return;try{consume(cursor.value);sample();cursor.continue()}catch(error){tx.abort();reject(error)}}})}
async function digest(database,id){const hash=crypto.createHash('sha256');let count=0,rows=0;await walk(database,id,chunk=>{hash.update(JSON.stringify({chunkIndex:chunk.chunkIndex,data:chunk.data}));count++;rows+=chunk.data.length});return{sha256:hash.digest('hex'),chunks:count,rows}}
(async()=>{
 const directory=await fs.mkdtemp(path.join(os.tmpdir(),'blackbox-backup-measure-'));
 const c=context();c.db=await database();c.trackDb=await database(true);
 const drive=new c.DriveSession(fixture.START);drive.apply({owner:fixture.START,type:'start',mono:0,wall:fixture.START});
 let sourceBytes=0,rowCount=0,chunkCount=0;
 try{
  await phase('populate-synthetic-source',async()=>{
   let first=true;
   for(const chunk of fixture.chunks(minutes)){
    const rows=chunk.data.filter(row=>row.rowType===1);
    if(first){drive.apply({owner:fixture.START,type:'gps',rows:rows.slice(0,25)});drive.apply({owner:fixture.START,type:'gps',rows:rows.slice(25)});first=false}else drive.apply({owner:fixture.START,type:'gps',rows});
    sourceBytes+=Buffer.byteLength(JSON.stringify(chunk));rowCount+=chunk.data.length;chunkCount++;
    await c.addBackupStores(c.db,{chunks:[chunk]});sample();
   }
   drive.apply({owner:fixture.START,type:'stop',mono:minutes*60000,wall:fixture.START+minutes*60000});
   await c.addBackupStores(c.db,{sessions:[{sessionId:fixture.START,startTime:fixture.START,status:'complete',label:'Synthetic '+minutes+' minute benchmark',drive:drive.snapshot()}]});
  });
  const before=await digest(c.db,fixture.START),files=[];let outputBytes=0,maxFileBytes=0;
  await phase('prepare-and-export-to-disk',async()=>{
   const job=await c.prepareMultipartBackup('all');let part;
   while((part=await job.next())){
    assert.equal(part.bytes,Buffer.byteLength(part.text));assert.ok(part.bytes<=8*1024*1024);
    const target=path.join(directory,part.name);await fs.writeFile(target,part.text);const size=(await fs.stat(target)).size;
    outputBytes+=size;maxFileBytes=Math.max(maxFileBytes,size);files.push({name:part.name,size,text:()=>fs.readFile(target,'utf8'),arrayBuffer:async()=>{const bytes=await fs.readFile(target);return bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)}});sample();
   }
   assert.ok(files.length>1);
  });
  const staged=await phase('read-disk-files-and-validate',()=>c.stageMultipartBackup(files.slice().reverse()));
  const recovered=context();recovered.db=c.db;recovered.trackDb=c.trackDb;
  const restored=await phase('restore-add-only-copies-after-context-reload',()=>recovered.executeMultipartRestore(staged.id));
  const restoredId=restored.mapping.sessions.find(([old])=>old===fixture.START)[1];
  const after=await phase('compare-original-and-restored-cursors',async()=>{
   assert.deepEqual(await digest(c.db,fixture.START),before,'original changed');const result=await digest(c.db,restoredId);assert.deepEqual(result,before,'restored recording differs');
   const roots=await c.readBackupStores(c.db,['sessions']);assert.equal(roots.sessions.length,2);const copy=roots.sessions.find(s=>s.sessionId===restoredId);assert.equal(copy.status,'complete');assert.equal(JSON.stringify(copy.drive),JSON.stringify(drive.snapshot()));
   const replay=new c.DriveReplay(copy.drive);await walk(c.db,restoredId,chunk=>replay.update(chunk.data));replay.finish();return result;
  });
  await phase('retry-without-duplicates',async()=>{const reloaded=context();reloaded.db=c.db;reloaded.trackDb=c.trackDb;await reloaded.executeMultipartRestore(staged.id);const roots=await c.readBackupStores(c.db,['sessions']);assert.equal(roots.sessions.length,2)});
  await c.discardMultipartJob(staged.id);
  Object.assign(measure,{sourceBytes,sourceMiB:sourceBytes/1048576,rows:rowCount,chunks:chunkCount,outputBytes,files:files.length,maxFileBytes,recordingSha256:after.sha256,roundTrip:'passed',originalPreserved:true,retryDuplicated:false,sampledPeakRss:rssPeak,sampledPeakHeapUsed:heapPeak});
  console.log(JSON.stringify(measure,null,2));
 }finally{c.db.close();c.trackDb.close();await fs.rm(directory,{recursive:true,force:true})}
})().catch(error=>{console.error(error);process.exitCode=1});
