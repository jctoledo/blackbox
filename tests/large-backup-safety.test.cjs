const {test}=require('node:test');
const assert=require('node:assert/strict');
const {context,database,exported}=require('./helpers/large-backup.cjs');
async function setup(){const c=context();c.db=await database();c.trackDb=await database(true);await c.addBackupStores(c.db,{sessions:[{sessionId:100,startTime:100,status:'complete',label:'keep original'}],chunks:Array.from({length:12},(_,i)=>({sessionId:100,chunkIndex:i,data:[{t:100+i,sp:i}]}))});return c}
async function original(c){return JSON.stringify(await c.readBackupStores(c.db,['sessions','chunks']))}
function reloaded(c){const next=context();next.db=c.db;next.trackDb=c.trackDb;return next}
function faultDatabase(database,after,onAdd=()=>{}){return{transaction(...args){const tx=database.transaction(...args),objectStore=tx.objectStore.bind(tx),stores=new Map();let adds=0;tx.objectStore=name=>{if(!stores.has(name)){const store=objectStore(name);stores.set(name,new Proxy(store,{get(target,key){if(key==='add')return(...values)=>{onAdd(name,values[0]);if(++adds===after)throw new DOMException('Injected disk quota failure','QuotaExceededError');return target.add(...values)};const value=Reflect.get(target,key);return typeof value==='function'?value.bind(target):value}}))}return stores.get(name)};return tx}}}
test('late quota failure aborts every added copy and preserves source bytes',async()=>{
 const c=await setup(),before=await original(c),job=await c.stageMultipartBackup(await exported(c)),real=c.db;c.db=faultDatabase(real,8);
 await assert.rejects(c.executeMultipartRestore(job.id),/quota|Quota|rolled back/);c.db=real;assert.equal(await original(c),before);
 await reloaded(c).executeMultipartRestore(job.id);assert.equal((await c.readBackupStores(real,['sessions'])).sessions.length,2);
});
test('a root ID collision after children are queued rolls back all child copies and preserves the foreign row',async()=>{
 const c=await setup(),job=await c.stageMultipartBackup(await exported(c)),promote=c.multipartPromote;let foreignId,childAdds=0;
 const real=c.db;c.db=faultDatabase(real,Infinity,(store,row)=>{if(store==='chunks'&&row.sessionId!==100)childAdds++});
 c.multipartPromote=async(...args)=>{if(args[3]==='recordings'){foreignId=args[1].mapping.sessions[0][1];await c.addBackupStores(c.db,{sessions:[{sessionId:foreignId,startTime:1,status:'complete',label:'foreign data'}]})}return promote(...args)};
 await assert.rejects(c.executeMultipartRestore(job.id),/conflict|rolled back|constraint/i);
 assert.equal(childAdds,12,'all child adds must have happened before the root collision');c.db=real;const rows=await c.readBackupStores(c.db,['sessions','chunks']);assert.equal(rows.sessions.length,2);assert.equal(rows.sessions.find(s=>s.sessionId===foreignId).label,'foreign data');assert.equal(rows.chunks.length,12);assert.ok(rows.chunks.every(chunk=>chunk.sessionId===100));
});
test('lost staging acknowledgement after a committed destination recovers from markers without duplicate copies',async()=>{
 const c=await setup(),job=await c.stageMultipartBackup(await exported(c)),update=c.multipartUpdateJob;let failed=false;
 c.multipartUpdateJob=(stage,id,action)=>update(stage,id,current=>{const value=action(current);if(!failed&&value.groups.recordings==='committed'){failed=true;throw Error('Injected lost commit acknowledgement')}return value});
 await assert.rejects(c.executeMultipartRestore(job.id),/acknowledgement/);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,2);
 const retry=reloaded(c);await retry.executeMultipartRestore(job.id);await retry.executeMultipartRestore(job.id);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,2);assert.equal((await c.readBackupStores(c.db,['chunks'])).chunks.length,24);
});
test('delayed staged reads keep the destination transaction alive and a later source error rolls it all back',async()=>{
 const c=await setup(),before=await original(c),job=await c.stageMultipartBackup(await exported(c)),rows=c.multipartStagedRows,promote=c.multipartPromote,real=c.db;let promoting=false,childAdds=0;
 c.db=faultDatabase(real,Infinity,store=>{if(store==='chunks')childAdds++});
 c.multipartPromote=async(...args)=>{promoting=true;try{return await promote(...args)}finally{promoting=false}};
 c.multipartStagedRows=async function*(stage,id,collection){let seen=0;for await(const entry of rows(stage,id,collection)){if(promoting){await new Promise(resolve=>setTimeout(resolve,2));if(collection==='chunks'&&++seen===8)throw Error('Injected staging read loss')}yield entry}};
 await assert.rejects(c.executeMultipartRestore(job.id),/staging read loss/);assert.equal(childAdds,7,'the destination must add seven rows before the staging read fails');c.db=real;assert.equal(await original(c),before);
 await reloaded(c).executeMultipartRestore(job.id);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,2);
});
test('cancelling file validation leaves the production databases untouched and does not consume the tail',async()=>{
 const c=await setup();c.sessionId=100;c.sessionStart=100;c.chunkIndex=12;c.chunkBuffer=[{t:200,sp:5}];const before=await original(c),tail=JSON.stringify(c.chunkBuffer),controller=new AbortController(),files=await exported(c);
 await assert.rejects(c.stageMultipartBackup(files,{signal:controller.signal,onProgress(){controller.abort()}}),/Cancelled/);assert.equal(await original(c),before);assert.equal(JSON.stringify(c.chunkBuffer),tail);
});
test('UTF-8 and escaped text spanning files stays within the exact byte limit and restores losslessly',async()=>{
 const c=await setup(),note='🏁"\n\\'.repeat(800000);await c.addBackupStores(c.db,{sessions:[{sessionId:200,startTime:200,status:'complete',note}]});const files=await exported(c);let parts=0;
 for(const file of files){const text=await file.text();assert.equal(file.size,Buffer.byteLength(text));assert.ok(file.size<=8*1024*1024);if(JSON.parse(text).kind==='part')parts++}assert.ok(parts>=2,'fixture must span multiple parts');
 const job=await c.stageMultipartBackup(files.reverse()),restored=await c.executeMultipartRestore(job.id),id=restored.mapping.sessions.find(([old])=>old===200)[1];const rows=await c.readBackupStores(c.db,['sessions']);assert.equal(rows.sessions.find(s=>s.sessionId===id).note,note);assert.equal(rows.sessions.find(s=>s.sessionId===200).note,note);
});
test('an oversized logical record is refused without mutating or deleting the original',async()=>{
 const c=await setup();await c.addBackupStores(c.db,{sessions:[{sessionId:200,startTime:200,status:'complete',note:'x'.repeat(16*1024*1024)}]});await assert.rejects(c.prepareMultipartBackup(),/record limit|16 MiB/);const rows=await c.readBackupStores(c.db,['sessions']);assert.equal(rows.sessions.find(s=>s.sessionId===200).note.length,16*1024*1024);assert.equal(rows.sessions.length,2);
});
test('concurrent stale resumes cannot downgrade committed proof and resurrect subsequently removed copies',async()=>{
 const c=await setup(),job=await c.stageMultipartBackup(await exported(c)),other=reloaded(c);let arrivals=0,release;const barrier=new Promise(resolve=>release=resolve);
 for(const tab of [c,other]){const presence=tab.multipartGroupPresence;let first=true;tab.multipartGroupPresence=async(...args)=>{const result=await presence(...args);if(args[2]==='recordings'&&first){first=false;assert.equal(result,'none');if(++arrivals===2)release();await barrier}return result}}
 const results=await Promise.allSettled([c.executeMultipartRestore(job.id),other.executeMultipartRestore(job.id)]);assert.ok(results.some(r=>r.status==='fulfilled'));assert.ok(results.some(r=>r.status==='rejected'));
 const roots=await c.readBackupStores(c.db,['sessions']),copy=roots.sessions.find(s=>s.sessionId!==100);assert.ok(copy);assert.equal(roots.sessions.length,2);
 await new Promise((resolve,reject)=>{const tx=c.db.transaction(['sessions','chunks'],'readwrite');tx.objectStore('sessions').delete(copy.sessionId);const r=tx.objectStore('chunks').index('sessionId').openCursor(copy.sessionId);r.onsuccess=()=>{const cursor=r.result;if(cursor){cursor.delete();cursor.continue()}};tx.oncomplete=resolve;tx.onabort=tx.onerror=()=>reject(tx.error)});
 await assert.rejects(reloaded(c).executeMultipartRestore(job.id),/uncertain|removed|automatic re-add|committed/i);assert.equal((await c.readBackupStores(c.db,['sessions'])).sessions.length,1);assert.equal((await c.readBackupStores(c.db,['chunks'])).chunks.length,12);
});
