const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const deferred=()=>{let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no});return{promise,resolve,reject}};
const settle=async()=>{for(let i=0;i<20;i++)await Promise.resolve()};
const track=id=>({id,name:'Track '+id,type:'loop',gpsOrigin:{lat:37,lon:-122},startLine:{p1:[0,0],p2:[0,20],direction:0},lapCount:0});
function fixture(){
 const requests=[],lookups=[],alerts=[],elements=new Map(),published=[],deletions=[],stored=new Map(['original','A','B'].map(id=>[id,track(id)]));let changes=0;
 const c={console,Promise,Math,referencePersistenceError:'',updateRecUI(){},activeTrack:track('original'),referenceLap:{id:'ref-original'},expectedLapRevision:9,lapResultGeneration:0,lapTracker:{reset(){}},lastDeltaMs:0,deltaTrend:0,prevLapFlags:0,prevLapTimeMs:0,prevLapCnt:0,lapTimerActive:true,p2pNeedsWarmup:false,suppressStartLineIndicator:false,
  trackAutoDetector:{beginTrackChange(){changes++},endTrackChange(){changes--}},
  requestDevice(url){const pending=deferred();requests.push({url,...pending});return pending.promise},
  getTrack:async id=>stored.get(id),
  getRefLap(id){const pending=deferred();lookups.push({id,...pending});return pending.promise},notifyUser:message=>alerts.push(message),startNewSession(){},clearSession(){},fmtLapTime:String,updateDeltaBar(){},
  trackDb:{},IDBKeyRange:{only:id=>id},commitTrackWrite(names,write){write({objectStore:name=>({delete(id){deletions.push([name,id]);if(name==='tracks')stored.delete(id)},clear(){deletions.push([name,'*']);if(name==='tracks')stored.clear()},index(){return{openCursor:()=>({result:null})}}})});return Promise.resolve()},
  updateActiveTrackDisplay(){published.push({id:c.activeTrack?.id??null,ref:c.referenceLap?.id??null,revision:c.expectedLapRevision})},
  $:id=>{if(!elements.has(id)){const values=new Set();elements.set(id,{textContent:'Original label',hidden:false,classList:{add(...keys){keys.forEach(k=>values.add(k))},remove(...keys){keys.forEach(k=>values.delete(k))},toggle(key,on){if(on===undefined)on=!values.has(key);on?values.add(key):values.delete(key);return on}}})}return elements.get(id)}};
 vm.createContext(c);
 const marker=html.includes('// Track command ownership.')?'// Track command ownership.':'async function activateTrack(';
 vm.runInContext(html.slice(html.indexOf(marker),html.indexOf('// Track Recording Functions'))+html.slice(html.indexOf('async function deleteTrackWithRef('),html.indexOf('function formatBytes(')),c);
 const ack=(index,revision=10,ok=true,error=null)=>requests[index].resolve({ok,status:ok?200:500,json:async()=>({revision,...(error?{error}:{})})});
 return{c,requests,lookups,alerts,elements,published,deletions,ack,changes:()=>changes};
}
test('rapid track choices serialize configuration and publish matching reference and labels',async()=>{
 const f=fixture(),a=f.c.activateTrack(track('A')),b=f.c.activateTrack(track('B'));await settle();
 assert.equal(f.requests.length,1,'B must not configure while A owns the command');f.ack(0,10);await settle();
 assert.equal(f.c.activeTrack.id,'original','reference lookup must not expose a half-published owner');assert.equal(f.c.expectedLapRevision,9);assert.equal(f.requests.length,1);
 f.lookups[0].resolve({id:'ref-A',lapTimeMs:1000});await a;await settle();assert.equal(f.requests.length,2);assert.equal(f.c.activeTrack.id,'A');
 f.ack(1,11);await settle();f.lookups[1].resolve({id:'ref-B',lapTimeMs:2000});await b;
 assert.equal(f.c.activeTrack.id,'B');assert.equal(f.c.referenceLap.id,'ref-B');assert.equal(f.elements.get('lap-track-name').textContent,'Track B');assert.equal(f.c.expectedLapRevision,11);assert.equal(f.changes(),0);
 assert.deepEqual(f.published.map(p=>[p.id,p.ref,p.revision]),[['A','ref-A',10],['B','ref-B',11]]);
});

test('a stop queued behind a replacement selection rechecks its confirmation owner',async()=>{
 const f=fixture(),previous=f.c.activeTrack,arm=f.c.activateSavedTrack('B');await settle();
 const stop=f.c.deactivateTrack(()=>f.c.activeTrack===previous);
 f.ack(0,10);await settle();f.lookups[0].resolve(null);await arm;
 assert.equal(await stop,false);assert.equal(f.requests.length,1);assert.equal(f.c.activeTrack.id,'B');
});
test('Stop queued during reference lookup cannot be revived by older activation',async()=>{
 const f=fixture(),a=f.c.activateTrack(track('A'));await settle();f.ack(0,10);await settle();const stop=f.c.deactivateTrack();await settle();assert.equal(f.requests.length,1);
 f.lookups[0].resolve({id:'ref-A'});await a;await settle();assert.match(f.requests[1].url,/type=clear/);f.ack(1,11);await stop;
 assert.equal(f.c.activeTrack,null);assert.equal(f.c.referenceLap,null);assert.equal(f.c.lapTimerActive,false);assert.equal(f.c.expectedLapRevision,11);assert.equal(f.changes(),0);
});
test('configuration rejection preserves owner and does not block the next selection',async()=>{
 const f=fixture(),a=f.c.activateTrack(track('A')),b=f.c.activateTrack(track('B'));await settle();f.requests[0].reject(Error('Connection lost'));await a;await settle();
 assert.equal(f.c.activeTrack.id,'original');assert.equal(f.c.referenceLap.id,'ref-original');assert.equal(f.c.expectedLapRevision,9);assert.match(f.alerts[0],/Connection lost/);assert.equal(f.requests.length,2);
 f.ack(1,12);await settle();f.lookups[0].reject(Error('Storage unavailable'));await b;
 assert.equal(f.c.activeTrack.id,'B');assert.equal(f.c.referenceLap,null);assert.equal(f.elements.get('lap-track-name').textContent,'Track B');assert.equal(f.changes(),0);
});
test('failed Stop preserves active owner and allows a subsequent activation',async()=>{
 const f=fixture(),stop=f.c.deactivateTrack(),a=f.c.activateTrack(track('A'));await settle();f.ack(0,10,false);assert.equal(await stop,false);await settle();
 assert.equal(f.c.activeTrack.id,'original');assert.equal(f.c.referenceLap.id,'ref-original');assert.equal(f.c.expectedLapRevision,9);assert.match(f.alerts[0],/Could not stop/);assert.equal(f.requests.length,2);
 f.ack(1,11);await settle();f.lookups[0].resolve(null);await a;assert.equal(f.c.activeTrack.id,'A');assert.equal(f.c.referenceLap,null);assert.equal(f.changes(),0);
});

test('saved-track lookup preserves the order of driver selections before configuration',async()=>{
 const f=fixture(),reads=[];f.c.getTrack=id=>{const pending=deferred();reads.push({id,...pending});return pending.promise};
 const a=f.c.activateSavedTrack('A'),b=f.c.activateSavedTrack('B');await settle();assert.equal(reads.length,1);assert.equal(reads[0].id,'A');assert.equal(f.requests.length,0);
 reads[0].resolve(track('A'));await settle();f.ack(0,10);await settle();f.lookups[0].resolve({id:'ref-A'});await a;await settle();assert.equal(reads.length,2);assert.equal(reads[1].id,'B');
 reads[1].resolve(track('B'));await settle();f.ack(1,11);await settle();f.lookups[1].resolve({id:'ref-B'});await b;
 assert.equal(f.c.activeTrack.id,'B');assert.equal(f.c.referenceLap.id,'ref-B');assert.equal(f.elements.get('lap-track-name').textContent,'Track B');assert.equal(f.changes(),0);
});

test('deleting a pending selection waits for its publication and stops that exact owner',async()=>{
 const f=fixture(),arm=f.c.activateTrack(track('B'));await settle();f.ack(0,10);await settle();const remove=f.c.deleteTrackWithRef('B');await settle();
 assert.equal(f.deletions.length,0,'pending B must not be deleted while it can still publish');f.lookups[0].resolve(null);await arm;await settle();assert.match(f.requests[1].url,/type=clear/);f.ack(1,11);await remove;
 assert.equal(f.c.activeTrack,null);assert.deepEqual(f.deletions,[['tracks','B']]);assert.equal(f.changes(),0);
});
test('clear all queued behind the first activation stops timing before removing tracks',async()=>{
 const f=fixture();f.c.activeTrack=null;const arm=f.c.activateTrack(track('B'));await settle();f.ack(0,10);await settle();const clear=f.c.clearAllTracks();await settle();assert.equal(f.deletions.length,0);
 f.lookups[0].resolve(null);await arm;await settle();assert.match(f.requests[1].url,/type=clear/);f.ack(1,11);await clear;
 assert.equal(f.c.activeTrack,null);assert.deepEqual(f.deletions,[['tracks','*'],['reference_laps','*'],['lap_history','*']]);assert.equal(f.changes(),0);
});
test('deleting the old track after selecting B does not stop B',async()=>{
 const f=fixture(),arm=f.c.activateTrack(track('B'));await settle();f.ack(0,10);await settle();const remove=f.c.deleteTrackWithRef('original');await settle();f.lookups[0].resolve(null);await arm;await settle();
 assert.equal(f.requests.length,1,'deletion must evaluate the current owner inside its command');await remove;assert.equal(f.c.activeTrack.id,'B');assert.deepEqual(f.deletions,[['tracks','original']]);assert.equal(f.changes(),0);
});
for(const operation of ['delete','clear'])test('failed timing stop retains tracks during '+operation+' and leaves queue usable',async()=>{
 const f=fixture(),removal=operation==='delete'?f.c.deleteTrackWithRef('original'):f.c.clearAllTracks(),rejected=assert.rejects(removal,/retained: device timing is still active/);await settle();f.ack(0,10,false);await rejected;
 assert.equal(f.c.activeTrack.id,'original');assert.equal(f.deletions.length,0);assert.equal(f.changes(),0);
 const arm=f.c.activateTrack(track('B'));await settle();f.ack(1,11);await settle();f.lookups[0].resolve(null);await arm;assert.equal(f.c.activeTrack.id,'B');assert.equal(f.changes(),0);
});

test('a raw track object queued after its deletion cannot arm a deleted row',async()=>{
 const f=fixture(),remove=f.c.deleteTrackWithRef('B'),arm=f.c.activateTrack(track('B'));await remove;await settle();
 assert.equal(f.requests.length,0,'stale object must be re-read before configuring');await arm;assert.equal(f.c.activeTrack.id,'original');assert.equal(f.c.referenceLap.id,'ref-original');assert.match(f.alerts[0],/no longer saved/);
});
test('a raw track object queued after clear-all cannot revive a removed track',async()=>{
 const f=fixture(),clear=f.c.clearAllTracks(),arm=f.c.activateTrack(track('B'));await settle();f.ack(0,10);await clear;await settle();
 assert.equal(f.requests.length,1,'only Stop may configure after rows are cleared');await arm;assert.equal(f.c.activeTrack,null);assert.equal(f.c.referenceLap,null);assert.match(f.alerts[0],/no longer saved/);
});
test('failed saved-track lookup cannot publish an object and later commands remain usable',async()=>{
 const f=fixture(),lookup=f.c.getTrack;f.c.getTrack=async()=>{throw Error('Database closed')};const arm=f.c.activateTrack(track('A'));await settle();
 assert.equal(f.requests.length,0);await arm;assert.equal(f.c.activeTrack.id,'original');assert.equal(f.c.referenceLap.id,'ref-original');assert.match(f.alerts[0],/Database closed/);
 f.c.getTrack=lookup;const next=f.c.activateTrack(track('B'));await settle();f.ack(0,10);await settle();f.lookups[0].resolve(null);await next;assert.equal(f.c.activeTrack.id,'B');
});
test('object activation uses the current saved geometry and new-stage state',async()=>{
 const f=fixture(),saved={...track('B'),name:'Saved stage',type:'point_to_point',isNew:true,finishLine:{p1:[100,0],p2:[100,20],direction:0}};f.c.getTrack=async()=>saved;
 const arm=f.c.activateTrack(track('B'));await settle();assert.match(f.requests[0].url,/type=point_to_point/);assert.match(f.requests[0].url,/f_p1_lat=/);f.ack(0,10);await settle();f.lookups[0].resolve(null);await arm;
 assert.equal(f.c.activeTrack.name,'Saved stage');assert.equal(f.c.activeTrack.isNew,true);assert.equal(f.c.p2pNeedsWarmup,true);assert.equal(f.elements.get('lap-track-name').textContent,'Saved stage');
});
