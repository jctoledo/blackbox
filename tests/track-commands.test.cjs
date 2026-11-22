const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const deferred=()=>{let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no});return{promise,resolve,reject}};
const settle=async()=>{for(let i=0;i<20;i++)await Promise.resolve()};
const track=id=>({id,name:'Track '+id,type:'loop',gpsOrigin:{lat:37,lon:-122},startLine:{p1:[0,0],p2:[0,20],direction:0},lapCount:0});
function fixture(){
 const requests=[],lookups=[],alerts=[],elements=new Map(),published=[];let changes=0;
 const c={console,Promise,Math,activeTrack:track('original'),referenceLap:{id:'ref-original'},expectedLapRevision:9,lapResultGeneration:0,lapTracker:{reset(){}},lastDeltaMs:0,deltaTrend:0,prevLapFlags:0,prevLapTimeMs:0,prevLapCnt:0,lapTimerActive:true,p2pNeedsWarmup:false,suppressStartLineIndicator:false,
  trackAutoDetector:{beginTrackChange(){changes++},endTrackChange(){changes--}},
  requestDevice(url){const pending=deferred();requests.push({url,...pending});return pending.promise},
  getRefLap(id){const pending=deferred();lookups.push({id,...pending});return pending.promise},alert:message=>alerts.push(message),startNewSession(){},clearSession(){},fmtLapTime:String,updateDeltaBar(){},
  updateActiveTrackDisplay(){published.push({id:c.activeTrack?.id??null,ref:c.referenceLap?.id??null,revision:c.expectedLapRevision})},
  $:id=>{if(!elements.has(id)){const values=new Set();elements.set(id,{textContent:'Original label',hidden:false,classList:{add(...keys){keys.forEach(k=>values.add(k))},remove(...keys){keys.forEach(k=>values.delete(k))},toggle(key,on){if(on===undefined)on=!values.has(key);on?values.add(key):values.delete(key);return on}}})}return elements.get(id)}};
 vm.createContext(c);
 const marker=html.includes('// Track command ownership.')?'// Track command ownership.':'async function activateTrack(';
 vm.runInContext(html.slice(html.indexOf(marker),html.indexOf('// Track Recording Functions')),c);
 const ack=(index,revision=10,ok=true,error=null)=>requests[index].resolve({ok,status:ok?200:500,json:async()=>({revision,...(error?{error}:{})})});
 return{c,requests,lookups,alerts,elements,published,ack,changes:()=>changes};
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
