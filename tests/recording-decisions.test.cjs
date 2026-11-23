const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const source=html.slice(html.indexOf('// Track Recording Functions'),html.indexOf('async function updateTrackBestLap('));
const settle=async()=>{for(let i=0;i<20;i++)await Promise.resolve()};
function fixture(){
 const elements=new Map(),decisions=[],saved=[],armed=[],notices=[];
 const $=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{},classList:{add(){},remove(){}}});return elements.get(id)};
 const recorder={trackType:'loop',keyPoints:[0,1],recording:true,loopDetected:true,cancel(){this.cancelled=true},finish(name){this.finished=true;return{id:'capture',name,type:'loop',centerline:[],startLine:{},gpsOrigin:{lat:1,lon:1}}}};
 const owner={recorder,frozen:true,track:null,saved:false,busy:false};
 const c={console,Promise,Math,Number,$,trackRecorder:recorder,routeCapture:owner,activeTrack:{id:'A',name:'First'},currentSessionId:'s1',expectedLapRevision:1,
  window:{addEventListener(){}},alert:m=>notices.push(m),notifyUser:m=>notices.push(m),prompt(){throw Error('Native prompt blocks recording')},confirm(){throw Error('Native confirm blocks recording')},
  requestDecision(message,value){let resolve;const promise=new Promise(r=>resolve=r);decisions.push({message,value,resolve});return promise},
  async saveTrack(t){saved.push(t)},async activateTrack(t){armed.push(t);return true},renderTrackList(){},closeTrackModal(){},async clearActiveTrack(){c.stopped=c.activeTrack.id},
 };
 vm.createContext(c);vm.runInContext(source,c);return{c,$,owner,recorder,decisions,saved,armed,notices};
}
test('track naming remains pending without blocking or duplicating save',async()=>{
 const f=fixture(),pending=f.c.finishTrackRecording();await settle();
 assert.equal(f.decisions.length,1);assert.equal(f.saved.length,0);
 await f.c.finishTrackRecording();assert.equal(f.decisions.length,1);
 f.decisions[0].resolve('Evening circuit');await pending;
 assert.equal(f.saved.length,1);assert.equal(f.armed.length,1);assert.equal(f.saved[0].name,'Evening circuit');
});
test('cancelled name keeps captured route and makes retry possible',async()=>{
 const f=fixture(),pending=f.c.finishTrackRecording();await settle();f.decisions[0].resolve(null);await pending;
 assert.equal(f.c.routeCapture,f.owner);assert.equal(f.owner.busy,false);assert.equal(f.saved.length,0);assert.equal(f.recorder.finished,undefined);
 const retry=f.c.finishTrackRecording();await settle();f.decisions[1].resolve('Retry');await retry;assert.equal(f.saved.length,1);
});
test('a delayed name cannot save a replaced capture',async()=>{
 const f=fixture(),pending=f.c.finishTrackRecording();await settle();f.c.routeCapture={recorder:{}};f.decisions[0].resolve('Old');await pending;
 assert.equal(f.saved.length,0);assert.equal(f.armed.length,0);
});
test('stop confirmation cannot stop a replacement timing owner',async()=>{
 const f=fixture(),pending=f.c.confirmStopTiming();await settle();f.c.activeTrack={id:'B',name:'Second'};f.c.currentSessionId='s2';f.c.expectedLapRevision=2;
 f.decisions[0].resolve(true);await pending;assert.equal(f.c.stopped,undefined);
});
test('stop confirmation cannot stop a restarted session on the same track',async()=>{
 const f=fixture(),pending=f.c.confirmStopTiming();await settle();f.c.currentSessionId='s2';f.decisions[0].resolve(true);await pending;assert.equal(f.c.stopped,undefined);
});
test('a delayed discard cannot cancel a replacement route',async()=>{
 const f=fixture(),pending=f.c.cancelTrackRecording();await settle();
 f.c.routeCapture={recorder:{}};f.decisions[0].resolve(true);
 assert.equal(await pending,false);assert.equal(f.recorder.cancelled,undefined);
});
test('a discard cannot cancel a save that began while awaiting confirmation',async()=>{
 const f=fixture(),pending=f.c.cancelTrackRecording();await settle();
 f.owner.busy=true;f.decisions[0].resolve(true);
 assert.equal(await pending,false);assert.equal(f.recorder.cancelled,undefined);
});
