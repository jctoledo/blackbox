const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function setup(){let now=10000,next=0;const timers=new Map(),elements=new Map(),classes=new Set(),events=new Map(),tracks=[{id:'a',name:'A very long saved track name with useful full identification',centerline:[{x:0,y:0,heading:0}]}];
 const document={activeElement:null,body:{classList:{add:k=>classes.add(k),remove:k=>classes.delete(k),contains:k=>classes.has(k)},appendChild(){}},createElement:()=>({classList:{add(){}},parentNode:null}),addEventListener(k,fn){events.set(k,fn)}};
 const $=id=>{if(!elements.has(id))elements.set(id,{id,hidden:id==='track-suggestion',classList:{add(){},remove(){}},setAttribute(k,v){this[k]=v},contains(el){return el===this||id==='track-suggestion'&&el?.id?.startsWith('track-suggestion-')},focus(){document.activeElement=this}});return elements.get(id)};
 const c={Math,Number,Set,Map,Date:{now:()=>now},performance:{now:()=>now},document,$,activeTrack:null,trackRecorder:null,currentPos:null,escHtml:String,getGpsRef:()=>null,wrapAngle:a=>Math.atan2(Math.sin(a),Math.cos(a)),getAllTracks:async()=>tracks,activateTrack:async()=>{c.activations++},activations:0,rec:true,setTimeout(fn,ms){timers.set(++next,{fn,at:now+ms});return next},clearTimeout:id=>timers.delete(id)};vm.createContext(c);vm.runInContext(slice('// Track Auto-Detection', 'function commitTrackWrite('),c);
 const fresh=()=>c.currentPos={valid:true,gpsAgeMs:0,receivedAtMs:now,x:0,y:0,yaw:0,validGpsCourse:false};fresh();
 const advance=ms=>{now+=ms;for(const [id,timer] of [...timers])if(timer.at<=now){timers.delete(id);timer.fn()}};
 const check=()=>vm.runInContext('trackAutoDetector.check(currentPos.x,currentPos.y,currentPos.yaw)',c);
 const detect=async()=>{fresh();await check();advance(3100);fresh();return check()};
 return {c,$,classes,events,tracks,fresh,advance,check,detect};
}
test('explicit Dismiss suppresses repeat candidate for this page while another saved track stays eligible',async()=>{
 const p=setup(),detected=await p.detect();assert.equal(detected.track.id,'a');p.c.showTrackDetectedToast(detected.track,detected.generation);p.c.dismissActiveToast();
 p.advance(3100);p.fresh();assert.equal(await p.check(),null);p.advance(3100);p.fresh();assert.equal(await p.check(),null);
 p.tracks[0]={...p.tracks[0],id:'b'};p.advance(3100);const other=await p.detect();assert.equal(other.track.id,'b');assert.equal(p.c.activations,0);assert.equal(p.c.rec,true);
});
for(const boundary of ['active','invalid','gps-age','receipt','moved','reset','activation'])test('delayed lookup rechecks current '+boundary+' context before presenting',async()=>{
 const p=setup();await p.check();p.advance(3100);p.fresh();let finish;p.c.getAllTracks=()=>new Promise(r=>finish=r);const pending=p.check();
 if(boundary==='active')p.c.activeTrack={id:'other'};if(boundary==='invalid')p.c.currentPos.valid=false;if(boundary==='gps-age')p.c.currentPos.gpsAgeMs=301;if(boundary==='receipt')p.advance(301);if(boundary==='moved')p.c.currentPos.x=1000;if(boundary==='reset')vm.runInContext('trackAutoDetector.reset()',p.c);if(boundary==='activation')vm.runInContext('trackAutoDetector.beginTrackChange();trackAutoDetector.endTrackChange()',p.c);
 finish(p.tracks);assert.equal(await pending,null);if(boundary==='reset')assert.equal(vm.runInContext('trackAutoDetector.matchCount',p.c),0);assert.equal(p.c.activations,0);
});
test('timeout cools down the candidate without creating a permanent dismissal',async()=>{
 const p=setup(),detected=await p.detect();p.c.showTrackDetectedToast(detected.track,detected.generation);p.advance(10001);p.fresh();assert.equal(p.$('track-suggestion').hidden,true);assert.equal(await p.check(),null);
 p.advance(3100);p.fresh();assert.equal(await p.check(),null);p.advance(60001);const again=await p.detect();assert.equal(again.track.id,'a');
});
test('freshness loss withdraws visible suggestion and keyboard dismissal returns focus without capture effects',async()=>{
 const p=setup(),detected=await p.detect();p.c.showTrackDetectedToast(detected.track,detected.generation);p.c.currentPos.valid=false;await p.check();assert.equal(p.$('track-suggestion').hidden,true);
 p.advance(3100);const again=await p.detect();p.c.showTrackDetectedToast(again.track,again.generation);p.$('track-suggestion-dismiss').focus();p.$('track-suggestion-dismiss').onclick();assert.equal(p.$('track-suggestion').hidden,true);assert.equal(p.c.document.activeElement,p.$('quick-tracks'));assert.equal(p.c.activations,0);assert.equal(p.c.rec,true);
});
test('dismissal storage stays bounded and page reload is its only reset',()=>{
 const p=setup();for(let i=0;i<300;i++){p.fresh();const shown=p.c.showTrackDetectedToast({...p.tracks[0],id:'track-'+i});if(shown)p.c.dismissActiveToast()}
 assert.equal(vm.runInContext('trackAutoDetector.dismissed.size',p.c),256);p.fresh();assert.equal(p.c.showTrackDetectedToast(p.tracks[0]),false);vm.runInContext('trackAutoDetector.reset()',p.c);assert.equal(vm.runInContext('trackAutoDetector.dismissed.size',p.c),256);assert.equal(vm.runInContext('trackAutoDetector.dismissed.size',setup().c),0);
});
test('presentation rechecks generation and fresh GPS; only explicit activation calls timing',async()=>{
 const p=setup(),detected=await p.detect();vm.runInContext('trackAutoDetector.reset()',p.c);assert.equal(p.c.showTrackDetectedToast(detected.track,detected.generation),false);
 p.fresh();assert.equal(p.c.showTrackDetectedToast(detected.track),true);assert.equal(p.$('track-suggestion-name').textContent,detected.track.name);assert.equal(p.$('track-suggestion-name').title,detected.track.name);assert.equal(p.c.activations,0);
 p.c.currentPos.gpsAgeMs=301;await p.$('track-suggestion-activate').onclick();assert.equal(p.c.activations,0);p.fresh();p.c.showTrackDetectedToast(detected.track);await p.$('track-suggestion-activate').onclick();assert.equal(p.c.activations,1);assert.equal(p.c.rec,true);
});
test('receipt watchdog and offline handler withdraw suggestions without another telemetry callback',async()=>{
 const p=setup(),detected=await p.detect();p.c.showTrackDetectedToast(detected.track,detected.generation);
 Object.assign(p.c,{clearForces(){},lapResultGeneration:0,lapTimerActive:false,lapTracker:null,referenceLap:null,updateLapProgress(){},updateDeltaBar(){}});
 vm.runInContext(slice('// Receipt freshness watchdog.', '// End receipt freshness watchdog.')+slice('function markTelemetryUnavailable(', 'async function poll()'),p.c);vm.runInContext('lastTelemetryAt=performance.now()',p.c);
 p.advance(301);p.c.checkTelemetryFreshness();assert.equal(p.$('track-suggestion').hidden,true);
 p.advance(3100);const again=await p.detect();p.c.showTrackDetectedToast(again.track,again.generation);p.c.markTelemetryUnavailable('Offline');assert.equal(p.$('track-suggestion').hidden,true);assert.equal(p.c.currentPos.valid,false);
});
test('saved-track mutation suspends detection until commit and invalidates older reads',async()=>{
 const p=setup();await p.check();p.advance(3100);p.fresh();let finish;p.c.getAllTracks=()=>new Promise(r=>finish=r);const lookup=p.check();let tx;p.c.trackDb={transaction(){tx={objectStore:()=>({put(){}})};return tx}};
 vm.runInContext(slice('async function saveTrack(', 'async function getTrack('),p.c);const save=p.c.saveTrack(p.tracks[0]);assert.equal(vm.runInContext('trackAutoDetector.trackChanges',p.c),1);finish(p.tracks);assert.equal(await lookup,null);assert.equal(await p.check(),null);tx.oncomplete();await save;assert.equal(vm.runInContext('trackAutoDetector.trackChanges',p.c),0);assert.equal(p.c.activations,0);
});
test('delayed lookup and presentation retain the point origin across firmware-origin replacement',async()=>{
 const p=setup(),oldOrigin={lat:37,lon:-122},newOrigin={lat:37.001,lon:-122};let liveOrigin=oldOrigin;
 p.c.getGpsRef=()=>liveOrigin;vm.runInContext(slice('function transformCoord(', 'function transformLine('),p.c);p.tracks[0].gpsOrigin=oldOrigin;p.c.currentPos.coordinateOrigin={...oldOrigin};
 await p.check();p.advance(3100);p.fresh();p.c.currentPos.coordinateOrigin={...oldOrigin};let finish;p.c.getAllTracks=()=>new Promise(r=>finish=r);const pending=p.check();liveOrigin=newOrigin;
 finish(p.tracks);const result=await pending;assert.equal(result?.track.id,'a');assert.equal(p.c.showTrackDetectedToast(result.track,result.generation),true);assert.equal(p.$('track-suggestion').hidden,false);
});
test('origin replacement cannot reinterpret an old-frame point as a new nearby track match',async()=>{
 const p=setup(),oldOrigin={lat:37,lon:-122},newOrigin={lat:37.001,lon:-122};let liveOrigin=oldOrigin;
 p.c.getGpsRef=()=>liveOrigin;vm.runInContext(slice('function transformCoord(', 'function transformLine('),p.c);p.tracks[0].gpsOrigin=newOrigin;p.c.currentPos.coordinateOrigin={...oldOrigin};p.c.currentPos.y=(newOrigin.lat-oldOrigin.lat)*111320;
 await p.check();p.advance(3100);p.fresh();p.c.currentPos.coordinateOrigin={...oldOrigin};let finish;p.c.getAllTracks=()=>new Promise(r=>finish=r);const pending=p.check();liveOrigin=newOrigin;finish(p.tracks);
 assert.equal(await pending,null);assert.equal(p.c.showTrackDetectedToast(p.tracks[0]),false);assert.equal(p.$('track-suggestion').hidden,true);
});
