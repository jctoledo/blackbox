const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const code=html.slice(html.indexOf('// Responsive driving layout.'),html.indexOf('// End responsive driving layout.'));
function context(landscape){
 const classes=new Set(),elements=new Map();let changed;
 const media={matches:landscape,addEventListener:(_,f)=>{changed=f}};
 const c={document:{body:{classList:{contains:v=>classes.has(v),remove:v=>classes.delete(v),toggle(v,on){if(on===undefined)on=!classes.has(v);if(on)classes.add(v);else classes.delete(v);return on}}}},matchMedia:()=>media,$:id=>{if(!elements.has(id))elements.set(id,{setAttribute(k,v){this[k]=v}});return elements.get(id)},requestAnimationFrame:fn=>fn(),resize(){},startRecording(){throw Error('Orientation started recording')},deactivateTrack(){throw Error('Orientation stopped timing')}};
 vm.createContext(c);vm.runInContext(code,c);return {classes,elements,rotate:landscape=>{media.matches=landscape;changed()}};
}
test('landscape automatically enters cluster and portrait restores ordinary dashboard',()=>{
 const p=context(false);assert.equal(p.classes.has('cluster-view'),false);p.rotate(true);assert.equal(p.classes.has('cluster-view'),true);assert.equal(p.classes.has('drive-view'),true);assert.equal(p.elements.get('drive-mode').textContent,'Dashboard');p.rotate(false);assert.equal(p.classes.has('drive-view'),false);
});
test('Dashboard gives an explicit escape from automatic cluster without changing orientation',()=>{
 const p=context(true);p.elements.get('drive-mode').onclick();assert.equal(p.classes.has('cluster-view'),false);assert.equal(p.classes.has('drive-view'),false);p.elements.get('drive-mode').onclick();assert.equal(p.classes.has('cluster-view'),true);
});
test('portrait manual Drive view and parked Analysis remain independently usable',()=>{
 const p=context(false);p.elements.get('drive-mode').onclick();assert.equal(p.classes.has('drive-view'),true);assert.equal(p.classes.has('cluster-view'),false);p.elements.get('analysis-mode').onclick();assert.equal(p.classes.has('analysis-view'),true);assert.equal(p.classes.has('drive-view'),false);p.rotate(true);assert.equal(p.classes.has('cluster-view'),false);p.elements.get('analysis-mode').onclick();assert.equal(p.classes.has('cluster-view'),true);
});
function peaks(){const c={Number,Math};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('class EffortPeaks{'),html.indexOf('const effortPeaks='))+';this.Peaks=EffortPeaks;',c);return new c.Peaks();}
test('sustained peaks use time-weighted windows and distinguish braking from acceleration',()=>{
 const p=peaks();for(let t=0;t<=1000;t+=50)p.update(t,.4,true);assert.ok(Math.abs(p.accel-.4)<1e-8);assert.equal(p.brake,0);
 for(let t=1050;t<=2000;t+=50)p.update(t,-.6,true);assert.ok(Math.abs(p.brake-.6)<1e-8);assert.ok(Math.abs(p.accel-.4)<1e-8);
});
test('spikes, outages and repeated packets cannot masquerade as sustained effort',()=>{
 const p=peaks();for(let t=0;t<=500;t+=50)p.update(t,t===250?1:0,true);assert.ok(p.accel<.11);
 p.reset();p.update(0,1,true);p.update(600,1,true);assert.equal(p.accel,null);
 for(let i=0;i<20;i++)p.update(600,1,true);assert.equal(p.accel,null);
 p.update(650,NaN,true);p.update(700,1,false);assert.equal(p.accel,null);
});
test('device millisecond rollover does not create an enormous effort window',()=>{
 const p=peaks();for(let i=0;i<=10;i++)p.update((0xfffffff0+i*50)>>>0,.3,true);assert.ok(Math.abs(p.accel-.3)<1e-8);
});
test('left and right sustained peaks keep independent signs',()=>{
 const p=peaks();for(let t=0;t<=500;t+=50)p.update(t,0,true,-.7);assert.ok(Math.abs(p.left-.7)<1e-8);assert.equal(p.right,0);
 for(let t=550;t<=1100;t+=50)p.update(t,0,true,.4);assert.ok(Math.abs(p.right-.4)<1e-8);
});

test('force scale preserves direction and marks clipping without dropping the dot offscreen',()=>{
 const c={Math,Number};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('function forcePoint('),html.indexOf('function clearForces(')),c);
 const p=c.forcePoint(3,-4,1);assert.equal(p.clipped,true);assert.ok(Math.abs(Math.hypot(p.x,p.y)-1)<1e-10);assert.ok(p.x>0&&p.y<0);
 assert.equal(c.forcesUsable(1,40,20),true);for(const age of [null,undefined,-1,301,NaN])assert.equal(c.forcesUsable(1,age,20),false);assert.equal(c.forcesUsable(0,40,20),false);for(const age of [null,undefined,-1,201,NaN])assert.equal(c.forcesUsable(1,40,age),false);
});
test('force plot, readouts and peaks share right-positive / acceleration-positive packet convention',()=>{
 const c={Math,Number,Date:{now:()=>1000},forcesUsable:()=>true,gpsOk:1,timing:{gps_age_ms:0},gx:.4,gy:.3,now:1000,trail:[],TRAIL_DURATION_MS:3000,currentScale:1,$:()=>({textContent:''}),clearForces:()=>assert.fail('fresh packet cleared')};vm.createContext(c);
 vm.runInContext(html.slice(html.indexOf('    // Vehicle acceleration convention:'),html.indexOf('    lastGpsState=gpsOk;')),c);
 assert.equal(c.trail[0].x,.4);assert.equal(c.trail[0].y,-.3);
 const p=peaks();for(let t=0;t<=500;t+=50)p.update(t,.3,true,.4);assert.ok(Math.abs(p.right-.4)<1e-9);assert.ok(Math.abs(p.accel-.3)<1e-9);
});
test('horizontal swipes cycle instrument tabs without changing timing or recording',()=>{
 const elements=new Map(),classes=new Set(['cluster-view']);const c={performance:{now:()=>100},requestAnimationFrame:fn=>fn(),resize(){},drawG(){},document:{body:{classList:{contains:k=>classes.has(k),toggle(k,on){on?classes.add(k):classes.delete(k)}}}},$:id=>{if(!elements.has(id))elements.set(id,{handlers:{},setAttribute(k,v){this[k]=v},addEventListener(k,fn){this.handlers[k]=fn}});return elements.get(id)}};vm.createContext(c);vm.runInContext(html.slice(html.indexOf("let clusterInstrument='speed'"),html.indexOf('// End selectable cluster instrument.')),c);
 const events=elements.get('g-card').handlers,start={touches:[{clientX:150,clientY:80}],target:{closest:()=>null}},end={changedTouches:[{clientX:50,clientY:82}]};
 events.touchstart(start);events.touchend(end);assert.equal(elements.get('cluster-forces')['aria-pressed'],'true');
 events.touchstart(start);events.touchcancel();events.touchend(end);assert.equal(elements.get('cluster-forces')['aria-pressed'],'true');
 events.touchstart(start);events.touchend(end);assert.equal(elements.get('cluster-peaks')['aria-pressed'],'true');
 events.touchstart(start);events.touchend(end);assert.equal(elements.get('cluster-runs')['aria-pressed'],'true');
 events.touchstart(start);events.touchend(end);assert.equal(elements.get('cluster-speed')['aria-pressed'],'true');
});
