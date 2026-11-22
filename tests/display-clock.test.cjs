const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b,start=0)=>html.slice(html.indexOf(a,start),html.indexOf(b,html.indexOf(a,start)));
function clock(extra={}){let wall=4000000,mono=0;class ClockDate extends Date{static now(){return wall}}const c={Math,Number,Date:ClockDate,performance:{now:()=>mono},...extra};vm.createContext(c);return{c,at(m,w){mono=m;wall=w}}}
function motion(){const f=clock({lastT:null,trail:[],emaGx:0,emaGy:0,latg:.4,lng:-.3,EMA_TAU:.07,effortPeaks:{update(){}}});const start=html.indexOf('function process(buf');vm.runInContext('function step(){'+slice('    const now=Date.now();','    speed_ema=',start)+'}',f.c);return f}
test('force EMA remains finite and follows elapsed time through backward and forward wall corrections',()=>{
 const normal=motion(),jumped=motion();for(let i=0;i<12;i++){normal.at(i*33,4000000+i*33);jumped.at(i*33,i<4?4000000+i*33:i<8?400000+i*33:90400000+i*33);normal.c.step();jumped.c.step();assert.ok(Number.isFinite(jumped.c.emaGx)&&Number.isFinite(jumped.c.emaGy),'fresh finite motion must not corrupt the filter');assert.ok(Math.abs(normal.c.emaGx-jumped.c.emaGx)<1e-12);assert.ok(Math.abs(normal.c.emaGy-jumped.c.emaGy)<1e-12)}
});
test('zero clock origin is valid and a real long frame gap resets motion history',()=>{
 const f=motion();f.c.step();const initial=f.c.emaGx;f.c.trail.push({x:1,y:1,t:0});f.at(100,4000100);f.c.step();assert.equal(f.c.lastT,100);assert.equal(f.c.trail.length,1);const alpha=1-Math.exp(-.1/.07);assert.ok(Math.abs(f.c.emaGx-(alpha*.4+(1-alpha)*initial))<1e-12);f.at(351,4000351);f.c.step();assert.equal(f.c.trail.length,0);assert.ok(Math.abs(f.c.emaGx-initial)<1e-12,'a real gap must restart filtering without prior force history');
});
test('plot history expiry uses the same elapsed clock as its points',()=>{
 const f=clock({trail:[{t:0},{t:1000}],TRAIL_DURATION_MS:3000});const draw=html.indexOf('function drawG(){');vm.runInContext('function recent(){'+slice('    const now=', '    const displayTrail=',draw)+'return recentTrail}',f.c);
 f.at(1500,400000);assert.equal(f.c.recent().length,2);f.at(3100,90400000);assert.equal(f.c.recent().length,1);f.at(4100,400001);assert.equal(f.c.recent().length,0);
});
test('page SESSION stopwatch follows elapsed time despite wall corrections',()=>{
 const elements=new Map(),f=clock({cnt:0,lastGpsState:false,fusion:{},rec:false,fmtTime:ms=>ms,updateRecUI(){},setInterval(fn){this.tick=fn},$:id=>{if(!elements.has(id))elements.set(id,{textContent:'',classList:{remove(){},toggle(){}}});return elements.get(id)}});
 f.c.setInterval=fn=>f.c.tick=fn;vm.runInContext(html.match(/let speed_ema=0,displaySessionStart=[^;]+;/)[0]+slice('let lastRateUpdateAt=', 'setInterval(updateDiag'),f.c);
 f.at(1000,400000);f.c.tick();assert.equal(f.c.$('session-time').textContent,1000);f.at(2000,90400000);f.c.tick();assert.equal(f.c.$('session-time').textContent,2000);
});
test('track capture elapsed time ignores wall corrections while point timestamps retain them',()=>{
 const f=clock();vm.runInContext(slice('class TrackRecorder{','// Track Manager IndexedDB')+';this.Capture=TrackRecorder;',f.c);const r=new f.c.Capture();const p={x:0,y:0,lat:37,lon:-122,heading:0,speed:10,sigma:3};r.start(p,'point_to_point');assert.equal(r.keyPoints[0].t,4000000);f.at(1000,400000);assert.equal(r.getStats().elapsedMs,1000);r.addSample({...p,x:3,lat:37.00003});r.addSample({...p,x:6,lat:37.00006});assert.equal(r.keyPoints.at(-1).t,400000);f.at(2000,90400000);assert.equal(r.getStats().elapsedMs,2000);
});
