const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const code=html.slice(html.indexOf('function referencePosition('),html.indexOf('// End spatial reference matching.'));
function context(){const c={Math,Number,Array};vm.createContext(c);vm.runInContext(code+';this.Matcher=SpatialMatcher;',c);return c}
const origin={lat:37,lon:-122};
function position(x,y=0,extra={}){return {valid:true,lat:origin.lat+y/111320,lon:origin.lon+x/(111320*Math.cos(origin.lat*Math.PI/180)),speed:36,gpsCourse:Math.PI/2,validGpsCourse:true,gpsAgeMs:0,...extra}}
function reference(points=Array.from({length:101},(_,i)=>({x:i,y:0,t:i*100}))){return {lapTimeMs:points.at(-1).t,spatial:{version:1,origin:{...origin},points}}}
test('same-place delta is independent of distance driven and reference origin survives reboot',()=>{
 const c=context(),m=new c.Matcher(reference());let result;
 for(let x=1;x<=50;x++)result=m.match(position(x,2,{x:999999,y:-999999}),x*100+500);
 assert.equal(result.valid,true);assert.ok(Math.abs(result.deltaMs-500)<.001);assert.ok(Math.abs(result.progress-50)<=1);
 // A different driven line and EKF coordinate frame do not shift the matched reference time.
 const p=c.referencePosition(position(50,2,{x:-30,y:10}),origin);assert.ok(Math.abs(p.x-50)<.00001);assert.ok(Math.abs(p.y-2)<.00001);
});
test('GPS sample age is removed from the time comparison',()=>{
 const m=new (context().Matcher)(reference());const r=m.match(position(5,0,{gpsAgeMs:100}),600);
 assert.equal(r.valid,true);assert.ok(Math.abs(r.deltaMs)<.001);
});
test('stale fixes, missing course, wrong direction, off-route and late acquisition suppress delta',()=>{
 const C=context().Matcher;
 for(const [p,t] of [[position(5,0,{gpsAgeMs:251}),500],[position(5,0,{gpsAgeMs:undefined}),500],[position(5,0,{validGpsCourse:false}),500],[position(5,0,{speed:0}),500],[position(5,0,{gpsCourse:-Math.PI/2}),500],[position(5,13),500],[position(50),5000],[position(-10),100]])assert.equal(new C(reference()).match(p,t).valid,false);
});
test('continuity rejects jumps, backward time, and reacquisition after long gaps',()=>{
 const C=context().Matcher;
 for(const [p,t] of [[position(80),600],[position(6),400],[position(25),2501]]){const m=new C(reference());assert.equal(m.match(position(5),500).valid,true);assert.equal(m.match(p,t).valid,false)}
});
test('same-direction overlapping branches are ambiguous even when continuity favors one',()=>{
 const points=[];let t=0;
 for(let x=0;x<=100;x+=5)points.push({x,y:0,t:t+=100});
 for(let x=95;x>=0;x-=5)points.push({x,y:2,t:t+=100});
 for(let x=5;x<=100;x+=5)points.push({x,y:1,t:t+=100});
 const r=new (context().Matcher)(reference(points)).match(position(5),500);
 assert.equal(r.valid,false);assert.equal(r.reason,'Ambiguous route position');
});
test('invalid spatial traces and legacy distance-only references never produce matched deltas',()=>{
 const c=context();assert.equal(new c.Matcher({samples:new ArrayBuffer(16)}).match(position(5),500).valid,false);
 for(const change of [r=>r.spatial.points[5].t=0,r=>r.spatial.points[5].x=1000,r=>r.spatial.origin.lat=90,r=>r.spatial.points[5].y=NaN,r=>r.lapTimeMs+=2000]){const r=reference();change(r);assert.equal(c.validSpatialReference(r.spatial,r.lapTimeMs),false)}
});
test('matched progress is capped below finish and does not extrapolate past the route',()=>{
 const m=new (context().Matcher)(reference());let r;
 for(let x=1;x<=100;x++){r=m.match(position(x),x*100);assert.equal(r.valid,true,JSON.stringify({x,r,last:m.last}));}
 assert.equal(r.valid,true);assert.equal(r.progress,99);assert.equal(m.match(position(101),10100).valid,false);
});

test('fresh stationary fixes preserve local continuity without inventing a stationary delta',()=>{
 const m=new (context().Matcher)(reference());assert.equal(m.match(position(5),500).valid,true);
 for(let t=600;t<=5600;t+=100)assert.equal(m.match(position(5,0,{speed:0,validGpsCourse:false}),t).valid,false);
 const r=m.match(position(6),5700);assert.equal(r.valid,true);assert.ok(Math.abs(r.deltaMs-5100)<.001);
});
