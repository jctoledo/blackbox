const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
function demo(){
 let now=0,ready,track,reference;const button={textContent:''};
 const c={console,URL,Response,DOMException,DataView,Uint8Array,Float32Array,Math,Date,btoa,window:{},location:{href:'https://example.test/demo.html'},performance:{now:()=>now},document:{addEventListener:(_,fn)=>{ready=fn},getElementById:()=>button},openTrackDB:async()=>{},getAllTracks:async()=>[],getRefLap:async()=>null,saveRefLap:async r=>{reference=r},saveTrack:async t=>{track=t},activateTrack:async()=>{}};
 vm.createContext(c);vm.runInContext(fs.readFileSync('docs/demo.js','utf8'),c);
 return {fetch:c.window.fetch,advance:ms=>{now+=ms},ready:()=>ready(),button,track:()=>track,reference:()=>reference};
}
async function sample(d){const j=await(await d.fetch('/api/telemetry')).json();return {j,p:Buffer.from(j.data,'base64')}}
test('synthetic telemetry preserves packet, clock, lap result and configuration contracts',async()=>{
 const d=demo();await d.fetch('/api/laptimer/configure?type=loop');d.advance(6000);
 const {j,p}=await sample(d);
 assert.equal(p.length,82);assert.equal(p[0],4);assert.equal(p.readUInt16LE(1),0xAA55);
 assert.equal(p.readUInt16LE(80),p.subarray(0,80).reduce((a,b)=>a+b,0)&65535);
 assert.equal(j.timing.state,'timing');assert.equal(j.timing.revision,1);assert.ok(Math.abs(p.readUInt32LE(73)-1000)<=1);
 let lastTime=p.readUInt32LE(73),lastResult;
 for(let i=0;i<1500;i++){
  d.advance(100);const s=await sample(d);
  if(s.p.readUInt16LE(77)>0){lastResult=s.j.timing.last_lap_ms;assert.ok(lastResult>=lastTime&&lastResult<=lastTime+101);break}
  lastTime=s.p.readUInt32LE(73);
 }
 assert.ok(lastResult>10000);assert.equal((await sample(d)).j.timing.best_lap_ms,lastResult);
 const unsupported=await(await d.fetch('/api/laptimer/configure?type=point_to_point')).json();assert.match(unsupported.error,/circuit timing only/);
});
test('demo trajectory has consistent velocity, varying speed, braking and acceleration with a monotonic reference',async()=>{
 const d=demo();await d.ready();const r=d.reference(),t=d.track(),pairs=new Float32Array(r.samples);
 assert.equal(r.trackId,t.id);assert.ok(r.totalDistance>500);assert.equal(pairs.length,r.sampleCount*2);
 for(let i=2;i<pairs.length;i+=2){assert.ok(pairs[i]>pairs[i-2]);assert.ok(pairs[i+1]>pairs[i-1])}
 let previous,minimum=Infinity,maximum=0,braking=false,accelerating=false;
 for(let i=0;i<1500;i++){
  const {p}=await sample(d),speed=p.readFloatLE(51)/3.6,vx=p.readFloatLE(43),vy=p.readFloatLE(47),x=p.readFloatLE(35),y=p.readFloatLE(39),a=p.readFloatLE(7);
  assert.ok(Math.abs(Math.hypot(vx,vy)-speed)<.0001);assert.ok(Number.isFinite(a));assert.ok(Math.abs(p.readFloatLE(11))<=5.01);assert.ok(Math.abs(a)<8);
  minimum=Math.min(minimum,speed);maximum=Math.max(maximum,speed);braking ||= a<-.35;accelerating ||= a>.35;
  if(previous){const derived=Math.hypot(x-previous.x,y-previous.y)/.1;assert.ok(Math.abs(derived-speed)<.6,`Position speed ${derived} vs ${speed}`)}
  previous={x,y};d.advance(100);
 }
 assert.ok(maximum-minimum>8);assert.ok(braking&&accelerating);
});
test('demo disconnect rejects telemetry and configuration until restored',async()=>{
 const d=demo();await d.ready();d.button.onclick({target:d.button});
 await assert.rejects(d.fetch('/api/telemetry'),/connection loss/);await assert.rejects(d.fetch('/api/laptimer/configure?type=clear'),/connection loss/);
 d.button.onclick({target:d.button});assert.equal((await d.fetch('/api/telemetry')).status,200);
 assert.equal((await d.fetch('/not-a-device-endpoint')).status,404);
});


test('spatial reference matches a complete synthetic circuit through its start/finish seam',async()=>{
 const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8'),c={Math,Number,Array};vm.createContext(c);
 vm.runInContext(html.slice(html.indexOf('function referencePosition('),html.indexOf('// End spatial reference matching.'))+';this.Matcher=SpatialMatcher;',c);
 const d=demo();await d.ready();await d.fetch('/api/laptimer/configure?type=loop');d.advance(5100);
 const m=new c.Matcher(d.reference());let matched=0,total=0;
 for(let i=0;i<700;i++){
  const {j,p}=await sample(d);if(p.readUInt16LE(77)>0)break;
  const r=m.match({valid:true,lat:j.timing.lat,lon:j.timing.lon,speed:p.readFloatLE(51),gpsAgeMs:0,gpsCourse:p.readFloatLE(69),validGpsCourse:true},p.readUInt32LE(73));
  total++;if(r.valid)matched++;else assert.fail(r.reason+' at '+p.readUInt32LE(73));d.advance(100);
 }
 assert.ok(total>400);assert.equal(matched,total);
});
test('demo GPS evidence is explicitly synthetic, bounded and uses 40ms receiver epochs',async()=>{
 const d=demo();const a=await (await d.fetch('/api/gps-log')).json();assert.equal(a.simulated,true);assert.equal(a.events.length,64);
 assert.equal(a.events[1].utc_us-a.events[0].utc_us,40000);d.advance(1000);const b=await (await d.fetch('/api/gps-log')).json();assert.equal(b.latest_seq-a.latest_seq,25);
});
