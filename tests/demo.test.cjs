const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
function demo(){
 let now=0,ready,track,reference;const buttons=new Map();const button={textContent:''};buttons.set('demo-connection',button);
 const c={console,URL,Response,DOMException,DataView,Uint8Array,Float32Array,Math,Date,btoa,window:{},location:{href:'https://example.test/demo.html'},performance:{now:()=>now},document:{addEventListener:(_,fn)=>{ready=fn},getElementById:id=>{if(!buttons.has(id))buttons.set(id,{textContent:''});return buttons.get(id)}},openTrackDB:async()=>{},getAllTracks:async()=>[],getRefLap:async()=>null,saveRefLap:async r=>{reference=r},saveTrack:async t=>{track=t},activateTrack:async()=>{}};
 vm.createContext(c);vm.runInContext(fs.readFileSync('docs/demo.js','utf8'),c);
 return {context:c,fetch:c.window.fetch,advance:ms=>{now+=ms},ready:()=>ready(),button,interval:()=>buttons.get('demo-interval').onclick({target:buttons.get('demo-interval')}),track:()=>track,reference:()=>reference};
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
  assert.ok(Math.abs(Math.hypot(vx,vy)-speed)<.0001);assert.ok(Number.isFinite(a));assert.ok(Math.abs(p.readFloatLE(11))<=5.7);assert.ok(Math.abs(a)<9);
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

test('demo road texture is deterministic and differs from ideal acceleration',async()=>{
 const a=demo(),b=demo();let residual=0;
 for(let i=0;i<600;i++){a.advance(40);b.advance(40);const x=await sample(a),y=await sample(b);assert.equal(x.p.readFloatLE(7),y.p.readFloatLE(7));const noise=-x.p.readFloatLE(7)-x.j.f.li;assert.ok(Math.abs(noise)<.13);residual=Math.max(residual,Math.abs(noise))}
 assert.ok(residual>.03);
});

test('explicit interval demo produces timestamped evidence for a complete rolling run',async()=>{
 const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8'),c={Math,Number};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('class RollingRuns{'),html.indexOf('const rollingRuns='))+';this.Runs=RollingRuns;',c);
 const d=demo();await d.ready();await d.interval();const runs=new c.Runs();let seen=0;
 for(let i=0;i<12;i++){d.advance(1000);const log=await(await d.fetch('/api/gps-log')).json();for(const e of log.events){if(e.seq<=seen)continue;seen=e.seq;runs.update({rowType:1,gpsActive:1,gpsAccepted:1,sp:e.speed_mps,gpsCourse:e.course_rad,gpsUtcUs:e.utc_us,gpsDate:e.date_ddmmyy,gpsFractionDigits:e.fractional_digits,gpsBootId:log.boot_id,gpsSeq:e.seq,gpsSynthetic:1})}}
 const result=runs.results.find(r=>r.status==='accepted');assert.ok(result);assert.ok(Math.abs(result.seconds-5.588)<1e-8);assert.equal(result.synthetic,1);
});

test('driver model brakes before corners and accelerates out with force tied to motion',async()=>{
 const d=demo();let prior,brake=0,exit=0;
 for(let i=0;i<1800;i++){
  const {j,p}=await sample(d),speed=p.readFloatLE(51)/3.6,lon=j.f.li,lat=p.readFloatLE(11),yawRate=p.readFloatLE(19);
  assert.ok(Math.abs(lat-speed*yawRate)<.55,'lateral force follows speed times yaw rate');
  if(prior){assert.ok((speed-prior.speed)/.04>=Math.min(lon,prior.lon)-.15&&(speed-prior.speed)/.04<=Math.max(lon,prior.lon)+.15,`longitudinal force follows changing speed at ${i}: speed ${prior.speed} -> ${speed}, acceleration ${prior.lon} -> ${lon}`);if(lon<-3&&Math.abs(lat)<4)brake++;if(lon>1&&Math.abs(lat)>1)exit++}
  prior={speed,lon};d.advance(40);
 }
 assert.ok(brake>10,'sustained braking approach');assert.ok(exit>10,'loaded corner exit');
});

// Exercise the actual persistence boundary rather than a reusable no-op save stub.
test('a fresh demo commits its track before creating the matching stored reference',async()=>{
 const {indexedDB}=require('fake-indexeddb'),database=await new Promise((resolve,reject)=>{
  const request=indexedDB.open('demo-real-persistence-'+Date.now());request.onerror=()=>reject(request.error);request.onsuccess=()=>resolve(request.result);
  request.onupgradeneeded=()=>{request.result.createObjectStore('tracks',{keyPath:'id'});request.result.createObjectStore('reference_laps',{keyPath:'id'}).createIndex('trackId','trackId',{unique:true})};
 });
 const d=demo(),c=d.context,html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');let activated=false;
 const section=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
 c.console={log(){},error(){}};c.trackDb=database;c.openTrackDB=async()=>{};c.trackAutoDetector={beginTrackChange(){},endTrackChange(){}};c.activateTrack=async()=>{activated=true};
 vm.runInContext(section('// Reference lap CRUD','async function deleteRefLap(')+section('function validSpatialReference(','class SpatialMatcher{')+section('async function saveTrack(','async function getTrack('),c);
 await d.ready();
 const read=store=>new Promise((resolve,reject)=>{const r=database.transaction(store).objectStore(store).getAll();r.onsuccess=()=>resolve(r.result);r.onerror=()=>reject(r.error)});
 const tracks=await read('tracks'),references=await read('reference_laps');
 assert.equal(activated,true,'fresh preview must reach activation');assert.equal(tracks.length,1);assert.equal(references.length,1);assert.equal(references[0].trackId,tracks[0].id);
 database.close();
});
