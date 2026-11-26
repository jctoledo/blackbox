const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function context(){const c={Math,Number,Error};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('class RollingRuns{'),html.indexOf('const rollingRuns='))+';this.Standing=StandingRuns;',c);return c}
function row(i,speed,step=.04,extra={}){return {rowType:1,gpsActive:1,gpsAccepted:1,sp:speed,gpsCourse:speed<2?null:1,gpsUtcUs:1000000+Math.round(i*step*1e6),gpsDate:251126,gpsFractionDigits:2,gpsBootId:7,gpsSeq:i+1,gpsMissing:0,gpsSynthetic:0,...extra}}
function launch(d,step=.04,mutate=()=>({})){const rows=[];for(let i=0;i<=Math.ceil(28/step);i++){const t=i*step,speed=Math.max(0,2*(t-2));const r=row(i,speed,step,mutate(i,t,speed));rows.push(r);d.update(r)}return rows}
test('standing speed needs stationary evidence; uses last stationary fix and exact speed crossing at 25 and 5 Hz',()=>{
 for(const step of [.04,.2]){const c=context(),d=new c.Standing({label:'0-60 mph',hi:60*.44704});const rows=launch(d,step),result=d.results.find(r=>r.status==='accepted');assert.ok(result);
  const anchor=rows.find(r=>r.gpsSeq===result.startSeq);assert.ok(anchor.sp<=.3);assert.equal(result.startUs,anchor.gpsUtcUs);assert.equal(result.startConvention,'stationary-fix-v1');assert.equal(result.rolloutM,0);assert.ok(result.launchBracketMs>0&&result.launchBracketMs<=600);
  const finishUs=1000000+(2+60*.44704/2)*1e6;assert.ok(Math.abs(result.endUs-finishUs)<1e-6);assert.ok(Math.abs(result.seconds-(finishUs-anchor.gpsUtcUs)/1e6)<1e-9);
  const mid=new c.Standing(d.interval);for(let i=0;i<400;i++)mid.update(row(i,10+i*.08));assert.equal(mid.results.length,0);
 }
});
test('quarter mile integrates receiver speed to exactly 402.336 m with a partial finish segment',()=>{
 const c=context(),d=new c.Standing({label:'1/4 mile',distanceM:402.336}),rows=launch(d),r=d.results.find(r=>r.status==='accepted');assert.ok(r);
 const start=rows.find(x=>x.gpsSeq===r.startSeq),expected=(-start.sp+Math.sqrt(start.sp**2+4*402.336))/2;
 assert.ok(Math.abs(r.seconds-expected)<1e-8);assert.equal(r.distanceM,402.336);assert.ok(Math.abs(r.finishSpeedMps-(start.sp+2*expected))<1e-8);assert.ok(r.endUs<rows.find(x=>x.gpsSeq===r.endSeq).gpsUtcUs);
});
test('stationary jitter and slow creep cannot produce an accepted launch',()=>{
 const c=context(),noise=new c.Standing({label:'0-60 mph',hi:60*.44704});for(let i=0;i<300;i++)noise.update(row(i,i%2?.29:.05));assert.equal(noise.results.length,0);
 const creep=new c.Standing(noise.interval);for(let i=0;i<500;i++)creep.update(row(i,i<50?0:Math.min(.4+(i-50)*.005,30)));
 assert.equal(creep.results.some(r=>r.status==='accepted'),false);assert.ok(creep.results.some(r=>/creeping/.test(r.reason)));
});
test('GPS failures, turns, boot/date/sequence changes and implausible acceleration reject launches',()=>{
 for(const bad of [{gpsSeq:900},{gpsBootId:8},{gpsDate:261126},{gpsAccepted:0},{gpsCourse:2},{gpsUtcUs:null},{gpsMissing:1},{sp:100}]){
  const c=context(),d=new c.Standing({label:'1/4 mile',distanceM:402.336});launch(d,.04,i=>i===160?bad:{});assert.equal(d.results.some(r=>r.status==='accepted'),false);assert.ok(d.results.some(r=>r.status==='rejected'));
 }
});
test('duplicates do not alter standing results; provenance includes stationary lead-in and exports reconstruct identically',()=>{
 const c=context(),d=new c.Standing({label:'1/4 mile',distanceM:402.336});const rows=launch(d,.04,i=>({gpsSynthetic:i===0?1:0})),result=d.results.find(r=>r.status==='accepted');assert.equal(result.synthetic,1);
 const twice=new c.Standing(d.interval);for(const r of rows){twice.update(r);twice.update(r)}assert.equal(twice.results[0].seconds,result.seconds);
 vm.runInContext(html.slice(html.indexOf('function runsToCSV('),html.indexOf('// End receiver-time rolling intervals.')),c);const csv=c.runsToCSV(rows).trim().split('\n'),header=csv[0].split(','),fields=csv.find(line=>line.startsWith('1/4 mile,accepted,')).split(','),out=Object.fromEntries(header.map((k,i)=>[k,fields[i]]));
 assert.equal(Number(out.seconds),result.seconds);assert.equal(out.start_convention,'stationary-fix-v1');assert.equal(out.rollout_m,'0');assert.equal(out.target_distance_m,'402.336');assert.equal(out.synthetic,'1');assert.equal(Number(out.launch_bracket_ms),result.launchBracketMs);
});
test('a finished run cannot repeat before a fresh stationary lead-in and an unfinished run is exported rejected',()=>{
 const c=context(),d=new c.Standing({label:'0-60 mph',hi:60*.44704});launch(d);assert.equal(d.results.filter(r=>r.status==='accepted').length,1);
 vm.runInContext(html.slice(html.indexOf('function runsToCSV('),html.indexOf('// End receiver-time rolling intervals.')),c);
 const rows=[];for(let i=0;i<100;i++)rows.push(row(i,Math.max(0,2*(i*.04-2))));const csv=c.runsToCSV(rows);assert.match(csv,/0-60 mph,rejected,Recording ended before completion/);
});


test('two-epoch launch confirmation is independent of small cadence jitter',()=>{
 const c=context();for(const secondMs of [380,400]){const d=new c.Standing({label:'0-60 mph',hi:60*.44704});for(let i=0;i<=6;i++)d.update(row(i,0,.2));d.update(row(7,.4,.2));d.update(row(8,.5,.2,{gpsUtcUs:2200000+secondMs*1000}));assert.equal(d.attempt?.confirmed,true)}
});

test('established heading remains mandatory after slowing below course-enrollment speed',()=>{for(const course of [null,2]){const c=context(),d=new c.Standing({label:'1/4 mile',distanceM:402.336});for(let i=0;i<=75;i++)d.update(row(i,Math.max(0,2*(i*.04-2))));assert.notEqual(d.attempt.course,null);d.update(row(76,1.9,.04,{gpsCourse:course}));assert.equal(d.attempt,null);assert.equal(d.results.at(-1).status,'rejected');assert.match(d.results.at(-1).reason,/course unavailable|Direction changed/);}});
