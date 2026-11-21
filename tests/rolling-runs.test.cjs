const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function context(){const c={Math,Number,Error};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('class RollingRuns{'),html.indexOf('const rollingRuns='))+';this.Runs=RollingRuns;',c);return c}
function row(i,speed,extra={}){return {rowType:1,gpsActive:1,gpsAccepted:1,sp:speed,gpsCourse:1,gpsUtcUs:1000000+i*40000,gpsDate:211126,gpsFractionDigits:2,gpsBootId:7,gpsSeq:i+1,gpsMissing:0,gpsSynthetic:0,...extra}}
function ramp(detector,extra=()=>({})){for(let i=0;i<220;i++)detector.update(row(i,18+i*.08,extra(i)))}
test('rolling interval uses interpolated receiver epochs, independent of browser batching',()=>{const c=context(),d=new c.Runs();ramp(d,()=>({t:9000}));const result=d.results.find(r=>r.status==='accepted');assert.ok(result);assert.ok(Math.abs(result.seconds-5.588)<1e-9);assert.equal(result.synthetic,0)});
test('gaps, reboots, date changes, invalid fixes and course deviation cannot complete a run',()=>{for(const bad of [{gpsSeq:900},{gpsBootId:8},{gpsDate:221126},{gpsAccepted:0},{gpsCourse:2},{gpsUtcUs:null}]){const c=context(),d=new c.Runs();ramp(d,i=>i===60?bad:{});assert.equal(d.results.filter(r=>r.status==='accepted').length,0);assert.ok(d.results.some(r=>r.status==='rejected'))}});
test('duplicate epochs cannot shorten a run and non-increasing UTC aborts it',()=>{const c=context(),d=new c.Runs();for(let i=0;i<220;i++){const r=row(i,18+i*.08);d.update(r);d.update(r)}assert.ok(Math.abs(d.results[0].seconds-5.588)<1e-9);const b=new c.Runs();ramp(b,i=>i===60?{gpsUtcUs:1000000}:{});assert.equal(b.results.some(r=>r.status==='accepted'),false)});
test('a brief finish-threshold spike is not a completed interval',()=>{const c=context(),d=new c.Runs();for(let i=0;i<220;i++){let speed=Math.min(18+i*.08,70*.44704-.2);if(i===180)speed=70*.44704+.01;d.update(row(i,speed))}assert.equal(d.results.some(r=>r.status==='accepted'),false)});
test('starting above the lower threshold cannot invent the missing start',()=>{const c=context(),d=new c.Runs();for(let i=0;i<150;i++)d.update(row(i,22+i*.08));assert.equal(d.results.length,0)});
test('synthetic provenance survives a complete estimate',()=>{const c=context(),d=new c.Runs();ramp(d,()=>({gpsSynthetic:1}));assert.equal(d.results[0].synthetic,1)});
test('Runs CSV is derived from saved GPS evidence and reports rejected attempts',()=>{
 const c=context();vm.runInContext(html.slice(html.indexOf('function runsToCSV('),html.indexOf('// End receiver-time rolling intervals.')),c);
 const rows=Array.from({length:220},(_,i)=>row(i,18+i*.08,{gpsSynthetic:1}));
 const csv=c.runsToCSV(rows).trim().split('\n');assert.equal(csv.length,2);assert.match(csv[1],/^45-70 mph,accepted,/);assert.ok(Math.abs(Number(csv[1].split(',')[3])-5.588)<1e-9);assert.equal(csv[1].split(',').at(-1),'1');
 rows[60]={...rows[60],gpsAccepted:0};assert.match(c.runsToCSV(rows),/rejected,GPS evidence incomplete/);
 assert.throws(()=>c.runsToCSV([]),/No 45–70 mph attempts/);
});
