const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(start,end)=>html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));
function fixture(){
 let wall=4000000,mono=0,sequence=0;const chunks=[];
 class ClockDate extends Date{static now(){return wall}}
 const c={console,Math,Number,Date:ClockDate,performance:{now:()=>mono},MAX_PENDING_SAMPLES:6000,CHUNK_INTERVAL:5000,db:{},d:{getUint32:()=>mono},timing:{gps_event:null,gps_boot_id:1},fusion:{}};
 for(const key of ['lapTimeMs','lapCnt','lapFlags','sp','ax','ay','wz','mo','latg','lng','lat','lon','gpsOk','ekfX','ekfY','ekfYaw','gpsCourse'])c[key]=0;
 c.requestDevice=async()=>({ok:true,json:async()=>{const events=Array.from({length:25},()=>{const seq=++sequence;return{seq,host_us:seq*40000,utc_us:seq*40000,date_ddmmyy:211126,fractional_digits:2,active:true,accepted:true,lat:37,lon:-122,speed_mps:10,course_rad:0}});return{version:1,simulated:false,boot_id:1,capacity:64,latest_seq:sequence,events}}});
 vm.createContext(c);vm.runInContext(slice('// Recording state.','function updateRecUI()')+'function updateRecUI(){};rec=true;sessionId=10;',c);
 c.commitStore=async(store,value)=>chunks.push(value);
 const telemetry=()=>{c.now=wall;vm.runInContext(slice('    // Recording\n    if(rec){','\n}\n\n// Read the complete bounded body'),c)};
 const settle=async()=>{for(let i=0;i<6;i++)await Promise.resolve()};
 return{c,chunks,telemetry,settle,clock(m,w){mono=m;wall=w}};
}
test('wall rollback cannot starve five-second saves or stop a healthy mixed recording at the pending cap',async()=>{
 const f=fixture();f.telemetry();await f.settle();assert.equal(f.chunks.length,1);
 for(let tick=1;tick<=5400;tick++){
  const mono=tick*1000/30;f.clock(mono,4000000-3600000+Math.floor(mono));f.telemetry();
  if(tick%30===0)await f.c.collectGpsLog();await f.settle();
 }
 assert.equal(vm.runInContext('rec',f.c),true,'wall rollback must not cause unsaved-buffer overflow');
 assert.ok(f.chunks.length>=35,'flushes must follow elapsed monotonic time');
 assert.ok(vm.runInContext('chunkBuffer.length',f.c)<300);
 const saved=f.chunks.flatMap(chunk=>chunk.data);assert.ok(saved.some(row=>row.t<4000000),'raw receipt wall timestamps must remain unchanged');assert.ok(saved.some(row=>row.rowType===1));
 assert.equal(saved.length+vm.runInContext('chunkBuffer.length',f.c),9901,'every incoming row must remain saved or pending');
});
test('forward wall correction cannot cause an early automatic save',async()=>{
 const f=fixture();f.telemetry();await f.settle();assert.equal(f.chunks.length,1);
 f.clock(1000,4000000+86400000);f.telemetry();await f.settle();assert.equal(f.chunks.length,1);
 f.clock(5001,4000000+86404001);f.telemetry();await f.settle();assert.equal(f.chunks.length,2);
});
test('GPS-only collection follows the same monotonic cadence after rollback',async()=>{
 const f=fixture();await f.c.collectGpsLog();assert.equal(f.chunks.length,1);
 for(let second=1;second<=6;second++){f.clock(second*1000,400000+second*1000);await f.c.collectGpsLog()}
 assert.equal(f.chunks.length,2);assert.equal(vm.runInContext('rec',f.c),true);
 assert.equal(f.chunks[1].data.length,125);assert.equal(vm.runInContext('chunkBuffer.length',f.c),25);
});
test('save age starts at completed commit and ignores wall corrections',async()=>{
 const f=fixture();let finish;f.c.commitStore=()=>new Promise(resolve=>finish=resolve);f.telemetry();await f.settle();
 f.clock(10000,400000);finish();await f.settle();assert.equal(f.c.recordingSaveAgeMs(),0);assert.equal(f.c.recordingSaveDue(),false);
 f.clock(11000,90400000);assert.equal(f.c.recordingSaveAgeMs(),1000);assert.equal(f.c.recordingSaveDue(),false);
 f.clock(15000,400001);assert.equal(f.c.recordingSaveAgeMs(),5000);assert.equal(f.c.recordingSaveDue(),true);
});
test('starting a new recording resets completed-save age and makes its first sample promptly eligible',async()=>{
 const f=fixture();f.telemetry();await f.settle();assert.equal(f.c.recordingSaveAgeMs(),0);
 vm.runInContext(slice('// Drive session core.','// End Drive session core.')+';rec=false;',f.c);
 f.c.createRecordingSession=async()=>20;f.c.captureRecordingMetadata=async()=>{};await f.c.startRecording();
 assert.equal(vm.runInContext('rec',f.c),true);assert.equal(vm.runInContext('sessionId',f.c),20);assert.equal(f.c.recordingSaveAgeMs(),null);assert.equal(f.c.recordingSaveDue(),true);
});
