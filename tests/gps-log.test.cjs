const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const section=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function context(){const c={Number,Math,Date,console};vm.createContext(c);vm.runInContext(section('function gpsRows(', 'function updateRecUI()')+section('function samplesToCSV(', 'async function exportCSV(')+section('function recordingHealth(', 'function lapConsistency('),c);return c;}
function batch(sequences,boot=7){return{version:1,simulated:false,boot_id:boot,capacity:64,latest_seq:sequences.at(-1)||0,events:sequences.map(seq=>({seq,host_us:seq*40000,utc_us:seq*40000,date_ddmmyy:201126,fractional_digits:2,active:true,accepted:true,lat:37,lon:-122,speed_mps:10,course_rad:1}))};}
test('GPS batches deduplicate, count overwritten events, and split reboot boundaries',()=>{
 const c=context(),first=c.gpsRows(batch([1,2,3]),null,1000);assert.equal(first.rows.length,3);assert.equal(first.rows[0].gpsMissing,null);assert.equal(first.rows[0].gpsBoundary,1);
 assert.equal(c.gpsRows(batch([1,2,3]),first.cursor,1100).rows.length,0);
 const gap=c.gpsRows(batch([8,9]),first.cursor,2000);assert.equal(gap.rows[0].gpsMissing,4);
 const reboot=c.gpsRows(batch([1],8),gap.cursor,3000);assert.equal(reboot.rows[0].gpsBoundary,2);assert.equal(reboot.rows[0].gpsMissing,null);
});
test('bad GPS snapshots fail before consuming their cursor',()=>{
 const c=context();for(const mutate of [b=>b.events[1].seq=1,b=>b.events[1].host_us=NaN,b=>b.events[0].lat='bad',b=>b.events[0].utc_us=86401000000,b=>b.latest_seq=99]){const b=batch([1,2]);mutate(b);assert.throws(()=>c.gpsRows(b,{boot:7,seq:0},1000));}
});
test('GPS and session CSV remain separate and preserve missing clocks, zeros and provenance',()=>{
 const c=context(),b=batch([1]);b.simulated=true;b.events[0].utc_us=null;b.events[0].speed_mps=0;const rows=c.gpsRows(b,null,1000).rows;
 const all=[{t:900,sp:1,gpsOk:1},...rows];const csv=c.gpsSamplesToCSV(all).trim().split('\n');assert.equal(csv.length,2);
 const result=Object.fromEntries(csv[0].split(',').map((k,i)=>[k,csv[1].split(',')[i]]));assert.equal(result.receiver_utc_us_of_day,'');assert.equal(result.speed_mps,'0');assert.equal(result.synthetic,'1');
 assert.equal(c.samplesToCSV(all).trim().split('\n').length,2);assert.equal(c.recordingHealth([{chunkIndex:0,data:all}]).samples,1);
});
test('stopping waits for the in-flight GPS fetch and saves its tail in the same session',async()=>{
 let deliver;const saved=[];const c={console,Math,Number,Date,performance,MAX_PENDING_SAMPLES:6000,CHUNK_INTERVAL:5000,db:{},requestDevice:()=>new Promise(resolve=>{deliver=resolve})};vm.createContext(c);
 vm.runInContext(section('// Recording state.', 'function updateRecUI()')+';function updateRecUI(){};rec=true;sessionId=10;sessionStart=100;lastSaveMono=performance.now();',c);
 c.commitStore=async(store,value)=>saved.push({store,value});
 c.patchRecordingSession=async(database,id,patch)=>saved.push({store:'sessions',value:{sessionId:id,...patch}});
 const pending=c.collectGpsLog(),stop=c.stopRecording();
 assert.equal(vm.runInContext('rec',c),false);
 deliver({ok:true,json:async()=>batch([1,2])});await pending;await stop;
 const chunk=saved.find(s=>s.store==='chunks').value;assert.equal(chunk.sessionId,10);assert.equal(chunk.data.length,2);assert.equal(chunk.data[1].gpsSeq,2);
 assert.equal(vm.runInContext('rec',c),false);assert.equal(saved.at(-1).value.status,'complete');
});
test('old firmware and failed GPS requests do not stop ordinary recording or invent data',async()=>{
 const c={console:{warn(){}},Math,Number,Date,db:{},requestDevice:async()=>({ok:false})};vm.createContext(c);
 vm.runInContext(section('// Recording state.', 'function updateRecUI()')+';function updateRecUI(){};rec=true;sessionId=10;',c);
 await c.collectGpsLog();assert.equal(vm.runInContext('rec',c),true);assert.equal(vm.runInContext('chunkBuffer.length',c),0);assert.match(vm.runInContext('gpsLogStatus',c),/unavailable/);
});
test('cursor pages drain retained history and expose unrecoverable gaps',async()=>{
 const c={console,Math,Number,Date,performance,MAX_PENDING_SAMPLES:6000,CHUNK_INTERVAL:5000,db:{}};vm.createContext(c);
 vm.runInContext(section('// Recording state.', 'function updateRecUI()')+';function updateRecUI(){};rec=true;sessionId=10;gpsCursor={boot:7,seq:10};lastSaveMono=performance.now();',c);
 const paths=[];let page=0;c.requestDevice=async path=>{paths.push(path);const b=page++===0?batch([20,21]):batch([22,23]);b.has_more=page===1;return{ok:true,json:async()=>b}};
 await c.collectGpsLog();assert.equal(paths[0],'/api/gps-log?boot=7&after=10');assert.equal(paths[1],'/api/gps-log?boot=7&after=21');
 assert.equal(vm.runInContext('chunkBuffer.length',c),4);assert.equal(vm.runInContext('chunkBuffer[0].gpsMissing',c),9);assert.match(vm.runInContext('gpsLogStatus',c),/9 events could not be recovered/);
});

test('GPS elevation survives ingestion and both CSV formats, with blanks for old firmware',()=>{
 const c=context();
 for(const elevation of [0,-12.5,1234.5,null,undefined]){
  const b=batch([1]);if(elevation!==undefined)b.events[0].altitude_msl_m=elevation;
  const row=c.gpsRows(b,null,1000).rows[0];assert.equal(row.gpsElevationM,elevation??null);
  for(const csv of [c.gpsSamplesToCSV([row]),c.samplesToCSV([{...row,rowType:0}])]){
   const [header,data]=csv.trim().split('\n'),index=header.split(',').indexOf('gps_elevation_m');
   assert.ok(index>=0);assert.equal(data.split(',')[index],elevation==null?'':String(elevation));
  }
 }
 for(const elevation of ['10',NaN,Infinity]){const b=batch([1]);b.events[0].altitude_msl_m=elevation;assert.throws(()=>c.gpsRows(b,null,1000),/elevation/)}
});
