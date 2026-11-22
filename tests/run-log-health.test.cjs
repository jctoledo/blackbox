const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(start,end)=>html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));
function setup(){
 const elements=new Map(),c={console:{warn(){}},Number,Math,Date,Promise,performance:{now:()=>c.now},now:100,rec:true,recordingBusy:false,sessionId:7,gpsCursor:null,gpsRequest:null,gpsLogStatus:'Waiting for GPS timestamp log…',MAX_PENDING_SAMPLES:6000,chunkBuffer:[],chunkWrite:null,recordingContext:null,recordingSaveDue:()=>false,checkpointDrive(){},selectedRunDetector:{status:'Collecting interval · GPS time',attempt:{startUs:123},results:[{status:'accepted',seconds:1.25}]}};
 c.$=id=>{if(!elements.has(id))elements.set(id,{textContent:'',disabled:false,attrs:{},setAttribute(k,v){this.attrs[k]=v}});return elements.get(id)};
 c.updatePerformanceRuns=()=>{};c.updateRecUI=()=>c.renderPerformanceRuns();
 vm.createContext(c);
 vm.runInContext(slice('function gpsRows(','function gpsSamplesToCSV('),c);vm.runInContext(slice('function renderPerformanceRuns(','function runsToCSV('),c);
 c.resetGpsLogHealth?.(7);return{c,elements,status:()=>c.$('run-status').textContent};
}
function batch(seq=1){return{version:1,simulated:true,boot_id:1,capacity:64,latest_seq:seq,events:[{seq,host_us:seq*40000,utc_us:43200000000+seq*40000,date_ddmmyy:221126,fractional_digits:3,active:true,accepted:true,lat:37,lon:-122,speed_mps:18,course_rad:0}]}}
const response=value=>({ok:true,json:async()=>value});
test('GPS-log failure is visible in Runs while telemetry recording and accepted results survive',async()=>{
 const {c,status}=setup(),attempt=c.selectedRunDetector.attempt,results=JSON.stringify(c.selectedRunDetector.results);c.requestDevice=async()=>({ok:false});await c.collectGpsLog();
 assert.match(status(),/GPS.*unavailable/i);assert.equal(c.rec,true);assert.equal(c.selectedRunDetector.attempt,attempt);assert.equal(JSON.stringify(c.selectedRunDetector.results),results);assert.equal(c.$('run-last').textContent,'1.25 s');
});
test('empty or repeated GPS evidence becomes waiting instead of indefinite collection, then recovers',async()=>{
 const {c,status}=setup();c.requestDevice=async()=>response({version:1,simulated:true,boot_id:1,capacity:64,latest_seq:0,events:[]});await c.collectGpsLog();assert.match(status(),/waiting.*GPS/i);
 c.requestDevice=async()=>response(batch());await c.collectGpsLog();assert.match(status(),/Collecting interval/);assert.equal(c.chunkBuffer.length,1);
 c.now=3000;await c.collectGpsLog();assert.equal(c.chunkBuffer.length,1);assert.match(status(),/waiting.*fresh.*GPS/i);
 c.now=3100;c.requestDevice=async()=>response(batch(2));await c.collectGpsLog();assert.match(status(),/Collecting interval/);assert.equal(c.$('run-best').textContent,'1.25 s');
});
test('a late failed GPS request from an old recording cannot replace the new owner status',async()=>{
 const {c,status}=setup();let reject;c.requestDevice=()=>new Promise((_,failure)=>reject=failure);const request=c.collectGpsLog();c.sessionId=8;c.resetGpsLogHealth?.(8);c.gpsLogStatus='New recording is waiting for receiver evidence';reject(Error('old owner timeout'));await request;
 assert.equal(c.gpsLogStatus,'New recording is waiting for receiver evidence');assert.doesNotMatch(status(),/unavailable/i);
 c.requestDevice=async()=>response(batch());await c.collectGpsLog();assert.match(status(),/Collecting interval/);c.rec=false;c.renderPerformanceRuns();assert.doesNotMatch(status(),/collecting|unavailable|waiting/i);
});
