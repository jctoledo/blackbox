const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function context(){const c={console,Date,Math,Number,performance:{now:()=>100}};vm.createContext(c);const marker=html.includes('function validGpsCoordinates(')?'function validGpsCoordinates(':'class TrackRecorder{';vm.runInContext(slice(marker,'// Track Manager IndexedDB')+';this.Capture=TrackRecorder;'+slice('let jsGpsRef=', '// Self-test for GPS coordinate functions'),c);return c}
for(const origin of [{lat:0,lon:30},{lat:30,lon:0},{lat:0,lon:0}]){
 test('valid zero-valued coordinate supports origin and circuit capture '+JSON.stringify(origin),()=>{
  const c=context();for(let i=0;i<5;i++)c.updateJsGpsRef(origin.lat,origin.lon);assert.deepEqual(JSON.parse(JSON.stringify(c.getGpsRef())),origin);
  const r=new c.Capture(),axis=origin.lat===0?'east':'north';const point=m=>({x:9000,y:9000,lat:origin.lat+(axis==='north'?m/111320:0),lon:origin.lon+(axis==='east'?m/(111320*Math.cos(origin.lat*Math.PI/180)):0),speed:10,heading:0,valid:true,sigma:3});
  assert.equal(r.start(point(0),'loop',origin),true);assert.ok(Math.abs(r.startPos.x)<.01&&Math.abs(r.startPos.y)<.01,'capture must use its GPS frame rather than inconsistent fallback x/y');
  for(let m=5;m<=100;m+=5)r.addSample(point(m));for(let m=95;m>=0&&!r.loopDetected;m-=5)r.addSample(point(m));
  assert.ok(r.gpsDistance>=150);assert.equal(r.loopDetected,true);assert.equal(r.keyPoints[0].lat,origin.lat);assert.equal(r.keyPoints[0].lon,origin.lon);
 });
}
test('invalid or missing coordinates cannot enroll JS reference or become real zero GPS points',()=>{
 for(const bad of [{lat:null,lon:30},{lat:undefined,lon:30},{lat:NaN,lon:30},{lat:Infinity,lon:30},{lat:91,lon:30},{lat:30,lon:181},{lat:30,lon:undefined}]){
  const c=context();for(let i=0;i<5;i++)c.updateJsGpsRef(bad.lat,bad.lon);assert.equal(c.getGpsRef(),null);
  const r=new c.Capture();assert.equal(r.start({x:5,y:6,speed:10,...bad},'point_to_point',{lat:30,lon:30}),true);assert.equal(r.startPos.x,5);assert.equal(r.startPos.y,6);assert.equal(r.startGps,null);assert.equal(r.lastGps,null);assert.equal(r.keyPoints[0].lat,null);assert.equal(r.keyPoints[0].lon,null);assert.equal(r._gpsToLocal(bad.lat,bad.lon),null);
 }
});
test('missing GPS breaks distance evidence while finite EKF fallback can still be captured',()=>{
 const c=context(),r=new c.Capture(),origin={lat:30,lon:30},p=m=>({x:m,y:0,lat:30,lon:30+m/(111320*Math.cos(Math.PI/6)),speed:10,heading:0,valid:true});r.start(p(0),'point_to_point',origin);r.addSample(p(5));r.addSample(p(10));const before=r.gpsDistance;r.addSample({x:15,y:0,speed:10,valid:false});assert.equal(r.lastGps,null);assert.equal(r.gpsDistance,before);r.addSample(p(20));assert.equal(r.gpsDistance,before,'GPS must not bridge unavailable coordinates');r.addSample(p(25));assert.ok(Math.abs(r.gpsDistance-before-5)<.01);
});
test('explicit invalid-fix flag cannot turn finite coordinates into GPS evidence',()=>{
 const c=context(),r=new c.Capture();r.start({x:5,y:6,lat:0,lon:0,valid:false,speed:10},'point_to_point',{lat:0,lon:0});assert.equal(r.startGps,null);assert.equal(r.keyPoints[0].lat,null);assert.equal(r.startPos.x,5);assert.equal(r.startPos.y,6);
 const missing=new c.Capture();assert.equal(missing.start({lat:null,lon:null},'point_to_point',{lat:0,lon:0}),false);assert.equal(missing.recording,false);
});
test('firmware reference distinguishes initialized zero from an unknown placeholder and supports legacy nonzero origins',async()=>{
 for(const [gps,expected] of [[{ref_lat:0,ref_lon:0,warmup:false},null],[{ref_lat:0,ref_lon:0},null],[{ref_lat:0,ref_lon:0,warmup:true},{lat:0,lon:0}],[{ref_lat:0,ref_lon:30,warmup:true},{lat:0,lon:30}],[{ref_lat:30,ref_lon:0,warmup:true},{lat:30,lon:0}],[{ref_lat:30,ref_lon:30},{lat:30,lon:30}],[{ref_lat:30,ref_lon:30,warmup:false},null],[{ref_lat:Infinity,ref_lon:30,warmup:true},null],[{ref_lat:30,ref_lon:181,warmup:true},null]]){
  const c=context();c.requestDevice=async()=>({json:async()=>({gps})});await c.fetchFirmwareGpsRef();assert.deepEqual(JSON.parse(JSON.stringify(c.getGpsRef())),expected,JSON.stringify(gps));
 }
});
