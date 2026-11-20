/* Synthetic telemetry for the public demo only. Never included in device firmware. */
(()=>{
 const start=performance.now()-24000,period=30000,origin={lat:37,lon:-122};
 let revision=0,armedCycle=null,paused=false;
 const json=data=>Promise.resolve(new Response(JSON.stringify(data),{headers:{'Content-Type':'application/json'}}));
 window.fetch=(input,options={})=>{
  const url=new URL(String(input),location.href),elapsed=performance.now()-start;
  if(options.signal?.aborted)return Promise.reject(new DOMException('Aborted','AbortError'));
  if(url.pathname==='/api/laptimer/configure'){
   revision++;armedCycle=url.searchParams.get('type')==='clear'?null:Math.floor(elapsed/period)+1;
   return json({ok:true,revision});
  }
  if(paused)return Promise.reject(new Error('Simulated connection loss'));
  const t=elapsed/period*2*Math.PI,radius=90,speed=radius*2*Math.PI/(period/1000);
  const east=radius*Math.cos(t),north=radius*Math.sin(t),lat=origin.lat+north/111320,lon=origin.lon+east/(111320*Math.cos(origin.lat*Math.PI/180));
  const cycle=Math.floor(elapsed/period),timing=armedCycle!==null&&cycle>=armedCycle;
  const laps=timing?cycle-armedCycle:0,lapMs=timing?Math.floor(elapsed%period):0;
  if(url.pathname==='/api/telemetry'){
   const data=new Uint8Array(82),v=new DataView(data.buffer);v.setUint8(0,4);v.setUint16(1,0xAA55,true);v.setUint32(3,Math.floor(elapsed),true);
   [0,speed*speed/radius,0,2*Math.PI/30,0,0,t+Math.PI/2,east,north,-speed*Math.sin(t),speed*Math.cos(t),speed*3.6,2.5].forEach((n,i)=>v.setFloat32(7+4*i,n,true));
   v.setUint8(59,4);v.setFloat32(60,lat,true);v.setFloat32(64,lon,true);v.setUint8(68,1);v.setFloat32(69,(-t%(2*Math.PI)+2*Math.PI)%(2*Math.PI),true);
   v.setUint32(73,lapMs,true);v.setUint16(77,laps,true);v.setUint16(80,data.slice(0,80).reduce((a,b)=>a+b,0)&65535,true);
   return json({seq:Math.floor(elapsed/40),data:btoa(String.fromCharCode(...data)),timing:{state:armedCycle===null?'idle':timing?'timing':'armed',revision,last_lap_ms:laps?period:null,best_lap_ms:laps?period:null,lat,lon,gps_valid:true,gps_age_ms:0},f:{li:0,lg:0,gw:.3,gr:25,pc:0,pf:95,rc:0,rf:95,tx:0,ty:0}});
  }
  if(url.pathname==='/api/diagnostics')return json({sensor_rates:{imu_hz:200,gps_hz:25,loop_hz:900,imu_expected:200,gps_expected:25,zupt_per_min:0,ekf_per_gps:8},ekf:{pos_sigma:2.5,vel_sigma:.2,yaw_sigma_deg:2,bias_x:0,bias_y:0},system:{heap_free:100000,uptime_s:Math.floor(elapsed/1000),tx_ok:100,tx_fail:0},gps:{model:'SIMULATED',rate_hz:25,fix:true,warmup:true,satellites:12,hdop:1,ref_lat:origin.lat,ref_lon:origin.lon},wifi:{mode:'DEMO',ssid:'SIMULATED DATA'},config:{telemetry_hz:30,gps_model:'demo',warmup_fixes:5},fusion:{lon_raw:0,lon_filtered:0,lon_blended:0,gps_weight:.3,gps_accel:0,gps_rate:25,gps_rejected:false,pitch_corr:0,roll_corr:0,pitch_conf:95,roll_conf:95,yaw_bias:0,yaw_calibrated:true,tilt_x:0,tilt_y:0,tilt_valid:true}});
  return Promise.resolve(new Response('{"error":"Unavailable in demo"}',{status:404}));
 };
 document.addEventListener('DOMContentLoaded',async()=>{
  document.getElementById('demo-connection').onclick=event=>{paused=!paused;event.target.textContent=paused?'Restore connection':'Simulate disconnect'};
  try{
   await openTrackDB();
   const track={id:'demo-circuit',name:'Demo circuit · simulated',type:'loop',gpsOrigin:origin,startLine:{p1:[70,0],p2:[110,0],direction:Math.PI/2},centerline:Array.from({length:120},(_,i)=>({x:90*Math.cos(i*Math.PI/60),y:90*Math.sin(i*Math.PI/60)})),pathLength:2*Math.PI*90,lapCount:0,bestLapMs:null,createdAt:Date.now(),quality:'good'};
   await saveTrack(track);await activateTrack(track);
  }catch(error){document.getElementById('demo-note').textContent='Simulated data. Browser storage unavailable; recording and saved tracks require storage access.';console.error(error)}
 });
})();
