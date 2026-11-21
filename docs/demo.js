/* Deterministic synthetic driving session. Not recorded driving or a firmware simulation. */
(()=>{
 const origin={lat:37,lon:-122},tau=2*Math.PI,steps=720;
 const route=Array.from({length:steps+1},(_,i)=>{
  const a=i/steps*tau;
  return {x:140*Math.cos(a)+35*Math.cos(2*a),y:95*Math.sin(a)-20*Math.sin(2*a),a};
 });
 // Integrate distance / speed rather than moving a point at constant angular speed.
 // Smooth sector pace changes produce useful gains and losses without position jumps.
 const paces=[1,1.025,.985,1.01];
 const speedAt=(a,pace,lap)=>{
  const dx=-140*Math.sin(a)-70*Math.sin(2*a),dy=95*Math.cos(a)-40*Math.cos(2*a);
  const ddx=-140*Math.cos(a)-140*Math.cos(2*a),ddy=-95*Math.sin(a)+80*Math.sin(2*a);
  const curvature=Math.abs(dx*ddy-dy*ddx)/Math.hypot(dx,dy)**3;
  const desired=(17+5*Math.sin(a+.4)+3*Math.cos(3*a))*(1+((pace-1)+.035*Math.sin(2*a+lap))*Math.sin(a/2)**2);
  const cornerLimit=Math.sqrt(5/Math.max(curvature,.0001));
  return (desired**-4+cornerLimit**-4)**-.25;
 };
 const laps=paces.map((pace,lap)=>{
  let distance=0,time=0;
  return route.map((point,i)=>{
   const speed=speedAt(point.a,pace,lap);
   if(i){const previous=route[i-1],ds=Math.hypot(point.x-previous.x,point.y-previous.y);distance+=ds;time+=ds/((speed+speedAt(previous.a,pace,lap))/2)*1000}
   return {...point,speed,distance,time};
  });
 });
 const durations=laps.map(lap=>lap[steps].time),superPeriod=durations.reduce((a,b)=>a+b,0);
 const start=performance.now()-(durations[0]-5000);
 let revision=0,armedCycle=null,paused=false;
 const json=data=>Promise.resolve(new Response(JSON.stringify(data),{headers:{'Content-Type':'application/json'}}));
 function clock(elapsed){
  let cycle=Math.floor(elapsed/superPeriod)*laps.length,lapMs=elapsed%superPeriod,index=0;
  while(index<laps.length-1&&lapMs>=durations[index]){lapMs-=durations[index];index++;cycle++}
  return {cycle,index,lapMs};
 }
 function motion(index,ms){
  const lap=laps[index];let lo=0,hi=steps;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(lap[mid].time<ms)lo=mid;else hi=mid}
  const p=lap[lo],q=lap[hi],f=Math.max(0,Math.min(1,(ms-p.time)/(q.time-p.time))),a=p.a+(q.a-p.a)*f;
  const dx=-140*Math.sin(a)-70*Math.sin(2*a),dy=95*Math.cos(a)-40*Math.cos(2*a);
  const ddx=-140*Math.cos(a)-140*Math.cos(2*a),ddy=-95*Math.sin(a)+80*Math.sin(2*a),norm=Math.hypot(dx,dy);
  const speed=p.speed+(q.speed-p.speed)*f,curvature=(dx*ddy-dy*ddx)/(norm**3);
  return {east:p.x+(q.x-p.x)*f,north:p.y+(q.y-p.y)*f,speed,yaw:Math.atan2(dx,dy),vx:speed*dx/norm,vy:speed*dy/norm,lonAccel:(q.speed-p.speed)/((q.time-p.time)/1000),latAccel:-speed*speed*curvature,yawRate:-speed*curvature};
 }
 window.fetch=(input,options={})=>{
  const url=new URL(String(input),location.href),elapsed=performance.now()-start,c=clock(elapsed);
  if(options.signal?.aborted)return Promise.reject(new DOMException('Aborted','AbortError'));
  if(paused)return Promise.reject(new Error('Simulated connection loss'));
  if(url.pathname==='/api/laptimer/configure'){
   if(!['loop','clear'].includes(url.searchParams.get('type')))return json({error:'This demo supports circuit timing only. Stage timing requires hardware validation.'});
   revision++;armedCycle=url.searchParams.get('type')==='clear'?null:c.cycle+1;
   return json({ok:true,revision});
  }
  const m=motion(c.index,c.lapMs),lat=origin.lat+m.north/111320,lon=origin.lon+m.east/(111320*Math.cos(origin.lat*Math.PI/180));
  const timing=armedCycle!==null&&c.cycle>=armedCycle,count=timing?c.cycle-armedCycle:0;
  const completed=Array.from({length:Math.min(count,4)},(_,i)=>durations[(c.cycle-1-i)%4]);
  if(url.pathname==='/api/telemetry'){
   const data=new Uint8Array(82),v=new DataView(data.buffer);v.setUint8(0,4);v.setUint16(1,0xAA55,true);v.setUint32(3,Math.floor(elapsed),true);
   [-m.lonAccel,m.latAccel,0,m.yawRate,0,0,m.yaw,m.east,m.north,m.vx,m.vy,m.speed*3.6,2.5].forEach((n,i)=>v.setFloat32(7+4*i,n,true));
   const turning=Math.abs(m.latAccel)>1.2,mode=(turning?4:0)|(m.lonAccel>.35?1:m.lonAccel<-.35?2:0);
   v.setUint8(59,mode);v.setFloat32(60,lat,true);v.setFloat32(64,lon,true);v.setUint8(68,1);v.setFloat32(69,(m.yaw+tau)%tau,true);
   v.setUint32(73,timing?Math.floor(c.lapMs):0,true);v.setUint16(77,count,true);v.setUint16(80,data.slice(0,80).reduce((a,b)=>a+b,0)&65535,true);
   return json({seq:Math.floor(elapsed/40),data:btoa(String.fromCharCode(...data)),timing:{state:armedCycle===null?'idle':timing?'timing':'armed',revision,last_lap_ms:count?Math.round(completed[0]):null,best_lap_ms:count?Math.round(Math.min(...completed)):null,lat,lon,gps_valid:true,gps_age_ms:0},f:{li:m.lonAccel,lg:m.lonAccel,gw:.3,gr:25,pc:0,pf:95,rc:0,rf:95,tx:0,ty:0}});
  }
  if(url.pathname==='/api/mount-calibration')return json({state:'unavailable',message:'Mount alignment needs a real device and fresh GPS/IMU evidence. The demo does not simulate a successful calibration.',saved:{is_calibrated:false,yaw_offset:0},candidate_degrees:null,samples:0,parked:false,storage_available:false});
  if(url.pathname==='/api/diagnostics')return json({sensor_rates:{imu_hz:200,gps_hz:25,loop_hz:900,imu_expected:200,gps_expected:25,zupt_per_min:0,ekf_per_gps:8},ekf:{pos_sigma:2.5,vel_sigma:.2,yaw_sigma_deg:2,bias_x:0,bias_y:0},system:{heap_free:100000,uptime_s:Math.floor(elapsed/1000),tx_ok:100,tx_fail:0},gps:{model:'SIMULATED',rate_hz:25,fix:true,warmup:true,satellites:12,hdop:1,ref_lat:origin.lat,ref_lon:origin.lon},wifi:{mode:'DEMO',ssid:'SIMULATED DATA'},config:{telemetry_hz:30,gps_model:'demo',warmup_fixes:5},fusion:{lon_raw:m.lonAccel,lon_filtered:m.lonAccel,lon_blended:m.lonAccel,gps_weight:.3,gps_accel:m.lonAccel,gps_rate:25,gps_rejected:false,pitch_corr:0,roll_corr:0,pitch_conf:95,roll_conf:95,yaw_bias:0,yaw_calibrated:true,tilt_x:0,tilt_y:0,tilt_valid:true}});
  return Promise.resolve(new Response('{"error":"Unavailable in demo"}',{status:404}));
 };
 document.addEventListener('DOMContentLoaded',async()=>{
  document.getElementById('demo-connection').onclick=event=>{paused=!paused;event.target.textContent=paused?'Restore connection':'Simulate disconnect'};
  try{
   await openTrackDB();
   const id='demo-circuit-v3',track={id,name:'Demo circuit · simulated',type:'loop',gpsOrigin:origin,startLine:{p1:[155,0],p2:[195,0],direction:Math.PI/2},centerline:route.map(p=>({x:p.x,y:p.y})),pathLength:laps[0][steps].distance,lapCount:0,bestLapMs:Math.round(durations[0]),createdAt:Date.now(),quality:'good'};
   const previous=(await getAllTracks()).find(item=>item.id===id);if(previous){track.bestLapMs=previous.bestLapMs;track.lapCount=previous.lapCount}
   if(!(await getRefLap(id))?.spatial){
    const pairs=new Float32Array(laps[0].flatMap(p=>[p.distance,p.time]));
    await saveRefLap({id:'demo-reference-v3',trackId:id,lapTimeMs:Math.round(durations[0]),created:Date.now(),samples:pairs.buffer,sampleCount:steps+1,totalDistance:track.pathLength,spatial:{version:1,closed:true,origin,points:laps[0].map(p=>({x:p.x,y:p.y,t:Math.min(p.time,Math.round(durations[0]))}))}});
   }
   await saveTrack(track);await activateTrack(track);
  }catch(error){document.getElementById('demo-note').textContent='Simulated data. Browser storage unavailable; recording and saved tracks require storage access.';console.error(error)}
 });
})();
