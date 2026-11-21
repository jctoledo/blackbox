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
 // A distance-domain driver: curvature limits lateral load; backward braking
 // and forward traction passes anticipate corners rather than adding display jitter.
 const laps=paces.map((pace,lap)=>{
  const points=route.map(point=>{
   const a=point.a,dx=-140*Math.sin(a)-70*Math.sin(2*a),dy=95*Math.cos(a)-40*Math.cos(2*a);
   const ddx=-140*Math.cos(a)-140*Math.cos(2*a),ddy=-95*Math.sin(a)+80*Math.sin(2*a);
   const curvature=Math.abs(dx*ddy-dy*ddx)/Math.hypot(dx,dy)**3;
   return {...point,curvature,speed:Math.min(31,Math.sqrt(5/Math.max(curvature,.0001)))*(1+(pace-1)*Math.sin(a/2)**2)};
  });
  // Repeated cyclic passes also constrain the start/finish seam.
  for(let pass=0;pass<8;pass++){
   for(let i=steps-1;i>=0;i--){const p=points[i],q=points[i+1],ds=Math.hypot(q.x-p.x,q.y-p.y);p.speed=Math.min(p.speed,Math.sqrt(q.speed*q.speed+2*4.5*ds))}
   points[steps].speed=points[0].speed=Math.min(points[steps].speed,points[0].speed);
   for(let i=1;i<=steps;i++){const p=points[i-1],q=points[i],ds=Math.hypot(q.x-p.x,q.y-p.y);const lateral=p.speed*p.speed*p.curvature;const traction=2.4*Math.sqrt(Math.max(.04,1-(lateral/5.5)**2));q.speed=Math.min(q.speed,Math.sqrt(p.speed*p.speed+2*traction*ds))}
   points[steps].speed=points[0].speed=Math.min(points[steps].speed,points[0].speed);
  }
  let distance=0,time=0;
  return points.map((p,i)=>{if(i){const q=points[i-1],ds=Math.hypot(p.x-q.x,p.y-q.y);distance+=ds;time+=ds/((p.speed+q.speed)/2)*1000}return {...p,distance,time}});
 });
 const durations=laps.map(lap=>lap[steps].time),superPeriod=durations.reduce((a,b)=>a+b,0);
 const start=performance.now()-(durations[0]-5000);
 let revision=0,armedCycle=null,paused=false,intervalStart=null,demoBoot=1,demoTrack=null;
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
 // Synthetic residuals after filtering: deterministic texture, not measured sensor noise.
 function texture(ms){
  const t=ms/1000,rough=Math.exp(-(((t%18-9)/.7)**2));
  return {lon:.025*Math.sin(t*19)+.015*Math.sin(t*43)+rough*.08*Math.sin(t*31),lat:.02*Math.sin(t*23)+.01*Math.sin(t*37)+rough*.06*Math.sin(t*29)};
 }
 // Explicit menu-selected straight-line scenario, repeated without speed/position jumps.
 function intervalMotion(ms){
  const t=Math.max(0,ms)/1000,cycle=Math.floor(t/20),p=t%20;
  let speed,distance,accel;
  if(p<8){speed=18+2*p;distance=18*p+p*p;accel=2}
  else if(p<10){speed=34;distance=208+34*(p-8);accel=0}
  else if(p<18){const q=p-10;speed=34-2*q;distance=276+34*q-q*q;accel=-2}
  else{speed=18;distance=484+18*(p-18);accel=0}
  return {east:cycle*520+distance,north:0,speed,yaw:Math.PI/2,vx:speed,vy:0,lonAccel:accel,latAccel:0,yawRate:0};
 }
 function sampleMotion(elapsed){const c=clock(elapsed);return intervalStart===null?motion(c.index,c.lapMs):intervalMotion(elapsed-intervalStart)}
 window.fetch=(input,options={})=>{
  const url=new URL(String(input),location.href),elapsed=performance.now()-start,c=clock(elapsed);
  if(options.signal?.aborted)return Promise.reject(new DOMException('Aborted','AbortError'));
  if(paused)return Promise.reject(new Error('Simulated connection loss'));
  if(url.pathname==='/api/laptimer/configure'){
   if(!['loop','clear'].includes(url.searchParams.get('type')))return json({error:'This demo supports circuit timing only. Stage timing requires hardware validation.'});
   if(intervalStart!==null&&url.searchParams.get('type')!=='clear')return json({error:'Switch to the circuit demo before arming lap timing.'});
   revision++;armedCycle=url.searchParams.get('type')==='clear'?null:c.cycle+1;
   return json({ok:true,revision});
  }
  const m=sampleMotion(elapsed),lat=origin.lat+m.north/111320,lon=origin.lon+m.east/(111320*Math.cos(origin.lat*Math.PI/180));
  const timing=armedCycle!==null&&c.cycle>=armedCycle,count=timing?c.cycle-armedCycle:0;
  const completed=Array.from({length:Math.min(count,4)},(_,i)=>durations[(c.cycle-1-i)%4]);
  if(url.pathname==='/api/gps-log'){
   const base=intervalStart??0,latest=Math.floor((elapsed-base)/40)+1;
   const events=Array.from({length:Math.min(64,latest)},(_,i)=>{
    const seq=Math.max(1,latest-63)+i,ms=base+(seq-1)*40,point=sampleMotion(ms);
    return {seq,host_us:ms*1000,utc_us:(43200000000+ms*1000)%86400000000,date_ddmmyy:10126,fractional_digits:2,active:true,accepted:true,lat:origin.lat+point.north/111320,lon:origin.lon+point.east/(111320*Math.cos(origin.lat*Math.PI/180)),speed_mps:point.speed,course_rad:(point.yaw+tau)%tau};
   });
   return json({version:1,simulated:true,boot_id:demoBoot,capacity:64,latest_seq:latest,events});
  }
  if(url.pathname==='/api/telemetry'){
   const data=new Uint8Array(82),v=new DataView(data.buffer);v.setUint8(0,4);v.setUint16(1,0xAA55,true);v.setUint32(3,Math.floor(elapsed),true);
   const noise=texture(elapsed);
   [-m.lonAccel-noise.lon,m.latAccel+noise.lat,0,m.yawRate,0,0,m.yaw,m.east,m.north,m.vx,m.vy,m.speed*3.6,2.5].forEach((n,i)=>v.setFloat32(7+4*i,n,true));
   const turning=Math.abs(m.latAccel)>1.2,mode=(turning?4:0)|(m.lonAccel>.35?1:m.lonAccel<-.35?2:0);
   v.setUint8(59,mode);v.setFloat32(60,lat,true);v.setFloat32(64,lon,true);v.setUint8(68,1);v.setFloat32(69,(m.yaw+tau)%tau,true);
   v.setUint32(73,timing?Math.floor(c.lapMs):0,true);v.setUint16(77,count,true);v.setUint16(80,data.slice(0,80).reduce((a,b)=>a+b,0)&65535,true);
   return json({seq:Math.floor(elapsed/40),data:btoa(String.fromCharCode(...data)),timing:{state:armedCycle===null?'idle':timing?'timing':'armed',revision,last_lap_ms:count?Math.round(completed[0]):null,best_lap_ms:count?Math.round(Math.min(...completed)):null,lat,lon,gps_valid:true,gps_age_ms:0,imu_age_ms:0},f:{li:m.lonAccel,lg:m.lonAccel,gw:.3,gr:25,pc:0,pf:95,rc:0,rf:95,tx:0,ty:0}});
  }
  if(url.pathname==='/api/mount-calibration')return json({state:'unavailable',message:'Sensor-to-car alignment needs a real device and fresh GPS/IMU evidence. The demo does not simulate a successful calibration.',saved:{is_calibrated:false,yaw_offset:0},candidate_degrees:null,samples:0,parked:false,storage_available:false});
  if(url.pathname==='/api/diagnostics')return json({sensor_rates:{imu_hz:200,gps_hz:25,loop_hz:900,imu_expected:200,gps_expected:25,zupt_per_min:0,ekf_per_gps:8},ekf:{pos_sigma:2.5,vel_sigma:.2,yaw_sigma_deg:2,bias_x:0,bias_y:0},system:{heap_free:100000,uptime_s:Math.floor(elapsed/1000),tx_ok:100,tx_fail:0},gps:{model:'SIMULATED',rate_hz:25,fix:true,warmup:true,satellites:12,hdop:1,ref_lat:origin.lat,ref_lon:origin.lon},wifi:{mode:'DEMO',ssid:'SIMULATED DATA'},config:{telemetry_hz:30,gps_model:'demo',warmup_fixes:5},fusion:{lon_raw:m.lonAccel,lon_filtered:m.lonAccel,lon_blended:m.lonAccel,gps_weight:.3,gps_accel:m.lonAccel,gps_rate:25,gps_rejected:false,pitch_corr:0,roll_corr:0,pitch_conf:95,roll_conf:95,yaw_bias:0,yaw_calibrated:true,tilt_x:0,tilt_y:0,tilt_valid:true}});
  return Promise.resolve(new Response('{"error":"Unavailable in demo"}',{status:404}));
 };
 document.addEventListener('DOMContentLoaded',async()=>{
  document.getElementById('demo-interval').onclick=async event=>{
   intervalStart=intervalStart===null?Math.floor(performance.now()-start):null;demoBoot++;revision++;armedCycle=null;
   event.target.textContent=intervalStart===null?'Simulate rolling interval':'Return to circuit demo';
   document.getElementById('demo-note').textContent=intervalStart===null?'Synthetic circuit and sensor texture. No measured driving data.':'Synthetic straight-line 45–70 mph intervals. Record a session, then open Runs. No measured driving data.';
   document.getElementById('menu-overlay')?.classList?.remove('open');
   if(intervalStart===null&&demoTrack)await activateTrack(demoTrack);
   else if(typeof deactivateTrack==='function')await deactivateTrack();
  };
  document.getElementById('demo-connection').onclick=event=>{paused=!paused;event.target.textContent=paused?'Restore connection':'Simulate disconnect'};
  try{
   await openTrackDB();
   const id='demo-circuit-v4',track={id,name:'Demo circuit · simulated',type:'loop',gpsOrigin:origin,startLine:{p1:[155,0],p2:[195,0],direction:Math.PI/2},centerline:route.map(p=>({x:p.x,y:p.y})),pathLength:laps[0][steps].distance,lapCount:0,bestLapMs:Math.round(durations[0]),createdAt:Date.now(),quality:'good'};
   const previous=(await getAllTracks()).find(item=>item.id===id);if(previous){track.bestLapMs=previous.bestLapMs;track.lapCount=previous.lapCount}
   if(!(await getRefLap(id))?.spatial){
    const pairs=new Float32Array(laps[0].flatMap(p=>[p.distance,p.time]));
    await saveRefLap({id:'demo-reference-v4',trackId:id,lapTimeMs:Math.round(durations[0]),created:Date.now(),samples:pairs.buffer,sampleCount:steps+1,totalDistance:track.pathLength,spatial:{version:1,closed:true,origin,points:laps[0].map(p=>({x:p.x,y:p.y,t:Math.min(p.time,Math.round(durations[0]))}))}});
   }
   demoTrack=track;await saveTrack(track);await activateTrack(track);
  }catch(error){document.getElementById('demo-note').textContent='Simulated data. Browser storage unavailable; recording and saved tracks require storage access.';console.error(error)}
 });
})();
