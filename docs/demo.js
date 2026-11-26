/* Deterministic synthetic driving session. Not recorded driving or a firmware simulation. */
(()=>{
 const origin={lat:37,lon:-122},tau=2*Math.PI,steps=720;
 const route=Array.from({length:steps+1},(_,i)=>{
  const a=i/steps*tau;
  return {x:140*Math.cos(a)+35*Math.cos(2*a),y:95*Math.sin(a)+25*Math.sin(3*a),a};
 });
 // Integrate distance / speed rather than moving a point at constant angular speed.
 // Smooth sector pace changes produce useful gains and losses without position jumps.
 const paces=[1,.94,1.025,.985];
 // A distance-domain driver: curvature limits lateral load; backward braking
 // and forward traction passes anticipate corners rather than adding display jitter.
 function constrainSpeeds(points){
  // Repeated cyclic passes also constrain the start/finish seam.
  for(let pass=0;pass<8;pass++){
   for(let i=steps-1;i>=0;i--){const p=points[i],q=points[i+1],ds=Math.hypot(q.x-p.x,q.y-p.y);p.speed=Math.min(p.speed,Math.sqrt(q.speed*q.speed+2*4.5*ds))}
   points[steps].speed=points[0].speed=Math.min(points[steps].speed,points[0].speed);
   for(let i=1;i<=steps;i++){const p=points[i-1],q=points[i],ds=Math.hypot(q.x-p.x,q.y-p.y);const lateral=p.speed*p.speed*p.curvature;const traction=2.4*Math.sqrt(Math.max(.04,1-(lateral/5.5)**2));q.speed=Math.min(q.speed,Math.sqrt(p.speed*p.speed+2*traction*ds))}
   points[steps].speed=points[0].speed=Math.min(points[steps].speed,points[0].speed);
  }
 }
 const profiles=paces.map((pace,lap)=>{
  const points=route.map(point=>{
   const a=point.a,dx=-140*Math.sin(a)-70*Math.sin(2*a),dy=95*Math.cos(a)+75*Math.cos(3*a);
   const ddx=-140*Math.cos(a)-140*Math.cos(2*a),ddy=-95*Math.sin(a)-225*Math.sin(3*a);
   const curvature=Math.abs(dx*ddy-dy*ddx)/Math.hypot(dx,dy)**3;
   return {...point,curvature,speed:Math.min(31,Math.sqrt(5/Math.max(curvature,.0001)))*(1+(pace-1)*Math.sin(a/2)**2)};
  });
  constrainSpeeds(points);return points;
 });
 // Adjacent laps share one boundary speed even when their interior pace differs.
 for(let pass=0;pass<8;pass++){
  const seam=Math.min(...profiles.map(p=>p[0].speed));
  for(const points of profiles){points[0].speed=points[steps].speed=seam;constrainSpeeds(points)}
 }
 const laps=profiles.map(points=>{
  let distance=0,time=0;
  return points.map((p,i)=>{if(i){const q=points[i-1],ds=Math.hypot(p.x-q.x,p.y-q.y);distance+=ds;time+=ds/((p.speed+q.speed)/2)*1000}return {...p,distance,time}});
 });
 const durations=laps.map(lap=>lap[steps].time),superPeriod=durations.reduce((a,b)=>a+b,0);
 const start=performance.now()-(durations[0]-5000);
 let revision=0,armedCycle=null,paused=false,gpsLogPaused=false,intervalStart=null,standingScenario=false,demoBoot=1,demoTrack=null;
 const json=data=>Promise.resolve(new Response(JSON.stringify(data),{headers:{'Content-Type':'application/json','Cache-Control':'no-store'}}));
 function clock(elapsed){
  let cycle=Math.floor(elapsed/superPeriod)*laps.length,lapMs=elapsed%superPeriod,index=0;
  while(index<laps.length-1&&lapMs>=durations[index]){lapMs-=durations[index];index++;cycle++}
  return {cycle,index,lapMs};
 }
 function motion(index,ms){
  const lap=laps[index];let lo=0,hi=steps;
  while(hi-lo>1){const mid=(lo+hi)>>1;if(lap[mid].time<ms)lo=mid;else hi=mid}
  const p=lap[lo],q=lap[hi],f=Math.max(0,Math.min(1,(ms-p.time)/(q.time-p.time))),a=p.a+(q.a-p.a)*f;
  const dx=-140*Math.sin(a)-70*Math.sin(2*a),dy=95*Math.cos(a)+75*Math.cos(3*a);
  const ddx=-140*Math.cos(a)-140*Math.cos(2*a),ddy=-95*Math.sin(a)-225*Math.sin(3*a),norm=Math.hypot(dx,dy);
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
  const t=Math.max(0,ms)/1000,cycle=Math.floor(t/28),p=t%28;
  let speed,distance,accel;
  if(p<12){speed=10+2*p;distance=10*p+p*p;accel=2}
  else if(p<14){speed=34;distance=264+34*(p-12);accel=0}
  else if(p<26){const q=p-14;speed=34-2*q;distance=332+34*q-q*q;accel=-2}
  else{speed=10;distance=596+10*(p-26);accel=0}
  return {east:cycle*616+distance,north:0,speed,yaw:Math.PI/2,vx:speed,vy:0,lonAccel:accel,latAccel:0,yawRate:0};
 }
 function launchMotion(ms){
  const t=Math.max(0,ms)/1000,cycle=Math.floor(t/48),p=t%48;
  let speed=0,distance=0,accel=0;
  if(p>=2&&p<22){const q=p-2;speed=2*q;distance=q*q;accel=2}
  else if(p>=22&&p<24){speed=40;distance=400+40*(p-22)}
  else if(p>=24&&p<44){const q=p-24;speed=40-2*q;distance=480+40*q-q*q;accel=-2}
  else if(p>=44)distance=880;
  return {east:cycle*880+distance,north:0,speed,yaw:Math.PI/2,vx:speed,vy:0,lonAccel:accel,latAccel:0,yawRate:0};
 }
 function sampleMotion(elapsed){const c=clock(elapsed);return intervalStart===null?motion(c.index,c.lapMs):standingScenario?launchMotion(Math.max(0,elapsed-intervalStart-4000)):intervalMotion(elapsed-intervalStart)}
 // Example recordings use the same motion model and receiver-time evidence as the live demo.
 function exampleRows(kind){
  const rows=[],base=Date.UTC(2026,0,1,12),duration=kind==='circuit'?Math.floor(durations[0]/40)*40:kind==='launch'?144000:84000;
  let priorSeq=0;
  for(let ms=0;ms<=duration;ms+=40){
   const p=kind==='circuit'?motion(0,ms):kind==='launch'?launchMotion(ms):intervalMotion(ms),seq=ms/40+1;
   // One deliberate receiver-log gap in the middle interval exercises rejection/export.
   const missing=kind==='rolling'&&ms>=35000&&ms<35240||kind==='launch'&&ms>=58000&&ms<58240;
   const lat=origin.lat+p.north/111320,lon=origin.lon+p.east/(111320*Math.cos(origin.lat*Math.PI/180));
   rows.push({t:base+ms,deviceTimeMs:ms,gpsBootId:kind==='circuit'?101:kind==='launch'?103:102,sp:p.speed*3.6,ax:-p.lonAccel,ay:p.latAccel,wz:p.yawRate,latg:p.latAccel/9.81,lng:p.lonAccel/9.81,lat,lon,gpsOk:1,gpsAgeMs:0,imuAgeMs:0,gpsSynthetic:1,ekfX:p.east,ekfY:p.north,ekfYaw:p.yaw,gpsCourse:(p.yaw+tau)%tau});
   if(!missing){rows.push({t:base+ms,rowType:1,gpsSynthetic:1,gpsBootId:kind==='circuit'?101:kind==='launch'?103:102,gpsSeq:seq,gpsHostUs:ms*1000,gpsUtcUs:43200000000+ms*1000,gpsDate:10126,gpsFractionDigits:2,gpsActive:1,gpsAccepted:1,lat,lon,sp:p.speed,gpsCourse:(p.yaw+tau)%tau,gpsMissing:priorSeq?seq-priorSeq-1:0,gpsBoundary:priorSeq?0:1});priorSeq=seq}
  }
  return rows;
 }
 function recordingSnapshot(uptime=0,boot=demoBoot){
  return {version:1,simulated:true,boot_id:boot,captured_uptime_ms:Math.max(0,Math.floor(uptime)),build:{app_version:'SIM demo',idf_version:null,source_sha:null,target:'esp32c3'},gps:{configured_model:'SIMULATED',configured_rate_hz:25,configured_uart_baud:115200,configuration_verified:false},imu:{model:'WT901',transport:'uart',configured_uart_baud:null,baud_selection:'pending',configured_output_rate_hz:null},calibration:{accelerometer_bias_mps2:null,mount_alignment:null}};
 }
 function exampleMetadata(time,boot){return {status:'captured',requestedAt:time,receivedAt:time,snapshot:recordingSnapshot(0,boot)}}
 function exampleDrive(id,rows){
  const d=new DriveSession(id),gps=rows.filter(r=>r.rowType===1),wall=rows[0].t;
  d.apply({owner:id,type:'start',mono:0,wall});
  d.apply({owner:id,type:'gps',rows:gps.slice(0,64)});
  d.apply({owner:id,type:'gps',rows:gps.slice(64)});
  d.apply({owner:id,type:'stop',mono:Math.max(0,rows.at(-1).t-wall),wall:rows.at(-1).t});
  return d.snapshot();
 }
 async function seedExampleRecording(database,label,rows,fixtureId){
  const boot=rows.find(row=>row.gpsBootId!==undefined)?.gpsBootId??({'circuit-v6':101,'circuit-v7':101,'rolling-v7':102,'rolling-v8':102}[fixtureId]);
  return new Promise((resolve,reject)=>{
   const tx=database.transaction(['sessions','chunks'],'readwrite'),sessions=tx.objectStore('sessions');let id;
   tx.oncomplete=()=>resolve(id);tx.onerror=tx.onabort=()=>reject(tx.error||new Error('Demo example could not be saved'));
   const request=sessions.getAll();request.onsuccess=()=>{
    const existing=request.result.find(s=>s.demoFixture===fixtureId);if(existing){
     id=existing.sessionId;const metadata=existing.recordingMetadata,patch={...existing,drive:existing.drive??(rows.length?exampleDrive(id,rows):undefined)};
     if(!metadata)patch.recordingMetadata=exampleMetadata(existing.startTime,boot);
     else if(metadata.status==='captured'&&metadata.snapshot.simulated&&metadata.snapshot.boot_id!==boot)patch.recordingMetadata={...metadata,snapshot:{...metadata.snapshot,boot_id:boot}};
     sessions.put(patch);return;
    }
    id=request.result.reduce((next,s)=>Math.max(next,s.sessionId+1),Date.now());
    sessions.add({sessionId:id,startTime:rows[0].t,status:'complete',label,demoFixture:fixtureId,drive:exampleDrive(id,rows),recordingMetadata:exampleMetadata(rows[0].t,boot)});
    for(let offset=0;offset<rows.length;offset+=250)tx.objectStore('chunks').add({sessionId:id,chunkIndex:offset/250,data:rows.slice(offset,offset+250),timestamp:rows[Math.min(offset+249,rows.length-1)].t});
   };
  });
 }
 async function populateExamples(track){
  if(typeof openDB!=='function')return;
  const database=await openDB(),circuit=exampleRows('circuit'),rolling=exampleRows('rolling'),launch=exampleRows('launch');
  // New evidence-complete fixture IDs add examples without rewriting old/user rows.
  const circuitId=await seedExampleRecording(database,'SIM · Circuit drive',circuit,'circuit-v7');
  const rollingId=await seedExampleRecording(database,'SIM · Rolling drive + GPS gap',rolling,'rolling-v8');
  const launchId=await seedExampleRecording(database,'SIM · Standing starts + GPS gap',launch,'launch-v1');
  updatePerformanceRuns(rolling);
  updatePerformanceRuns(launch);
  // Warm the page-session force summary from the simulated lead-in, not invented peaks.
  const elapsed=Math.floor(performance.now()-start);
  for(let ms=0;ms<=elapsed;ms+=40){const c=clock(ms),p=motion(c.index,c.lapMs);effortPeaks.update(ms,p.lonAccel/9.81,sustainedEffortUsable(p.speed,1,0,0),p.latAccel/9.81)}
  if(typeof commitTrackWrite==='function')await commitTrackWrite('lap_history',tx=>{
   let at=Date.UTC(2026,0,1,12);
   durations.slice(0,3).forEach((lapTimeMs,i)=>{at+=Math.round(lapTimeMs);tx.objectStore('lap_history').put({id:'demo-history-v6-'+i,trackId:track.id,sessionId:'demo-history-v6',sessionStart:Date.UTC(2026,0,1,12),timestamp:at,lapTimeMs:Math.round(lapTimeMs),lapNumber:i+1})});
  });
  document.getElementById('demo-note').textContent='Synthetic circuit, rolling and standing-start examples. Saved GPS evidence includes deliberate gaps. No measured driving data.';
  return {circuitId,rollingId,launchId};
 }
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
  const completed=Array.from({length:Math.min(count,4)},(_,i)=>durations[((c.cycle-1-i)%4+4)%4]);
  if(url.pathname==='/api/gps-log'){
   if(gpsLogPaused)return Promise.resolve(new Response('{"error":"Synthetic GPS log paused"}',{status:503}));
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
   return json({seq:Math.floor(elapsed/40),data:btoa(String.fromCharCode(...data)),timing:{simulated:true,gps_boot_id:demoBoot,state:armedCycle===null?'idle':timing?'timing':'armed',revision,last_lap_ms:count?Math.round(completed[0]):null,best_lap_ms:count?Math.round(Math.min(...completed)):null,lat,lon,gps_valid:true,gps_age_ms:0,imu_age_ms:0},f:{li:m.lonAccel,lg:m.lonAccel,gw:.3,gr:25,pc:0,pf:95,rc:0,rf:95,tx:0,ty:0}});
  }
  if(url.pathname==='/api/mount-calibration')return json({state:'unavailable',message:'Sensor-to-car alignment needs a real device and fresh GPS/IMU evidence. The demo does not simulate a successful calibration.',saved:{is_calibrated:false,yaw_offset:0},candidate_degrees:null,samples:0,parked:false,storage_available:false});
  if(url.pathname==='/api/recording-metadata')return json(recordingSnapshot(elapsed));
  if(url.pathname==='/api/diagnostics')return json({sensor_rates:{imu_hz:200,gps_hz:25,loop_hz:900,imu_expected:200,gps_expected:25,zupt_per_min:0,ekf_per_gps:8},ekf:{pos_sigma:2.5,vel_sigma:.2,yaw_sigma_deg:2,bias_x:0,bias_y:0},system:{heap_free:100000,uptime_s:Math.floor(elapsed/1000),tx_ok:100,tx_fail:0},gps:{model:'SIMULATED',rate_hz:25,fix:true,warmup:true,satellites:12,hdop:1,ref_lat:origin.lat,ref_lon:origin.lon},wifi:{mode:'DEMO',ssid:'SIMULATED DATA'},config:{telemetry_hz:30,gps_model:'demo',warmup_fixes:5},fusion:{lon_raw:m.lonAccel,lon_filtered:m.lonAccel,lon_blended:m.lonAccel,gps_weight:.3,gps_accel:m.lonAccel,gps_rate:25,gps_rejected:false,pitch_corr:0,roll_corr:0,pitch_conf:95,roll_conf:95,yaw_bias:0,yaw_calibrated:true,tilt_x:0,tilt_y:0,tilt_valid:true}});
  return Promise.resolve(new Response('{"error":"Unavailable in demo"}',{status:404}));
 };
 function suppressDriveSuggestions(){
  // Explicit synthetic preview only; production detection is unchanged.
  if(typeof trackAutoDetector!=='undefined')trackAutoDetector.check=async()=>null;
  if(typeof showTrackDetectedToast==='function')showTrackDetectedToast=()=>{};
  if(typeof dismissActiveToast==='function')dismissActiveToast();
 }
 document.addEventListener('DOMContentLoaded',async()=>{
  const drivePreview=new URL(location.href).searchParams.get('view')==='drive';
  if(drivePreview)suppressDriveSuggestions();
  document.getElementById('demo-drive').onclick=async()=>{suppressDriveSuggestions();if(activeTrack)await deactivateTrack();document.getElementById('menu-overlay')?.classList?.remove('open');renderDrive()};
  async function switchRunScenario(standing){
   intervalStart=intervalStart!==null&&standingScenario===standing?null:Math.floor(performance.now()-start);standingScenario=standing;demoBoot++;revision++;armedCycle=null;
   document.getElementById('demo-interval').textContent=intervalStart!==null&&!standingScenario?'Return to circuit demo':'Simulate rolling interval';
   document.getElementById('demo-launch').textContent=intervalStart!==null&&standingScenario?'Return to circuit demo':'Simulate standing start';
   document.getElementById('demo-note').textContent=intervalStart===null?'Synthetic circuit and sensor texture. No measured driving data.':standing?'Synthetic stationary launches and quarter-mile estimates. Record a session, then open Runs. No measured driving data.':'Synthetic straight-line mph and km/h intervals. Record a session, then open Runs. No measured driving data.';
   document.getElementById('menu-overlay')?.classList?.remove('open');
   if(intervalStart===null&&demoTrack)await activateTrack(demoTrack);
   else if(typeof deactivateTrack==='function')await deactivateTrack();
  }
  document.getElementById('demo-interval').onclick=()=>switchRunScenario(false);
  document.getElementById('demo-launch').onclick=()=>switchRunScenario(true);
  document.getElementById('demo-gps-log').onclick=event=>{gpsLogPaused=!gpsLogPaused;event.target.textContent=gpsLogPaused?'Resume GPS timestamp log':'Pause GPS timestamp log'};
  document.getElementById('demo-connection').onclick=event=>{paused=!paused;event.target.textContent=paused?'Restore connection':'Simulate disconnect'};
  try{
   await openTrackDB();
   const id='demo-circuit-v6',track={id,name:'Demo circuit · S-bends',type:'loop',gpsOrigin:origin,startLine:{p1:[155,0],p2:[195,0],direction:Math.PI/2},centerline:route.map(p=>({x:p.x,y:p.y})),pathLength:laps[0][steps].distance,lapCount:3,bestLapMs:Math.round(Math.min(...durations.slice(0,3))),createdAt:Date.now(),quality:'good'};
   const previous=(await getAllTracks()).find(item=>item.id===id);if(previous){track.bestLapMs=previous.bestLapMs;track.lapCount=previous.lapCount}
   // A reference belongs to an already committed track, just as on a real device.
   await saveTrack(track);
   if(!(await getRefLap(id))?.spatial){
    const pairs=new Float32Array(laps[0].flatMap(p=>[p.distance,p.time]));
    await saveRefLap({id:'demo-reference-v6',trackId:id,lapTimeMs:Math.round(durations[0]),created:Date.now(),samples:pairs.buffer,sampleCount:steps+1,totalDistance:track.pathLength,spatial:{version:1,closed:true,origin,points:laps[0].map(p=>({x:p.x,y:p.y,t:Math.min(p.time,Math.round(durations[0]))}))}});
   }
   demoTrack=track;if(!drivePreview){await activateTrack(track);armedCycle=clock(performance.now()-start).cycle-1;}
   const examples=await populateExamples(track);
   if(new URL(location.href).searchParams.get('view')==='replay'&&examples&&typeof openRecordingReplay==='function')await openRecordingReplay(examples.circuitId);
   if(drivePreview){const databaseForDrive=await openDB();await loadLatestDrive(databaseForDrive,await idbPromise(databaseForDrive.transaction('sessions','readonly').objectStore('sessions').getAll()));renderDrive()}
  }catch(error){document.getElementById('demo-note').textContent='Simulated data. Browser storage unavailable; recording and saved tracks require storage access.';console.error(error)}
 });
})();
