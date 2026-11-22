// Deterministic synthetic workload, not a measured receiver rate or phone capacity.
const START=Date.UTC(2026,10,21,12),GPS_HZ=25,TELEMETRY_HZ=30,SECONDS_PER_CHUNK=5;
function gps(i){
 const ms=i*40,speed=18+4*Math.sin(ms/90000);
 return{t:START+ms,rowType:1,gpsSynthetic:1,gpsBootId:7,gpsSeq:i+1,gpsHostUs:ms*1000,gpsUtcUs:43200000000+ms*1000,gpsDate:211126,gpsFractionDigits:3,gpsActive:1,gpsAccepted:1,lat:37.3317+0.001*Math.sin(ms/60000),lon:-122.0301+0.001*Math.cos(ms/60000),sp:speed,gpsCourse:ms/60000%(2*Math.PI),gpsMissing:0,gpsBoundary:i===0?1:0};
}
function telemetry(i){
 const ms=i*1000/TELEMETRY_HZ,g=gps(Math.floor(ms/40));
 return{t:START+ms,gpsSynthetic:1,deviceTimeMs:Math.floor(ms),lapTimeMs:0,lapCnt:0,lapFlags:0,lastLapMs:null,bestLapMs:null,gpsAgeMs:Math.floor(ms%40),imuAgeMs:2,timingRevision:1,gpsBootId:7,gpsSeq:g.gpsSeq,gpsHostUs:g.gpsHostUs,gpsUtcUs:g.gpsUtcUs,gpsDate:211126,gpsFractionDigits:3,sp:g.sp*3.6,ax:0.15*Math.sin(ms/10000),ay:0.2*Math.cos(ms/8000),wz:0.01,mo:1,latg:0.02,lng:0.03,lat:g.lat,lon:g.lon,gpsOk:1,ekfX:ms/100,ekfY:20*Math.sin(ms/60000),ekfYaw:g.gpsCourse,gpsCourse:g.gpsCourse,lon_imu:0.1,lon_gps:0.08,gps_wt:0.6,pitch_c:0.01,pitch_cf:0.01,roll_c:0.02,roll_cf:0.02,tilt_x:0.01,tilt_y:0.02};
}
function* chunks(minutes,sessionId=START){
 if(!Number.isInteger(minutes)||minutes<=0)throw Error('Positive integer minutes required');
 const seconds=minutes*60;
 for(let s=0;s<seconds;s+=SECONDS_PER_CHUNK){
  const data=[];
  for(let i=s*GPS_HZ;i<Math.min(seconds,s+SECONDS_PER_CHUNK)*GPS_HZ;i++)data.push(gps(i));
  for(let i=s*TELEMETRY_HZ;i<Math.min(seconds,s+SECONDS_PER_CHUNK)*TELEMETRY_HZ;i++)data.push(telemetry(i));
  data.sort((a,b)=>a.t-b.t||(b.rowType||0)-(a.rowType||0));
  yield{sessionId,chunkIndex:s/SECONDS_PER_CHUNK,timestamp:START+(s+SECONDS_PER_CHUNK)*1000,data};
 }
}
module.exports={START,GPS_HZ,TELEMETRY_HZ,SECONDS_PER_CHUNK,gps,telemetry,chunks};
