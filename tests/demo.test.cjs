const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
function demo(){
 let now=0,ready;const button={textContent:''};
 const c={console,URL,Response,DOMException,DataView,Uint8Array,Math,Date,btoa,window:{},location:{href:'https://example.test/demo.html'},performance:{now:()=>now},document:{addEventListener:(_,fn)=>{ready=fn},getElementById:()=>button},openTrackDB:async()=>{},getAllTracks:async()=>[],saveTrack:async()=>{},activateTrack:async()=>{}};
 vm.createContext(c);vm.runInContext(fs.readFileSync('docs/demo.js','utf8'),c);
 return {fetch:c.window.fetch,advance:ms=>{now+=ms},ready:()=>ready(),button};
}
test('synthetic telemetry matches the v4 packet and exact lap result contract',async()=>{
 const d=demo();await d.fetch('/api/laptimer/configure?type=loop');d.advance(7000);
 const j=await(await d.fetch('/api/telemetry')).json(),packet=Buffer.from(j.data,'base64');
 assert.equal(packet.length,82);assert.equal(packet[0],4);assert.equal(packet.readUInt16LE(1),0xAA55);
 assert.equal(packet.readUInt16LE(80),packet.subarray(0,80).reduce((a,b)=>a+b,0)&65535);
 assert.equal(j.timing.state,'timing');assert.equal(j.timing.revision,1);assert.equal(packet.readUInt32LE(73),1000);
 d.advance(30000);const next=await(await d.fetch('/api/telemetry')).json();assert.equal(next.timing.last_lap_ms,30000);assert.equal(next.timing.best_lap_ms,30000);
});
test('demo disconnect rejects telemetry and configuration until restored',async()=>{
 const d=demo();await d.ready();d.button.onclick({target:d.button});
 await assert.rejects(d.fetch('/api/telemetry'),/connection loss/);await assert.rejects(d.fetch('/api/laptimer/configure?type=clear'),/connection loss/);
 d.button.onclick({target:d.button});assert.equal((await d.fetch('/api/telemetry')).status,200);
 assert.equal((await d.fetch('/not-a-device-endpoint')).status,404);
});
