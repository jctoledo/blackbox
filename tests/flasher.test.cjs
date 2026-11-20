const {test}=require('node:test');
const assert=require('node:assert/strict');
const {webcrypto}=require('node:crypto');
const fs=require('node:fs');
const {loadVerifiedFirmware,createBlackboxFlasher}=require('../docs/flasher.js');
function fixture(options={}){
 const bytes=new Uint8Array(64);bytes[0]=0xe9;bytes[12]=5;
 const manifest={chip:'esp32c3',offset:0,size:64,version:'test',...options};
 return webcrypto.subtle.digest('SHA-256',bytes).then(hash=>{
  manifest.sha256=Buffer.from(hash).toString('hex');
  return {bytes,manifest,fetcher:async url=>url==='manifest'?{ok:true,json:async()=>manifest}:{ok:true,arrayBuffer:async()=>bytes.buffer}};
 });
}
test('merged firmware uses zero offset and a verified blob',async()=>{
 const f=await fixture();const result=await loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto);
 assert.equal(result.blob.size,64);
 let options;createBlackboxFlasher(class{constructor(o){options=o}},{firmwareUrl:'image'});
 assert.equal(options.firmwareOffset,0);assert.equal(options.chip,'esp32c3');
});
test('corruption and wrong chip images are blocked before serial writing',async()=>{
 const f=await fixture();f.bytes[20]^=1;
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto),/checksum mismatch/);
 f.bytes[12]=0;
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto),/expected merged/);
});
test('application-only offsets cannot be accepted for a merged image',async()=>{
 const f=await fixture({offset:0x10000});
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto),/Unsupported/);
});
test('release fetch failures stop the installation flow',async()=>{
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest'},async()=>({ok:false}),webcrypto),/manifest unavailable/);
});
