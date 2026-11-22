const {test}=require('node:test');const assert=require('node:assert/strict');const fs=require('node:fs');
function imageDimensions(bytes,name){
 if(bytes.subarray(0,8).equals(Buffer.from([137,80,78,71,13,10,26,10])))return{width:bytes.readUInt32BE(16),height:bytes.readUInt32BE(20)};
 assert.equal(bytes.readUInt16BE(0),0xffd8,name+' JPEG signature');
 for(let offset=2;offset+4<=bytes.length;){
  assert.equal(bytes[offset],0xff,name+' JPEG marker');const marker=bytes[offset+1],length=bytes.readUInt16BE(offset+2);
  assert.ok(length>=2&&offset+2+length<=bytes.length,name+' bounded JPEG segment');
  if(marker>=0xc0&&marker<=0xcf&&![0xc4,0xc8,0xcc].includes(marker)){
   assert.ok(length>=8,name+' JPEG frame header');return{width:bytes.readUInt16BE(offset+7),height:bytes.readUInt16BE(offset+5)};
  }
  if(marker===0xda)break;
  offset+=2+length;
 }
 assert.fail(name+' has no supported image dimensions');
}
test('website screenshot dimensions match PNG/JPEG files and intended orientations',()=>{
 const html=fs.readFileSync('docs/index.html','utf8');
 const images=[...html.matchAll(/<img\b[^>]*\bsrc="((?:dashboard-[^"]+|assets\/(?:recording|diagnostics)-[^"]+)\.(?:png|jpe?g))"[^>]*>/g)];
 assert.ok(images.length>=6,'website must expose its dashboard views');const orientations=new Set();
 for(const [tag,name] of images){
  const {width,height}=imageDimensions(fs.readFileSync('docs/'+name),name);
  const portrait=name.includes('portrait')||name.startsWith('assets/recording-');orientations.add(portrait?'portrait':'landscape');
  assert.equal(Number(tag.match(/width="(\d+)"/)[1]),width,name+' width');assert.equal(Number(tag.match(/height="(\d+)"/)[1]),height,name+' height');assert.equal(height>width,portrait,name+' orientation');
 }
 assert.deepEqual([...orientations].sort(),['landscape','portrait']);
});
