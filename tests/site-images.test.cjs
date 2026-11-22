const {test}=require('node:test');const assert=require('node:assert/strict');const fs=require('node:fs');
test('website screenshot dimensions match the real PNGs and intended orientations',()=>{
 const html=fs.readFileSync('docs/index.html','utf8');
 const images=[...html.matchAll(/<img\b[^>]*\bsrc="((?:dashboard-[^"]+|assets\/recording-[^"]+)\.png)"[^>]*>/g)];
 assert.ok(images.length>=2,'website must expose dashboard screenshots');const orientations=new Set();
 for(const [tag,name] of images){
  const png=fs.readFileSync('docs/'+name);assert.equal(png.subarray(1,4).toString(),'PNG',name+' file format');const w=png.readUInt32BE(16),h=png.readUInt32BE(20);
  const portrait=name.includes('portrait')||name.startsWith('assets/recording-');orientations.add(portrait?'portrait':'landscape');
  assert.equal(Number(tag.match(/width="(\d+)"/)[1]),w,name+' width');assert.equal(Number(tag.match(/height="(\d+)"/)[1]),h,name+' height');assert.equal(h>w,portrait,name+' orientation');
 }
 assert.deepEqual([...orientations].sort(),['landscape','portrait']);
});
