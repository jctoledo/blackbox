const {test}=require('node:test');const assert=require('node:assert/strict');const fs=require('node:fs');
test('website screenshot dimensions match the real PNGs and intended orientations',()=>{
 const html=fs.readFileSync('docs/index.html','utf8');
 for(const [name,portrait] of [['dashboard-portrait-v6.png',true],['dashboard-cluster-v6.png',false],['dashboard-forces-v6.png',false],['dashboard-peaks-v6.png',false],['dashboard-runs-v7.png',false]]){
  const png=fs.readFileSync('docs/'+name);assert.equal(png.subarray(1,4).toString(),'PNG',name+' file format');const w=png.readUInt32BE(16),h=png.readUInt32BE(20);
  const tag=html.match(new RegExp('<img[^>]+src="'+name.replace('.','\\.')+'"[^>]*>'))[0];
  assert.equal(Number(tag.match(/width="(\d+)"/)[1]),w,name+' width');assert.equal(Number(tag.match(/height="(\d+)"/)[1]),h,name+' height');assert.equal(h>w,portrait,name+' orientation');
 }
});
