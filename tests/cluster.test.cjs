const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const code=html.slice(html.indexOf('// Responsive driving layout.'),html.indexOf('// End responsive driving layout.'));
function context(landscape){
 const classes=new Set(),elements=new Map();let changed;
 const media={matches:landscape,addEventListener:(_,f)=>{changed=f}};
 const c={document:{body:{classList:{contains:v=>classes.has(v),remove:v=>classes.delete(v),toggle(v,on){if(on===undefined)on=!classes.has(v);if(on)classes.add(v);else classes.delete(v);return on}}}},matchMedia:()=>media,$:id=>{if(!elements.has(id))elements.set(id,{setAttribute(k,v){this[k]=v}});return elements.get(id)},requestAnimationFrame:fn=>fn(),resize(){},startRecording(){throw Error('Orientation started recording')},deactivateTrack(){throw Error('Orientation stopped timing')}};
 vm.createContext(c);vm.runInContext(code,c);return {classes,elements,rotate:landscape=>{media.matches=landscape;changed()}};
}
test('landscape automatically enters cluster and portrait restores ordinary dashboard',()=>{
 const p=context(false);assert.equal(p.classes.has('cluster-view'),false);p.rotate(true);assert.equal(p.classes.has('cluster-view'),true);assert.equal(p.classes.has('drive-view'),true);assert.equal(p.elements.get('drive-mode').textContent,'Dashboard');p.rotate(false);assert.equal(p.classes.has('drive-view'),false);
});
test('Dashboard gives an explicit escape from automatic cluster without changing orientation',()=>{
 const p=context(true);p.elements.get('drive-mode').onclick();assert.equal(p.classes.has('cluster-view'),false);assert.equal(p.classes.has('drive-view'),false);p.elements.get('drive-mode').onclick();assert.equal(p.classes.has('cluster-view'),true);
});
test('portrait manual Drive view and parked Analysis remain independently usable',()=>{
 const p=context(false);p.elements.get('drive-mode').onclick();assert.equal(p.classes.has('drive-view'),true);assert.equal(p.classes.has('cluster-view'),false);p.elements.get('analysis-mode').onclick();assert.equal(p.classes.has('analysis-view'),true);assert.equal(p.classes.has('drive-view'),false);p.rotate(true);assert.equal(p.classes.has('cluster-view'),false);p.elements.get('analysis-mode').onclick();assert.equal(p.classes.has('cluster-view'),true);
});
