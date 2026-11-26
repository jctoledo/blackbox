const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function setup(unit='km/h',preferences=false){
 const elements=new Map(),events=new Map();let focused;
 const $=id=>{if(!elements.has(id))elements.set(id,{id,hidden:id==='run-options',offsetTop:id==='run-overtaking'?48:4,scrollTop:0,textContent:'',attrs:{},setAttribute(k,v){this.attrs[k]=v},focus(){focused=this;for(const fn of events.get('focusin')||[])fn({target:this})},contains(target){return target===this||this.id==='run-chooser'&&['run-interval','run-lower','run-overtaking','run-short','run-upper','run-standing','run-quarter','run-options'].includes(target?.id)}});return elements.get(id)};
 const c={Math,Number,Error,$,gpsLogReadiness:()=>null,rec:true,recordingBusy:false,speedUnit:unit,document:{addEventListener(name,fn){if(!events.has(name))events.set(name,[]);events.get(name).push(fn)}}};if(preferences){delete c.speedUnit;c.speed_ema=100;c.maximumFreshSpeed=120;c.localStorage={getItem:()=>unit,setItem(){}}}vm.createContext(c);
 vm.runInContext(slice('class RollingRuns{', 'function runsToCSV(')+slice('// Runs interval disclosure:', '// End Runs interval disclosure.')+(preferences?slice('// Speed display preferences.', '// End speed display preferences.'):'') ,c);c.syncRunUnits();
 return {c,$,events,focus:()=>focused,emit(name,event){for(const fn of events.get(name)||[])fn(event)}};
}
test('unit changes preserve family and independent literal detector results',()=>{
 const p=setup();assert.equal(p.$('run-interval').textContent,'80–120 km/h');
 vm.runInContext("runDetectors[0].results.push({status:'accepted',seconds:5.588});runDetectors[3].results.push({status:'accepted',seconds:5.555555})",p.c);p.c.renderPerformanceRuns();assert.equal(p.$('run-last').textContent,'5.56 s');
 p.c.speedUnit='mph';p.c.syncRunUnits();assert.equal(p.$('run-interval').textContent,'45–70 mph');assert.equal(p.$('run-last').textContent,'5.59 s');
 p.$('run-interval').onclick();p.$('run-lower').onclick();assert.equal(p.$('run-interval').textContent,'30–60 mph');assert.equal(p.$('run-lower').attrs['aria-pressed'],'true');
 p.c.speedUnit='km/h';p.c.syncRunUnits();assert.equal(p.$('run-interval').textContent,'50–100 km/h');assert.equal(p.$('run-last').textContent,'—');assert.equal(vm.runInContext('runDetectors[0].results.length+runDetectors[3].results.length',p.c),2);assert.equal(p.c.rec,true);
});
test('production disclosure handlers support focus, Escape, selection and outside dismissal',()=>{
 const p=setup('mph');p.$('run-interval').onclick();assert.equal(p.$('run-options').hidden,false);assert.equal(p.$('run-interval').attrs['aria-expanded'],'true');assert.equal(p.focus(),p.$('run-overtaking'));
 let prevented=false;p.emit('keydown',{key:'Escape',preventDefault(){prevented=true}});assert.equal(prevented,true);assert.equal(p.$('run-options').hidden,true);assert.equal(p.focus(),p.$('run-interval'));
 p.$('run-interval').onclick();p.$('run-lower').onclick();assert.equal(p.$('run-options').hidden,true);assert.equal(p.focus(),p.$('run-interval'));assert.equal(p.$('run-interval').textContent,'30–60 mph');
 p.$('run-interval').onclick();p.emit('pointerdown',{target:p.$('run-lower')});assert.equal(p.$('run-options').hidden,false);p.emit('pointerdown',{target:{}});assert.equal(p.$('run-options').hidden,true);
 p.$('run-interval').onclick();p.emit('focusin',{target:{}});assert.equal(p.$('run-options').hidden,true);
});
test('changing instrument or leaving landscape dismisses the disclosure',()=>{
 const p=setup(),classes=new Set(['cluster-view']);p.c.document.body={classList:{remove:k=>classes.delete(k),toggle(k,on){on?classes.add(k):classes.delete(k)}}};p.c.requestAnimationFrame=()=>{};p.c.resize=()=>{};p.c.drawG=()=>{};
 vm.runInContext('let clusterInstrument="runs";'+slice('function closeInstrumentPicker(', 'function selectClusterInstrument(')+slice('function selectClusterInstrument(', "for(const mode of ['speed','forces','peaks','runs'])$('cluster-'+mode).onclick"),p.c);
 p.$('run-interval').onclick();p.c.selectClusterInstrument('peaks');assert.equal(p.$('run-options').hidden,true);assert.equal(p.c.rec,true);
 const media={matches:true,addEventListener(){}};p.c.matchMedia=()=>media;p.c.document.body.classList.contains=k=>classes.has(k);vm.runInContext(slice('// Responsive driving layout.', '// End responsive driving layout.'),p.c);
 p.$('run-interval').onclick();media.matches=false;p.c.syncDrivingLayout();assert.equal(p.$('run-options').hidden,true);assert.equal(p.c.rec,true);
});
test('global unit control synchronizes Runs without changing the selected family',()=>{
 const p=setup('mph',true);
 p.$('run-interval').onclick();p.$('run-lower').onclick();p.$('menu-units').onclick();assert.equal(p.$('run-interval').textContent,'50–100 km/h');assert.equal(p.$('run-lower').attrs['aria-pressed'],'true');assert.equal(p.c.rec,true);
});


test('six visible choices preserve short/standing counterparts and the same quarter-mile result across units',()=>{
 const p=setup('mph');for(const family of ['short','upper','standing','quarter']){p.$('run-interval').onclick();p.$('run-'+family).onclick();assert.equal(p.$('run-'+family).attrs['aria-pressed'],'true')}
 assert.equal(p.$('run-interval').textContent,'1/4 mile');vm.runInContext("selectedRunDetector.results.push({status:'accepted',seconds:14.2})",p.c);p.c.renderPerformanceRuns();assert.equal(p.$('run-last').textContent,'14.20 s');p.c.speedUnit='km/h';p.c.syncRunUnits();assert.equal(p.$('run-interval').textContent,'1/4 mile');assert.equal(p.$('run-last').textContent,'14.20 s');p.$('run-interval').onclick();p.$('run-standing').onclick();assert.equal(p.$('run-interval').textContent,'0–100 km/h');assert.equal(p.$('run-last').textContent,'—');assert.equal(p.$('run-note').textContent,'Experimental · no rollout');
});
