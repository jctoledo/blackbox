const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const c={};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('function timeTextParts('),html.indexOf('function decorateTimeElement(')),c);
test('all duration formats retain exact text and fade only leading integer zeros',()=>{
 for(const [text,dim] of [['0:03.284','00'],['0:00.003','000'],['1:03:09.020','00'],['10:10.100',''],['+0.18s','0'],['−0.02 s','0'],['±0.00s','0'],['0.12 s','0'],['0:00','000'],['≥ 0:03','00'],['BEST LAP 0:46.744','0'],['0s','0']]){
  const parts=c.timeTextParts(text);assert.equal(parts.map(p=>p.text).join(''),text);assert.equal(parts.filter(p=>p.dim).map(p=>p.text).join(''),dim,text);
 }
});
test('non-time values and fractional precision do not receive padding treatment',()=>{
 for(const text of ['—:——','No reference','0.00 g','0.05 km','25 Hz','2026-11-25','Speed 035','0.003','.00s','.01 s'])assert.equal(c.timeTextParts(text).some(p=>p.dim),false,text);
 const parts=c.timeTextParts('0:01.000');assert.equal(parts.filter(p=>!p.dim).map(p=>p.text).join(''),':1.000');
});
test('delta trends cannot change the displayed numeric string or hide unavailable state',()=>{
 const fields=new Map();const d={Number,Math,fmtDelta:ms=>(ms>0?'+':'')+(ms/1000).toFixed(2),$:id=>{if(!fields.has(id))fields.set(id,{textContent:'',style:{},classList:{add(){},remove(){}}});return fields.get(id)}};vm.createContext(d);vm.runInContext(html.slice(html.indexOf('function updateDeltaBar('),html.indexOf('async function saveNewRefLap(')),d);
 for(const trend of [-100,0,100]){d.updateDeltaBar({deltaMs:180,hasRef:true,trend});assert.equal(d.$('delta-text').textContent,'+0.18s');assert.match(d.$('delta-text').className,/bbNum/)}
 d.updateDeltaBar({deltaMs:null,hasRef:false});assert.equal(d.$('delta-text').textContent,'—');assert.equal(d.$('delta-reason').textContent,'No reference');
 d.updateDeltaBar({deltaMs:null,hasRef:true,reason:'Off reference route'});assert.equal(d.$('delta-reason').textContent,'Off reference');assert.equal(d.$('delta-reason').title,'Off reference route');
});
