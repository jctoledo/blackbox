const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const {context,vm}=require('./helpers/large-backup.cjs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(start,end)=>html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));
test('Escape, backdrop and Close keep backup controls reachable until cancellation or completion unlocks them',()=>{
 const c=context(),elements=new Map(),handlers=new Map();let returnedFocus=0;c.getComputedStyle=()=>({visibility:'visible'});
 c.$=id=>{if(!elements.has(id)){const classes=new Set();elements.set(id,{id,classList:{add:name=>classes.add(name),remove:name=>classes.delete(name),contains:name=>classes.has(name)},focus(){},getClientRects:()=>[{}],querySelectorAll:()=>[]})}return elements.get(id)};
 c.document={activeElement:{focus(){returnedFocus++},getClientRects:()=>[{}]},addEventListener:(type,handler)=>handlers.set(type,handler),querySelector:()=>[c.$('data-modal'),c.$('track-modal')].find(modal=>modal.classList.contains('open'))};
 vm.runInContext(slice('// Track Manager UI','function openTrackModal()'),c);
 vm.runInContext("function closeDataModal(){hideModal('data-modal')}\n"+slice("$('data-modal-close').onclick=", "$('btn-clear-all-recordings').onclick="),c);
 c.revealModal('data-modal','data-modal-close');vm.runInContext('backupBusy=true',c);
 let prevented=false;handlers.get('keydown')({key:'Escape',preventDefault(){prevented=true}});assert.equal(prevented,true);assert.equal(c.$('data-modal').classList.contains('open'),true);
 c.$('data-modal').onclick({target:c.$('data-modal')});assert.equal(c.$('data-modal').classList.contains('open'),true);
 c.$('data-modal-close').onclick();assert.equal(c.$('data-modal').classList.contains('open'),true);assert.equal(returnedFocus,0);
 vm.runInContext('backupBusy=false',c);handlers.get('keydown')({key:'Escape',preventDefault(){}});assert.equal(c.$('data-modal').classList.contains('open'),false);assert.equal(returnedFocus,1);
 c.revealModal('track-modal','track-modal-close');vm.runInContext('backupBusy=true',c);handlers.get('keydown')({key:'Escape',preventDefault(){}});assert.equal(c.$('track-modal').classList.contains('open'),false);
});
