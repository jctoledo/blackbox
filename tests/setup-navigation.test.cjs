const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(start,end)=>html.slice(html.indexOf(start),html.indexOf(end,html.indexOf(start)));

function context(){
 const elements=new Map(),listeners=new Map(),intervals=new Set();let nextInterval=0,returned;
 const document={activeElement:null,fullscreenEnabled:false,fullscreenElement:null,documentElement:{},addEventListener(type,handler){if(!listeners.has(type))listeners.set(type,[]);listeners.get(type).push(handler)},querySelector(selector){assert.equal(selector,'.bbModal.open');return ['phone-modal','mount-modal','track-modal','data-modal'].map($).find(el=>el.classList.contains('open'))}};
 function $(id){
  if(!elements.has(id)){
   const classes=new Set(),tag=html.match(new RegExp('<(\\w+)\\b[^>]*\\bid="'+id+'"'))?.[1];
   const el={id,tagName:tag?.toUpperCase(),disabled:false,hidden:false,textContent:'',classList:{add:n=>classes.add(n),remove:n=>classes.delete(n),contains:n=>classes.has(n)},getClientRects(){return this.hidden?[]:[{}]},focus(){document.activeElement=this;returned=id},click(){this.focus();this.onclick?.()},querySelectorAll(){return []}};
   elements.set(id,el);
  }
  return elements.get(id);
 }
 const c={$,document,getComputedStyle:el=>({visibility:el.id.startsWith('menu-')&&el.id!=='menu-btn'&&!$('menu-overlay').classList.contains('open')?'hidden':'visible'}),navigator:{standalone:false},matchMedia:()=>({matches:false}),requestAnimationFrame:fn=>fn(),resize(){},rec:true,lapTimerActive:true,backupBusy:false,renderTrackList(){},updateActiveTrackDisplay(){},openDataModal(){},requestDevice:async()=>({ok:true,json:async()=>({state:'idle',message:'Ready',saved:{is_calibrated:false},parked:true,storage_available:true})}),setInterval(){const token=++nextInterval;intervals.add(token);return token},clearInterval:token=>intervals.delete(token)};
 vm.createContext(c);
 // Preserve application registration order. Isolated listener fixtures miss the
 // generic handler consuming Escape before the setup-specific handler can clean up.
 for(const code of [slice('// View switching','function getModeColorHex'),slice('// Track Manager UI','function updateActiveTrackDisplay'),slice("$('menu-btn').onclick=","$('menu-rec').onclick="),slice('// Mobile display controls.','// End mobile display controls.'),slice('// Mount alignment controls.','// End mount alignment controls.')])vm.runInContext(code,c);
 return {c,$,document,intervals,focus:()=>returned,escape(){const event={key:'Escape',preventDefault(){}};for(const listener of listeners.get('keydown')||[])listener(event)},enter(id){const el=$(id);el.focus();if(el.tagName==='BUTTON')el.click();else el.onkeydown?.({key:'Enter',preventDefault(){}})}};
}

test('Escape stops mount polling, restores the visible menu trigger and preserves recording/timing',()=>{
 const t=context();t.$('menu-btn').click();t.$('menu-mount').click();assert.equal(t.intervals.size,1);assert.equal(t.focus(),'mount-close');
 t.escape();assert.equal(t.$('mount-modal').classList.contains('open'),false);assert.equal(t.intervals.size,0);assert.equal(t.focus(),'menu-btn');assert.equal(t.c.rec,true);assert.equal(t.c.lapTimerActive,true);
});

for(const close of ['escape','button','backdrop'])test('Phone setup returns to its visible opener after '+close,async()=>{
 const t=context();t.$('display-mode').focus();await t.c.toggleDisplayMode();
 if(close==='escape')t.escape();else if(close==='button')t.$('phone-close').click();else t.$('phone-modal').onclick({target:t.$('phone-modal')});
 assert.equal(t.$('phone-modal').classList.contains('open'),false);assert.equal(t.focus(),'display-mode');assert.equal(t.c.rec,true);assert.equal(t.c.lapTimerActive,true);
});

for(const close of ['button','backdrop'])test('Mount setup stops polling on '+close,()=>{
 const t=context();t.$('menu-btn').click();t.$('menu-mount').click();
 if(close==='button')t.$('mount-close').click();else t.$('mount-modal').onclick({target:t.$('mount-modal')});
 assert.equal(t.intervals.size,0);assert.equal(t.focus(),'menu-btn');assert.equal(t.$('mount-modal').classList.contains('open'),false);
});

test('menu-launched Phone setup and a hidden opener fall back to visible controls',()=>{
 const t=context();t.$('menu-btn').click();t.$('menu-app').click();t.escape();assert.equal(t.focus(),'menu-btn');
 t.$('menu-btn').click();t.$('menu-app').click();t.$('menu-btn').hidden=true;t.escape();assert.equal(t.focus(),'cluster-controls');
});

test('setup close routing preserves the busy backup dismissal lock',()=>{
 const t=context();t.$('menu-btn').focus();t.c.revealModal('data-modal','data-modal-close');t.c.backupBusy=true;t.escape();
 assert.equal(t.$('data-modal').classList.contains('open'),true);assert.equal(t.focus(),'data-modal-close');
 t.c.backupBusy=false;t.escape();assert.equal(t.$('data-modal').classList.contains('open'),false);assert.equal(t.focus(),'menu-btn');
});

test('native keyboard activation enters Diagnostics and returns focus to a visible dashboard control',()=>{
 const t=context();t.$('view-dashboard').classList.add('active');t.enter('link-diag');assert.equal(t.$('view-diagnostics').classList.contains('active'),true);
 const enteredFocus=t.focus();t.enter('link-dashboard');assert.equal(t.$('view-dashboard').classList.contains('active'),true);assert.equal(t.$('view-diagnostics').classList.contains('active'),false);assert.equal(enteredFocus,'link-dashboard');assert.equal(t.focus(),'link-diag');
});
