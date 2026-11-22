const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function context({supported=true,standalone=false,reject=false}={}){
 const elements=new Map(),listeners={};let enters=0,exits=0;
 const $=id=>{if(!elements.has(id)){const set=new Set();elements.set(id,{hidden:false,textContent:'',focus(){},tabIndex:0,getClientRects:()=>[{}],classList:{add:x=>set.add(x),remove:x=>set.delete(x),contains:x=>set.has(x)}})}return elements.get(id)};
 const document={fullscreenEnabled:supported,fullscreenElement:null,documentElement:{async requestFullscreen(){enters++;if(reject)throw Error('denied');document.fullscreenElement={}}},async exitFullscreen(){exits++;document.fullscreenElement=null},addEventListener:(n,f)=>listeners[n]=f};
 const c={document,getComputedStyle:()=>({visibility:'visible'}),navigator:{standalone},matchMedia:()=>({matches:false}),$,requestAnimationFrame:f=>f(),resize(){},openDataModal(){},startRecording(){throw Error('Display changed recording')},stopRecording(){throw Error('Display stopped recording')}};
 vm.createContext(c);vm.runInContext(html.slice(html.indexOf('// Track Manager UI'),html.indexOf('function openTrackModal()'))+html.slice(html.indexOf('// Mobile display controls.'),html.indexOf('// End mobile display controls.')),c);
 return {c,$,listeners,counts:()=>({enters,exits})};
}
test('fullscreen is only entered after a tap and native exit restores the label',async()=>{
 const t=context();assert.deepEqual(t.counts(),{enters:0,exits:0});assert.equal(t.$('display-mode').textContent,'Full screen');
 await t.$('display-mode').onclick();assert.equal(t.$('display-mode').textContent,'Exit full screen');
 await t.$('display-mode').onclick();assert.equal(t.$('display-mode').textContent,'Full screen');
 t.c.document.fullscreenElement={};t.listeners.fullscreenchange();assert.equal(t.$('display-mode').textContent,'Exit full screen');
 t.c.document.fullscreenElement=null;t.listeners.fullscreenchange();assert.equal(t.$('display-mode').textContent,'Full screen');
});
test('unsupported iPhone path shows setup, not an unworkable fullscreen button',async()=>{
 const t=context({supported:false});assert.equal(t.$('display-mode').textContent,'App setup');await t.$('display-mode').onclick();assert.equal(t.$('phone-modal').classList.contains('open'),true);assert.equal(t.counts().enters,0);
});
test('denied fullscreen falls back to instructions and standalone hides redundant control',async()=>{
 const t=context({reject:true});await t.$('display-mode').onclick();assert.match(t.$('display-message').textContent,/unavailable/);assert.equal(t.$('phone-modal').classList.contains('open'),true);
 assert.equal(context({standalone:true}).$('display-mode').hidden,true);
});
test('phone guidance preserves data and explains the local HTTP limitation',()=>{
 assert.match(html,/Storage may be separate/);assert.match(html,/restore the backup there as copies/);assert.match(html,/Open as Web App/);assert.match(html,/HTTP address may still open with browser controls/);
});
