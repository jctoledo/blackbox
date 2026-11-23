const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const source=html.slice(html.indexOf('// Nonblocking decisions and notices:'),html.indexOf('// Track Manager UI'));
function fixture(){
 const elements=new Map();let focused,restored;
 const $=id=>{if(!elements.has(id))elements.set(id,{id,open:false,value:'',returnValue:'',textContent:'',hidden:false,focus(){focused=id},select(){},showModal(){this.open=true},close(){this.open=false}});return elements.get(id)};
 const opener={id:'opener'};const c={$,Promise,String,document:{activeElement:opener},focusVisibleControl(el){restored=el}};
 vm.createContext(c);vm.runInContext(source,c);return{c,$,opener,focused:()=>focused,restored:()=>restored};
}
test('a closing dialog retains its owner until the close event resolves',async()=>{
 const f=fixture(),first=f.c.requestDecision('First');f.$('decision-dialog').close();
 assert.equal(await f.c.requestDecision('Second'),null);
 f.$('decision-dialog').close();f.$('decision-dialog').onclose();assert.equal(await first,null);assert.equal(f.restored(),f.opener);
 const next=f.c.requestDecision('Next');f.$('decision-dialog').returnValue='accept';f.$('decision-dialog').close();f.$('decision-dialog').onclose();assert.equal(await next,true);
});
test('confirmation defaults to Cancel and naming returns trimmed user input',async()=>{
 const f=fixture(),confirmation=f.c.requestDecision('Delete?');assert.equal(f.focused(),'decision-cancel');
 f.$('decision-dialog').close();f.$('decision-dialog').onclose();assert.equal(await confirmation,null);
 const name=f.c.requestDecision('Name','Default');assert.equal(f.focused(),'decision-name');
 f.$('decision-name').value='  Evening route  ';f.$('decision-dialog').returnValue='accept';f.$('decision-dialog').close();f.$('decision-dialog').onclose();assert.equal(await name,'Evening route');
});
test('unsupported dialog fails closed, exposes a notice and allows retry',async()=>{
 const f=fixture(),dialog=f.$('decision-dialog');dialog.showModal=()=>{throw Error('unavailable')};
 assert.equal(await f.c.requestDecision('Delete?'),null);assert.equal(f.$('app-notice').hidden,false);
 assert.match(f.$('app-notice-text').textContent,/No action was taken/);
 dialog.showModal=()=>{dialog.open=true};const retry=f.c.requestDecision('Retry');dialog.close();dialog.onclose();assert.equal(await retry,null);
});
