const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function setup(cost,fail=false){let now=0,requests=0;const delays=[];const c={Math,Date,performance:{now:()=>now},lastTelemetryAt:Date.now(),markTelemetryUnavailable(){},console,setTimeout:(fn,delay)=>delays.push(delay),requestDevice:async()=>{requests++;now+=cost;if(fail)throw Error('offline');return{ok:true,json:async()=>({data:null})}}};vm.createContext(c);vm.runInContext(html.slice(html.indexOf('async function poll(){'),html.indexOf('// Diagnostics polling')),c);return{c,delays,requests:()=>requests};}
test('poll period includes request time instead of adding 33ms after it',async()=>{
 for(const [cost,expected] of [[0,33],[10,23],[30,3],[60,0]]){const p=setup(cost);await p.c.poll();assert.equal(p.delays[0],expected);assert.equal(p.requests(),1);}
});
test('failed requests keep the 500ms retry backoff',async()=>{const p=setup(60,true);await p.c.poll();assert.equal(p.delays[0],500)});
