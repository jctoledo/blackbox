const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
test('GPS transport diagnostics distinguish missing, verified and failed startup',()=>{
 const source=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
 const nodes=new Map();const $=id=>{if(!nodes.has(id))nodes.set(id,{});return nodes.get(id)};
 const context=vm.createContext({$});
 vm.runInContext(source.slice(source.indexOf('function renderGpsTransport('),source.indexOf('function rateClass(')),context);
 context.renderGpsTransport(null);assert.equal($('gps-setup').textContent,'Unavailable');assert.equal($('gps-uart-overflow').textContent,'--');
 context.renderGpsTransport({configuration_verified:true,uart_baud:115200,fifo_overflows:0,buffer_full:2,frame_errors:3,parity_errors:0,read_errors:1,checksum_rejections:4,active_rmc:25,inactive_rmc:0});
 assert.equal($('gps-setup').textContent,'Verified at startup');assert.equal($('gps-uart-overflow').textContent,'0 / 2');
 assert.equal($('gps-uart-reads').textContent,'1');assert.equal($('gps-rmc-status').textContent,'25 / 0');
 context.renderGpsTransport({configuration_verified:false,setup_error:'readback_mismatch'});
 assert.equal($('gps-setup').textContent,'Failed: readback_mismatch');assert.equal($('gps-uart-baud').textContent,'--');
});
