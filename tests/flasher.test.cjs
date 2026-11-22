const {test}=require('node:test');
const assert=require('node:assert/strict');
const {webcrypto}=require('node:crypto');
const fs=require('node:fs');
const {loadVerifiedFirmware,createBlackboxFlasher,flashTransportMode,prepareFlashTransport}=require('../docs/flasher.js');
function fixture(options={}){
 const bytes=new Uint8Array(64);bytes[0]=0xe9;bytes[12]=5;
 const manifest={chip:'esp32c3',offset:0,size:64,version:'test',...options};
 return webcrypto.subtle.digest('SHA-256',bytes).then(hash=>{
  manifest.sha256=Buffer.from(hash).toString('hex');
  return {bytes,manifest,fetcher:async url=>url==='manifest'?{ok:true,json:async()=>manifest}:{ok:true,arrayBuffer:async()=>bytes.buffer}};
 });
}
test('merged firmware uses zero offset and a verified blob',async()=>{
 const f=await fixture();const result=await loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto);
 assert.equal(result.blob.size,64);
 let options;createBlackboxFlasher(class{constructor(o){options=o}},{firmwareUrl:'image'});
 assert.equal(options.firmwareOffset,0);assert.equal(options.chip,'esp32c3');
});
test('corruption and wrong chip images are blocked before serial writing',async()=>{
 const f=await fixture();f.bytes[20]^=1;
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto),/checksum mismatch/);
 f.bytes[12]=0;
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto),/expected merged/);
});
test('application-only offsets cannot be accepted for a merged image',async()=>{
 const f=await fixture({offset:0x10000});
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest',firmwareUrl:'image'},f.fetcher,webcrypto),/Unsupported/);
});
test('release fetch failures stop the installation flow',async()=>{
 await assert.rejects(loadVerifiedFirmware({manifestUrl:'manifest'},async()=>({ok:false}),webcrypto),/manifest unavailable/);
});
test('Android uses USB even when Chrome exposes Bluetooth Web Serial',()=>{
 assert.equal(flashTransportMode({userAgent:'Mozilla Android',serial:{},usb:{}},true),'usb');
 assert.equal(flashTransportMode({userAgent:'Desktop mode',userAgentData:{platform:'Android'},serial:{},usb:{}},true),'usb');
 assert.equal(flashTransportMode({serial:{},usb:{}},true),'serial');
 assert.equal(flashTransportMode({usb:{}},true),'usb');
 assert.equal(flashTransportMode({usb:{}},false),null);
 assert.equal(flashTransportMode({},true),null);
 assert.equal(flashTransportMode({userAgent:'Android',serial:{}},true),null);
});
test('USB drivers preload without requesting permission; desktop never loads them',async()=>{
 let loads=0,picks=0,nativePicks=0;
 const load=async()=>{loads++;return {requestUsbSerialPort:()=>{picks++;return 'usb-port'}}};
 const android=await prepareFlashTransport({userAgent:'Android',serial:{},usb:{}},true,load);
 assert.equal(loads,1);assert.equal(picks,0);
 assert.equal(await android.requestPort(),'usb-port');assert.equal(picks,1);
 const desktop=await prepareFlashTransport({serial:{requestPort:()=>{nativePicks++;return 'native-port'}}},true,load);
 assert.equal(await desktop.requestPort(),'native-port');assert.equal(nativePicks,1);assert.equal(loads,1);
 await assert.rejects(prepareFlashTransport({usb:{}},true,async()=>{throw Error('offline')}),/offline/);
 assert.equal(await prepareFlashTransport({usb:{}},false,load),null);
});
test('USB port reaches existing chip checks and preserves the toolkit connection flow',async()=>{
 let nativeCalls=0,seen;
 class Flasher {
  constructor(options){this.options=options;this.device={connect:async(chip,opts)=>{seen={chip,opts};if(!opts.port)nativeCalls++;return 'connected'}}}
  getDevice(){return this.device}
  connect(){return this.device.connect(this.options.chip,{baudrate:115200,timeout:15000})}
 }
 const port={getInfo:()=>({usbVendorId:0x303a,usbProductId:0x1001})};
 const flasher=createBlackboxFlasher(Flasher,{firmwareUrl:'m9n'},{requestPort:async()=>port});
 assert.equal(await flasher.connect(),'connected');assert.equal(nativeCalls,0);
 assert.equal(seen.chip,'esp32c3');assert.equal(seen.opts.port,port);
 assert.equal(seen.opts.timeout,15000);assert.equal(seen.opts.skipChipCheck,undefined);
 assert.equal(flasher.options.firmwareOffset,0);assert.equal(flasher.options.firmwareUrl,'m9n');
});
test('cancelled USB selection never opens a port and the next click can retry',async()=>{
 let picks=0,connections=0;
 class Flasher {
  constructor(){this.device={connect:async()=>{connections++}}}
  getDevice(){return this.device}
  connect(){return this.device.connect('esp32c3')}
 }
 const flasher=createBlackboxFlasher(Flasher,{}, {requestPort:async()=>{
  if(++picks===1)throw new DOMException('No device selected','NotFoundError');return {};
 }});
 await assert.rejects(flasher.connect(),{name:'NotFoundError'});assert.equal(connections,0);
 await flasher.connect();assert.equal(connections,1);
});
test('bundled USB registry recognizes C3 native USB and common bridges',async()=>{
 const {findUsbSerialDriver,usbSerialFilters}=await import('../docs/vendor/usb-serial.mjs');
 const cdc={vendorId:0x303a,productId:0x1001,deviceClass:0xef,configurations:[{interfaces:[{alternates:[{interfaceClass:2,interfaceSubclass:2,interfaceProtocol:1}]}]}]};
 assert.equal(findUsbSerialDriver(cdc)?.name,'CDC-ACM');
 for (const [vendorId,productId,name] of [[0x10c4,0xea60,'CP210x'],[0x1a86,0x7523,'CH340/CH341'],[0x0403,0x6001,'FTDI']]) {
  assert.equal(findUsbSerialDriver({vendorId,productId,deviceClass:0,configurations:[]})?.name,name);
 }
 assert.equal(findUsbSerialDriver({vendorId:0xffff,productId:0xffff,deviceClass:0,configurations:[]}),undefined);
 assert.ok(usbSerialFilters().length>3);
});
async function installer({cancel=false,failWrite=false,corrupt=false,failDrivers=false}={}) {
 const vm=require('node:vm'),f=await fixture(),events=[],elements=new Map();
 if(corrupt)f.bytes[20]^=1;
 const node=()=>({disabled:false,value:'',textContent:'',className:'',style:{},scrollHeight:0,
  classList:{toggle(){}},listeners:{},addEventListener(type,fn){this.listeners[type]=fn},appendChild(){},
  querySelector(id){return this.children??={};}});
 const get=id=>{if(!elements.has(id))elements.set(id,node());return elements.get(id)};
 // Status text and subtext need independent nodes.
 get('status-box').querySelector=id=>get(id);
 let selected=false;
 class Flasher extends EventTarget {
  constructor(options){super();events.push(['create',options]);this.device={connect:async(chip,opts)=>{
   events.push(['connect',chip,opts]);selected=true;
  }}}
  getDevice(){return this.device}
  connect(){return this.device.connect('esp32c3',{timeout:15000})}
  isConnected(){return selected}
  async flash(opts){events.push(['write',opts]);if(failWrite)throw Error('USB disconnected')}
  async disconnect(){selected=false;events.push(['disconnect'])}
  dispose(){}
 }
 const context=vm.createContext({window:{isSecureContext:true,ESPWebFlash:{ESPFlasher:Flasher},addEventListener(){}},
  navigator:{wakeLock:{request:async()=>({release:async()=>events.push(['release-wake-lock'])})}},
  PROJECTS:{m9n:{name:'NEO-M9N / 25 Hz',manifestUrl:'manifest',firmwareUrl:'m9n'}},
  fetch:f.fetcher,crypto:webcrypto,Blob,queueMicrotask});
 vm.runInContext(fs.readFileSync(require.resolve('../docs/flasher.js'),'utf8'),context);
 context.document={getElementById:get,createElement:node};
 await context.initializeBlackboxFlasher(async()=>{
  if(failDrivers)throw Error('driver download failed');
  return {kind:'usb',requestPort:async()=>{
   events.push(['picker']);if(cancel){cancel=false;throw new DOMException('cancelled','NotFoundError')}return {usb:true};
  }};
 });
 const select=get('project-select'),button=get('flash-button');
 const choose=async()=>{select.value='m9n';await select.listeners.change()};
 const click=()=>button.listeners.click();
 return {get,events,choose,click,button};
}
test('Android installer selects verified M9N, connects without writing, then writes at zero and releases wake lock',async()=>{
 const ui=await installer();await ui.choose();assert.equal(ui.button.disabled,false);
 await ui.click();assert.equal(ui.events.filter(e=>e[0]==='write').length,0);
 assert.equal(ui.button.textContent,'Flash firmware');
 assert.equal(ui.events.find(e=>e[0]==='connect')[2].port.usb,true);
 await ui.click();const write=ui.events.find(e=>e[0]==='write')[1];
 assert.equal(write.firmwareOffset,0);assert.equal(write.customFirmware.size,64);
 assert.ok(ui.events.some(e=>e[0]==='release-wake-lock'));
 assert.equal(ui.button.textContent,'Installation complete');assert.equal(ui.button.disabled,true);
});
test('Android installer recovers from cancelled permission and failed USB writes',async()=>{
 const ui=await installer({cancel:true,failWrite:true});await ui.choose();await ui.click();
 assert.equal(ui.button.textContent,'Connect device');assert.equal(ui.button.disabled,false);
 assert.equal(ui.events.filter(e=>e[0]==='connect').length,0);
 await ui.click();await ui.click();
 assert.equal(ui.button.textContent,'Connect device');assert.equal(ui.button.disabled,false);
 assert.ok(ui.events.some(e=>e[0]==='release-wake-lock'));
 await ui.click();assert.equal(ui.button.textContent,'Flash firmware');
});
test('Android installer blocks connection when image verification or USB driver initialization fails',async()=>{
 const corrupt=await installer({corrupt:true});await corrupt.choose();
 assert.equal(corrupt.button.disabled,true);assert.equal(corrupt.events.length,0);
 const unavailable=await installer({failDrivers:true});
 assert.equal(unavailable.button.disabled,true);assert.equal(unavailable.get('project-select').disabled,true);
 assert.equal(unavailable.get('.status-text').textContent,'USB support could not load');
});
