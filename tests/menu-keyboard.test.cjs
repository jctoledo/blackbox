const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
function setup(){const elements=new Map(),events=new Map();let focused;const c={rec:true,lapTimerActive:true,$:id=>{if(!elements.has(id)){const classes=new Set();elements.set(id,{classList:{add:name=>classes.add(name),remove:name=>classes.delete(name),contains:name=>classes.has(name)},focus(){focused=id}})}return elements.get(id)},document:{addEventListener(type,handler){events.set(type,handler)}}};vm.createContext(c);vm.runInContext(html.slice(html.indexOf("$('menu-btn').onclick="),html.indexOf("$('menu-rec').onclick=")),c);return{c,events,focus:()=>focused}}
test('Escape dismisses the open menu, restores focus and preserves recording/timing',()=>{const {c,events,focus}=setup();c.$('menu-btn').onclick();let prevented=false;events.get('keydown')({key:'Escape',preventDefault(){prevented=true}});assert.equal(c.$('menu-overlay').classList.contains('open'),false);assert.equal(focus(),'menu-btn');assert.equal(prevented,true);assert.equal(c.rec,true);assert.equal(c.lapTimerActive,true)});
test('Escape with the menu closed does not steal focus or consume another control action',()=>{const {events,focus}=setup();let prevented=false;events.get('keydown')({key:'Escape',preventDefault(){prevented=true}});assert.equal(focus(),undefined);assert.equal(prevented,false)});
