const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const html=fs.readFileSync('sensors/blackbox/web/dashboard.html','utf8');
const slice=(a,b)=>html.slice(html.indexOf(a),html.indexOf(b,html.indexOf(a)));
function fixture(extra={}){const elements=new Map();const c={console,Date,Math,Number,setTimeout(){},$:id=>{if(!elements.has(id)){const classes=new Set(['timing']);elements.set(id,{textContent:'old value',hidden:false,classList:{remove(...names){names.forEach(n=>classes.delete(n))},contains:n=>classes.has(n)}})}return elements.get(id)},...extra};vm.createContext(c);return {c,elements}}
for(const state of ['idle','mismatch'])test('unconfirmed timing clears matched delta, result and active styling: '+state,()=>{
 let invalidated=0,delta;const {c}=fixture({expectedLapRevision:2,lapTimerActive:true,activeTrack:{type:'loop'},referenceLap:{spatial:{}},lapTracker:{invalidate(){invalidated++}},updateDeltaBar:value=>delta=value,lapResultGeneration:0});
 vm.runInContext(slice('function fmtLapTime(', 'function updateReadouts'),c);
 c.updateLapTimer(0,0,0,{state:state==='idle'?'idle':'timing',revision:1});
 assert.equal(delta?.deltaMs,null);assert.equal(invalidated,1);assert.equal(c.$('lap-result').hidden,true);assert.equal(c.$('lap-state').classList.contains('timing'),false);assert.equal(c.$('lap-section').classList.contains('timing'),false);
});
test('reset renders cleared speed and force statistics without telemetry',()=>{
 let drawn=0;const {c}=fixture({maxL:1,maxR:1,maxA:1,maxB:1,peak:1,maximumFreshSpeed:100,trail:[1],currentScale:2,emaGx:1,emaGy:1,displaySessionStart:1,effortPeaks:{reset(){}},drawG(){drawn++}});
 vm.runInContext(slice('function resetState()', 'let peakHighlightTimeout'),c);c.resetState();
 for(const id of ['speed-maximum','effort-accel','effort-brake','effort-left','effort-right'])assert.equal(c.$(id).textContent,'—',id);assert.equal(drawn,1);
});
test('recovered pending metadata does not claim a running request',()=>{
 const {c}=fixture({escHtml:String});vm.runInContext(slice('function recordingDetailsHTML(', 'async function exportRecordingDetails('),c);
 const out=c.recordingDetailsHTML({recordingMetadata:{status:'pending',requestedAt:1}});assert.match(out,/interrupted/i);assert.match(out,/no request is running/i);
});
test('empty interrupted recording has no saved samples promise',()=>{
 const {c}=fixture();vm.runInContext(slice('function recordingRecoveryLabel(', 'function recordingDetailsHTML('),c);
 assert.match(c.recordingRecoveryLabel({sampleCount:0}),/no saved samples/i);assert.match(c.recordingRecoveryLabel({sampleCount:1}),/saved data available/i);
});
test('live metadata refresh preserves the details element, expanded state, focus and modal scroll',()=>{
 let selected,focused=false,action;const button={focus(options){assert.equal(options.preventScroll,true);focused=true}},body={contains:()=>true,replaceChildren(...nodes){this.nodes=nodes},querySelector:()=>button};
 const details={open:true,querySelector:()=>body},row={querySelector:()=>details},scroller={scrollTop:123},modal={classList:{contains:()=>true},querySelector(selector){if(selector==='.bbModalBody')return scroller;selected=selector;return row}};
 const {c}=fixture({document:{activeElement:{},createElement:()=>({content:{querySelector:()=>({childNodes:['new metadata']})}})},$:()=>modal,recordingDetailsHTML:()=>'<details/>',handleSessionAction:value=>action=value});
 vm.runInContext(slice('function refreshRecordingDetails(', 'function recordingDetailsHTML('),c);c.refreshRecordingDetails({sessionId:42});
 assert.equal(selected,'[data-session-id="42"]');assert.equal(details.open,true);assert.equal(focused,true);assert.equal(scroller.scrollTop,123);assert.deepEqual(body.nodes,['new metadata']);button.onclick();assert.equal(action,button);
});
