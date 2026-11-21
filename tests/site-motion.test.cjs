const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const source=fs.readFileSync('docs/site-motion.js','utf8');
function page({reduce=false,saveData=false,reject=false}={}){
 const handlers={},button={setAttribute(k,v){this[k]=v},addEventListener(k,v){handlers['button:'+k]=v}},video={dataset:{src:'assets/road-hero.mp4'},plays:0,pauses:0,getAttribute(){return this.src},play(){this.plays++;return reject?Promise.reject(Error('autoplay blocked')):Promise.resolve()},pause(){this.pauses++},addEventListener(k,v){handlers['video:'+k]=v}};
 const media={matches:reduce,addEventListener(k,v){handlers.media=v}},connection={saveData,addEventListener(k,v){handlers.connection=v}},document={hidden:false,getElementById:id=>id==='motion-toggle'?button:video,querySelector:()=>({}),addEventListener(k,v){handlers[k]=v}};
 class Observer{constructor(cb){handlers.observe=cb}observe(){}}
 const c={document,navigator:{connection},matchMedia:()=>media,window:{IntersectionObserver:Observer},IntersectionObserver:Observer};vm.createContext(c);vm.runInContext(source,c);
 return {button,video,media,connection,document,handlers};
}
test('background video is not fetched until visible and allowed by motion/data preferences',()=>{
 for(const options of [{reduce:true},{saveData:true}]){const p=page(options);p.handlers.observe([{isIntersecting:true}]);assert.equal(p.video.src,undefined);assert.equal(p.video.plays,0);assert.equal(p.button.hidden,true)}
 const p=page();assert.equal(p.video.src,undefined);p.handlers.observe([{isIntersecting:true}]);assert.equal(p.video.src,'assets/road-hero.mp4');assert.equal(p.video.muted,true);assert.equal(p.video.plays,1);
});
test('explicit pause persists through visibility changes and can be resumed',()=>{
 const p=page();p.handlers.observe([{isIntersecting:true}]);p.handlers['button:click']();assert.equal(p.button['aria-pressed'],'true');const plays=p.video.plays;
 p.handlers.observe([{isIntersecting:false}]);p.handlers.observe([{isIntersecting:true}]);assert.equal(p.video.plays,plays);
 p.handlers['button:click']();assert.equal(p.video.plays,plays+1);assert.equal(p.button['aria-pressed'],'false');
});
test('hidden tabs and changed preferences pause playback',()=>{
 const p=page();p.handlers.observe([{isIntersecting:true}]);const before=p.video.pauses;p.document.hidden=true;p.handlers.visibilitychange();assert.ok(p.video.pauses>before);
 p.document.hidden=false;p.media.matches=true;p.handlers.media();assert.equal(p.button.hidden,true);
});
test('autoplay refusal offers play and a broken media file leaves the poster fallback',async()=>{
 const p=page({reject:true});p.handlers.observe([{isIntersecting:true}]);await Promise.resolve();assert.equal(p.button.textContent,'Play background');
 p.handlers['video:error']();assert.equal(p.video.hidden,true);assert.equal(p.button.hidden,true);
});
