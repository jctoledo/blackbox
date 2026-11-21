/* Decorative licensed footage; no remote media requests, sound, or timing claims. */
(()=>{
 const button=document.getElementById('motion-toggle'),hero=document.querySelector('.hero'),video=document.getElementById('hero-video');
 if(!button||!hero||!video)return;
 const reduced=matchMedia('(prefers-reduced-motion: reduce)'),connection=navigator.connection;
 let paused=false,visible=!('IntersectionObserver' in window),failed=false,request=0;
 const update=()=>{
  const currentRequest=++request;
  const stopped=paused||document.hidden||!visible||reduced.matches||connection?.saveData||failed;
  button.hidden=reduced.matches||!!connection?.saveData||failed;
  button.setAttribute('aria-pressed',String(paused));button.textContent=paused?'Play background':'Pause background';
  if(stopped){video.pause();return}
  if(!video.getAttribute('src'))video.src=video.dataset.src;
  video.muted=true;
  const play=video.play();
  if(play?.catch)play.catch(()=>{if(currentRequest!==request)return;paused=true;button.setAttribute('aria-pressed','true');button.textContent='Play background'});
 };
 button.addEventListener('click',()=>{paused=!paused;update()});
 video.addEventListener('error',()=>{failed=true;video.hidden=true;update()});
 document.addEventListener('visibilitychange',update);
 reduced.addEventListener('change',update);connection?.addEventListener?.('change',update);
 if('IntersectionObserver' in window)new IntersectionObserver(entries=>{visible=entries[0].isIntersecting;update()}).observe(hero);
 update();
})();
