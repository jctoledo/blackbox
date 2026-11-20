/* Decorative only. Stop animation while hidden, offscreen, or explicitly paused. */
(()=>{
 const button=document.getElementById('motion-toggle'),hero=document.querySelector('.hero');
 let paused=false,visible=true;
 const update=()=>document.body.classList.toggle('motion-paused',paused||document.hidden||!visible);
 button.addEventListener('click',()=>{paused=!paused;button.setAttribute('aria-pressed',String(paused));button.textContent=paused?'Resume motion':'Pause motion';update()});
 document.addEventListener('visibilitychange',update);
 if('IntersectionObserver' in window)new IntersectionObserver(entries=>{visible=entries[0].isIntersecting;update()}).observe(hero);
 update();
})();
