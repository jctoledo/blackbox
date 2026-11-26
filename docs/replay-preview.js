(()=>{
 const button=document.getElementById('load-replay-preview');if(!button)return;
 button.addEventListener('click',()=>{
  const frame=document.createElement('iframe');frame.src='demo.html?view=replay';frame.title='Interactive saved SIM recording replay';frame.setAttribute('allow','fullscreen');
  const container=document.getElementById('replay-preview');container.replaceChildren(frame);frame.focus();
 });
})();
