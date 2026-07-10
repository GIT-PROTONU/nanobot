// ---- Merged log drawer: decision log + health log, one chronological stream ---------
// An expandable bottom drawer (click the handle bar to open/close), NOT a modal — the
// rest of the page (any tab's settings) stays visible and usable while it's open.
// Reachable from every tab since it's outside the tab-switching #panels entirely.
// Polls GET /logs (web_server.get_merged_log) only while open.
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  const drawer=$("logsDrawer"), handle=$("logsHandle"), body=$("logsBody");
  if(!drawer||!handle||!body) return;

  let lastEntries=[], filter="all", timer=null;

  function fmtHealthRow(e){
    const time=new Date((e.t||0)*1000).toLocaleTimeString();
    return `<div class="logrow src-health"><span class="lt">${time}</span> 🩺 ${e.text||""}</div>`;
  }
  function render(){
    const es=filter==="all"?lastEntries:lastEntries.filter(e=>e.source===filter);
    body.innerHTML = es.length
      ? es.map(e=>e.source==="health"?fmtHealthRow(e):fmtLogRow(e)).join("")
      : "<i>no log entries yet</i>";
  }
  function load(){
    fetch("/logs").then(r=>r.ok?r.json():null).then(d=>{
      lastEntries=(d&&d.entries)||[]; render();
    }).catch(()=>{ body.innerHTML="<i>log unavailable (dev harness / server too old)</i>"; });
  }
  function setOpen(on){
    drawer.classList.toggle("open",on);
    clearInterval(timer); timer=null;
    if(on){ load(); timer=setInterval(load,4000); }
  }

  handle.onclick=()=>setOpen(!drawer.classList.contains("open"));
  document.addEventListener("keydown",e=>{
    if(e.key==="Escape"&&drawer.classList.contains("open")) setOpen(false);
  });
  document.querySelectorAll("#logsFilter button").forEach(b=>b.onclick=()=>{
    document.querySelectorAll("#logsFilter button").forEach(x=>x.classList.toggle("active",x===b));
    filter=b.dataset.src; render();
  });
})();
