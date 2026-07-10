// ---- Merged log panel: decision log + health log, one chronological stream ----------
// A genuine collapsible panel in normal document flow (no position:fixed, no z-index
// tricks) -- toggled by the topbar's "📜 Logs" button, it takes real layout space and
// pushes the hero view / tabs aside instead of overlaying them, so it structurally
// cannot cover the tab bar. Not a modal: no backdrop, the currently-open tab's
// settings stay visible/editable alongside it. Polls GET /logs only while open.
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  const btn=$("logsBtn"), closeBtn=$("logsPanelClose"), panel=$("logsPanel"), body=$("logsBody");
  if(!btn||!panel||!body) return;

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
    btn.classList.toggle("active",on);
    panel.classList.toggle("open",on);
    clearInterval(timer); timer=null;
    if(on){ load(); timer=setInterval(load,4000); }
  }

  btn.onclick=()=>setOpen(!panel.classList.contains("open"));
  if(closeBtn) closeBtn.onclick=()=>setOpen(false);
  document.addEventListener("keydown",e=>{
    if(e.key==="Escape"&&panel.classList.contains("open")) setOpen(false);
  });
  document.querySelectorAll("#logsFilter button").forEach(b=>b.onclick=()=>{
    document.querySelectorAll("#logsFilter button").forEach(x=>x.classList.toggle("active",x===b));
    filter=b.dataset.src; render();
  });
})();
