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

  // Dock the drawer's bottom edge exactly at the tab bar's real footprint (not a
  // guessed px constant, which drifted out of sync and let the drawer overlap/hide
  // the tabs — icon+label font metrics and the safe-area inset both vary by
  // device). #tabbar is only truly `position:fixed` on the mobile single-column
  // layout (see style.css's 880px breakpoint); on desktop it's a normal top-of-
  // sidebar nav, so the drawer docks flush to the viewport bottom there instead.
  function syncTabbarOffset(){
    const tb=$("tabbar"); if(!tb) return;
    const fixed=getComputedStyle(tb).position==="fixed";
    document.documentElement.style.setProperty("--tabbar-h", (fixed?tb.offsetHeight:0)+"px");
  }
  syncTabbarOffset();
  window.addEventListener("resize",syncTabbarOffset);
  window.addEventListener("orientationchange",syncTabbarOffset);

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
    if(on) syncTabbarOffset();       // re-measure in case the viewport changed since load
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
