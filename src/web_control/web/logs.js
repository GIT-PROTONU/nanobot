// ---- Merged log sheet: decision log + health log, one chronological stream ----------
// Toggled by the "📜 Logs" button that lives IN the tab bar (#logsToggle) — not a
// separate always-on bar, so there's nothing to ever collide with the tab bar's own
// space when collapsed. Not a modal: no backdrop, the currently-open tab's settings
// stay visible/editable behind it while it's open. Polls GET /logs only while open.
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  const toggle=$("logsToggle"), drawer=$("logsDrawer"), body=$("logsBody");
  if(!toggle||!drawer||!body) return;

  // The sheet docks directly above the tab bar's real rendered height (measured
  // live, not guessed — icon/label font metrics + the safe-area inset vary by
  // device). #tabbar is only truly `position:fixed` on the mobile single-column
  // layout (see style.css's 880px breakpoint); on desktop it's a normal top-of-
  // sidebar nav, so the sheet docks flush to the viewport bottom there instead.
  // This only affects the sheet while it's OPEN — collapsed, #logsDrawer has no
  // footprint on screen at all, so a stale measurement can no longer hide a tab.
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
    toggle.classList.toggle("active",on);
    drawer.classList.toggle("open",on);
    clearInterval(timer); timer=null;
    if(on){ load(); timer=setInterval(load,4000); }
  }

  toggle.onclick=()=>setOpen(!drawer.classList.contains("open"));
  document.addEventListener("keydown",e=>{
    if(e.key==="Escape"&&drawer.classList.contains("open")) setOpen(false);
  });
  document.querySelectorAll("#logsFilter button").forEach(b=>b.onclick=()=>{
    document.querySelectorAll("#logsFilter button").forEach(x=>x.classList.toggle("active",x===b));
    filter=b.dataset.src; render();
  });
})();
