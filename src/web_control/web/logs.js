// ---- Merged log overlay: decision log + health log, one chronological stream --------
// Reachable from any tab via the topbar's always-visible 📜 Logs button — deliberately
// NOT another tab/card, since the whole point is "the one place to check regardless of
// what you're doing". Polls GET /logs (web_server.get_merged_log) only while open.
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  const overlay=$("logsOverlay"), body=$("logsBody"); if(!overlay||!body) return;

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
  function open(){
    overlay.classList.remove("hidden");
    load(); clearInterval(timer); timer=setInterval(load,4000);
  }
  function close(){
    overlay.classList.add("hidden");
    clearInterval(timer); timer=null;
  }

  $("logsBtn").onclick=open;
  $("logsClose").onclick=close;
  overlay.addEventListener("click",e=>{ if(e.target===overlay) close(); });
  document.addEventListener("keydown",e=>{ if(e.key==="Escape"&&!overlay.classList.contains("hidden")) close(); });
  document.querySelectorAll("#logsFilter button").forEach(b=>b.onclick=()=>{
    document.querySelectorAll("#logsFilter button").forEach(x=>x.classList.toggle("active",x===b));
    filter=b.dataset.src; render();
  });
})();
