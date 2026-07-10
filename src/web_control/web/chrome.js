// ---- UI chrome: bottom tabs, hero view switcher, tips, virtual joystick ------
(function(){
  "use strict";
  const $=id=>document.getElementById(id);

  // bottom tab bar -> show one panel. #logsToggle lives in the same bar but isn't a
  // real tab (no data-tab, no matching .panel) — it toggles the log sheet instead
  // (see logs.js), so it's excluded here.
  function showTab(name){
    document.querySelectorAll("#tabbar button[data-tab]").forEach(b=>b.classList.toggle("active",b.dataset.tab===name));
    document.querySelectorAll(".panel").forEach(p=>p.classList.toggle("active",p.id==="panel-"+name));
  }
  document.querySelectorAll("#tabbar button[data-tab]").forEach(b=>b.onclick=()=>showTab(b.dataset.tab));
  $("statusChip").onclick=()=>showTab("system");

  // hero view switcher: Lidar / Map / Camera. Reuses the existing (now hidden)
  // mapOn / camOn checkboxes — flipping them runs the original start/stop handlers.
  function setChk(id,val){ const el=$(id); if(el && el.checked!==val){ el.checked=val; el.dispatchEvent(new Event("change")); } }
  function showView(name){
    curHeroView=name;   // lets pollScan run at full rate only while the lidar view is up
    document.querySelectorAll("#viewSwitch button").forEach(b=>b.classList.toggle("active",b.dataset.view===name));
    $("view-lidar").classList.toggle("on",name==="lidar");
    $("view-map").classList.toggle("on",name==="map");
    $("view-cam").classList.toggle("on",name==="cam");
    $("view-oled").classList.toggle("on",name==="oled");
    $("ctlMap").classList.toggle("on",name==="map");
    $("ctlCam").classList.toggle("on",name==="cam");
    setChk("mapOn",name==="map");
    setChk("camOn",name==="cam");
    if(window.OLED) OLED.setActive(name==="oled");   // only animate while visible
  }
  document.querySelectorAll("#viewSwitch button").forEach(b=>b.onclick=()=>showView(b.dataset.view));

  // tips: hide all the explanatory help text by default for a cleaner UI
  $("tipsOn").onchange=()=>document.body.classList.toggle("no-tips",!$("tipsOn").checked);

  // virtual joystick -> analog setCmd(v,w). Push forward = +v, push left = +w
  // (matches the keyboard mapping). Speed scales with how far it's pushed; the
  // global 10 Hz publisher keeps streaming /cmd_vel while it's held off-centre.
  // Publishes during a drag are capped at ~15 Hz: pointermove fires at 60-120+ Hz
  // and forwarding every event floods the server + the 115200-baud ESP32 link, so
  // delivery gaps blow past the ESP32's 500 ms cmd watchdog and driving stutters.
  (function(){
    const joy=$("joy"), knob=$("joyKnob"); if(!joy) return;
    let id=null, cx=0, cy=0, R=60, lastPub=0;
    const begin=e=>{
      id=e.pointerId;
      const jr=joy.getBoundingClientRect(), kr=knob.getBoundingClientRect();
      cx=jr.left+jr.width/2; cy=jr.top+jr.height/2; R=(jr.width-kr.width)/2;
      joy.setPointerCapture(id); move(e); e.preventDefault();
    };
    const move=e=>{
      if(e.pointerId!==id) return;
      let dx=e.clientX-cx, dy=e.clientY-cy; const d=Math.hypot(dx,dy);
      if(d>R){ dx*=R/d; dy*=R/d; }
      knob.style.transform=`translate(${dx}px,${dy}px)`;
      setCmd(-dy/R, -dx/R);                    // up = forward, left = +yaw
      const t=performance.now();
      if(t-lastPub>=66){ lastPub=t; publishCmd(); }
    };
    const end=e=>{
      if(e.pointerId!==id) return; id=null;
      knob.style.transform="translate(0,0)"; setCmd(0,0); publishCmd();
    };
    joy.addEventListener("pointerdown",begin);
    joy.addEventListener("pointermove",move);
    joy.addEventListener("pointerup",end);
    joy.addEventListener("pointercancel",end);

    // Mirror every command source on the knob, so keyboard (WASD / arrows) visibly
    // drives it too. Both inputs funnel through the global setCmd(v,w); wrap it to
    // reposition the knob — skip while a finger is dragging (move() already owns it).
    const curR=()=>{ const jr=joy.getBoundingClientRect(), kr=knob.getBoundingClientRect();
      return (jr.width-kr.width)/2 || R; };
    function place(v,w){ const r=curR(); let x=-w*r, y=-v*r; const d=Math.hypot(x,y);
      if(d>r){ x*=r/d; y*=r/d; } knob.style.transform=`translate(${x}px,${y}px)`; }
    const baseSet=setCmd;
    setCmd=(v,w)=>{ baseSet(v,w); if(id===null) place(v,w); };
  })();

  // initial UI state
  showTab("drive");
  showView("lidar");
})();
