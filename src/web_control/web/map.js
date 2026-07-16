// ---- SLAM map panel ----------------------------------------------------------
// Self-contained, pure HTTP: poll the raw occupancy map from /map (same-origin,
// written by slam_nav to /dev/shm) and render it. The map file is a JSON metadata
// line, '\n', then raw int8 occupancy (-1 unknown, 0 free .. 100 occupied; row 0 =
// origin_y). Canvas clicks -> POST /publish /goal_pose (app.js mapSetGoal).
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  const cv=$("mapcv"), ctx=cv.getContext("2d");
  const dec=new TextDecoder();
  let timer=null, img=null, lastMeta=null;

  async function poll(){
    try{
      const r=await fetch("/map?t="+Date.now());
      if(!r.ok) return;                        // 503 until the first map is written
      const buf=new Uint8Array(await r.arrayBuffer());
      const nl=buf.indexOf(10);                // end of the JSON header line
      if(nl<0) return;
      const meta=JSON.parse(dec.decode(buf.subarray(0,nl)));
      draw(meta, buf.subarray(nl+1));
    }catch(e){ /* transient: keep polling */ }
  }

  function draw(m, grid){
    const w=m.w, h=m.h;
    if(cv.width!==w||cv.height!==h){ cv.width=w; cv.height=h; img=ctx.createImageData(w,h); }
    const d=img.data;
    for(let row=0;row<h;row++){
      const cy=h-1-row, base=row*w;            // grid row 0 = bottom -> flip for canvas
      for(let c=0;c<w;c++){
        let v=grid[base+c]; if(v>127) v-=256;  // byte -> signed int8
        const o=(cy*w+c)*4;
        const s=v<0?44:(255-Math.round(v*2.55)); // unknown grey; 0 white..100 black
        d[o]=d[o+1]=d[o+2]=s; d[o+3]=255;
      }
    }
    ctx.putImageData(img,0,0);
    lastMeta=m;
    $("mapWait").style.display="none";    // got a real frame: drop the placeholder
    cv.style.cursor="crosshair";          // ...and arm click-to-go only now
    const W2P=(x,y)=>[(x-m.ox)/m.res, h-1-(y-m.oy)/m.res];  // world -> canvas px
    // breadcrumb trail (cyan polyline) — where the robot has been
    if(m.trail && m.trail.length>1){
      ctx.strokeStyle="#22d3ee"; ctx.lineWidth=1; ctx.globalAlpha=0.7; ctx.beginPath();
      m.trail.forEach((pt,i)=>{ const [x,y]=W2P(pt[0],pt[1]); i?ctx.lineTo(x,y):ctx.moveTo(x,y); });
      ctx.stroke(); ctx.globalAlpha=1;
    }
    // planned path (green polyline)
    if(mapPlan && mapPlan.length>1){
      ctx.strokeStyle="#34c759"; ctx.lineWidth=2; ctx.beginPath();
      mapPlan.forEach((pt,i)=>{ const [x,y]=W2P(pt[0],pt[1]); i?ctx.lineTo(x,y):ctx.moveTo(x,y); });
      ctx.stroke();
    }
    // home marker (origin) — small magenta square
    if(m.hx!==undefined){
      const [hx,hy]=W2P(m.hx,m.hy);
      ctx.strokeStyle="#c084fc"; ctx.lineWidth=2; ctx.strokeRect(hx-4,hy-4,8,8);
    }
    // goal marker (yellow ring)
    if(mapGoal){
      const [gx,gy]=W2P(mapGoal[0],mapGoal[1]);
      ctx.strokeStyle="#ffd60a"; ctx.lineWidth=2;
      ctx.beginPath(); ctx.arc(gx,gy,5,0,6.2832); ctx.stroke();
    }
    // robot pose: red dot + heading tick
    const [rx,ry]=W2P(m.px,m.py);
    ctx.fillStyle=ctx.strokeStyle="#ff3b30"; ctx.lineWidth=2;
    ctx.beginPath(); ctx.arc(rx,ry,4,0,6.2832); ctx.fill();
    ctx.beginPath(); ctx.moveTo(rx,ry);
    ctx.lineTo(rx+9*Math.cos(m.pth), ry-9*Math.sin(m.pth)); ctx.stroke();
    // telemetry line (coverage + localization health + mode)
    if(m.seen!==undefined){
      const q = m.score>=8?"good":(m.score>=2?"ok":"weak");
      const loc = m.loc && m.loc!=="ok" ? ` · ⚠ ${m.loc}` : "";
      $("mapStats").textContent =
        `mode: ${m.mode||"—"} · explored ${(m.seen*100).toFixed(0)}% `
        + `(${(m.free_m2||0).toFixed(1)} m² free) · match ${q} (${(m.score||0).toFixed(0)})`
        + ` · motion ${m.motion?"ON":"off"}${loc}`;
    }
  }

  $("mapOn").addEventListener("change",e=>{
    const on=e.target.checked;
    $("mapStats").style.display=on?"block":"none";
    $("mapHint").style.display=on?"block":"none";
    if(on){
      $("mapWait").style.display="flex"; cv.style.cursor="default"; // until a frame lands
      poll(); timer=setInterval(poll,500);
    } else { clearInterval(timer); timer=null; }
  });

  // ---- pan / zoom (scroll or pinch to zoom, drag to pan, ⤢ Fit resets) ----
  // Done as a CSS transform on the canvas so the goal-click math below keeps
  // working unchanged: getBoundingClientRect() returns the post-transform box.
  const wrap=$("mapWrap");
  let zoom=1, panX=0, panY=0, moved=0;
  const ptrs=new Map(); let pinchD=0;
  function applyView(){
    cv.style.transform=`translate(-50%,-50%) translate(${panX}px,${panY}px) scale(${zoom})`;
  }
  function zoomAt(f,sx,sy){            // zoom keeping the screen point (sx,sy) fixed
    const r=wrap.getBoundingClientRect();
    const z=Math.min(16,Math.max(1,zoom*f)); f=z/zoom; zoom=z;
    const ox=sx-(r.left+r.width/2), oy=sy-(r.top+r.height/2);
    panX=ox*(1-f)+f*panX; panY=oy*(1-f)+f*panY;
    applyView();
  }
  wrap.addEventListener("wheel",e=>{ e.preventDefault();
    zoomAt(e.deltaY<0?1.15:1/1.15,e.clientX,e.clientY); },{passive:false});
  wrap.addEventListener("pointerdown",e=>{
    wrap.setPointerCapture(e.pointerId);
    ptrs.set(e.pointerId,[e.clientX,e.clientY]); moved=0;
    if(ptrs.size===2){ const a=[...ptrs.values()];
      pinchD=Math.hypot(a[0][0]-a[1][0],a[0][1]-a[1][1]); }
  });
  wrap.addEventListener("pointermove",e=>{
    if(!ptrs.has(e.pointerId)) return;
    const [px,py]=ptrs.get(e.pointerId);
    ptrs.set(e.pointerId,[e.clientX,e.clientY]);
    if(ptrs.size===1){
      panX+=e.clientX-px; panY+=e.clientY-py;
      moved+=Math.abs(e.clientX-px)+Math.abs(e.clientY-py);
      applyView();
    }else if(ptrs.size===2){
      const a=[...ptrs.values()];
      const d=Math.hypot(a[0][0]-a[1][0],a[0][1]-a[1][1]);
      if(pinchD>0) zoomAt(d/pinchD,(a[0][0]+a[1][0])/2,(a[0][1]+a[1][1])/2);
      pinchD=d; moved=99;
    }
  });
  const ptrEnd=e=>{ ptrs.delete(e.pointerId); pinchD=0; };
  wrap.addEventListener("pointerup",ptrEnd);
  wrap.addEventListener("pointercancel",ptrEnd);
  $("mapFit").addEventListener("click",()=>{ zoom=1; panX=panY=0; applyView(); });

  // tap the map -> goal in world metres (account for CSS scale/zoom + the y-flip).
  // Listens on the wrap, not the canvas: pointer capture retargets the click here.
  // A drag/pinch (moved) is not a goal tap, nor is a tap outside the canvas.
  wrap.addEventListener("click",e=>{
    if(!lastMeta||moved>6) return;
    const rect=cv.getBoundingClientRect();
    const cx=(e.clientX-rect.left)*cv.width/rect.width;
    const cy=(e.clientY-rect.top)*cv.height/rect.height;
    if(cx<0||cy<0||cx>=cv.width||cy>=cv.height) return;
    const m=lastMeta;
    mapSetGoal(m.ox+cx*m.res, m.oy+(cv.height-1-cy)*m.res);
  });
  $("mapMotion").addEventListener("change",e=>setNavMotion(e.target.checked));
  $("mapExplore").addEventListener("change",e=>setNavExplore(e.target.checked));
  $("mapHome").addEventListener("click",()=>pub("/slam_nav/go_home",true));
  $("mapSave").addEventListener("click",()=>pub("/slam_nav/save_map",true));
  $("mapClear").addEventListener("click",()=>{
    if(!confirm("Clear the map? This wipes the current SLAM grid and cannot be undone.")) return;
    pub("/slam_nav/clear_map",true);
  });
  $("mapTest").addEventListener("click",()=>{
    if(!$("mapMotion").checked){ alert("Enable Motion first — the self-test drives the robot."); return; }
    if(!confirm("Run the calibration self-test?\nThe robot will drive forward, back, then spin in place.")) return;
    const el=$("mapTestOut"); el.style.display="block"; el.textContent="running self-test…";
    pub("/selftest",true);
  });
  $("mapStop").addEventListener("click",()=>{
    $("mapMotion").checked=false; setNavMotion(false);
    $("mapExplore").checked=false; setNavExplore(false);   // also halt auto-exploration
    sendDrive(0,0);                                        // belt-and-braces halt
  });
})();
