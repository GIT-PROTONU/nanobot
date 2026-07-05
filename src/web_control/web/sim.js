// ---- In-browser robot simulator -------------------------------------------
// Lightweight, ROS-free. When "Enable simulation" is on we (a) override fetch for
// the two HTTP sources the page already polls — /scan.bin (lidar) and /map
// (occupancy grid) — returning synthetic bytes in the *exact* wire format, so the
// existing Lidar + Map hero views render the sim with NO changes to their code; and
// (b) read the same teleop command the joystick / WASD feed into setCmd to drive a
// diff-drive robot around a user-drawn / imported world grid. No rosbridge involved,
// so it runs by just opening the page on a dev PC (Windows included).
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  let on=false;                                // master sim switch

  // ---- world: occupancy grid, 0 = free, 100 = wall. row 0 = world min-y (bottom),
  //      matching the /map format the page expects (it flips for the canvas). -----
  const W={res:0.05, w:0, h:0, ox:0, oy:0, cells:null};
  function newWorld(sizeM){
    W.res=0.05; W.w=W.h=Math.round(sizeM/W.res);
    W.ox=-sizeM/2; W.oy=-sizeM/2;
    W.cells=new Int8Array(W.w*W.h);            // all free
  }
  newWorld(12);
  const idx=(c,r)=>r*W.w+c;
  function occ(wx,wy){                          // world metres -> is it a wall?
    const c=Math.floor((wx-W.ox)/W.res), r=Math.floor((wy-W.oy)/W.res);
    if(c<0||r<0||c>=W.w||r>=W.h) return true;   // outside the map = solid
    return W.cells[idx(c,r)]>=50;
  }

  // ---- robot pose + start pose ----------------------------------------------
  const R={x:0,y:0,th:0, sx:0,sy:0,sth:0};
  let trail=[], lastTrail=0, simGoal=null;
  function resetRobot(){ R.x=R.sx; R.y=R.sy; R.th=R.sth; trail.length=0; }

  // command comes from the SAME setCmd the joystick + keyboard already call.
  const cmd={v:0,w:0};
  const _setCmd=setCmd;
  setCmd=(v,w)=>{ _setCmd(v,w); cmd.v=v; cmd.w=w; };
  // tapping the Map view drops a goal -> remember it for the sim controller too.
  const _mapSetGoal=mapSetGoal;
  mapSetGoal=(wx,wy)=>{ _mapSetGoal(wx,wy); if(on){ simGoal=[wx,wy]; nav.nextPlan=0; } };

  // ---- physics: 20 Hz unicycle integration with crude footprint collision ---
  const RAD=0.16;
  function blocked(x,y){
    if(occ(x,y)) return true;
    for(let a=0;a<6.28;a+=1.05) if(occ(x+RAD*Math.cos(a), y+RAD*Math.sin(a))) return true;
    return false;
  }
  // ---- navigation: a JS port of slam_nav's planner (occupancy.plan) + pure-
  //      pursuit (nav_node._control). Wavefront BFS over a downsampled grid with
  //      robot-radius-inflated obstacles, descend it start->goal, follow with
  //      pure pursuit, and stop+replan on a reactive front cone. Same defaults as
  //      robot.yaml (downsample 4, radius 0.16, lookahead 0.30, stop 0.25). -------
  const nav={path:[], nextPlan:0};
  const DS=4, LOOK=0.30, STOP=0.25, FRONT=0.6;
  function planPath(sx,sy,gx,gy){
    const resC=W.res*DS, cw=Math.ceil(W.w/DS), ch=Math.ceil(W.h/DS);
    const blk=new Uint8Array(cw*ch);
    for(let r=0;r<W.h;r++) for(let c=0;c<W.w;c++)
      if(W.cells[idx(c,r)]>=50) blk[((r/DS)|0)*cw + ((c/DS)|0)]=1;
    for(let p=0,passes=Math.max(1,Math.round(RAD/resC)); p<passes; p++){  // inflate by robot radius
      const s=blk.slice();
      for(let r=0;r<ch;r++) for(let c=0;c<cw;c++){ if(s[r*cw+c]) continue;
        if((c>0&&s[r*cw+c-1])||(c<cw-1&&s[r*cw+c+1])||(r>0&&s[(r-1)*cw+c])||(r<ch-1&&s[(r+1)*cw+c])) blk[r*cw+c]=1; }
    }
    const w2c=(x,y)=>[Math.floor((x-W.ox)/resC), Math.floor((y-W.oy)/resC)];
    const okFree=(c,r)=>c>=0&&r>=0&&c<cw&&r<ch&&!blk[r*cw+c];
    const snap=(c,r)=>{ if(okFree(c,r))return[c,r];
      for(let rad=1;rad<25;rad++) for(let dr=-rad;dr<=rad;dr++) for(let dc=-rad;dc<=rad;dc++)
        if(okFree(c+dc,r+dr)) return[c+dc,r+dr];
      return null; };
    let s=snap(...w2c(sx,sy)), g=snap(...w2c(gx,gy)); if(!s||!g) return [];
    const [sc,sr]=s,[gc,gr]=g, BIG=1e9, dist=new Float32Array(cw*ch).fill(BIG);
    dist[gr*cw+gc]=0; const q=[[gc,gr]]; let qi=0;        // unit-cost wavefront from goal
    while(qi<q.length){ const [c,r]=q[qi++], d=dist[r*cw+c]+1;
      for(const [dc,dr] of [[1,0],[-1,0],[0,1],[0,-1]]){ const cc=c+dc,rr=r+dr;
        if(cc<0||rr<0||cc>=cw||rr>=ch||blk[rr*cw+cc]) continue;
        if(d<dist[rr*cw+cc]){ dist[rr*cw+cc]=d; q.push([cc,rr]); } } }
    if(dist[sr*cw+sc]>=BIG) return [];                    // unreachable
    const path=[]; let c=sc,r=sr;                          // descend the field start->goal
    for(let i=0;i<cw*ch;i++){
      path.push([W.ox+(c+0.5)*resC, W.oy+(r+0.5)*resC]);
      if(c===gc&&r===gr) break;
      let best=dist[r*cw+c],nc=c,nr=r;
      for(const [dc,dr] of [[1,0],[-1,0],[0,1],[0,-1]]){ const cc=c+dc,rr=r+dr;
        if(cc>=0&&rr>=0&&cc<cw&&rr<ch&&dist[rr*cw+cc]<best){ best=dist[rr*cw+cc]; nc=cc; nr=rr; } }
      if(nc===c&&nr===r) break; c=nc; r=nr;
    }
    return path;
  }
  function frontBlocked(){                                 // reactive cone, like nav_node._front_blocked
    for(let da=-FRONT;da<=FRONT;da+=0.2){ const a=R.th+da, ca=Math.cos(a), sa=Math.sin(a);
      for(let t=RAD;t<=RAD+STOP;t+=W.res) if(occ(R.x+t*ca, R.y+t*sa)) return true; }
    return false;
  }

  function tick(){
    const dt=0.05;
    const lin=Number($("lin").value)||0.25, ang=Number($("ang").value)||1.5;
    let v=cmd.v*lin, w=cmd.w*ang;
    // auto-nav to a tapped goal: plan (always, to show the path) + drive when Motion
    // is on and the stick is idle — mirrors slam_nav (plan even with motion off).
    if(simGoal){
      const ts=performance.now()/1000;
      const dgoal=Math.hypot(simGoal[0]-R.x, simGoal[1]-R.y);
      if(dgoal<0.12){ simGoal=null; mapGoal=null; nav.path=[]; mapPlan=[]; }
      else{
        if(ts>=nav.nextPlan){    // ~1 Hz replan (also retries when no path was found)
          nav.nextPlan=ts+1.0; nav.path=planPath(R.x,R.y,simGoal[0],simGoal[1]); mapPlan=nav.path.slice();
        }
        if($("mapMotion") && $("mapMotion").checked && !cmd.v && !cmd.w){
          if(frontBlocked()){ v=0; w=ang*0.5; nav.nextPlan=Math.min(nav.nextPlan,ts+0.2); } // stop+replan
          else if(nav.path.length){
            let tx=nav.path[nav.path.length-1][0], ty=nav.path[nav.path.length-1][1];
            for(const [wx,wy] of nav.path) if(Math.hypot(wx-R.x,wy-R.y)>=LOOK){ tx=wx; ty=wy; break; }
            let e=Math.atan2(ty-R.y,tx-R.x)-R.th; e=Math.atan2(Math.sin(e),Math.cos(e));
            w=Math.max(-ang,Math.min(ang, 2.0*e));
            v=Math.abs(e)<0.6 ? lin*0.8 : 0;              // turn in place if badly misaligned
          } else { v=0; w=ang*0.5; }                       // no path: rotate to look for one
        }
      }
    }
    R.th+=w*dt; R.th=Math.atan2(Math.sin(R.th),Math.cos(R.th));
    const nx=R.x+v*Math.cos(R.th)*dt, ny=R.y+v*Math.sin(R.th)*dt;
    if(!blocked(nx,ny)){ R.x=nx; R.y=ny; }      // wall = stop translating (can still turn)
    const now=performance.now();
    if(now-lastTrail>250){ trail.push([R.x,R.y]); if(trail.length>400) trail.shift(); lastTrail=now; }
    $("px").textContent=R.x.toFixed(2); $("py").textContent=R.y.toFixed(2);
    $("pth").textContent=(R.th*180/Math.PI).toFixed(0);
  }

  // ---- raycast lidar (LDS02RR-ish: 12 cm .. 6 m) ----------------------------
  function castRanges(){
    const n=Number($("simBeams").value)||360;
    const rmin=0.12, rmax=6.0, step=W.res*0.5;
    const noise=(Number($("simNoise").value)||0)/100;
    const out=new Float32Array(n);
    for(let i=0;i<n;i++){
      const a=R.th + (-Math.PI + i*(2*Math.PI/n));  // world angle of beam i
      const ca=Math.cos(a), sa=Math.sin(a);
      let hit=Infinity;
      for(let t=rmin;t<=rmax;t+=step){
        if(occ(R.x+t*ca, R.y+t*sa)){ hit=t + (noise?(Math.random()*2-1)*noise:0); break; }
      }
      out[i]=hit;
    }
    return {n,amin:-Math.PI,ainc:2*Math.PI/n,ranges:out};
  }

  // ---- synthetic HTTP responses (byte-for-byte what the page's parsers want) -
  const enc=new TextEncoder();
  let scanSeq=0;
  function scanResponse(){
    const s=castRanges();
    const head=enc.encode(JSON.stringify({seq:++scanSeq,amin:s.amin,ainc:s.ainc,n:s.n})+"\n");
    const body=new Uint8Array(head.length + s.n*4); body.set(head,0);
    const dv=new DataView(body.buffer, head.length);
    for(let i=0;i<s.n;i++) dv.setFloat32(i*4, s.ranges[i], true);
    return new Response(body,{status:200});
  }
  function mapResponse(){
    let free=0; for(let i=0;i<W.cells.length;i++) if(W.cells[i]<50) free++;
    const meta={w:W.w,h:W.h,res:W.res,ox:W.ox,oy:W.oy, px:R.x,py:R.y,pth:R.th,
      mode:"sim", motion:!!($("mapMotion")&&$("mapMotion").checked),
      score:10, seen:1, free_m2:free*W.res*W.res, hx:R.sx,hy:R.sy, trail:trail.slice()};
    const head=enc.encode(JSON.stringify(meta)+"\n");
    const body=new Uint8Array(head.length + W.cells.length); body.set(head,0);
    body.set(new Uint8Array(W.cells.buffer), head.length);
    return new Response(body,{status:200});
  }
  const _fetch=window.fetch.bind(window);
  window.fetch=(input,init)=>{
    if(on){
      const url=(typeof input==="string")?input:((input&&input.url)||"");
      if(url.indexOf("/scan.bin")>=0) return Promise.resolve(scanResponse());
      if(url.indexOf("/map")>=0)      return Promise.resolve(mapResponse());
    }
    return _fetch(input,init);
  };

  // ---- map editor canvas ----------------------------------------------------
  const ec=$("simEdit"), ex=ec.getContext("2d");
  let baseImg=null, baseDirty=true;
  function rebuildBase(){
    if(ec.width!==W.w||ec.height!==W.h){ ec.width=W.w; ec.height=W.h; }
    baseImg=ex.createImageData(W.w,W.h); const d=baseImg.data;
    for(let r=0;r<W.h;r++){ const cy=W.h-1-r;   // row 0 = bottom -> canvas top = max y
      for(let c=0;c<W.w;c++){ const wall=W.cells[idx(c,r)]>=50, o=(cy*W.w+c)*4;
        d[o]=wall?139:14; d[o+1]=wall?149:19; d[o+2]=wall?165:27; d[o+3]=255; } }
    baseDirty=false;
  }
  function drawEditor(){
    if(baseDirty) rebuildBase();
    ex.putImageData(baseImg,0,0);
    const P=(x,y)=>[(x-W.ox)/W.res, W.h-1-(y-W.oy)/W.res];
    const [sx,sy]=P(R.sx,R.sy);                  // start = green ring
    ex.lineWidth=Math.max(1,W.w/200); ex.strokeStyle="#34c759";
    ex.beginPath(); ex.arc(sx,sy,W.w/45,0,6.28); ex.stroke();
    if(simGoal){ const [gx,gy]=P(simGoal[0],simGoal[1]); ex.strokeStyle="#ffd60a";
      ex.beginPath(); ex.arc(gx,gy,W.w/55,0,6.28); ex.stroke(); }
    const [rx,ry]=P(R.x,R.y);                    // robot = red dot + heading
    ex.fillStyle=ex.strokeStyle="#ff3b30";
    ex.beginPath(); ex.arc(rx,ry,W.w/55,0,6.28); ex.fill();
    ex.beginPath(); ex.moveTo(rx,ry);
    ex.lineTo(rx+(W.w/22)*Math.cos(R.th), ry-(W.w/22)*Math.sin(R.th)); ex.stroke();
  }

  let tool="wall", painting=false;
  function evtCell(e){
    const rc=ec.getBoundingClientRect();
    const cx=(e.clientX-rc.left)*ec.width/rc.width, cy=(e.clientY-rc.top)*ec.height/rc.height;
    return {c:Math.floor(cx), r:W.h-1-Math.floor(cy)};
  }
  function paint(e){
    const {c,r}=evtCell(e);
    if(tool==="robot"){ R.sx=W.ox+(c+0.5)*W.res; R.sy=W.oy+(r+0.5)*W.res; resetRobot(); return; }
    const br=Number($("simBrush").value)||3, val=tool==="erase"?0:100;
    for(let dr=-br;dr<=br;dr++) for(let dc=-br;dc<=br;dc++){
      const cc=c+dc, rr=r+dr;
      if(cc<0||rr<0||cc>=W.w||rr>=W.h||dc*dc+dr*dr>br*br) continue;
      W.cells[idx(cc,rr)]=val;
    }
    baseDirty=true;
  }
  ec.addEventListener("pointerdown",e=>{ painting=true; ec.setPointerCapture(e.pointerId); paint(e); });
  ec.addEventListener("pointermove",e=>{ if(painting) paint(e); });
  ec.addEventListener("pointerup",()=>painting=false);
  ec.addEventListener("pointercancel",()=>painting=false);

  $("simTools").querySelectorAll(".tool").forEach(b=>b.onclick=()=>{ tool=b.dataset.tool;
    $("simTools").querySelectorAll(".tool").forEach(x=>x.classList.toggle("active",x===b)); });
  const bind=(id,out)=>{ $(id).oninput=()=>$(out).textContent=$(id).value; };
  bind("simBrush","simBrushV"); bind("simBeams","simBeamsV"); bind("simNoise","simNoiseV");

  $("simBorder").onclick=()=>{
    for(let c=0;c<W.w;c++){ W.cells[idx(c,0)]=100; W.cells[idx(c,W.h-1)]=100; }
    for(let r=0;r<W.h;r++){ W.cells[idx(0,r)]=100; W.cells[idx(W.w-1,r)]=100; }
    baseDirty=true;
  };
  $("simClear").onclick=()=>{ W.cells.fill(0); baseDirty=true; };
  $("simReset").onclick=resetRobot;
  $("simNew").onclick=()=>{ newWorld(Number($("simSize").value)||12);
    R.sx=R.sy=R.sth=0; simGoal=null; resetRobot(); baseDirty=true; };

  // import: a floor-plan image (dark = wall) OR a robot .bin occupancy map
  $("simImport").onchange=e=>{ const f=e.target.files[0]; if(!f) return;
    (/\.bin$/i.test(f.name)?importBin:importImage)(f); e.target.value=""; };
  function importImage(f){
    const img=new Image(), url=URL.createObjectURL(f);
    img.onload=()=>{
      const tc=document.createElement("canvas"); tc.width=W.w; tc.height=W.h;
      const tx=tc.getContext("2d"); tx.drawImage(img,0,0,W.w,W.h);
      const d=tx.getImageData(0,0,W.w,W.h).data;
      for(let r=0;r<W.h;r++) for(let c=0;c<W.w;c++){
        const p=((W.h-1-r)*W.w+c)*4;             // image row 0 = top = max y
        const lum=0.299*d[p]+0.587*d[p+1]+0.114*d[p+2];
        W.cells[idx(c,r)]=(d[p+3]>10 && lum<128)?100:0;
      }
      URL.revokeObjectURL(url); baseDirty=true;
    };
    img.src=url;
  }
  function importBin(f){ f.arrayBuffer().then(ab=>{
    const buf=new Uint8Array(ab), nl=buf.indexOf(10); if(nl<0) return;
    const m=JSON.parse(new TextDecoder().decode(buf.subarray(0,nl)));
    W.res=m.res; W.w=m.w; W.h=m.h; W.ox=m.ox; W.oy=m.oy;
    const grid=buf.subarray(nl+1); W.cells=new Int8Array(W.w*W.h);
    for(let i=0;i<W.cells.length;i++){ let v=grid[i]; if(v>127) v-=256; W.cells[i]=v>=50?100:0; }
    R.sx=R.sy=R.sth=0; simGoal=null; resetRobot(); baseDirty=true;
  }); }

  function download(blob,name){ const a=document.createElement("a");
    a.href=URL.createObjectURL(blob); a.download=name; a.click();
    setTimeout(()=>URL.revokeObjectURL(a.href),1000); }
  $("simExportPng").onclick=()=>{ rebuildBase(); ex.putImageData(baseImg,0,0);
    ec.toBlob(b=>download(b,"nano_world.png")); };
  $("simExportBin").onclick=()=>{
    const head=enc.encode(JSON.stringify({w:W.w,h:W.h,res:W.res,ox:W.ox,oy:W.oy})+"\n");
    const body=new Uint8Array(head.length+W.cells.length);
    body.set(head,0); body.set(new Uint8Array(W.cells.buffer),head.length);
    download(new Blob([body]),"nano_world.bin");
  };

  // ---- master switch + loops ------------------------------------------------
  let physTimer=null;
  function setSim(en){
    on=en;
    if(on){
      if(!physTimer) physTimer=setInterval(tick,50);                 // 20 Hz physics
      const mb=document.querySelector('#viewSwitch button[data-view="map"]'); if(mb) mb.click();
    }else if(physTimer){ clearInterval(physTimer); physTimer=null; }
  }
  $("simOn").addEventListener("change",e=>setSim(e.target.checked));
  // redraw the editor only while it's on screen (or sim running) — cheap, idle-free.
  setInterval(()=>{ if(on || $("panel-sim").classList.contains("active")) drawEditor(); },100);
  drawEditor();
})();
