// ===== OLED mirror — a pixel-exact, fully client-side copy of the physical SSD1306 =====
// Renders the SAME screen the panel shows (dashboard / mood face / TTS karaoke / shutdown),
// from the SAME inputs: /oled_face, /oled_word, /oled_text, /oled_system + live telemetry.
// This is a faithful port of src/oled_display/oled_display/display_node.py — all the drawing
// happens here in the browser, so it works identically on the robot (state in the SSE
// telemetry frame) and on the dev harness (state polled from /oled/state; dev_webui.py). Blink &
// gaze are animated locally with their own RNG, so they aren't frame-identical to the panel,
// but the mode / mood / words / dashboard all mirror exactly.
window.OLED=(function(){
  "use strict";
  const W=128, H=64;                       // logical SSD1306 resolution
  const ON="#cfe9ff", OFF="#000";          // lit pixel (blue-white OLED) / dark
  const FONT='8px "DejaVu Sans Mono","Courier New",monospace';
  const KNOWN=["happy","angry","focused","stress","sleepy","looking"];
  const IDLE_FACE="neutral";               // resting face when no mood is up (== oled_display idle_face)
  const EYE_DX=37, EYE_W=20, EYE_H=26, BLINK_DUR=0.16;   // eye geometry (== display_node.py)

  let cv=null, ctx=null, timer=0;
  // ---- panel state (mirrors DisplayNode fields) ----
  let face="", accent="", word="", brand="", sys="", pinned=false, showWords=true;
  // The EFFECTIVE mood rendered: "" = dashboard (pinned), else a face (the requested mood, or
  // IDLE_FACE between beats) — the single arbiter, mirroring DisplayNode._recompute_mood.
  const effMood=()=>pinned ? "" : (face||IDLE_FACE);
  // The emotion accent rides only an actively-requested mood (not the resting idle face), and
  // `has` treats it as in-play alongside the base shape — mirroring DisplayNode._has.
  const effAccent=()=>(pinned || !face) ? "" : accent;
  const has=m=>effMood()===m || effAccent()===m;
  const tel={espTemp:NaN, espAt:-1e9, imuHz:0, imuAt:-1e9, roll:0, pitch:0, lds:0, ldsAt:-1e9,
             cpu:NaN, mem:NaN, sbc:NaN, ip:"0.0.0.0"};
  // ---- animation state (== _anim_update) ----
  let aT=0, open=1, blinkT=null, dbl=false, nextBlink=0,
      gxf=0, gyf=0, gtx=0, gty=0, nextGaze=0, nextSmile=0, smileUntil=0, frame=0;
  const now=()=>performance.now()/1000;
  const rand=(a,b)=>a+Math.random()*(b-a);

  // ---- primitives (1:1 with PIL's ImageDraw on a 128x64 mono buffer) ----
  function clear(){ ctx.fillStyle=OFF; ctx.fillRect(0,0,W,H); }
  function rect(x0,y0,x1,y1,on){ ctx.fillStyle=on?ON:OFF; ctx.fillRect(x0,y0,x1-x0+1,y1-y0+1); }
  function line(x0,y0,x1,y1){ ctx.strokeStyle=ON; ctx.lineWidth=1; ctx.beginPath();
    ctx.moveTo(x0+0.5,y0+0.5); ctx.lineTo(x1+0.5,y1+0.5); ctx.stroke(); }
  // PIL ellipse takes a bounding box; centre/radii derived from it.
  function bell(x0,y0,x1,y1,on){ const cx=(x0+x1)/2, cy=(y0+y1)/2,
      rx=Math.max(0.5,Math.abs(x1-x0)/2), ry=Math.max(0.5,Math.abs(y1-y0)/2);
    ctx.beginPath(); ctx.ellipse(cx,cy,rx,ry,0,0,Math.PI*2); ctx.fillStyle=on?ON:OFF; ctx.fill(); }
  function arc(x0,y0,x1,y1,a0,a1){ const cx=(x0+x1)/2, cy=(y0+y1)/2,
      rx=Math.abs(x1-x0)/2, ry=Math.abs(y1-y0)/2;
    ctx.strokeStyle=ON; ctx.lineWidth=1; ctx.beginPath();
    ctx.ellipse(cx,cy,rx,ry,0,a0*Math.PI/180,a1*Math.PI/180); ctx.stroke(); }
  function poly(pts,on){ ctx.beginPath(); ctx.moveTo(pts[0][0],pts[0][1]);
    for(let i=1;i<pts.length;i++) ctx.lineTo(pts[i][0],pts[i][1]);
    ctx.closePath(); ctx.fillStyle=on?ON:OFF; ctx.fill(); }
  function textW(s){ ctx.font=FONT; return ctx.measureText(s).width; }
  function text(x,y,s,on){ ctx.fillStyle=on?ON:OFF; ctx.font=FONT;
    ctx.textBaseline="top"; ctx.textAlign="left"; ctx.fillText(s,x,y); }

  // ---- dashboard (== _dashboard_tick / _row) ----
  function row(y,name,alive,value){
    text(2,y,name,1);
    const cx=30, cy=y+1;                    // 6px disc; ring when not alive
    bell(cx,cy,cx+6,cy+6,1);
    if(!alive) bell(cx+1,cy+1,cx+5,cy+5,0);
    text(48,y,value,1);
  }
  function clock(){ return new Date().toTimeString().slice(0,8); }
  function dashboard(){
    const t=performance.now();
    const espUp=t-tel.espAt<4000, imuUp=t-tel.imuAt<2500,
          ldsUp=(t-tel.ldsAt<3000)&&tel.lds>0.1;
    clear();
    rect(0,0,W-1,11,1);                                       // header bar (inverted)
    text(2,2,(brand||"NANOBOT").slice(0,12),0);
    const clk=clock(); text(W-2-textW(clk),2,clk,0);
    text(2,14,tel.ip||"0.0.0.0",1);
    if(!isNaN(tel.sbc)){ const s=Math.round(tel.sbc)+"C"; text(W-2-textW(s),14,s,1); }
    line(0,25,W-1,25);
    row(28,"ESP",espUp, (espUp&&!isNaN(tel.espTemp))?Math.round(tel.espTemp)+"C":"off");
    row(40,"IMU",imuUp, imuUp?Math.round(tel.imuHz)+"Hz":"off");
    row(52,"LDS",ldsUp, ldsUp?tel.lds.toFixed(1)+"Hz":"off");
    [[28,tel.cpu,"CPU"],[40,tel.mem,"RAM"]].forEach(([y,val,lab])=>{
      if(isNaN(val)) return;
      const s=lab+" "+Math.round(val)+"%";
      text(Math.max(0,W-3-textW(s)),y,s,1);
    });
    if(imuUp){                                                // row 52: IMU roll/pitch tilt (deg)
      const sgn=v=>(v>=0?"+":"")+Math.round(v);
      const s="R"+sgn(tel.roll)+" P"+sgn(tel.pitch);
      text(Math.max(0,W-3-textW(s)),52,s,1);
    }
  }

  // ---- TTS karaoke: one word, as big as it fits, centred (== _draw_word) ----
  function drawWord(w){
    clear();
    ctx.textAlign="center"; ctx.textBaseline="middle";
    let fs=H-8; ctx.font="bold "+fs+'px "DejaVu Sans Mono","Courier New",monospace';
    while(fs>6 && ctx.measureText(w).width>W-4){
      fs--; ctx.font="bold "+fs+'px "DejaVu Sans Mono","Courier New",monospace'; }
    ctx.fillStyle=ON; ctx.fillText(w,W/2,H/2);
    ctx.textAlign="left"; ctx.textBaseline="top";
  }

  // ---- face / mood animation (== _anim_update + the *_eye helpers) ----
  function animUpdate(n){
    const dt=n-aT; aT=n;
    if(blinkT!==null){
      blinkT+=dt; const p=blinkT/BLINK_DUR;
      if(p>=1){ blinkT=null; open=1;
        if(dbl){ dbl=false; nextBlink=n+0.18; }
        else if(has("focused")){ nextBlink=n+rand(5,9); dbl=false; }
        else { nextBlink=n+rand(2,5); dbl=Math.random()<0.3; }
      } else open=Math.abs(1-2*p);
    } else if(n>=nextBlink){ blinkT=0; open=1; }
    else open=1;
    if(n>=nextGaze){
      if(Math.random()<0.35){ gtx=0; gty=0; }
      else { gtx=rand(-3,3); gty=rand(-3,3); }
      const fast=(has("focused")||has("looking"));
      nextGaze=n+rand(fast?0.6:1.0, fast?1.8:3.0);
    }
    if(has("happy") && n>=nextSmile){ smileUntil=n+0.45; nextSmile=n+rand(3,7); }
    const k=Math.min(1,dt*6);
    gxf+=(gtx-gxf)*k; gyf+=(gty-gyf)*k;
    if(Math.abs(gtx-gxf)<0.4) gxf=gtx;
    if(Math.abs(gty-gyf)<0.4) gyf=gty;
  }
  function pupil(cx,cy,eh,px,py,pw,ph,sparkle){
    const mx=Math.max(0,EYE_W-pw-2), my=Math.max(0,eh-ph-2);
    const pcx=cx+Math.max(-mx,Math.min(mx,px)), pcy=cy+Math.max(-my,Math.min(my,py));
    bell(pcx-pw,pcy-ph,pcx+pw,pcy+ph,0);
    if(sparkle) bell(pcx-pw+1,pcy-ph+1,pcx-pw+4,pcy-ph+4,1);
  }
  function smileEye(cx,cy){           // happy upward crescent (== _smile_eye)
    const top=cy-16, bot=cy+10;
    bell(cx-EYE_W,top,cx+EYE_W,bot,1);
    bell(cx-EYE_W,top+8,cx+EYE_W,bot+8,0);
  }
  function happyEye(cx,cy,px,py,smiling){
    if(smiling || open<0.3){ smileEye(cx,cy); return; }
    const eh=Math.max(2,Math.round(EYE_H*open));
    bell(cx-EYE_W,cy-eh,cx+EYE_W,cy+eh,1);
    pupil(cx,cy,eh,px,py,7,9,true);
  }
  function brow(cx,cy,inner,eh){      // slanted scowl brow (== _brow)
    const xin=cx+inner*(EYE_W+2), xout=cx-inner*(EYE_W+2), b=Math.round(eh*1.1);
    poly([[xout,cy-eh-3],[xin,cy-eh-3],[xin,cy-eh+b],[xout,cy-eh+3]],0);
  }
  function droop(cx,cy){              // heavy upper eyelid (sleepy accent, == _droop)
    const eh=Math.max(2,Math.round(EYE_H*open));
    rect(cx-EYE_W-1,cy-eh-1,cx+EYE_W+1,cy+Math.round(eh*0.35),0);
  }
  function angryEye(cx,cy,inner,px,py){
    const eh=Math.max(2,Math.round(EYE_H*open));
    if(open<0.25){ line(cx-EYE_W,cy,cx+EYE_W,cy); return; }
    bell(cx-EYE_W,cy-eh,cx+EYE_W,cy+eh,1);
    pupil(cx,cy,eh,px,py,6,7,false);
    brow(cx,cy,inner,eh);
  }
  // Emotion accent ON TOP of the base shape (== _accent_overlay): angry brows / sleepy lids.
  // Happy folds to a smile crescent earlier (faceTick); focused/looking/neutral are cadence-only.
  function accentOverlay(lcx,rcx,cy){
    const a=effAccent();
    if(!a || a===effMood()) return;
    if(a==="angry"){ const eh=Math.max(2,Math.round(EYE_H*open)); brow(lcx,cy,1,eh); brow(rcx,cy,-1,eh); }
    else if(a==="sleepy"){ droop(lcx,cy); droop(rcx,cy); }
  }
  function focusedEye(cx,cy,px,py){
    if(open<0.25){ line(cx-EYE_W,cy,cx+EYE_W,cy); return; }
    const eh=Math.max(3,Math.round(EYE_H*0.6*open));
    bell(cx-EYE_W,cy-eh,cx+EYE_W,cy+eh,1);
    pupil(cx,cy,eh,px,py,5,Math.max(2,eh-3),false);
  }
  function lookingEye(cx,cy,px,py){
    if(open<0.25){ line(cx-EYE_W,cy,cx+EYE_W,cy); return; }
    const eh=Math.max(2,Math.round(EYE_H*open));
    bell(cx-EYE_W,cy-eh,cx+EYE_W,cy+eh,1);
    pupil(cx,cy,eh,px,py,6,8,false);
  }
  function sleepy(n){
    const zc=Math.floor(n*1.2)%3+1;
    clear();
    const cy=H/2+4;
    for(const cx of [W/2-EYE_DX, W/2+EYE_DX])
      for(const dy of [0,1,2]) arc(cx-EYE_W,cy-8+dy,cx+EYE_W,cy+8+dy,20,160);
    const zx=W-30, zy=18;
    for(let i=0;i<zc;i++) text(zx+i*8,zy-i*6,"z",1);
  }
  function stress(){
    clear(); const ph=frame, o=ph%8;
    for(let x=-H;x<W;x+=8) line(x+o,0,x+o+H,H);
    const bx=Math.round(W/2+(W/2-9)*Math.sin(ph*0.5)),
          by=Math.round(H/2+(H/2-9)*Math.cos(ph*0.33));
    bell(bx-9,by-9,bx+9,by+9,0);
    ctx.strokeStyle=ON; ctx.lineWidth=1; ctx.beginPath();
    ctx.ellipse(bx,by,9,9,0,0,Math.PI*2); ctx.stroke();
  }
  function faceTick(){
    const n=now(); animUpdate(n); frame++; const m=effMood();
    if(m==="stress"){ stress(); return; }
    if(m==="sleepy"){ sleepy(n); return; }
    const cy=H/2, lcx=W/2-EYE_DX, rcx=W/2+EYE_DX;
    const px=Math.round(gxf*3.0), py=Math.round(gyf*2.5);
    const smiling=has("happy") && n<smileUntil;
    clear();
    // Happy (base OR accent) folds the whole eye into a smile crescent on the smile beat/blink.
    if(has("happy") && (smiling || open<0.3)){ smileEye(lcx,cy); smileEye(rcx,cy); return; }
    if(m==="angry"){ angryEye(lcx,cy,1,px,py); angryEye(rcx,cy,-1,px,py); }
    else if(m==="focused"){ focusedEye(lcx,cy,px,py); focusedEye(rcx,cy,px,py); }
    else if(m==="looking"){ lookingEye(lcx,cy,px,py); lookingEye(rcx,cy,px,py); }
    else { happyEye(lcx,cy,px,py,smiling); happyEye(rcx,cy,px,py,smiling); }   // happy / neutral
    accentOverlay(lcx,rcx,cy);                                                 // emotion on top
  }

  // ---- end screens (== _shutdown_screen / _restart_screen) ----
  function shutdownScreen(){
    clear(); const cx=W/2, gy=H/2-12, r=8;
    arc(cx-r,gy-r,cx+r,gy+r,300,240); line(cx,gy-r-1,cx,gy+2);
    const msg="Shutting down"; text((W-textW(msg))/2,gy+r+4,msg,1);
  }
  function restartScreen(msg){
    clear(); const cx=W/2, gy=H/2-12, r=8;
    arc(cx-r,gy-r,cx+r,gy+r,70,360);
    const ax=cx+Math.round(r*Math.cos(70*Math.PI/180)),
          ay=gy+Math.round(r*Math.sin(70*Math.PI/180));
    poly([[ax-3,ay-1],[ax+3,ay-1],[ax,ay+4]],1);
    text((W-textW(msg))/2,gy+r+4,msg,1);
  }

  // ---- frame router (== the owner priority in display_node.py) ----
  // Face is the default; the dashboard shows only when pinned (effMood()==="").  Karaoke words
  // take over only while showWords is on — else speech keeps the face up.
  function render(){
    if(!ctx) return;
    const m=effMood();
    if(sys==="shutdown") shutdownScreen();
    else if(sys==="restart"||sys==="reboot") restartScreen(sys==="restart"?"Restarting stack":"Restarting");
    else if(word && showWords) drawWord(word);
    else if(m) faceTick();
    else dashboard();
  }

  function reseed(){ const n=now(); aT=n; open=1; blinkT=null; dbl=false;
    nextBlink=n+rand(1.5,3.5); gxf=gyf=gtx=gty=0;
    nextGaze=n+rand(1.0,2.5); nextSmile=n+rand(2.0,5.0); smileUntil=0; }

  return {
    init(){
      cv=document.getElementById("oledcv"); if(!cv) return;
      cv.width=W; cv.height=H; ctx=cv.getContext("2d");
      ctx.imageSmoothingEnabled=false; reseed();
    },
    // Only animate while the OLED hero view is visible (zero cost otherwise).
    setActive(on){
      if(on && !timer){ render(); timer=setInterval(render,50); }   // ~20 fps
      else if(!on && timer){ clearInterval(timer); timer=0; }
    },
    setFace(raw){ raw=(raw||"").trim().toLowerCase();
      let next="", acc="";
      if(!["","off","dashboard","none"].includes(raw)){
        const i=raw.indexOf(":");                                    // compound "shape:emotion"
        const shape=i<0?raw:raw.slice(0,i); acc=i<0?"":raw.slice(i+1);
        next=KNOWN.includes(shape)?shape:"neutral";
        acc=(KNOWN.includes(acc)&&acc!==next)?acc:"";
      }
      if(next===face && acc===accent) return;
      if(next && !face) reseed();                                    // entering face mode
      face=next; accent=acc;
    },
    setWord(w){ word=(w||"").trim(); },
    setBrand(t){ brand=t||""; },
    setDashboard(on){ on=!!on; if(on===pinned) return;
      if(pinned && !on) reseed();          // unpinning -> re-enter the resting face cleanly
      pinned=on; },
    setShowWords(on){ showWords=!!on; },
    setSystem(s){ s=(s||"").trim().toLowerCase();
      if(["restart","reboot","shutdown"].includes(s)) sys=s; },
    tel(p){ const t=performance.now();
      if("espTemp" in p){ tel.espTemp=p.espTemp; tel.espAt=t; }     // temp also proves ESP alive
      if("espBeat" in p){ tel.espAt=t; }
      if("imuHz" in p){ tel.imuHz=p.imuHz; tel.imuAt=t; }
      if("roll" in p) tel.roll=p.roll;
      if("pitch" in p) tel.pitch=p.pitch;
      if("lds" in p){ tel.lds=p.lds; tel.ldsAt=t; }
      if("cpu" in p) tel.cpu=p.cpu;
      if("mem" in p) tel.mem=p.mem;
      if("sbc" in p) tel.sbc=p.sbc;
      if("ip" in p) tel.ip=p.ip;
    },
  };
})();
OLED.init();
// Dev harness (no /telemetry stream): poll the panel state the robot would stream.
// On the robot we're `connected`, so this never fires (the telemetry frame drives the
// mirror); a 404 here is harmless.
setInterval(()=>{
  if(typeof connected!=="undefined" && connected) return;
  fetch("/oled/state").then(r=>r.ok?r.json():null).then(s=>{
    if(!s) return;
    OLED.setFace(s.face||""); OLED.setWord(s.word||""); OLED.setBrand(s.brand||"");
    if(s.system) OLED.setSystem(s.system);
    const p={imuHz:s.imu_hz, roll:s.roll, pitch:s.pitch, lds:s.lds_hz, cpu:s.cpu, mem:s.mem, sbc:s.temp, ip:s.ip};
    if(s.esp_alive) p.espTemp=s.esp_temp;
    OLED.tel(p);
  }).catch(()=>{});
},500);
