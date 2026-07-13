"use strict";
const $=id=>document.getElementById(id);
const cv=$("cv"), ctx=cv.getContext("2d");

// ---- view state ----
let scale=40, panX=0, panY=0;            // px per metre
let dragging=false,dsx=0,dsy=0,dpx=0,dpy=0;
let frame=[];                            // [{x,y}] metres, latest scan
let scanCount=0, lastHzT=performance.now(), scanHz=0;

// ---- server link state (SSE /telemetry — there is no rosbridge anymore) ----
let connected=false;                     // the /telemetry stream is live
// slam_nav view+plan state (read by the map panel script)
let mapPlan=[], mapGoal=null;

$("connect").onclick=()=>connected?disconnect():connect();
$("fit").onclick=autoFit;
$("lin").oninput=()=>$("linv").textContent=$("lin").value;
$("ang").oninput=()=>$("angv").textContent=$("ang").value;

// set one WHITELISTED parameter on a node live via POST /param (the server calls the
// node's /<node>/set_parameters service). All the tuning sliders/toggles funnel here.
function setParam(node,name,value){
  fetch("/param",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({node,name,value})}).catch(()=>{});
}
// publish on a WHITELISTED topic via POST /publish (see telemetry.py's whitelist).
function pub(topic,value){
  fetch("/publish",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({topic,value})}).catch(()=>{});
}
const setNodeRate=(node,hz)=>setParam(node,"publish_rate",hz);
$("imuRate").oninput=()=>$("imuRateV").textContent=$("imuRate").value;
$("imuRate").onchange=()=>setNodeRate("imu_driver",Number($("imuRate").value));
$("ldsRate").oninput=()=>$("ldsRateV").textContent=$("ldsRate").value;
$("ldsRate").onchange=()=>setNodeRate("lds_driver",Number($("ldsRate").value));
$("odoRate").oninput=()=>$("odoRateV").textContent=$("odoRate").value;
$("odoRate").onchange=()=>setNodeRate("wheel_odometry",Number($("odoRate").value));
// publish a navigation goal in the map frame; slam_nav plans a path toward it.
function mapSetGoal(wx,wy){ pub("/goal_pose",{x:wx,y:wy}); mapGoal=[wx,wy]; }
const setNavMotion=on=>setParam("slam_nav","enable_motion",on);
const setNavExplore=on=>setParam("slam_nav","auto_explore",on);
// slam_nav navigation tuning (Sensors tab) — all whitelisted + live, no restart needed.
$("navMaxLin").oninput=()=>$("navMaxLinV").textContent=$("navMaxLin").value;
$("navMaxLin").onchange=()=>setParam("slam_nav","max_lin",Number($("navMaxLin").value));
$("navMaxAng").oninput=()=>$("navMaxAngV").textContent=$("navMaxAng").value;
$("navMaxAng").onchange=()=>setParam("slam_nav","max_ang",Number($("navMaxAng").value));
$("navStopDist").oninput=()=>$("navStopDistV").textContent=$("navStopDist").value;
$("navStopDist").onchange=()=>setParam("slam_nav","stop_distance",Number($("navStopDist").value));
$("navRadius").oninput=()=>$("navRadiusV").textContent=$("navRadius").value;
$("navRadius").onchange=()=>setParam("slam_nav","robot_radius",Number($("navRadius").value));
$("navStuck").oninput=()=>$("navStuckV").textContent=Number($("navStuck").value)===0?"off":$("navStuck").value+" s";
$("navStuck").onchange=()=>setParam("slam_nav","stuck_timeout",Number($("navStuck").value));
$("navRelocalize").onchange=e=>setParam("slam_nav","relocalize",e.target.checked);
$("navPickupPause").onchange=e=>setParam("slam_nav","pickup_pause",e.target.checked);
$("navLdsIdleEnable").onchange=e=>setParam("slam_nav","lds_idle_enable",e.target.checked);
$("navLdsIdle").oninput=()=>$("navLdsIdleV").textContent=Number($("navLdsIdle").value)===0?"off":$("navLdsIdle").value;
$("navLdsIdle").onchange=()=>setParam("slam_nav","lds_idle_timeout",Number($("navLdsIdle").value));
// LDS spin-speed setpoint -> /lds_target_rpm (Float32). The ESP32 PID holds it.
$("ldsTgt").oninput=()=>$("ldsTgtV").textContent=$("ldsTgt").value;
$("ldsTgt").onchange=()=>publishLdsTgt();
function publishLdsTgt(){ pub("/lds_target_rpm",Number($("ldsTgt").value)); }
// Wheels-up test override -> /pickup_override (Int8, latched): -1 auto, 0 down, 1 up.
$("pickupOv").onchange=()=>publishPickupOv();
function publishPickupOv(){ pub("/pickup_override",Number($("pickupOv").value)); }
// Fan override -> sys_monitor fan_override param.
// v<0 => auto (track CPU temp); 0..1 => forced fixed duty.
const setFanOverride=v=>setParam("sys_monitor","fan_override",v);
function fanApply(){
  const auto=$("fanAuto").checked;
  $("fanOv").disabled=auto;
  $("fanMode").textContent=auto?"auto":"manual";
  setFanOverride(auto?-1:Number($("fanOv").value)/100);
}
$("fanAuto").onchange=fanApply;
$("fanOv").oninput=()=>$("fanOvV").textContent=$("fanOv").value;
$("fanOv").onchange=()=>{ if(!$("fanAuto").checked) setFanOverride(Number($("fanOv").value)/100); };
// Fan start temperature -> sys_monitor fan_temp_min param (°C below which the auto
// curve idles at 0% duty; the ramp runs from here up to fan_temp_max=70°C).
$("fanStart").oninput=()=>$("fanStartV").textContent=$("fanStart").value;
$("fanStart").onchange=()=>setParam("sys_monitor","fan_temp_min",Number($("fanStart").value));
// On (re)connect, push the current Auto/override state once so the node matches the UI.
function syncFan(){ setFanOverride($("fanAuto").checked?-1:Number($("fanOv").value)/100); }
// On (re)connect, push the slider's current value once so the robot's spin setpoint
// matches the UI without a manual drag (the firmware boots at 300 rpm; this keeps the
// shown value authoritative even if it differs).
function syncLdsTgt(){ publishLdsTgt(); }

// Webcam MJPEG: the <img> streams /stream.mjpg (same-origin, served by the web
// server). Toggling off drops the connection so the camera stops (ref-counted).
// camMode swaps which endpoint is used -- the normal feed, the GPU vision colour-
// threshold tracking mask (/stream_mask.mjpg), or the motion-diff mask
// (/stream_motion_mask.mjpg) -- see gpu_vision.py's mask viewers. Only one mode is
// active at a time; clicking the active mode's button returns to the normal feed.
let camMode="normal";
function camStreamUrl(){
  const path = camMode==="color" ? "/stream_mask.mjpg"
             : camMode==="motion" ? "/stream_motion_mask.mjpg"
             : "/stream.mjpg";
  return path+"?t="+Date.now();
}
// camWait: a proper "camera disabled"/"camera unavailable" message instead of a
// blank red <img> broken-image square. lastCameraEnabled tracks the master switch
// (see set_camera_enable in web_server.py) as last reported by telemetry (onVision
// below), so the message is specific ("disabled" vs a generic connection failure) and
// -- the main point -- re-enabling the camera auto-retries the stream on its own
// instead of requiring a manual page refresh.
let lastCameraEnabled=true;
function showCamWait(){
  const img=$("cam"), wait=$("camWait");
  img.style.display="none";
  wait.textContent = lastCameraEnabled
    ? "⚠ Camera unavailable — check the connection"
    : "📷 Camera disabled — enable it in Sensors → Camera (GPU vision)";
  wait.style.display="flex";
}
function hideCamWait(){ $("camWait").style.display="none"; }
$("camOn").addEventListener("change",e=>{
  const img=$("cam"), hint=$("camHint");
  if(e.target.checked){
    hint.style.display="block";
    if(lastCameraEnabled){ img.src=camStreamUrl(); img.style.display="block"; hideCamWait(); }
    else showCamWait();
  }
  else { img.removeAttribute("src"); img.style.display="none"; hideCamWait(); hint.style.display="none"; }
});
$("cam").onerror=()=>{ if($("camOn").checked) showCamWait(); };
$("cam").onload=()=>{ hideCamWait(); $("cam").style.display="block"; };
function setCamMode(mode){
  camMode = (camMode===mode) ? "normal" : mode;
  $("visionMaskToggle").textContent=camMode==="color"?"🎥 Show normal feed":"🎭 Show tracking mask";
  $("visionMaskToggle").classList.toggle("primary",camMode==="color");
  $("visionMotionMaskToggle").textContent=camMode==="motion"?"🎥 Show normal feed":"👣 Show motion mask";
  $("visionMotionMaskToggle").classList.toggle("primary",camMode==="motion");
  if($("camOn").checked){ $("cam").style.background=""; $("cam").src=camStreamUrl(); }
}
$("visionMaskToggle").onclick=()=>setCamMode("color");
$("visionMotionMaskToggle").onclick=()=>setCamMode("motion");

// OLED mask mirror: show the tracking mask on the robot's physical 128x64 panel
// (POST /vision/oled_mask; gpu_vision writes /dev/shm/nano_oled_mask.bin, oled_display
// renders it). lastOledMask is synced FROM telemetry in onVision, so this button always
// toggles the real server state, not a local guess.
let lastOledMask=false;
$("visionOledMask").onclick=()=>{
  fetch("/vision/oled_mask",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({enabled:!lastOledMask})})
    .then(r=>r.json()).then(d=>{ if(d&&d.error) alert(d.error); }).catch(()=>{});
};

// Named colour-target palette (GET /vision/targets + select/delete): calibrations are
// saved server-side under the name typed in #visionTargetName and survive restarts.
function loadVisionTargets(){
  fetch("/vision/targets").then(r=>r.ok?r.json():null).then(d=>{
    if(!d||!d.targets) return;
    const sel=$("visTargetSel"), names=Object.keys(d.targets);
    sel.innerHTML="";
    if(!names.length){ sel.innerHTML='<option value="">(none stored)</option>'; return; }
    names.forEach(n=>{
      const o=document.createElement("option");
      o.value=n; o.textContent=n+(n===d.active?" (active)":"");
      if(n===d.active) o.selected=true;
      sel.appendChild(o);
    });
  }).catch(()=>{});
}
$("visTargetUse").onclick=()=>{
  const name=$("visTargetSel").value;
  if(!name) return;
  fetch("/vision/target_select",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name})}).then(r=>r.json()).then(d=>{
      if(d&&d.error){ alert(d.error); return; }
      // the server restored the stored blob tuning -- sync the sliders to it
      if(d&&d.target){
        const t=d.target;
        $("visBlobThresh").value=Math.round((t.threshold||0.22)*100);
        $("visBlobThreshV").textContent=$("visBlobThresh").value;
        $("visBlobMin").value=Math.round((t.min_confidence||0)*100);
        $("visBlobMinV").textContent=$("visBlobMin").value;
        $("visBlobMax").value=Math.round((t.max_confidence!=null?t.max_confidence:1)*100);
        $("visBlobMaxV").textContent=$("visBlobMax").value;
      }
      loadVisionTargets();
    }).catch(()=>{});
};
$("visTargetDel").onclick=()=>{
  const name=$("visTargetSel").value;
  if(!name || !confirm(`Forget the stored target colour "${name}"?`)) return;
  fetch("/vision/target_delete",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name})}).then(()=>loadVisionTargets()).catch(()=>{});
};
loadVisionTargets();

// Webcam mic: stream raw S16LE PCM from /audio.pcm and play it via the Web Audio
// API. We schedule small AudioBuffers on a short jitter buffer (~180 ms) for low
// latency; if playback falls behind (underrun) we resync to "now". Toggling off
// aborts the fetch, which drops the connection and stops arecord (ref-counted).
let audioCtx=null, micAbort=null, micPlayhead=0, micRaf=0, micAnalyser=null, micBuf=null;
const MIC_LEAD=0.18;                 // s of jitter buffer (latency vs. dropouts)
function micSet(s){ $("micState").textContent=s; }
// Level meter: read the live audio through an AnalyserNode each animation frame
// (the canonical Web Audio way), take the peak deviation, and show it with a
// smooth fall-off (VU style). Driven by the same audio you hear.
function micMeter(){
  const bar=$("micBar"), pct=$("micPct"); let shown=0;
  const tick=()=>{
    let peak=0;
    if(micAnalyser){
      micAnalyser.getByteTimeDomainData(micBuf);
      for(let i=0;i<micBuf.length;i++){ const d=Math.abs(micBuf[i]-128); if(d>peak) peak=d; }
      peak/=128;                      // 0..1
    }
    shown = Math.max(peak, shown*0.86);
    bar.style.width = Math.min(100, Math.sqrt(shown)*100).toFixed(1)+"%";  // perceptual
    bar.style.background = shown>0.8 ? "var(--red)" : shown>0.4 ? "var(--amber)" : "var(--green)";
    if(pct) pct.textContent = Math.round(shown*100)+"%";
    micRaf = requestAnimationFrame(tick);
  };
  tick();
}
async function micStart(){
  try{
    audioCtx = audioCtx || new (window.AudioContext||window.webkitAudioContext)();
    await audioCtx.resume();
    micAnalyser = audioCtx.createAnalyser();        // metering tap (source->analyser->out)
    micAnalyser.fftSize = 2048;
    micBuf = new Uint8Array(micAnalyser.fftSize);
    micAnalyser.connect(audioCtx.destination);
    micAbort = new AbortController();
    const res = await fetch("/audio.pcm?t="+Date.now(), {signal:micAbort.signal});
    if(!res.ok || !res.body){ micSet("(unavailable)"); return; }
    const rate = Number(res.headers.get("X-Sample-Rate")) || 16000;
    micPlayhead = 0; micSet("● live"); micMeter();
    const reader = res.body.getReader();
    let carry = new Uint8Array(0);
    for(;;){
      const {value,done} = await reader.read();
      if(done) break;
      // join any odd trailing byte from the previous chunk (16-bit samples)
      let buf = value;
      if(carry.length){ const t=new Uint8Array(carry.length+value.length); t.set(carry); t.set(value,carry.length); buf=t; carry=new Uint8Array(0); }
      const n = buf.length & ~1; if(n<buf.length) carry = buf.slice(n);
      const samples = n>>1; if(!samples) continue;
      const dv = new DataView(buf.buffer, buf.byteOffset, n);
      const ab = audioCtx.createBuffer(1, samples, rate);
      const ch = ab.getChannelData(0);
      for(let i=0;i<samples;i++) ch[i] = dv.getInt16(i*2,true)/32768;
      const src = audioCtx.createBufferSource();
      src.buffer = ab; src.connect(micAnalyser);   // through the meter tap to output
      const now = audioCtx.currentTime;
      if(micPlayhead < now + 0.01) micPlayhead = now + MIC_LEAD;  // (re)sync on underrun
      src.start(micPlayhead); micPlayhead += ab.duration;
    }
  }catch(e){ if(e.name!=="AbortError") micSet("(error)"); }
}
function micStop(){
  if(micAbort){ micAbort.abort(); micAbort=null; }
  if(micRaf){ cancelAnimationFrame(micRaf); micRaf=0; }
  micAnalyser=null; $("micBar").style.width="0%"; $("micPct").textContent="";
  micSet("");
}
$("micOn").addEventListener("change",e=>{ e.target.checked ? micStart() : micStop(); });

// Power controls -> POST to the web server (same origin). Both are confirmed.
// The server publishes /oled_system itself so the physical panel flips immediately.
$("btnReset").onclick=()=>{
  if(!confirm("Restart the whole ROS stack? The page will reconnect in a few seconds.")) return;
  fetch("/system/restart",{method:"POST"}).catch(()=>{});
  setConn(false);
};
$("btnReboot").onclick=()=>{
  if(!confirm("REBOOT the SBC?\nThe whole board restarts — the page reconnects once it boots back up.")) return;
  fetch("/system/reboot",{method:"POST"}).catch(()=>{});
  setConn(false);
};
$("btnShutdown").onclick=()=>{
  if(!confirm("SHUT DOWN the SBC?\nIt powers off completely — you must turn it back on by hand.")) return;
  fetch("/system/shutdown",{method:"POST"}).catch(()=>{});
  setConn(false);
};

// ---- Stress test mode ---- pegs every CPU core with niced (lowest-priority) worker
// processes server-side (see stress.py) to validate the watchdog/fan-curve hardening
// under real load; the niceness is what keeps this very page responding throughout, not
// a reserved core. Status is polled only while a run might be active (started here, or
// left running by another tab/reload — one poll on load picks that up).
let stressTimer=null;
function stressRender(d){
  const st=$("stressStatus");
  if(!d || !d.active){
    st.textContent="idle"; $("stressWorkers").textContent="–"; $("stressRemaining").textContent="–";
    if(stressTimer){ clearInterval(stressTimer); stressTimer=null; }
    return;
  }
  st.textContent="running";
  $("stressWorkers").textContent=d.cpu_workers;
  $("stressRemaining").textContent=Math.ceil(d.remaining)+"s";
  if(!stressTimer) stressTimer=setInterval(stressPoll,1000);
}
function stressPoll(){
  fetch("/stress/status").then(r=>r.ok?r.json():null).then(stressRender).catch(()=>{});
}
$("stressDur").oninput=()=>$("stressDurV").textContent=$("stressDur").value;
$("stressStart").onclick=()=>{
  fetch("/stress/start",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({duration:Number($("stressDur").value)})})
    .then(r=>r.json()).then(d=>{ if(d&&d.error){ alert(d.error); return; } stressRender(d); })
    .catch(()=>{});
};
$("stressStop").onclick=()=>fetch("/stress/stop",{method:"POST"})
  .then(r=>r.json()).then(stressRender).catch(()=>{});
stressPoll();

// Camera snapshot: one still JPEG in a new tab (the server ref-counts the camera,
// so this works with the live stream off). Cache-busted so each click is a new grab.
$("camShot").onclick=()=>window.open("/snapshot.jpg?t="+Date.now(),"_blank");

// GPU vision click-to-calibrate: arm, then the next click on the video samples that
// pixel's colour client-side (canvas) and POSTs it -- see onCamPickClick above.
$("visionPick").onclick=()=>visionPickArmed?disarmVisionPick():armVisionPick();
$("visionClear").onclick=()=>{
  disarmVisionPick();
  fetch("/vision/calibrate",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({clear:true})}).then(()=>loadVisionTargets()).catch(()=>{});
  resetBlobTuneUI();   // clearing also resets tuning server-side, same as a fresh pick
};
$("cam").addEventListener("click",onCamPickClick);

// Manual mode: direct hardware-MJPEG passthrough, bypassing GPU vision entirely
// (zero CPU/GPU cost, but pauses PIR/blob-tracking/dark-reflex -- see WebServerNode.
// vision_manual). Two checkboxes drive the same server state -- one over the Camera
// view, one on the Sensors "Camera (GPU vision)" card, so the pipeline can be killed
// without switching views. Both are synced FROM telemetry (see onVision), so these
// handlers only fire on a genuine user click, not the programmatic sync; each also
// mirrors its own state onto the other checkbox immediately, instead of waiting for
// the next telemetry tick to catch up.
function setVisManual(enabled){
  $("visManual").checked=enabled; $("visManual2").checked=enabled;
  fetch("/vision/manual",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({enabled})}).catch(()=>{});
}
$("visManual").onchange=()=>setVisManual($("visManual").checked);
$("visManual2").onchange=()=>setVisManual($("visManual2").checked);

// Master camera on/off (see set_camera_enable in web_server.py) -- stops GpuVision AND
// the direct passthrough entirely, distinct from manual mode (which still runs the
// direct passthrough). Same synced-from-telemetry shape as setVisManual above.
$("visCameraEnable").onchange=()=>{
  const enabled=$("visCameraEnable").checked;
  fetch("/vision/camera_enable",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({enabled})}).catch(()=>{});
};

// Health events: poll sys_monitor's durable outage log only while the box is open
// (each poll is an HTTP round-trip to the board — no point when nobody's looking).
(function(){
  const box=$("healthBox"), out=$("healthLog"); let timer=null;
  function load(){
    fetch("/health/log").then(r=>r.json()).then(d=>{
      const lines=(d.lines||[]);
      out.textContent=lines.length?lines.slice().reverse().join("\n"):"no events logged yet";
    }).catch(()=>{ out.textContent="log unavailable (dev harness / server too old)"; });
  }
  box.addEventListener("toggle",()=>{
    clearInterval(timer); timer=null;
    if(box.open){ load(); timer=setInterval(load,5000); }
  });
})();

// Live telemetry: ONE EventSource on /telemetry replaces every rosbridge
// subscription — the server pushes a compact JSON frame ~5x/s with all the light
// readouts (odom/IMU/diagnostics/ESP32/LDS/OLED mirror/brain). /scan and /map stay
// on their /dev/shm+HTTP polls (heaviest data, unchanged). Auto-reconnects with a
// gentle backoff across a stack restart / reboot; manual Disconnect stops retrying.
// On the dev harness /telemetry doesn't exist, so the page shows disconnected and
// the HTTP pollers (brain card, OLED mirror) take over — same behaviour as before.
let es=null, wantConn=true, reconnT=null, reconnDelay=1000;
function scheduleReconnect(){
  if(!wantConn || reconnT) return;
  $("conn").textContent="reconnecting…";
  reconnT=setTimeout(()=>{ reconnT=null; if(wantConn && !connected) connect(); }, reconnDelay);
  reconnDelay=Math.min(10000, reconnDelay*1.5);
}
function connect(){
  wantConn=true;
  if(reconnT){ clearTimeout(reconnT); reconnT=null; }
  if(es){ es.close(); es=null; }
  es=new EventSource("/telemetry");
  es.onopen=()=>{
    connected=true; reconnDelay=1000; setConn(true);
    OLED.tel({ip:location.hostname||"robot"});
    // Re-assert the page's authoritative state on every (re)connect: LDS setpoint,
    // fan override, pickup override (a fresh load publishes auto = clears stale
    // overrides), and the OLED dashboard/words toggles.
    syncLdsTgt(); syncFan(); publishPickupOv(); sendDash(); sendWords();
  };
  es.onmessage=e=>{ try{ onFrame(JSON.parse(e.data)); }catch(err){} };
  es.onerror=()=>{
    if(es){ es.close(); es=null; }
    connected=false; setConn(false); scheduleReconnect();
  };
}
function disconnect(){
  wantConn=false;
  if(reconnT){ clearTimeout(reconnT); reconnT=null; }
  if(es){ es.close(); es=null; }
  connected=false; setConn(false);
}
function setConn(ok){
  $("dot").classList.toggle("ok",ok);
  $("conn").textContent=ok?"connected":"disconnected";
  $("connect").textContent=ok?"Disconnect":"Connect";
  $("connect").classList.toggle("primary",!ok);
}

// ---- telemetry frame fan-out: one frame updates every readout ----------------
let rawPurpose="", rawTask="", rawExp="", rawSelftest="", rawSchedule="";
function onFrame(f){
  if(f.odom) onOdom(f.odom);
  if(f.imu) onImu(f.imu);
  if(f.eul) onEul(f.eul);
  if(f.diag) onDiag(f.diag);
  onEsp(f.esp||{}, f.susp||[]);
  onLds(f.lds||{});
  if(f.fan!==undefined) $("fanDuty").textContent=(f.fan*100).toFixed(0)+"%";
  if(f.plan) mapPlan=f.plan;
  onVision(f.vision);
  if(f.selftest && f.selftest!==rawSelftest){ rawSelftest=f.selftest;
    const el=$("mapTestOut"); el.style.display="block"; el.textContent=f.selftest; }
  // Brain readouts arrive as the same latched JSON strings the behaviour node
  // publishes; only re-render when they actually change (frames tick ~5 Hz).
  if(f.purpose && f.purpose!==rawPurpose){ rawPurpose=f.purpose; renderPurpose(f.purpose); }
  if(f.task && f.task!==rawTask){ rawTask=f.task; renderTask(f.task); }
  if(f.experiments && f.experiments!==rawExp){ rawExp=f.experiments; renderExperiments(f.experiments); }
  if(f.schedule && f.schedule!==rawSchedule){ rawSchedule=f.schedule; onScheduleFrame(f.schedule); }
  // OLED mirror: feed the client-side panel copy the same inputs the physical
  // panel renders from. (Dashboard/words toggles are page-owned — sendDash/sendWords.)
  const o=f.oled||{};
  OLED.setFace(o.face||""); OLED.setWord(o.word||""); OLED.setBrand(o.brand||"");
  if(o.system) OLED.setSystem(o.system);
}

// GPU vision (gpu_vision.py): absent from the frame entirely when gpu_vision_enable
// is off, so the Sensors card just shows "off" -- no behavior change otherwise. Runs
// continuously server-side regardless of whether the Camera tab is open, so this
// readout works even with the video view never opened. The status numbers live in the
// Sensors panel only (no on-video badge, per user preference) -- the crosshair is kept
// on the video itself since it's a spatial indicator, not a text label.
// Hover explanations for the Sensors tab (both "System health" and "Camera (GPU
// vision)" cards) -- readouts AND every tunable slider/toggle. Applied as plain
// native `title` attributes (zero new UI component, browser handles the hover) via
// element ID, so this needed no changes to the HTML markup itself. Toggleable
// (#hintsToggle) and persisted across reloads via localStorage, since it's a pure
// display preference, not robot state.
const HINTS = {
  // System health
  sysCpu: "Aggregate CPU busy % across all cores (from /proc/stat deltas). Tap to expand per-core.",
  sysLoad: "1-minute load average (/proc/loadavg) -- roughly how many processes are waiting for CPU.",
  sysMem: "RAM used/total (from /proc/meminfo: MemTotal - MemAvailable).",
  sysTemp: "SBC CPU die temperature, from the cpu-thermal sysfs zone.",
  sysDisk: "Root filesystem usage percent.",
  sysWifi: "WiFi signal strength (dBm) and link quality (green ≥ -60dBm, amber ≥ -75, red below).",
  sysUp: "Time since this stack process last started (not the board's boot time).",
  // Camera (GPU vision) card -- toggles
  visCameraEnable: "Master camera switch -- turns off BOTH GPU vision AND the direct passthrough entirely. Distinct from Manual mode, which still runs the direct feed.",
  visManual2: "Bypasses the whole GPU pipeline for a raw hardware MJPEG passthrough -- zero CPU/GPU cost, but PIR/blob-tracking/dark-reflex/mask views all pause while on.",
  visDarkAuto: "Auto-toggles the ESP32's /led from frame brightness, with hysteresis (on/off thresholds must stay apart to avoid flicker).",
  // Camera card -- readouts
  visMotion2: "PIR-style motion score: average per-pixel frame-to-frame change (0-100%). Near zero = static scene.",
  visMotionCenter2: "Where in frame motion is concentrated (x%,y% from top-left), weighted by change magnitude.",
  visTarget2: "Colour-blob tracker lock status -- \"locked N%\" means N% of the frame matches the calibrated colour.",
  visIntercept2: "Kinetic intercept: growth rate of the tracked colour target's size (per second) -- high + sustained = approaching the lens.",
  visMotionIntercept2: "Same growth-rate idea, but for raw motion -- no calibrated colour needed, flags anything looming.",
  visMotionTargetMatch2: "Distance between the motion centroid and the tracked colour's centroid -- small = the thing that's moving IS the tracked target.",
  visLuma2: "Average frame brightness (0-100%), same shader pass the flashlight/dark reflex uses.",
  visObstructed2: "Flags a flat, dark frame (hand/dust over the lens) -- shows the raw flatness (variance) number plus the alert.",
  visColorCast2: "Average scene colour split by channel (R/G/B%) -- flags a strong colour cast (e.g. warm incandescent light).",
  visEdgeDensity2: "How much texture/contrast is in the whole frame -- a static complement to \"motion,\" flags visual clutter.",
  visOverhead2: "Same texture signal, cropped to just the top 30% of frame -- a heuristic for structure above the lidar's 2D scan plane (an overhang the lidar can't see).",
  visShiny2: "Fraction of frame matching a fixed near-white colour -- flags shiny/wet/reflective surfaces.",
  visBacklit2: "Flags a bright spot in an otherwise dim scene (e.g. a window behind a dark subject) -- shows the brightness delta.",
  visFocusBlur2: "Flags a sudden texture drop while the scene is still lit -- something close/blocking the lens, distinct from full darkness.",
  visGpuDuty2: "Software-measured fraction of each frame's time budget spent in the shader+readback pipeline. Over 100% means the vision loop is falling behind the configured fps.",
  visBumper2: "Optical virtual bumper: commanded to move but no visual motion detected for a while -- a possible wheel stall/slip.",
  visBumperCmd2: "The /cmd_vel currently being commanded, as seen by the bumper check.",
  visBumperHeld2: "How long the stall condition has held, while commanded to move.",
  // Dark reflex sliders
  visDarkOn: "Brightness below which the LED turns on.",
  visDarkOff: "Brightness above which the LED turns back off (kept above the on-threshold to avoid flicker).",
  // Optical bumper tuning
  visBumpEps: "Minimum commanded speed (linear or angular) to count as \"actually driving.\"",
  visBumpFloor: "GPU motion score below this counts as \"nothing visibly moved.\"",
  visBumpSecs: "How long the stall condition must hold before the bumper alerts.",
  // Blob tracking tuning
  visBlobThresh: "Colour-match tolerance -- how close a pixel must be to the calibrated colour to count (higher = more forgiving).",
  visBlobMin: "Minimum matched fraction of frame to count as a real lock (raise to reject a few stray matching pixels).",
  visBlobMax: "Maximum matched fraction of frame to still count as a lock (lower to reject a colour that also matches the background).",
  // Vision alerts tuning (all initial guesses except var_max, informed by a real reading)
  visObstrVar: "Luma-variance ceiling for \"flat/textureless\" -- lower = stricter. Tuned from a real reading (400, vs. an ordinary scene's ~2700).",
  visObstrDark: "Brightness ceiling, combined with flatness above, for the obstruction alert -- both conditions must hold.",
  visClutter: "Visual-interest (edge-density) ceiling above which the scene counts as \"cluttered.\"",
  visOverhead: "Same edge-density ceiling, for the top-of-frame \"overhead structure\" signal.",
  visFocusBlur: "Edge-density floor below which (while still lit) the scene counts as defocused/blocked.",
  visBacklit: "Brightness-delta floor (brightest patch minus average) for the backlit alert.",
  visHighlight: "Highlight-fraction ceiling above which the scene counts as \"shiny.\"",
  visLooming: "Motion-intercept-rate ceiling above which something counts as looming.",
  visColorcast: "Colour-channel spread (max minus min of R/G/B) above which the scene counts as colour-cast.",
  visMotionTarget: "Distance BELOW which the motion centroid counts as matching the tracked colour target.",
  // 2026-07-13 batch
  visTargetName2: "Which named palette entry is currently being tracked (see the Named targets row / the target-name box in the Camera view).",
  visApproach2: "Motion growing fast AND centred in frame = someone/something walking up to the robot. Drives the behaviour layer's anticipatory greeting.",
  visNovelty2: "How different the scene looks vs. a slow ~20s background average -- stays high while something is genuinely different, then habituates. Boosts the autonomous \"looking\" beat.",
  visCamFreeze2: "Flags a capture path that stopped delivering frames (or keeps delivering the identical buffer) -- a USB/driver wedge, distinct from the optical bumper's wheel-stall case.",
  visVibration2: "Image much blurrier while driving than the standing-still baseline -- excess chassis vibration (loose screw, wheel imbalance). A maintenance hint, not a stop.",
  visTargetSel: "The persistent calibration palette -- pick a stored colour target and press Track to make it live.",
  visTargetUse: "Make the selected stored target the live tracked colour (also restores its saved tuning).",
  visTargetDel: "Forget the selected stored target. Deleting the active one stops tracking.",
  visGlare: "Glare rejection: derates the blob lock confidence by the frame's specular-highlight fraction, so a shiny reflection matching the tracked hue can't hold a false lock. 0 = off.",
  visNoveltyAlert: "Novelty score above which the \"scene changed\" alert fires.",
  visCamStall: "Seconds without a new frame (or with an exactly-zero diff) before the camera counts as frozen.",
  visVibRatio: "Driving edge-density below this fraction of the standing-still baseline counts as excess blur.",
  visVibSecs: "How long the excess blur must hold before the vibration flag fires.",
  visApproachRate: "Motion growth rate above which something counts as approaching.",
  visApproachBand: "How close to frame centre the motion must be to count as approaching (rules out edge clutter).",
  visionTargetName: "Name this calibration ('ball', 'dock marker', ...) -- every pick is saved to a persistent palette under this name.",
  visionOledMask: "Mirror the tracking mask to the robot's physical OLED (128x64) -- the face yields while it's on.",
};
function applyHints(on){
  for(const [id,text] of Object.entries(HINTS)){
    const el=document.getElementById(id);
    if(!el) continue;
    if(on) el.title=text; else el.removeAttribute("title");
  }
}
let hintsOn = localStorage.getItem("nano_hints")!=="off";
$("hintsToggle").checked=hintsOn;
applyHints(hintsOn);
$("hintsToggle").onchange=()=>{
  hintsOn=$("hintsToggle").checked;
  localStorage.setItem("nano_hints", hintsOn?"on":"off");
  applyHints(hintsOn);
};

let visManualSynced=false, blobTuneSynced=false, cameraEnableSynced=false;
function onVision(v){
  const manualRow=$("visManualRow"), manualRow2=$("visManualRow2"), cross=$("visionCrosshair");
  const ids=["visMotion2","visMotionCenter2","visTarget2","visTargetName2","visIntercept2",
    "visMotionIntercept2","visMotionTargetMatch2","visApproach2","visNovelty2","visLuma2",
    "visObstructed2","visCamFreeze2","visVibration2","visColorCast2","visEdgeDensity2",
    "visOverhead2","visShiny2","visBacklit2","visFocusBlur2","visGpuDuty2","visBumper2"];
  if(!v){
    if(manualRow) manualRow.style.display="none";   // no GpuVision instance at all -- nothing to toggle
    if(manualRow2) manualRow2.style.display="none";
    if(cross) cross.style.display="none";
    ids.forEach(id=>{ const el=$(id); if(el) el.textContent="off"; });
    return;
  }
  // The manual-mode rows only make sense when a GpuVision instance exists to switch
  // away from/back to (gpu_vision_enable was true at startup) -- shown regardless of
  // whether manual mode is currently on, so the operator can toggle it either way.
  if(manualRow) manualRow.style.display="flex";
  if(manualRow2) manualRow2.style.display="flex";
  if(!visManualSynced){    // reflect server state once, without fighting the user's own clicks
    $("visManual").checked=!!v.manual; $("visManual2").checked=!!v.manual; visManualSynced=true;
  }
  if(!cameraEnableSynced && v.camera_enabled!=null){
    $("visCameraEnable").checked=!!v.camera_enabled; cameraEnableSynced=true;
  }
  // Sync the blob-tuning sliders to whatever's ACTUALLY active once (e.g. a target was
  // already calibrated from a previous session, or another browser tab) -- after that,
  // resetBlobTuneUI() on a fresh pick/clear keeps them in sync without fighting drags.
  if(!blobTuneSynced && v.has_target_color && v.blob_tuning){
    const [th,mn,mx]=v.blob_tuning;
    $("visBlobThresh").value=Math.round(th*100); $("visBlobThreshV").textContent=Math.round(th*100);
    $("visBlobMin").value=Math.round(mn*100); $("visBlobMinV").textContent=Math.round(mn*100);
    $("visBlobMax").value=Math.round(mx*100); $("visBlobMaxV").textContent=Math.round(mx*100);
    blobTuneSynced=true;
  }
  const camOff=(v.camera_enabled===false);
  // Track the master camera switch's state across ticks so the live-view <img> reacts
  // immediately (not just on its own onerror) -- disabling shows the message even if
  // this specific connection hasn't errored yet (a mid-stream toggle only takes effect
  // on the NEXT connection, see web_server.py's _cam property comment); re-enabling
  // auto-retries the stream instead of leaving the user needing a manual page refresh.
  const wasCameraEnabled=lastCameraEnabled;
  lastCameraEnabled=!camOff;
  if($("camOn").checked){
    if(camOff) showCamWait();
    else if(!wasCameraEnabled){
      $("cam").src=camStreamUrl(); $("cam").style.display="block"; hideCamWait();
    }
  }
  const paused=!!v.manual||camOff;
  const pausedReason=camOff?"camera off":"paused (manual mode)";
  [$("visionPick"),$("visionClear")].forEach(b=>{ if(b) b.disabled=paused; });
  // OLED mask mirror: label + highlight follow the server state (v.oled_mask), so a
  // toggle from another tab -- or the server auto-dropping it when the camera stops --
  // is always reflected here.
  lastOledMask=!!v.oled_mask;
  const om=$("visionOledMask");
  if(om){ om.disabled=paused;
    om.classList.toggle("primary",lastOledMask);
    om.textContent=lastOledMask?"🖥 Stop OLED mirror":"🖥 Mask → OLED"; }
  // A reported target has already passed the server's min/max blob-size gate (see
  // GpuVision._loop / the "Blob tracking tuning" sliders) -- that IS the lock condition
  // now, so there's no separate confidence floor here to fight the tuning sliders.
  const locked=!paused && !!v.target;
  const set=(id,txt)=>{ const el=$(id); if(el){ el.textContent=txt; el.style.opacity=paused?0.5:1; } };
  set("visMotion2", paused?pausedReason:(v.motion*100).toFixed(1)+"%");
  set("visMotionCenter2", paused?"–":(v.motion_center
    ? `${(v.motion_center[0]*100).toFixed(0)}%,${(v.motion_center[1]*100).toFixed(0)}%` : "none"));
  set("visTarget2", paused?"–":(v.target ? "locked "+(v.target[2]*100).toFixed(0)+"%"
                              : (v.has_target_color ? "searching" : "no target set")));
  set("visTargetName2", v.target_name||"–");
  const alerts=v.alerts||{};
  set("visApproach2", paused?"–":(v.approach?"⚠ something approaching":"no"));
  set("visNovelty2", paused?"–":((v.novelty||0)*100).toFixed(1)+"%"
    +(alerts.novelty?" ⚠ scene changed":""));
  const cfEl=$("visCamFreeze2");
  if(cfEl){ cfEl.textContent=paused?"–":(alerts.camera_freeze?"⚠ FROZEN":"live"
      +(v.frame_age!=null?` (${v.frame_age.toFixed(1)}s)`:""));
    cfEl.style.color=(!paused && alerts.camera_freeze)?"var(--red)":""; cfEl.style.opacity=paused?0.5:1; }
  set("visVibration2", paused?"–":(alerts.vibration?"⚠ excess blur while driving":"clear"));
  set("visIntercept2", paused?"–":(v.target ? (v.intercept_rate*100).toFixed(0)+"%/s" : "–"));
  set("visMotionIntercept2", paused?"–":(v.motion_intercept_rate*100).toFixed(0)+"%/s"
    +(!paused && alerts.looming?" ⚠ looming":""));
  set("visMotionTargetMatch2", paused?"–":(v.motion_target_match!=null
    ? (v.motion_target_match*100).toFixed(0)+"%"+(alerts.motion_matches_target?" ✓ match":"") : "–"));
  set("visLuma2", paused?"–":(v.luma*100).toFixed(0)+"%");
  const obEl=$("visObstructed2");
  if(obEl){ obEl.textContent=paused?"–":
      `var ${v.luma_variance.toFixed(0)}`+(alerts.obstructed?" ⚠ covered/dark":" clear");
    obEl.style.color=(!paused && alerts.obstructed)?"var(--red)":""; obEl.style.opacity=paused?0.5:1; }
  set("visColorCast2", paused?"–":(v.color_cast
    ? `R${(v.color_cast[0]*100).toFixed(0)} G${(v.color_cast[1]*100).toFixed(0)} B${(v.color_cast[2]*100).toFixed(0)}`
      +(alerts.colorcast?" ⚠ cast":"")
    : "–"));
  set("visEdgeDensity2", paused?"–":(v.edge_density*100).toFixed(0)+"%"+(alerts.clutter?" ⚠ cluttered":""));
  set("visOverhead2", paused?"–":(v.overhead_edge_density*100).toFixed(0)+"%"+(alerts.overhead_alert?" ⚠ possible overhang":""));
  set("visShiny2", paused?"–":(v.highlight_fraction*100).toFixed(0)+"%"+(alerts.shiny?" ⚠ shiny":""));
  set("visBacklit2", paused?"–":
    `Δ${((v.luma_max-v.luma)*100).toFixed(0)}%`+(alerts.backlit?" ⚠ backlit":" clear"));
  set("visFocusBlur2", paused?"–":(alerts.focus_blur?"⚠ blurred/close":"clear"));
  set("visGpuDuty2", paused?"–":(v.gpu_duty*100).toFixed(0)+"%");
  const bump=v.bumper||{alert:false,commanded:false,cmd_vel:[0,0],low_motion_secs:0};
  const bEl=$("visBumper2");
  if(bEl){ bEl.textContent=paused?"–":(bump.alert?"⚠ possible stall":"clear");
    bEl.style.color=(!paused && bump.alert)?"var(--red)":""; bEl.style.opacity=paused?0.5:1; }
  set("visBumperCmd2", paused?"–":(bump.commanded
    ? `lin ${bump.cmd_vel[0].toFixed(2)} ang ${bump.cmd_vel[1].toFixed(2)}` : "not driving"));
  set("visBumperHeld2", paused?"–":(bump.commanded ? bump.low_motion_secs.toFixed(1)+"s" : "–"));
  if(cross){
    const r=locked && camDisplayRect();
    if(r){
      cross.style.left=(r.left+v.target[0]*r.width)+"px";
      cross.style.top=(r.top+v.target[1]*r.height)+"px";
      cross.style.display="block";
    } else cross.style.display="none";
  }
}

// Dark reflex config -> web_control params (see PARAM_WHITELIST in telemetry.py).
// UI sliders are 0-100%, the backend stores 0..1 luma fractions.
function syncDarkReflex(){
  let on=Number($("visDarkOn").value), off=Number($("visDarkOff").value);
  if(off<=on){ off=Math.min(100,on+5); $("visDarkOff").value=off; }
  $("visDarkOnV").textContent=on; $("visDarkOffV").textContent=off;
  setParam("web_control","vision_dark_threshold",on/100);
  setParam("web_control","vision_dark_recover",off/100);
}
$("visDarkAuto").onchange=()=>setParam("web_control","vision_dark_reflex_enable",$("visDarkAuto").checked);

$("bumperTuneToggle").onclick=()=>{
  const box=$("bumperTune"), open=box.style.display!=="block";
  box.style.display=open?"block":"none";
  $("bumperTuneToggle").textContent=(open?"▾":"▸")+" Optical bumper tuning";
};
// Optical bumper thresholds -> web_control params. cmd_eps/confirm_secs are already in
// the backend's native units (m/s-or-rad/s, seconds); motion_floor is a 0-100% slider
// over the backend's 0..1 gpu motion-score fraction, same conversion idiom as dark reflex.
$("visBumpEps").oninput=()=>$("visBumpEpsV").textContent=$("visBumpEps").value;
$("visBumpEps").onchange=()=>setParam("web_control","vision_bumper_cmd_eps",Number($("visBumpEps").value));
$("visBumpFloor").oninput=()=>$("visBumpFloorV").textContent=$("visBumpFloor").value;
$("visBumpFloor").onchange=()=>setParam("web_control","vision_bumper_motion_floor",Number($("visBumpFloor").value)/100);
$("visBumpSecs").oninput=()=>$("visBumpSecsV").textContent=$("visBumpSecs").value;
$("visBumpSecs").onchange=()=>setParam("web_control","vision_bumper_confirm_secs",Number($("visBumpSecs").value));

$("blobTuneToggle").onclick=()=>{
  const box=$("blobTune"), open=box.style.display!=="block";
  box.style.display=open?"block":"none";
  $("blobTuneToggle").textContent=(open?"▾":"▸")+" Blob tracking tuning";
};
// Blob tuning -> POST /vision/blob_tune (NOT a ROS param -- this lives inside
// gpu_vision.py's own state, adjusted via a dedicated action endpoint, same reasoning
// as /vision/calibrate). All three sliders are 0-100% over the backend's 0..1 fractions.
function blobTune(body){
  fetch("/vision/blob_tune",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)}).catch(()=>{});
}
$("visBlobThresh").oninput=()=>$("visBlobThreshV").textContent=$("visBlobThresh").value;
$("visBlobThresh").onchange=()=>blobTune({threshold:Number($("visBlobThresh").value)/100});
$("visBlobMin").oninput=()=>$("visBlobMinV").textContent=$("visBlobMin").value;
$("visBlobMin").onchange=()=>blobTune({min_confidence:Number($("visBlobMin").value)/100});
$("visBlobMax").oninput=()=>$("visBlobMaxV").textContent=$("visBlobMax").value;
$("visBlobMax").onchange=()=>blobTune({max_confidence:Number($("visBlobMax").value)/100});
$("visDarkOn").oninput=()=>$("visDarkOnV").textContent=$("visDarkOn").value;
$("visDarkOff").oninput=()=>$("visDarkOffV").textContent=$("visDarkOff").value;
$("visDarkOn").onchange=syncDarkReflex;
$("visDarkOff").onchange=syncDarkReflex;

$("visionAlertsToggle").onclick=()=>{
  const box=$("visionAlertsTune"), open=box.style.display!=="block";
  box.style.display=open?"block":"none";
  $("visionAlertsToggle").textContent=(open?"▾":"▸")+" Vision alerts tuning";
};
// Vision alert thresholds -> web_control params (see PARAM_WHITELIST/telemetry.py's
// _vision_alerts). All 0-100(-ish)% sliders over the backend's 0..1 fractions, same
// conversion idiom as dark reflex/bumper, except the obstruction-flatness slider which
// is raw variance units (not a percentage -- see gpu_vision.py's luma_variance).
const VISION_ALERT_SLIDERS=[
  ["visObstrVar","vision_obstruction_var_max",1],       // raw units, no /100
  ["visObstrDark","vision_obstruction_dark_max",100],
  ["visClutter","vision_clutter_alert",100],
  ["visOverhead","vision_overhead_alert",100],
  ["visFocusBlur","vision_focus_blur_max",100],
  ["visBacklit","vision_backlit_delta_min",100],
  ["visHighlight","vision_highlight_alert",100],
  ["visLooming","vision_looming_alert",100],
  ["visColorcast","vision_colorcast_alert",100],
  ["visMotionTarget","vision_motiontarget_match_max",100],
  ["visNoveltyAlert","vision_novelty_alert",100],
  ["visCamStall","vision_camera_stall_secs",1],          // raw seconds, no /100
  ["visVibRatio","vision_vibration_ratio",100],
  ["visVibSecs","vision_vibration_confirm_secs",1],      // raw seconds
  ["visApproachRate","vision_approach_rate",100],
  ["visApproachBand","vision_approach_band",100],
  ["visGlare","vision_glare_derate",1],                  // raw multiplier, no /100
];
VISION_ALERT_SLIDERS.forEach(([id,param,scale])=>{
  const el=$(id), vEl=$(id+"V");
  if(!el||!vEl) return;
  el.oninput=()=>vEl.textContent=el.value;
  el.onchange=()=>setParam("web_control",param,Number(el.value)/scale);
});

// Maps the displayed <img>'s rendered rect (accounting for object-fit:contain
// letterboxing) in coordinates relative to #cam itself -- shared by the crosshair
// overlay and the click-to-calibrate colour picker below.
function camDisplayRect(){
  const img=$("cam");
  const iw=img.naturalWidth||640, ih=img.naturalHeight||480;
  const cw=img.clientWidth, ch=img.clientHeight;
  if(!iw||!ih||!cw||!ch) return null;
  const scale=Math.min(cw/iw, ch/ih);
  const dw=iw*scale, dh=ih*scale;
  return {left:(cw-dw)/2, top:(ch-dh)/2, width:dw, height:dh, iw, ih};
}

let visionPickArmed=false;
function armVisionPick(){
  visionPickArmed=true;
  $("cam").classList.add("pick-armed");
  $("visionPickHint").style.display="block";
}
function disarmVisionPick(){
  visionPickArmed=false;
  $("cam").classList.remove("pick-armed");
  $("visionPickHint").style.display="none";
}
function onCamPickClick(e){
  if(!visionPickArmed) return;
  const r=camDisplayRect();
  if(!r){ disarmVisionPick(); return; }
  const px=e.offsetX-r.left, py=e.offsetY-r.top;
  if(px<0||py<0||px>=r.width||py>=r.height) return;    // clicked the letterbox margin
  const nx=Math.min(r.iw-1, Math.max(0, Math.floor(px/r.width*r.iw)));
  const ny=Math.min(r.ih-1, Math.max(0, Math.floor(py/r.height*r.ih)));
  const cv=document.createElement("canvas");
  cv.width=r.iw; cv.height=r.ih;
  let data;
  try{
    const ctx=cv.getContext("2d");
    ctx.drawImage($("cam"), 0, 0, r.iw, r.ih);
    data=ctx.getImageData(nx, ny, 1, 1).data;
  }catch(err){ disarmVisionPick(); return; }
  const name=($("visionTargetName").value||"").trim()||"default";
  fetch("/vision/calibrate",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({r:data[0]/255, g:data[1]/255, b:data[2]/255, threshold:0.22, name})})
    .then(()=>loadVisionTargets()).catch(()=>{});
  resetBlobTuneUI();   // server resets threshold/min/max on every fresh colour pick -- match it
  disarmVisionPick();
}
function resetBlobTuneUI(){
  $("visBlobThresh").value=22; $("visBlobThreshV").textContent=22;
  $("visBlobMin").value=0; $("visBlobMinV").textContent=0;
  $("visBlobMax").value=100; $("visBlobMaxV").textContent=100;
}

// ---- ESP32 coprocessor handlers ----
let lastHb=null, lastHbT=0;
function onEsp(e, susp){
  if(e.hb!=null && e.hb_age<2.5){
    lastHb=e.hb; lastHbT=performance.now()-e.hb_age*1000; OLED.tel({espBeat:1});
  }
  if(e.ticks) $("espTicks").textContent=`${e.ticks[0]} / ${e.ticks[1]}`;
  if(e.tick_hz!=null) $("espTickHz").textContent=e.tick_hz.toFixed(0)+" Hz";
  suspTxt("espSuspL",susp[0]); suspTxt("espSuspR",susp[1]);
  if(e.temp!=null && e.temp_age<5){
    $("espTemp").textContent=e.temp.toFixed(1)+"°C"; OLED.tel({espTemp:e.temp});
  }
  if(e.hall!=null) $("espHall").textContent=e.hall;
}
function onLds(l){
  if(l.rpm!=null) $("espLdsRpm").textContent=l.rpm.toFixed(0)+" rpm";
  if(l.hz!=null){ $("espLdsHz").textContent=l.hz.toFixed(1)+" Hz"; OLED.tel({lds:l.hz}); }
  if(l.duty!=null) $("espLdsDuty").textContent=(l.duty*100).toFixed(0)+"%";
}
// suspension: green = on the ground (ready to drive), amber = up/suspended.
function suspTxt(id,val){
  const el=$(id);
  if(val===true){ el.textContent="UP (suspended)"; el.style.color="var(--amber)"; }
  else if(val===false){ el.textContent="down"; el.style.color="var(--green)"; }
  else { el.textContent="–"; el.style.color=""; }
}
// Heartbeat liveness: the counter ticks ~1 Hz; if it stops advancing the link is down.
setInterval(()=>{
  const el=$("espHb");
  if(lastHb==null){ el.textContent="–"; el.style.color=""; return; }
  const alive=(performance.now()-lastHbT)<2500;
  el.textContent = alive ? `alive (#${lastHb})` : "lost";
  el.style.color = alive ? "var(--green)" : "var(--red)";
},1000);

let lastImuT=0;
function onImu(m){
  // frame imu: {a:|accel| m/s^2, g:|gyro| rad/s, hz:actual /imu/data Hz, age:s}.
  // Sourced from sys_monitor's 1 Hz vitals blob, so a healthy age can reach ~2 s.
  if(m.age>=3 || m.a==null) return;        // stale: the "lost" watchdog below owns the text
  lastImuT=performance.now()-m.age*1000;
  $("imuA").textContent=m.a.toFixed(2);
  $("imuG").textContent=(m.g*180/Math.PI).toFixed(1);
  const el=$("imuHz"); el.textContent=m.hz.toFixed(0)+" Hz"; el.style.color="";
  OLED.tel({imuHz:m.hz});
}
// IMU connectivity: if the IMU stream stops (USB unplugged, or the driver lost the
// port) the rate readout goes red "lost" — so even with the sensor nodes merged into
// one process you can still see the IMU drop out.
setInterval(()=>{
  if(lastImuT && performance.now()-lastImuT<3500) return;   // fresh: onImu owns the text
  const el=$("imuHz"); el.textContent="lost"; el.style.color="var(--red)";
},1000);
function onEul(m){
  if(m.age>=4 || m.r==null) return;
  $("imuR").textContent=m.r.toFixed(1)+"°";
  $("imuP").textContent=m.p.toFixed(1)+"°";
  $("imuY").textContent=m.y.toFixed(1)+"°";
  OLED.tel({roll:m.r, pitch:m.p});
}

function fmtUptime(s){
  const d=Math.floor(s/86400), h=Math.floor(s%86400/3600), m=Math.floor(s%3600/60);
  return d>0 ? `${d}d${h}h` : `${h}h${String(m).padStart(2,"0")}m`;
}
$("cpuToggle").onclick=()=>{
  const box=$("sysCores"), open=box.style.display!=="flex";
  box.style.display=open?"flex":"none";
  $("cpuToggle").textContent=(open?"▾":"▸")+" CPU";
};
function renderCores(s){
  const box=$("sysCores");
  const vals=(s||"").split(",").filter(x=>x!=="").map(Number);
  if(box.childElementCount!==vals.length)            // (re)build rows on count change
    box.innerHTML=vals.map((_,i)=>
      `<div class="core"><span class="lbl">c${i}</span>`+
      `<span class="bar"><i id="core${i}"></i></span><b id="corev${i}">–</b></div>`).join("");
  vals.forEach((v,i)=>{
    const f=$("core"+i);
    if(f){f.style.width=Math.max(0,Math.min(100,v))+"%";
      f.style.background=v>=85?"var(--red)":v>=60?"var(--amber)":"var(--green)";}
    const b=$("corev"+i); if(b) b.textContent=v.toFixed(0)+"%";
  });
}
function onDiag(kv){
  // frame diag: the /diagnostics "system" status as a flat {key: value} dict
  $("sysCpu").textContent=(kv.cpu_percent??"–")+"%";
  OLED.tel({cpu:Number(kv.cpu_percent), mem:Number(kv.mem_percent), sbc:Number(kv.cpu_temp_c)});
  renderCores(kv.cpu_cores);
  $("sysLoad").textContent=kv.load1??"–";
  $("sysMem").textContent=`${kv.mem_used_mb??"?"}/${kv.mem_total_mb??"?"}MB (${kv.mem_percent??"?"}%)`;
  $("sysTemp").textContent=(kv.cpu_temp_c??"–")+"°C";
  $("sysDisk").textContent=(kv.disk_percent??"–")+"%";
  $("sysUp").textContent=fmtUptime(Number(kv.uptime_s||0));
  $("sysTemp").style.color = Number(kv.cpu_temp_c)>=75 ? "var(--red)" : "";
  $("sysMem").style.color  = Number(kv.mem_percent)>=90 ? "var(--amber)" : "";
  // WiFi: signal strength only ("-54dBm (78%)"), colour-coded so you can spot a weak link.
  const w=$("sysWifi"), dbm=kv.wifi_signal_dbm;
  if(!kv.wifi_iface || !dbm){ w.textContent="—"; w.style.color=""; }
  else{
    w.textContent = `${dbm}dBm${kv.wifi_quality_pct?` (${kv.wifi_quality_pct}%)`:""}`;
    const d=Number(dbm);
    w.style.color = d>=-60 ? "var(--green)" : d>=-75 ? "var(--amber)" : "var(--red)";
  }
}

// ---- Speech (TTS) ---- POSTs to web_control, which speaks via espeak-ng and
// streams the words to the OLED (/oled_word), owning the audio + word timing in
// one place. Voice/volume/
// speed/pitch + the stats-announcer are persisted server-side (survive a reboot),
// so the page just mirrors them: GET them on load, POST /tts/config on change.
function sendOled(){
  const text=$("oledText").value.trim();
  if(!text) return;
  fetch("/tts",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({voice:$("ttsVoice").value,text})}).catch(()=>{});
}
$("oledSend").onclick=sendOled;
$("ttsStop").onclick=()=>fetch("/tts/stop",{method:"POST"}).catch(()=>{});
$("oledText").addEventListener("keydown",e=>{ if(e.key==="Enter"){e.preventDefault();sendOled();} });

function saveTts(patch){
  fetch("/tts/config",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(patch)}).catch(()=>{});
}
// Live label on drag (input); persist only on release/commit (change) to avoid spam.
function bindSlider(id,labelId,key){
  const el=$(id), lab=$(labelId);
  el.addEventListener("input",()=>{ lab.textContent=el.value; });
  el.addEventListener("change",()=>saveTts({[key]:Number(el.value)}));
}
bindSlider("ttsVol","ttsVolV","volume");
bindSlider("ttsSpeed","ttsSpeedV","speed");
bindSlider("ttsPitch","ttsPitchV","pitch");
bindSlider("ttsLead","ttsLeadV","lead_silence");
$("ttsVoice").addEventListener("change",()=>saveTts({voice:$("ttsVoice").value}));
$("announceOn").addEventListener("change",()=>saveTts({announce:$("announceOn").checked}));
$("announceInterval").addEventListener("change",
  ()=>saveTts({announce_interval:Number($("announceInterval").value)}));
$("announceSay").onclick=()=>fetch("/tts/announce",{method:"POST"}).catch(()=>{});

// Restore the persisted TTS settings into the controls on page load.
function loadTts(){
  fetch("/tts/config").then(r=>r.ok?r.json():null).then(s=>{
    if(!s) return;
    const set=(id,v)=>{ if($(id)) $(id).value=v; };
    set("ttsVoice",s.voice); set("ttsVol",s.volume); set("ttsSpeed",s.speed);
    set("ttsPitch",s.pitch); set("ttsLead",s.lead_silence);
    set("announceInterval",s.announce_interval);
    $("ttsVolV").textContent=s.volume; $("ttsSpeedV").textContent=s.speed;
    $("ttsPitchV").textContent=s.pitch;
    $("ttsLeadV").textContent=s.lead_silence; $("announceOn").checked=!!s.announce;
  }).catch(()=>{});
}
loadTts();

// ---- AI (OpenRouter) ---- the spoken line + a matching OLED face are generated
// server-side (POST /llm/say|chat|observe|look); the server speaks it (TTS) and
// publishes the mood on /oled_face. enable/model/persona persist server-side. Autonomous
// chatter is driven by the behaviour statechart, not here. The decision log (/llm/log)
// records every decision + outcome (incl. statechart beats + skips).
function llmApply(s){
  if(!s) return;
  const set=(id,v)=>{ if($(id)&&v!=null) $(id).value=v; };
  if($("llmOn")) $("llmOn").checked=!!s.enabled;
  set("llmModel",s.model);
  set("llmSmartModel",s.smart_model);
  set("llmVisionModel",s.vision_model);
  set("llmVisionFallbackModel",s.vision_fallback_model);
  set("llmFreeModel",s.free_model);
  set("llmFreeSmartModel",s.free_smart_model);
  set("llmPersona",s.persona);
  if($("llmKeyStatus")) $("llmKeyStatus").textContent = s.api_key_set ? "· saved" : "· not set";
  if($("llmStatus")) $("llmStatus").textContent =
    s.available ? ("ready · "+(s.model_effective||"")) :
    (s.enabled ? "enabled but no API key configured" : "disabled");
}
function llmSave(patch){
  fetch("/llm/config",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(patch)}).then(r=>r.ok?r.json():null).then(llmApply).catch(()=>{});
}
function showReply(r){
  const el=$("llmReply"); if(!el) return;
  if(!r){ el.textContent="(no reply)"; }
  else if(r.error){ el.textContent="⚠ "+r.error; }
  else el.textContent="🤖 "+(r.say||"")+(r.mood?("  ["+r.mood+"]"):"");
  loadLlmLog();                                  // an action just made a new decision
}
function llmPost(url,body){
  $("llmReply").textContent="…";
  fetch(url,{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)})
    .then(r=>r.ok?r.json():{error:r.status===503?"AI unavailable (no key / disabled)":("http "+r.status)})
    .then(showReply).catch(()=>showReply({error:"network"}));
}
$("llmSay").onclick=()=>llmPost("/llm/say",{prompt:$("llmPrompt").value.trim()});
$("llmObserve").onclick=()=>llmPost("/llm/observe",{});  // comment on its own sensors
$("llmLook").onclick=()=>llmPost("/llm/look",{});        // comment on what the camera sees
function sendChat(){
  const m=$("llmChat").value.trim(); if(!m) return;
  $("llmChat").value=""; llmPost("/llm/chat",{message:m});
}
$("llmSend").onclick=sendChat;
$("llmChat").addEventListener("keydown",e=>{ if(e.key==="Enter"){e.preventDefault();sendChat();} });
$("llmPrompt").addEventListener("keydown",e=>{ if(e.key==="Enter"){e.preventDefault();$("llmSay").click();} });
$("llmOn").addEventListener("change",()=>llmSave({enabled:$("llmOn").checked}));
$("llmModel").addEventListener("change",()=>llmSave({model:$("llmModel").value.trim()}));
$("llmSmartModel").addEventListener("change",()=>llmSave({smart_model:$("llmSmartModel").value.trim()}));
$("llmVisionModel").addEventListener("change",()=>llmSave({vision_model:$("llmVisionModel").value.trim()}));
$("llmVisionFallbackModel").addEventListener("change",()=>llmSave({vision_fallback_model:$("llmVisionFallbackModel").value.trim()}));
$("llmFreeModel").addEventListener("change",()=>llmSave({free_model:$("llmFreeModel").value.trim()}));
$("llmFreeSmartModel").addEventListener("change",()=>llmSave({free_smart_model:$("llmFreeSmartModel").value.trim()}));
// API key: never pre-filled (the server never sends the saved key back — see llmApply),
// so the field starts blank. Typing one + leaving the field saves it (persists server-side,
// survives a reboot); the field is cleared right after so the secret doesn't linger in the
// DOM. There's no way to "blank out" via the field itself (blur with no edits is a no-op),
// so clearing a saved key is a dedicated action.
$("llmApiKey").addEventListener("change",()=>{
  const v=$("llmApiKey").value;
  if(!v) return;
  llmSave({api_key:v});
  $("llmApiKey").value="";
});
$("llmKeyClear").onclick=()=>{
  if(!confirm("Clear the saved OpenRouter API key?")) return;
  llmSave({api_key:""});
};
// Persona is read-only here — it's single-sourced from personality.json (the creator's
// output). Edit it by re-running scripts/personality_creator.py and restarting.
fetch("/llm/config").then(r=>r.ok?r.json():null).then(llmApply).catch(()=>{});

// ---- Decision log ---- where + how each AI decision got made (incl. statechart beats
// and skips). Poll while the <details> is open; also refreshed right after each action.
const STATUS_ICON={spoke:"🗣",  "no-reply":"🤐", "skipped-busy":"⏳",
  "llm-unavailable":"🚫", "no-frame":"📷✗", error:"⚠"};
function fmtLogRow(e){
  const time=new Date((e.t||0)*1000).toLocaleTimeString();
  const who=e.trigger+(e.state&&e.state!==e.trigger.replace(/^beat:/,"")?(" "+e.state):"");
  const cam=e.camera?" 📷":"";
  const out=e.say?("“"+e.say+"”"+(e.mood?(" ["+e.mood+"]"):"")):(e.detail||"");
  const ic=STATUS_ICON[e.status]||"•";
  return `<div class="logrow"><span class="lt">${time}</span> ${ic} <b>${who}</b>${cam}`
    +` <span class="ls">${e.status}</span>`+(out?(" — "+out):"")
    +(e.model?` <span class="lm">${e.model}</span>`:"")+`</div>`;
}
function loadLlmLog(){
  const box=$("llmLog"); if(!box) return;
  fetch("/llm/log").then(r=>r.ok?r.json():null).then(d=>{
    const es=(d&&d.entries)||[];
    box.innerHTML = es.length ? es.slice(0,40).map(fmtLogRow).join("")
                              : "<i>no decisions yet</i>";
  }).catch(()=>{});
}
const llmLogDetails=$("llmLog")&&$("llmLog").closest("details");
let llmLogTimer=null;
if(llmLogDetails){
  llmLogDetails.addEventListener("toggle",()=>{
    clearInterval(llmLogTimer);
    if(llmLogDetails.open){ loadLlmLog(); llmLogTimer=setInterval(loadLlmLog,4000); }
  });
}

// ---- Skills · capability library ---- list skills/*.md, run one on demand, reload the
// library. Each .md is a self-documenting capability; the brain also picks one autonomously
// on a "skill" beat (logged in the decision log above). Narrative skills speak a line;
// action skills (gated) publish a whitelisted ROS message.
const SKILL_ICON={say:"💬",observe:"📟",look:"📷",topic:"⚙"};
function fmtSkill(s){
  const ic=SKILL_ICON[s.kind]||"•";
  const gated=s.is_action && !s.enabled;            // an off-by-default action skill
  const act=s.is_action?` <span class="lm">${s.topic||"action"}</span>`:"";
  const tag=gated?` <span class="ls">disabled</span>`:"";
  const likes=s.likes||0;                            // 👍 count: the brain favours liked skills
  return `<div class="skillrow${gated?" disabled":""}"><div class="skmeta">`
    +`<b>${ic} ${s.name}</b>${act}${tag}<div class="skdesc">${s.description||""}</div></div>`
    +`<button class="btn sklike${likes>0?" liked":""}" data-name="${s.name}"`
    +` title="Like — the brain performs liked skills more often. Click again to like more;`
    +` shift-click to take one back.">👍 ${likes}</button>`
    +`<button class="btn skinvoke" data-name="${s.name}">Run</button></div>`;
}
let skillNames=[];   // populated by loadSkills(); used to build the Schedule card's skill picker
function loadSkills(){
  const box=$("skillsList"), st=$("skillsStatus"); if(!box||!st) return;
  fetch("/skills").then(r=>r.ok?r.json():null).then(d=>{
    if(!d){ st.textContent="unavailable"; box.innerHTML=""; return; }
    if(d.enabled===false){ st.textContent="skills disabled"; box.innerHTML=""; return; }
    const sk=d.skills||[];
    st.textContent=`${sk.length} skill${sk.length===1?"":"s"} · action tier `
      +(d.allow_actions?"on":"off");
    box.innerHTML=sk.length?sk.map(fmtSkill).join(""):"<i>no skills found — add a .md to skills/</i>";
    box.querySelectorAll(".skinvoke").forEach(b=>b.onclick=()=>invokeSkill(b.dataset.name));
    box.querySelectorAll(".sklike").forEach(b=>b.onclick=e=>likeSkill(b.dataset.name,e.shiftKey?-1:1));
    skillNames=sk.map(s=>s.name);
    renderSchedule();   // refresh the skill <select> options now that names are known
  }).catch(()=>{ st.textContent="offline"; });
}
// Like (👍) a skill so the brain performs it more often — repeatable (each click +1), shift-click
// takes one back. The count is a per-skill weight in the autonomous skill-beat lottery.
function likeSkill(name,delta){
  fetch("/skills/like",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name,delta})}).then(r=>r.ok?r.json():null)
    .then(()=>loadSkills()).catch(()=>{});
}
function invokeSkill(name){
  const st=$("skillsStatus"); if(st) st.textContent="running "+name+"…";
  fetch("/skills/invoke",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name})})
    .then(r=>r.ok?r.json():{error:"http "+r.status}).then(res=>{
      if(res && (res.say!==undefined || res.mood!==undefined)) showReply(res);  // narrative
      else { if(st) st.textContent=name+": "+(res.status||res.error||res.detail||"done");
             loadLlmLog(); }
    }).catch(()=>{ if(st) st.textContent="network error"; });
}
if($("skillsReload")) $("skillsReload").onclick=()=>{
  const st=$("skillsStatus"); if(st) st.textContent="reloading…";
  fetch("/skills/reload",{method:"POST"}).then(()=>{loadSkills();loadWorkshop();})
    .catch(()=>{loadSkills();loadWorkshop();});
};
loadSkills();

// ---- Workshop · skills on trial ---- reflection mode mints/adapts a skill from experience and
// puts it on trial; it auto-adopts after enough good runs + 👍 (no errors) or auto-retires on
// errors/👎. Keep/Kill are the manual overrides (POST to web_control / the dev harness).
function fmtTrial(t){
  const stat=`runs ${t.runs||0} · 👍${t.reward_pos||0} 👎${t.reward_neg||0}`
    +((t.errors)?` · ⚠${t.errors}`:"");
  const badge=t.status==="adopted"?`<span class="ls" style="color:#7fd">adopted</span>`
    :t.status==="retired"?`<span class="ls">retired</span>`
    :`<span class="ls" style="color:#fd7">trial</span>`;
  const origin=t.origin==="adapt"?`adapt of ${t.parent||"?"}`:"new";
  const btns=(t.status==="trial")
    ?`<button class="btn skkeep" data-name="${t.name}">Keep</button>`
      +`<button class="btn skkill" data-name="${t.name}">Kill</button>`:"";
  return `<div class="skillrow"><div class="skmeta"><b>🧪 ${t.name}</b> ${badge}`
    +` <span class="lm">${origin}</span><div class="skdesc">${t.rationale||""}</div>`
    +`<div class="skdesc">${stat}</div></div>${btns}</div>`;
}
function loadWorkshop(){
  const box=$("workshopList"), st=$("workshopStatus"); if(!box||!st) return;
  fetch("/skills/workshop").then(r=>r.ok?r.json():null).then(d=>{
    if(!d){ st.textContent="unavailable"; box.innerHTML=""; return; }
    if(d.enabled===false){ st.textContent="workshop disabled"; box.innerHTML=""; return; }
    const tr=d.trials||[];
    const live=tr.filter(t=>t.status==="trial").length;
    st.textContent=(d.busy?"reflecting — proposing a skill… · ":"")
      +`${live} on trial · ${tr.length} tracked`;
    box.innerHTML=tr.length?tr.map(fmtTrial).join("")
      :"<i>none yet — reflect to invent one</i>";
    box.querySelectorAll(".skkeep").forEach(b=>b.onclick=()=>workshopAct("keep",b.dataset.name));
    box.querySelectorAll(".skkill").forEach(b=>b.onclick=()=>workshopAct("kill",b.dataset.name));
  }).catch(()=>{ st.textContent="offline"; });
}
function workshopAct(act,name){
  const st=$("workshopStatus"); if(st) st.textContent=act+"ing "+name+"…";
  fetch("/skills/workshop/"+act,{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name})}).then(()=>{loadWorkshop();loadSkills();})
    .catch(()=>{ if(st) st.textContent="network error"; });
}
loadWorkshop();

// ---- Schedule · routines ---- fires a NAMED skill once a day at a local HH:MM (a manual-
// style invocation, unlike the autonomous skill-beat picker above). The behaviour node
// echoes the normalized list on the latched /schedule topic (see onFrame); editing here
// replaces the whole list via the whitelisted /schedule_edit topic (see telemetry.py).
let scheduleEntries=[];   // the editable working copy: [{time,skill}]
function renderSchedule(){
  const box=$("scheduleList"); if(!box) return;
  if(!scheduleEntries.length){ box.innerHTML="<i>no routines — Add one</i>"; return; }
  const opts=n=>(skillNames.length?skillNames:[n]).map(s=>
    `<option value="${s}"${s===n?" selected":""}>${s||"(no skills loaded)"}</option>`).join("");
  box.innerHTML=scheduleEntries.map((e,i)=>
    `<div class="row" data-i="${i}">`
    +`<input type="time" class="schTime" value="${e.time||""}">`
    +`<select class="schSkill grow">${opts(e.skill)}</select>`
    +`<button class="btn danger schDel" data-i="${i}">✕</button></div>`).join("");
  box.querySelectorAll(".schTime").forEach(el=>el.onchange=e=>{
    scheduleEntries[+e.target.closest("[data-i]").dataset.i].time=e.target.value; });
  box.querySelectorAll(".schSkill").forEach(el=>el.onchange=e=>{
    scheduleEntries[+e.target.closest("[data-i]").dataset.i].skill=e.target.value; });
  box.querySelectorAll(".schDel").forEach(b=>b.onclick=()=>{
    scheduleEntries.splice(+b.dataset.i,1); renderSchedule(); });
}
function onScheduleFrame(raw){
  try{ scheduleEntries=(JSON.parse(raw)||[]).map(e=>({time:e.time||"",skill:e.skill||""})); }
  catch(e){ scheduleEntries=[]; }
  const st=$("scheduleStatus");
  if(st) st.textContent=scheduleEntries.length
    ?`${scheduleEntries.length} routine${scheduleEntries.length===1?"":"s"}`:"no routines configured";
  renderSchedule();
}
if($("scheduleAdd")) $("scheduleAdd").onclick=()=>{
  scheduleEntries.push({time:"09:00",skill:skillNames[0]||""}); renderSchedule(); };
if($("scheduleSave")) $("scheduleSave").onclick=()=>{
  pub("/schedule_edit", scheduleEntries.filter(e=>e.time&&e.skill));
  const st=$("scheduleStatus"); if(st) st.textContent="saved";
};

// ---- Brain · purpose & learning ---- the Purpose Engine (goals + intrinsic reward) and
// the Horizon Planner's A/B bandit live in the behaviour node and publish latched JSON on
// /purpose, /task_current, /experiments. Reward (👍/👎) + reflection POST to web_control,
// which logs + republishes them for the behaviour node. All narrative-only (no motion).
let lastTask=null;
function renderPurpose(s){
  try{ const p=JSON.parse(s);
    $("purposeObjective").textContent="objective: "+((p.objective&&p.objective.text)||"—");
    const r=p.intrinsic_reward||{};
    $("rewardBars").innerHTML=Object.keys(r).map(k=>
      `<span>${k}</span><b>${Math.round(r[k]*100)}%</b>`).join("");
  }catch(e){}
}
function renderTask(s){
  try{ const t=JSON.parse(s);
    lastTask = (t && t.task) ? t : null;          // only a real task is a reward target
    $("taskCurrent").textContent = (t && (t.text||t.task)) || "—";
  }catch(e){ lastTask=null; }
}
function renderExperiments(s){
  const box=$("abLog"); if(!box) return;
  try{ const d=JSON.parse(s), exps=d.experiments||{};
    box.innerHTML=Object.keys(exps).map(eid=>{
      const e=exps[eid], vs=e.variants||{};
      const rows=Object.keys(vs).map(v=>
        `<div class="logrow">${v===e.winner?"★":"·"} <b>${v}</b>`
        +` <span class="ls">n=${vs[v].n||0}</span> reward ${(vs[v].mean||0).toFixed(2)}</div>`
      ).join("");
      return `<div class="logrow"><b>${eid}</b></div>${rows}`;
    }).join("")||"<i>no experiments</i>";
  }catch(e){}
}
function sendReward(value){
  const body={value, scope: lastTask?"contextual":"global", target: lastTask};
  fetch("/brain/reward",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(body)}).then(()=>loadLlmLog()).catch(()=>{});
}
if($("rewardUp")) $("rewardUp").onclick=()=>sendReward(1);
if($("rewardDown")) $("rewardDown").onclick=()=>sendReward(-1);
if($("reflectOn")) $("reflectOn").addEventListener("change",()=>{
  fetch("/brain/reflect",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({on:$("reflectOn").checked})}).catch(()=>{});
});
// Dev-harness fallback (scripts/dev_webui.py): with no /telemetry stream, poll the
// brain readouts over plain HTTP. On the real robot these come in the telemetry frame
// (and these GETs 404), so this only does anything on the dev host while disconnected.
function pollBrainHttp(){
  if(connected) return;
  fetch("/purpose").then(r=>r.ok?r.text():null).then(s=>{ if(s) renderPurpose(s); }).catch(()=>{});
  fetch("/task_current").then(r=>r.ok?r.text():null).then(s=>{ if(s) renderTask(s); }).catch(()=>{});
  fetch("/experiments").then(r=>r.ok?r.text():null).then(s=>{ if(s) renderExperiments(s); }).catch(()=>{});
}
setInterval(pollBrainHttp, 3000); pollBrainHttp();

// ---- OLED face / mood ---- empty string = the resting idle face, else the selected mood.
function sendFace(){
  const mood=$("faceOn").checked ? $("faceMood").value : "";
  pub("/oled_face",mood);
}
$("faceOn").addEventListener("change",sendFace);
$("faceMood").addEventListener("change",()=>{ if($("faceOn").checked) sendFace(); });

// ---- OLED dashboard pin + spoken-text toggle ----
// Each updates the client-side mirror immediately AND publishes the Bool (via /publish)
// so the physical panel follows. Off-robot (dev harness) the publish 404s harmlessly and
// the mirror is the only display, so the local set is what makes the toggle work there.
function sendDash(){
  const on=$("dashOn").checked; OLED.setDashboard(on);
  pub("/oled_dashboard",on);
}
function sendWords(){
  const on=$("wordsOn").checked; OLED.setShowWords(on);
  pub("/oled_show_words",on);
}
$("dashOn").addEventListener("change",sendDash);
$("wordsOn").addEventListener("change",sendWords);

// ---- lidar scan over HTTP (same-origin /dev/shm) ----
// /scan.bin = a JSON header line ({seq,amin,ainc,n}), '\n', then n raw float32 ranges
// (inf = no hit). The driver rewrites it per scan; we poll a touch faster and skip
// unchanged seqs. Deliberately NOT in the telemetry frame: it's the heaviest data and
// polling lets the page control the rate per view (see the interval below).
let lastScanSeq=-1, lastScanErr=null, lastScanRx=null;
const scanDec=new TextDecoder();
let curHeroView="lidar";     // tracked by the hero view switcher (showView)
async function pollScan(){
  try{
    const r=await fetch("/scan.bin?t="+Date.now());
    if(!r.ok) return;                          // 503 until the first scan is written
    const buf=new Uint8Array(await r.arrayBuffer());
    const nl=buf.indexOf(10); if(nl<0) return;
    const h=JSON.parse(scanDec.decode(buf.subarray(0,nl)));
    if(h.seq===lastScanSeq) return;            // no new scan since the last poll
    lastScanSeq=h.seq;
    if(h.stale){
      // Driver heartbeat: the port-level truth when no revolutions are arriving.
      // Distinguish the failure modes instead of showing nothing at all.
      const el=$("ldsErr");
      if(!h.open)                el.textContent="port open failed";
      else if(!h.rx)             el.textContent="no RX data (wiring?)";
      else if(h.rx===lastScanRx) el.textContent="RX stopped ("+h.rx+" B)";
      else                       el.textContent="RX garbled · err "+h.err;
      el.style.color="var(--red)";
      $("ldsLost").textContent="–";
      lastScanRx=h.rx; lastScanErr=h.err;
      frame=[]; $("pts").textContent=0; $("ldsPts").textContent=0;
      $("hz").textContent="0"; $("ldsHz").textContent="0 Hz";
      return;
    }
    if(h.rx!==undefined) lastScanRx=h.rx;
    // RX-health counters from the real driver's blob header (absent in the sim).
    if(h.lost!==undefined){ const el=$("ldsLost"); el.textContent=h.lost;
      el.style.color = h.lost>36 ? "var(--amber)" : ""; }      // >10% of a rev missing
    if(h.err!==undefined){ const el=$("ldsErr"); el.textContent=h.err;
      el.style.color = (lastScanErr!==null && h.err>lastScanErr) ? "var(--red)" : "";
      lastScanErr=h.err; }
    const dv=new DataView(buf.buffer, buf.byteOffset+nl+1);
    const maxr=Number($("maxr").value)||1e9, pts=[];
    for(let i=0;i<h.n;i++){
      const rng=dv.getFloat32(i*4,true);
      if(!isFinite(rng)||rng<=0||rng>maxr) continue;
      const a=h.amin+i*h.ainc;
      pts.push({x:rng*Math.cos(a), y:rng*Math.sin(a)});
    }
    frame=pts; $("pts").textContent=pts.length; $("ldsPts").textContent=pts.length;
    scanCount++;
    const now=performance.now();
    if(now-lastHzT>=1000){ scanHz=scanCount*1000/(now-lastHzT); scanCount=0; lastHzT=now;
      $("hz").textContent=scanHz.toFixed(1); $("ldsHz").textContent=scanHz.toFixed(1)+" Hz"; }
  }catch(e){ /* transient: keep polling */ }
}
// Poll fast (a touch above the scan rate, which runs ~5-10 Hz) only while the lidar hero
// view is up AND the tab is visible — every poll is an HTTP round-trip the board serves
// from /dev/shm, so there's no point hammering it when nobody's looking at the lidar.
// Off-view we still tick ~1 Hz to keep the header scan-Hz / pts readouts alive; when the
// tab is backgrounded we stop entirely.
let scanTick=0;
setInterval(()=>{
  if(document.hidden) return;
  if(curHeroView==="lidar" || (++scanTick % 12)===0) pollScan();
},80);
function onOdom(o){
  const [x,y,yaw]=o;
  $("px").textContent=x.toFixed(2); $("py").textContent=y.toFixed(2);
  $("pth").textContent=(yaw*180/Math.PI).toFixed(0);
  $("odoX").textContent=x.toFixed(2)+" m"; $("odoY").textContent=y.toFixed(2)+" m";
  $("odoTh").textContent=(yaw*180/Math.PI).toFixed(0)+"°";
}

// ---- teleop ----
// POST /drive to the web server, which publishes /cmd_vel itself and runs its OWN
// 10 Hz keepalive + dead-man on the board, so browser jank can't outlast the ESP32's
// 500 ms /cmd_vel watchdog and stutter the drive. The dev harness accepts it as a no-op.
let curV=0, curW=0, driveBusy=false;
function setCmd(v,w){ curV=v; curW=w; }
function publishCmd(){
  const v=curV*Number($("lin").value), w=curW*Number($("ang").value);
  sendDrive(v,w);
}
function sendDrive(v,w){
  if(driveBusy) return;              // one POST in flight; the 10 Hz tick re-sends
  driveBusy=true;
  fetch("/drive",{method:"POST",body:JSON.stringify({v:v,w:w})})
    .catch(()=>{})
    .finally(()=>{ driveBusy=false; });
}
// 10 Hz refresh while moving: the server's dead-man stops the motors if commands stop
// arriving. Skip it when the tab is backgrounded; and if we get hidden mid-drive, send
// one stop right away so the robot halts immediately instead of coasting to the watchdog.
setInterval(()=>{ if(!document.hidden && (curV||curW)) publishCmd(); },100);
document.addEventListener("visibilitychange",()=>{
  if(document.hidden && (curV||curW)){ setCmd(0,0); publishCmd(); }
});
$("stop").onclick=()=>{setCmd(0,0);publishCmd();};

// keyboard
const keys={};
const typing=e=>/^(input|textarea|select)$/i.test(e.target.tagName);
addEventListener("keydown",e=>{
  if(typing(e)) return;   // don't teleop while typing in a field (e.g. OLED text)
  if(["w","a","s","d","arrowup","arrowdown","arrowleft","arrowright"," "].includes(e.key.toLowerCase()))
    e.preventDefault();
  keys[e.key.toLowerCase()]=true; updateKeys();
});
addEventListener("keyup",e=>{ if(typing(e)) return; keys[e.key.toLowerCase()]=false; updateKeys(); });
function updateKeys(){
  let v=0,w=0;
  if(keys["w"]||keys["arrowup"])v+=1;
  if(keys["s"]||keys["arrowdown"])v-=1;
  if(keys["a"]||keys["arrowleft"])w+=1;
  if(keys["d"]||keys["arrowright"])w-=1;
  if(keys[" "]){v=0;w=0;}
  if(v===curV && w===curW) return;  // key autorepeat: unchanged command, the 10 Hz keepalive covers it
  setCmd(v,w); publishCmd();
}

// ---- render ----
function resize(){const r=cv.getBoundingClientRect(),d=devicePixelRatio||1;
  cv.width=r.width*d;cv.height=r.height*d;ctx.setTransform(d,0,0,d,0,0);}
new ResizeObserver(resize).observe($("wrap"));

function draw(){
  const w=cv.clientWidth,h=cv.clientHeight; ctx.clearRect(0,0,w,h);
  const cx=w/2+panX, cy=h/2+panY;
  ctx.strokeStyle="#1c2330"; ctx.fillStyle="#3a4150";
  for(let r=1;r<=8;r++){ctx.beginPath();ctx.arc(cx,cy,r*scale,0,7);ctx.stroke();}
  ctx.strokeStyle="#262d3a";ctx.beginPath();
  ctx.moveTo(0,cy);ctx.lineTo(w,cy);ctx.moveTo(cx,0);ctx.lineTo(cx,h);ctx.stroke();
  ctx.fillStyle="#6e7681";ctx.font="11px system-ui,sans-serif";ctx.fillText("rings = 1 m",10,h-10);
  // robot heading +x is to the right; draw a little nose
  ctx.fillStyle="#f0556a";ctx.beginPath();ctx.arc(cx,cy,5,0,7);ctx.fill();
  ctx.strokeStyle="#f0556a";ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+14,cy);ctx.stroke();
  // points: ROS x forward (up on screen), y left
  ctx.fillStyle="#4d9fff";
  for(const p of frame){ const X=cx-p.y*scale, Y=cy-p.x*scale; ctx.fillRect(X-1.5,Y-1.5,3,3); }
  requestAnimationFrame(draw);
}
function autoFit(){
  if(!frame.length) return; let mx=0;
  for(const p of frame) mx=Math.max(mx,Math.hypot(p.x,p.y));
  if(mx>0){ scale=(Math.min(cv.clientWidth,cv.clientHeight)*0.45)/mx; panX=panY=0; }
}
cv.addEventListener("wheel",e=>{e.preventDefault();
  scale*=e.deltaY<0?1.12:1/1.12; scale=Math.min(400,Math.max(4,scale));},{passive:false});
cv.addEventListener("pointerdown",e=>{dragging=true;dsx=e.clientX;dsy=e.clientY;dpx=panX;dpy=panY;cv.setPointerCapture(e.pointerId);});
cv.addEventListener("pointermove",e=>{if(dragging){panX=dpx+(e.clientX-dsx);panY=dpy+(e.clientY-dsy);}});
cv.addEventListener("pointerup",()=>dragging=false);

resize(); requestAnimationFrame(draw);
connect();   // auto-connect on load
