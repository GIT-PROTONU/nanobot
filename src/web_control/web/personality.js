// ---- Personality panel (AI tab) ----------------------------------------------
// Self-contained, pure HTTP (same philosophy as map.js): poll GET /personality and
// render traits/drives/beat-registry sliders; edits POST /personality as a HARD patch
// (see web_server.set_personality / presence.apply_evolve) — set exactly + re-baselined
// in the behaviour node, so a deliberate web edit sticks instead of being smoothed away
// or reverted like a slow LLM-reflection nudge. Works identically against the robot and
// scripts/dev_webui.py (both serve the same GET/POST /personality contract).
(function(){
  "use strict";
  const $=id=>document.getElementById(id);

  const TRAITS=["curiosity","extraversion","caution","playfulness"];
  const DRIVES=["energy","focus","introspection"];
  const idFor=k=>"p"+k[0].toUpperCase()+k.slice(1);

  function patch(body){
    fetch("/personality",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify(body)}).then(r=>r.ok?r.json():null).then(d=>{ if(d) render(d); })
      .catch(()=>{});
  }
  // range inputs only fire "change" on release, so no extra debounce is needed — matches
  // the slam_nav slider convention in app.js (oninput = live label, onchange = send).
  function wireSlider(key, group){
    const el=$(idFor(key)), v=$(idFor(key)+"V");
    if(!el) return;
    el.oninput=()=>{ v.textContent=Number(el.value).toFixed(2); };
    el.onchange=()=>patch({[group]:{[key]:Number(el.value)}});
  }
  TRAITS.forEach(k=>wireSlider(k,"traits"));
  DRIVES.forEach(k=>wireSlider(k,"drives"));
  if($("pMood")) $("pMood").onchange=()=>patch({drives:{mood:$("pMood").value}});

  function beatRow(name, cfg){
    const pr=(cfg.priority!=null?cfg.priority:0.5);
    return `<span>${name}${cfg.trait?` <i class="hint-inline">(${cfg.trait})</i>`:""}</span>`
      +`<span class="row" style="gap:6px">`
      +`<input type="range" min="0" max="1" step="0.01" value="${pr}" `
      +`data-beat="${name}" class="persBeatPriority" style="width:70px">`
      +`<b class="persBeatPriorityV" style="width:32px">${pr.toFixed(2)}</b>`
      +`<label style="display:flex;align-items:center;gap:4px;font-size:11.5px;color:var(--muted)">`
      +`<input type="checkbox" data-beat="${name}" class="persBeatEnabled" `
      +`${cfg.enabled?"checked":""}> on</label>`
      +`</span>`;
  }

  function wireBeatRows(){
    document.querySelectorAll(".persBeatPriority").forEach(el=>{
      const label=el.parentElement.querySelector(".persBeatPriorityV");
      el.oninput=()=>{ label.textContent=Number(el.value).toFixed(2); };
      el.onchange=()=>patch({registry:{[el.dataset.beat]:{priority:Number(el.value)}}});
    });
    document.querySelectorAll(".persBeatEnabled").forEach(el=>{
      el.onchange=()=>patch({registry:{[el.dataset.beat]:{enabled:el.checked}}});
    });
  }

  // Don't stomp a slider mid-drag: skip re-rendering any control the user is actively on.
  function busy(){
    const a=document.activeElement;
    return a && (a.classList.contains("persBeatPriority") || a.classList.contains("persBeatEnabled")
      || TRAITS.concat(DRIVES).some(k=>a.id===idFor(k)) || a.id==="pMood");
  }

  function render(p){
    if(busy()) return;
    $("persName").textContent=(p.name||"Nano")+(p.persona?" — "+p.persona:"");
    const traits=p.traits||{}, drives=p.drives||{};
    TRAITS.forEach(k=>{
      if(!(k in traits)) return;
      $(idFor(k)).value=traits[k]; $(idFor(k)+"V").textContent=Number(traits[k]).toFixed(2);
    });
    DRIVES.forEach(k=>{
      if(!(k in drives)) return;
      $(idFor(k)).value=drives[k]; $(idFor(k)+"V").textContent=Number(drives[k]).toFixed(2);
    });
    if($("pMood") && drives.mood!==undefined) $("pMood").value=drives.mood||"";
    const reg=p.registry||{}, box=$("persBeats");
    if(box) box.innerHTML=Object.keys(reg).sort().map(n=>beatRow(n,reg[n])).join("")
      || "<i>no beats registered yet</i>";
    wireBeatRows();
  }

  function poll(){
    fetch("/personality").then(r=>r.ok?r.json():null).then(d=>{ if(d) render(d); }).catch(()=>{});
  }
  setInterval(poll, 3000); poll();
})();
