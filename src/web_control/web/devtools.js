// ---- Synthetic sensors (dev harness only) -----------------------------------
// Polls /dev/sensors to detect if the dev harness is running. If so, shows the
// "Synthetic sensors (dev)" card in the Sim panel and wires up the form. The form
// is generated from the `fields` spec returned by /dev/sensors, so it never drifts
// from the server's SENSOR_FIELDS table. Manual mode freezes the synthetic sensor
// values; random mode lets them jitter lifelike. Changes POST to /dev/sensors and
// persist across dev harness restarts.
(function(){
  "use strict";
  const $=id=>document.getElementById(id);
  const card=$("devSensorsCard");
  if(!card) return;
  let fields={}, vals={};

  function buildForm(){
    const box=$("devSensorsFields"); if(!box) return;
    box.innerHTML="";
    for(const [k,spec] of Object.entries(fields)){
      const row=document.createElement("div"); row.className="field";
      const lab=document.createElement("label");
      const valSpan=document.createElement("span"); valSpan.id="devFv_"+k;
      valSpan.textContent=vals[k];
      lab.textContent=spec.label+" "; lab.appendChild(valSpan);
      row.appendChild(lab);
      if(spec.type==="bool"){
        const sw=document.createElement("label"); sw.className="switch";
        const cb=document.createElement("input"); cb.type="checkbox"; cb.id="devF_"+k;
        cb.checked=!!vals[k];
        cb.onchange=()=>{ vals[k]=cb.checked; valSpan.textContent=cb.checked?"1":"0"; };
        const track=document.createElement("span"); track.className="track";
        sw.appendChild(cb); sw.appendChild(track);
        row.appendChild(sw);
      } else {
        const rng=document.createElement("input"); rng.type="range"; rng.id="devF_"+k;
        rng.min=spec.min; rng.max=spec.max; rng.step=(spec.type==="int")?1:0.5; rng.value=vals[k];
        rng.oninput=()=>{ valSpan.textContent=rng.value; vals[k]=parseFloat(rng.value); };
        row.appendChild(rng);
      }
      box.appendChild(row);
    }
    // disable fields when not in manual mode
    Array.from(box.querySelectorAll("input")).forEach(el=>{
      if(el.type!=="checkbox") el.disabled=!$("devManual").checked;
    });
  }

  function loadSensors(){
    fetch("/dev/sensors").then(r=>{
      if(!r.ok) throw new Error("not available");
      return r.json();
    }).then(s=>{
      fields=s.fields||{}; vals=Object.assign({},s.values||{});
      $("devManual").checked=!!s.manual;
      buildForm();
      card.style.display="";  // show the card
      const st=$("devSensorsStatus");
      if(st) st.textContent="source: "+(s.manual?"manual":"random");
    }).catch(()=>{
      // /dev/sensors not available (real robot) — hide the card silently
      card.style.display="none";
    });
  }

  $("devManual").addEventListener("change",()=>{
    const box=$("devSensorsFields");
    Array.from(box.querySelectorAll("input")).forEach(el=>{
      if(el.type!=="checkbox") el.disabled=!$("devManual").checked;
    });
  });

  $("devSensorsApply").onclick=()=>{
    fetch("/dev/sensors",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({manual:$("devManual").checked, values:vals})})
      .then(r=>r.ok?r.json():null).then(s=>{
        if(!s) return;
        vals=Object.assign({},s.values||{});
        $("devManual").checked=!!s.manual;
        buildForm();
        const st=$("devSensorsStatus");
        if(st) st.textContent="source: "+(s.manual?"manual":"random")+" — saved";
      }).catch(()=>{});
  };

  $("devSensorsReload").onclick=loadSensors;

  // Try to load on page load; if the endpoint doesn't exist (real robot), the card stays hidden.
  loadSensors();
})();
