from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from .scenarios import build


DEFAULT_TRACE_SCENARIOS = ("head_on", "crossing", "noisy", "multi_threat")


def _load(path: Path, *, required: bool = True) -> dict[str, Any] | None:
    if not path.exists():
        if required:
            raise FileNotFoundError(f"missing visualization input: {path}")
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _scenario_trace(name: str, seed: int) -> dict[str, Any]:
    simulation = build(name, seed, event_logging=True)
    simulation.config.trace_interval = 0.1
    summary = simulation.run()
    return {
        "name": name,
        "seed": seed,
        "summary": summary,
        "trace": simulation.trace,
        "events": [
            event for event in simulation.events
            if event["type"] in {"collision_alarm", "avoidance_phase", "node_reboot"}
        ],
    }


def dashboard_data(
    results_dir: str | Path,
    *,
    scenarios: Iterable[str] = DEFAULT_TRACE_SCENARIOS,
    seed: int = 7,
) -> dict[str, Any]:
    root = Path(results_dir)
    return {
        "readiness": _load(root / "pre_hardware_readiness.json"),
        "verification": _load(root / "verification.json"),
        "network": _load(root / "network_matrix.json"),
        "synthesis": _load(root / "fpga_synthesis.json"),
        "px4": _load(root / "px4_sitl.json", required=False),
        "closed_loop": _load(root / "px4_closed_loop.json", required=False),
        "traces": [_scenario_trace(name, seed) for name in scenarios],
    }


def build_dashboard(
    results_dir: str | Path,
    output: str | Path,
    *,
    scenarios: Iterable[str] = DEFAULT_TRACE_SCENARIOS,
    seed: int = 7,
) -> Path:
    payload = json.dumps(
        dashboard_data(results_dir, scenarios=scenarios, seed=seed),
        separators=(",", ":"),
    ).replace("</", "<\\/")
    rendered = _HTML.replace("__DASHBOARD_DATA__", payload)
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(rendered, encoding="utf-8")
    return destination


_HTML = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone simulation verification dashboard</title>
<style>
:root{color-scheme:dark;--bg:#071018;--panel:#101d28;--panel2:#152736;--ink:#e8f1f6;--muted:#93a9b8;--cyan:#37d6c0;--blue:#58a6ff;--gold:#f4c95d;--red:#ff6b6b;--line:#294252}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(145deg,#071018,#0b1924 55%,#071018);color:var(--ink);font:15px/1.45 ui-sans-serif,system-ui,sans-serif}main{max-width:1280px;margin:auto;padding:28px}h1{font-size:clamp(28px,5vw,52px);line-height:1.05;margin:0 0 8px}h2{margin:34px 0 14px;font-size:22px}h3{margin:0 0 10px;font-size:15px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}.lede{color:var(--muted);max-width:800px}.grid{display:grid;gap:14px}.kpis{grid-template-columns:repeat(auto-fit,minmax(175px,1fr));margin:24px 0}.two{grid-template-columns:repeat(auto-fit,minmax(360px,1fr))}.card{background:rgba(16,29,40,.94);border:1px solid var(--line);border-radius:14px;padding:16px;box-shadow:0 12px 35px #0005}.value{font-size:28px;font-weight:750;color:var(--cyan)}.label,.note{color:var(--muted);font-size:13px}.gates{grid-template-columns:repeat(auto-fit,minmax(190px,1fr))}.gate{display:flex;gap:9px;align-items:center;padding:10px 12px;border-radius:9px;background:var(--panel2)}.dot{width:10px;height:10px;border-radius:50%;background:var(--red);box-shadow:0 0 12px currentColor}.pass .dot{background:var(--cyan)}svg{width:100%;height:auto;display:block;overflow:visible}.axis{stroke:#557083;stroke-width:1}.threshold{stroke:var(--gold);stroke-dasharray:6 5;stroke-width:2}.safe{fill:var(--cyan)}.expected{fill:var(--red)}.barbg{fill:#203746}.tiny{font-size:11px;fill:var(--muted)}.text{font-size:12px;fill:var(--ink)}select,input{accent-color:var(--cyan);background:#0b1720;color:var(--ink);border:1px solid var(--line);border-radius:7px;padding:7px}input{width:100%}.controls{display:flex;gap:12px;align-items:center;margin-bottom:10px}.controls label{color:var(--muted)}#networkPlot .fast{fill:var(--cyan)}#networkPlot .balanced{fill:var(--gold)}#networkPlot .long_range{fill:var(--red)}.legend{display:flex;gap:16px;color:var(--muted);font-size:12px}.legend i{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px}.metric-row{display:grid;grid-template-columns:120px 1fr 62px;gap:8px;align-items:center;margin:10px 0}.track{height:13px;background:#213845;border-radius:7px;overflow:hidden}.fill{height:100%;background:var(--blue)}.fill.cyan{background:var(--cyan)}.fill.gold{background:var(--gold)}.event{padding:7px 0;border-bottom:1px solid var(--line);font-size:12px;color:var(--muted)}code{color:#b8e1ff}.footer{margin-top:35px;color:var(--muted);font-size:12px}@media(max-width:600px){main{padding:18px}.two{grid-template-columns:1fr}.controls{align-items:flex-start;flex-direction:column}}
</style>
</head>
<body><main>
<h1>Pre-hardware flight-safety dashboard</h1>
<p class="lede">Interactive, self-contained evidence view. All charts are generated from the JSON verification artifacts and deterministic scenario traces.</p>
<section id="kpis" class="grid kpis"></section>
<h2>Acceptance gates</h2><section id="gates" class="grid gates"></section>
<section class="grid two">
  <article class="card"><h2>Scenario separation</h2><p class="note">Worst run from each campaign; gold line is the operational requirement.</p><svg id="scenarioChart" viewBox="0 0 620 540"></svg></article>
  <article class="card"><h2>Network design space</h2><p class="note">Delivery ratio versus mean latency. Farther up and left is better.</p><div class="legend"><span><i style="background:var(--cyan)"></i>SF7</span><span><i style="background:var(--gold)"></i>SF9</span><span><i style="background:var(--red)"></i>SF12</span></div><svg id="networkPlot" viewBox="0 0 620 430"></svg></article>
</section>
<section class="grid two">
  <article class="card"><h2>Tracking tradeoff</h2><div id="tracking"></div></article>
  <article class="card"><h2>RTL synthesis footprint</h2><div id="rtl"></div></article>
</section>
<h2>Scenario trajectory replay</h2>
<article class="card">
  <div class="controls"><label for="scenario">Scenario</label><select id="scenario"></select><label id="timeLabel"></label></div>
  <input id="time" type="range" min="0" max="0" value="0">
  <svg id="trajectory" viewBox="0 0 900 500"></svg>
  <svg id="separation" viewBox="0 0 900 150"></svg>
  <div id="events"></div>
</article>
<p class="footer">Generated locally by <code>make visualize</code>. No network connection or JavaScript library is required.</p>
</main>
<script>const DATA=__DASHBOARD_DATA__;
const NS='http://www.w3.org/2000/svg';
const el=(name,attrs={},text='')=>{const n=document.createElementNS(NS,name);for(const[k,v]of Object.entries(attrs))n.setAttribute(k,v);if(text)n.textContent=text;return n};
const pct=v=>(100*v).toFixed(1)+'%'; const fixed=(v,n=1)=>Number(v).toFixed(n);
function kpis(){const v=DATA.verification,r=DATA.readiness,n=DATA.network.recommendation,c=DATA.closed_loop;const items=[['Readiness',r.passed+'/'+r.total+' gates'],['Deterministic checks',v.deterministic_acceptance.passed+'/'+v.deterministic_acceptance.total],['Campaign runs',v.network_and_system_campaign.runs.toLocaleString()],['Protocol CRC',v.protocol.single_bit_corruptions_detected.toLocaleString()+'/'+v.protocol.cases.toLocaleString()],['Recommended PDR',pct(n.delivery_ratio_mean)],['Closed-loop separation',c?fixed(c.actual_minimum_separation_m,2)+' m':'not run']];document.querySelector('#kpis').innerHTML=items.map(([l,x])=>`<div class="card"><div class="label">${l}</div><div class="value">${x}</div></div>`).join('')}
function gates(){document.querySelector('#gates').innerHTML=Object.entries(DATA.readiness.gates).map(([n,p])=>`<div class="gate ${p?'pass':''}"><span class="dot"></span><span>${n.replaceAll('_',' ')}</span></div>`).join('')}
function scenarioChart(){const svg=document.querySelector('#scenarioChart'),campaign=DATA.verification.network_and_system_campaign,req=campaign.minimum_separation_requirement_m||DATA.readiness.recommended_network.minimum_separation_m||4,rows=Object.entries(campaign.scenarios),max=Math.max(10,...rows.map(([,v])=>Math.min(v.minimum_separation_worst_m,20))),left=145,w=430,row=32;svg.setAttribute('viewBox',`0 0 620 ${rows.length*row+45}`);svg.append(el('line',{x1:left+req/max*w,x2:left+req/max*w,y1:8,y2:rows.length*row+5,class:'threshold'}));rows.forEach(([name,v],i)=>{const y=12+i*row,expected=['no_avoidance','command_loss','congested'].includes(name),value=Math.min(v.minimum_separation_worst_m,20);svg.append(el('text',{x:0,y:y+15,class:'text'},name));svg.append(el('rect',{x:left,y,width:w,height:18,rx:4,class:'barbg'}));svg.append(el('rect',{x:left,y,width:Math.max(1,value/max*w),height:18,rx:4,class:expected?'expected':'safe'}));svg.append(el('text',{x:left+Math.min(value/max*w,w)+7,y:y+14,class:'text'},fixed(value,2)+' m'))});svg.append(el('text',{x:left+req/max*w+5,y:rows.length*row+25,class:'tiny'},'requirement '+req+' m'))}
function networkPlot(){const svg=document.querySelector('#networkPlot'),rows=DATA.network.rows.filter(r=>r.latency_ms_mean!=null),W=540,H=340,L=55,T=20,maxX=Math.max(...rows.map(r=>r.latency_ms_mean));const x=v=>L+Math.log10(1+v)/Math.log10(1+maxX)*W,y=v=>T+(1-v)*H;svg.append(el('line',{x1:L,x2:L,y1:T,y2:T+H,class:'axis'}));svg.append(el('line',{x1:L,x2:L+W,y1:T+H,y2:T+H,class:'axis'}));[0,.25,.5,.75,1].forEach(v=>{svg.append(el('text',{x:12,y:y(v)+4,class:'tiny'},pct(v)));svg.append(el('line',{x1:L,x2:L+W,y1:y(v),y2:y(v),class:'axis',opacity:.22}))});rows.forEach(r=>{const viable=r.delivery_ratio_mean>=.7&&r.latency_ms_mean<=1500&&r.minimum_separation_worst_m>=4;const c=el('circle',{cx:x(r.latency_ms_mean),cy:y(r.delivery_ratio_mean),r:viable?7:4,class:r.phy,opacity:viable?1:.62,stroke:viable?'white':'none','stroke-width':2});c.append(el('title',{},`${r.phy} / ${r.mac} / ${r.routing} / ${r.telemetry_interval_s}s — PDR ${pct(r.delivery_ratio_mean)}, ${fixed(r.latency_ms_mean)} ms`));svg.append(c)});svg.append(el('text',{x:L+W-80,y:T+H+32,class:'tiny'},'latency →'));svg.append(el('text',{x:4,y:12,class:'tiny'},'PDR ↑'))}
function metricRow(label,value,kind=''){return `<div class="metric-row"><span>${label}</span><div class="track"><div class="fill ${kind}" style="width:${Math.min(100,value*100)}%"></div></div><strong>${pct(value)}</strong></div>`}
function tracking(){const t=DATA.readiness.tracking;document.querySelector('#tracking').innerHTML='<h3>Raw measurements</h3>'+metricRow('Recall',t.raw.recall)+metricRow('Precision',t.raw.precision,'gold')+'<h3 style="margin-top:22px">Filtered + uncertainty</h3>'+metricRow('Recall',t.filtered_uncertainty_aware.recall,'cyan')+metricRow('Precision',t.filtered_uncertainty_aware.precision,'gold')}
function rtl(){const mods=DATA.synthesis.modules,max=Math.max(...Object.values(mods).map(v=>v.cells));document.querySelector('#rtl').innerHTML=Object.entries(mods).map(([n,v])=>`<div class="metric-row"><span>${n}</span><div class="track"><div class="fill cyan" style="width:${v.cells/max*100}%"></div></div><strong>${v.cells}</strong></div>`).join('')+'<p class="note">Generic Yosys cells; this is elaboration evidence, not device utilization or timing closure.</p>'}
const colors=['#37d6c0','#ff6b6b','#58a6ff','#f4c95d','#c792ea','#82aaff','#89ddff','#f78c6c','#c3e88d','#ffcb6b','#f07178','#b2ccd6'];
function renderTrace(index){const item=DATA.traces[index],trace=item.trace,svg=document.querySelector('#trajectory'),sep=document.querySelector('#separation'),slider=document.querySelector('#time');svg.replaceChildren();sep.replaceChildren();slider.max=trace.length-1;slider.value=0;const points=trace.flatMap(t=>t.nodes.map(n=>n.position)),xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),pad=50,minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),spanX=Math.max(1,maxX-minX),spanY=Math.max(1,maxY-minY),scale=Math.min(800/spanX,400/spanY),sx=x=>pad+(x-minX)*scale,sy=y=>450-(y-minY)*scale;const ids=trace[0].nodes.map(n=>n.id);ids.forEach((id,j)=>{const d=trace.map((t,i)=>`${i?'L':'M'}${sx(t.nodes.find(n=>n.id===id).position[0])},${sy(t.nodes.find(n=>n.id===id).position[1])}`).join(' ');svg.append(el('path',{d,fill:'none',stroke:colors[j%colors.length],'stroke-width':j<2?3:1.5,opacity:j<2?1:.55}));const m=el('circle',{id:'node-'+id,cx:0,cy:0,r:j<2?8:5,fill:colors[j%colors.length],stroke:'white','stroke-width':1.5});svg.append(m);svg.append(el('text',{id:'label-'+id,x:0,y:0,class:'text'},'D'+id))});svg.append(el('text',{x:8,y:20,class:'tiny'},'NED horizontal plane (meters)'));const req=DATA.verification.network_and_system_campaign.minimum_separation_requirement_m||4,maxSep=Math.max(req,...trace.map(t=>t.separation_m)),x=i=>45+i/(trace.length-1)*825,y=v=>125-v/maxSep*100;sep.append(el('line',{x1:45,x2:870,y1:y(req),y2:y(req),class:'threshold'}));sep.append(el('path',{d:trace.map((t,i)=>`${i?'L':'M'}${x(i)},${y(t.separation_m)}`).join(' '),fill:'none',stroke:'#58a6ff','stroke-width':3}));sep.append(el('text',{x:5,y:y(req)+4,class:'tiny'},req+'m'));sep.append(el('text',{x:5,y:15,class:'tiny'},'separation'));document.querySelector('#events').innerHTML=item.events.map(e=>`<div class="event"><strong>${fixed(e.time,2)}s</strong> — ${e.type.replaceAll('_',' ')} ${e.maneuver?`(${e.maneuver})`:''}${e.after?`→ ${e.after}`:''}</div>`).join('');const update=()=>{const i=+slider.value,t=trace[i];ids.forEach(id=>{const n=t.nodes.find(n=>n.id===id),cx=sx(n.position[0]),cy=sy(n.position[1]);document.querySelector('#node-'+id).setAttribute('cx',cx);document.querySelector('#node-'+id).setAttribute('cy',cy);document.querySelector('#label-'+id).setAttribute('x',cx+10);document.querySelector('#label-'+id).setAttribute('y',cy+4)});document.querySelector('#timeLabel').textContent=`${fixed(t.time,1)} s · separation ${fixed(t.separation_m,2)} m`};slider.oninput=update;update()}
function traces(){const select=document.querySelector('#scenario');DATA.traces.forEach((t,i)=>select.add(new Option(`${t.name} (seed ${t.seed})`,i)));select.onchange=()=>renderTrace(+select.value);renderTrace(0)}
kpis();gates();scenarioChart();networkPlot();tracking();rtl();traces();
</script></body></html>'''
