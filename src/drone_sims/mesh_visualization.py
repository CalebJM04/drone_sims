from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Any


def build_mesh_dashboard(report: dict[str, object], output: str | Path) -> Path:
    payload = json.dumps(report, separators=(",", ":")).replace("</", "<\\/")
    rendered = _HTML.replace("__MESH_REPORT__", payload)
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(rendered, encoding="utf-8")
    return destination


class LiveMeshDashboard:
    def __init__(self, port: int) -> None:
        self.state: dict[str, object] = {"time_s": 0, "nodes": [], "links": [], "alerts": []}
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/api/state":
                    payload = json.dumps(owner.state).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                payload = _HTML.replace("__MESH_REPORT__", "null").encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, _format: str, *_args: object) -> None:
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    @property
    def address(self) -> str:
        host, port = self.server.server_address
        return f"http://{host}:{port}"

    def start(self) -> None:
        self.thread.start()

    def update(self, state: dict[str, object]) -> None:
        self.state = state

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


_HTML = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Live drone mesh</title><style>
:root{color-scheme:dark;--bg:#061018;--panel:#10212d;--line:#294653;--ink:#e9f5f5;--muted:#91a9b2;--good:#35d6b4;--warn:#f3c75f;--bad:#ff6577;--blue:#59a5ff}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 20% 0,#123044,#061018 50%);color:var(--ink);font:14px/1.4 system-ui,sans-serif}main{max-width:1250px;margin:auto;padding:24px}h1{font-size:clamp(28px,5vw,50px);margin:0}p{color:var(--muted)}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin:20px 0}.card{background:#10212ddd;border:1px solid var(--line);border-radius:14px;padding:15px}.value{font-size:28px;font-weight:750;color:var(--good)}.label{color:var(--muted)}.stage{display:grid;grid-template-columns:minmax(500px,2fr) minmax(280px,1fr);gap:14px}.map{min-height:570px}svg{width:100%;height:auto}.link{stroke-width:4}.healthy{stroke:var(--good)}.degrading{stroke:var(--warn)}.critical,.out_of_range{stroke:var(--bad)}.route{stroke:var(--blue);stroke-width:9;opacity:.42}.node{fill:#142f40;stroke:#dff;stroke-width:3}.node-label{fill:white;font-size:13px;text-anchor:middle}.alert{border-left:4px solid var(--bad);padding:8px 10px;margin:8px 0;background:#301822}.event{padding:8px 0;border-bottom:1px solid var(--line)}input{width:100%}@media(max-width:850px){.stage{grid-template-columns:1fr}.map{min-height:0}}
</style></head><body><main><h1>4+ node awareness mesh</h1><p>Actual companion services · proactive route look-ahead · collision alerts only · flight control disabled</p><section id="metrics" class="grid"></section><section class="stage"><article class="card map"><svg id="map" viewBox="0 0 800 560"></svg><input id="time" type="range" min="0" max="0" value="0"></article><aside><article class="card"><h2>Planned route D1 → D2</h2><div id="route" class="value">discovering</div></article><article class="card"><h2>Collision awareness</h2><div id="alerts">No active alerts</div></article><article class="card"><h2>Events</h2><div id="events"></div></article></aside></section></main><script>
const REPORT=__MESH_REPORT__,slider=document.querySelector('#time');let frames=REPORT?REPORT.snapshots:[],state=null;
const pct=v=>(v*100).toFixed(1)+'%',num=(v,n=1)=>Number(v||0).toFixed(n),colors=['#35d6b4','#ff6577','#59a5ff','#f3c75f','#b58cff','#7bdff2','#ff9f68','#a8e06c'];
function render(s){state=s;if(!s)return;const m=REPORT?REPORT.summary:{delivery_ratio:s.delivery_ratio,p95_latency_ms:0,maximum_state_age_s:0,control_commands:0};document.querySelector('#metrics').innerHTML=[['Time',num(s.time_s)+' s'],['Nodes',s.nodes.length],['Delivery',pct(s.delivery_ratio||m.delivery_ratio||0)],['P95 latency',num(m.p95_latency_ms)+' ms'],['Max state age',num(m.maximum_state_age_s)+' s'],['Control commands',m.control_commands||0]].map(([a,b])=>`<div class="card"><div class="label">${a}</div><div class="value">${b}</div></div>`).join('');const svg=document.querySelector('#map');svg.replaceChildren();const xs=s.nodes.map(n=>n.position[0]),ys=s.nodes.map(n=>n.position[1]),minX=Math.min(...xs)-5,maxX=Math.max(...xs)+5,minY=Math.min(...ys)-5,maxY=Math.max(...ys)+5,sx=x=>45+(x-minX)/Math.max(1,maxX-minX)*710,sy=y=>515-(y-minY)/Math.max(1,maxY-minY)*470,byId=Object.fromEntries(s.nodes.map(n=>[n.id,n]));const route=s.route_1_to_2||[];for(let i=0;i+1<route.length;i++){const a=byId[route[i]],b=byId[route[i+1]];if(a&&b)svg.insertAdjacentHTML('beforeend',`<line class="route" x1="${sx(a.position[0])}" y1="${sy(a.position[1])}" x2="${sx(b.position[0])}" y2="${sy(b.position[1])}"/>`)}s.links.filter(l=>l.connected_now).forEach(l=>{const a=byId[l.a],b=byId[l.b];svg.insertAdjacentHTML('beforeend',`<line class="link ${l.status}" x1="${sx(a.position[0])}" y1="${sy(a.position[1])}" x2="${sx(b.position[0])}" y2="${sy(b.position[1])}"><title>${l.status}: ${num(l.distance_m)}m, projected ${num(l.projected_distance_m)}m</title></line>`)});s.nodes.forEach((n,i)=>svg.insertAdjacentHTML('beforeend',`<circle class="node" style="fill:${colors[i%colors.length]}44" cx="${sx(n.position[0])}" cy="${sy(n.position[1])}" r="19"/><text class="node-label" x="${sx(n.position[0])}" y="${sy(n.position[1])+5}">D${n.id}</text>`));document.querySelector('#route').textContent=route.length?route.map(x=>'D'+x).join(' → '):'unavailable';document.querySelector('#alerts').innerHTML=s.alerts.length?s.alerts.map(a=>`<div class="alert">D${a.node_id} sees D${a.peer_id}<br>${num(a.current_distance_m)} m now · ${num(a.closest_distance_m)} m closest · ${num(a.time_to_closest_s)} s</div>`).join(''):'No active alerts';}
if(REPORT){slider.max=Math.max(0,frames.length-1);slider.oninput=()=>render(frames[+slider.value]);document.querySelector('#events').innerHTML=REPORT.events.map(e=>`<div class="event"><strong>${num(e.time_s)}s</strong> ${e.type.replaceAll('_',' ')}</div>`).join('');render(frames[0]);setInterval(()=>{if(+slider.value<frames.length-1){slider.value=+slider.value+1;render(frames[+slider.value])}},500)}else{slider.style.display='none';document.querySelector('#events').textContent='Live run in progress';setInterval(async()=>{try{render(await(await fetch('/api/state',{cache:'no-store'})).json())}catch(e){}},250)}
</script></body></html>'''
