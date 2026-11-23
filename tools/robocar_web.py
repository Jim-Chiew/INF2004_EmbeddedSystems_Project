#!/usr/bin/env python3
"""
robocar_web.py - a live web page for the car: the MQTT log, the car's status and the
START / STOP / RESET buttons, in your browser.

  python tools/robocar_web.py --host localhost      then open http://localhost:8080

It reads the car's messages from the Mosquitto broker (the same way robocar.py does), decodes them
with robocar.py, and pushes each one to the page as it arrives. It needs no extra Python packages
(paho-mqtt is used if installed, like robocar.py gui). Keep this file next to robocar.py.

Options: --host / --port (the MQTT broker), --web-port (default 8080), --no-browser,
         --lan (let other devices on your network open the page; they can then press the buttons too).
By default only this computer can open the page.
"""
import argparse
import collections
import json
import queue
import re
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import robocar as rc

BACKLOG = 300  # events kept so a page opened late (or refreshed) still shows recent history


class Hub:
    """Receives every MQTT message, turns it into an event, and hands it to all open pages."""

    def __init__(self):
        self.lock = threading.Lock()
        self.backlog = collections.deque(maxlen=BACKLOG)
        self.clients = []
        self.stats = {"frames": 0, "missing": 0, "bad": 0, "START": 0, "STOP": 0, "RESET": 0,
                      "barcodes": 0, "obstacles": 0, "humps": 0,
                      "car": "unknown", "car_at": ""}  # from robocar/car1/status
        self.last_seq = None

    def on_message(self, topic: str, data: bytes) -> None:
        with self.lock:
            ev = self._event(topic, data)
            ev.update(t=time.strftime("%H:%M:%S"), ts=time.time(), hex=data.hex(" "), stats=dict(self.stats))
            self.backlog.append(ev)
            for q in self.clients:
                try:
                    q.put_nowait(ev)
                except queue.Full:  # a stuck page must not slow the others down
                    pass

    def _event(self, topic: str, data: bytes) -> dict:
        s = self.stats
        if topic == rc.TOPIC_CMD:
            name = rc.COMMAND_NAMES.get(data[0]) if len(data) == 1 else None
            if name:
                s[name] += 1
                return {"kind": "cmd", "text": f"{name} sent to the car"}
            s["bad"] += 1
            return {"kind": "bad", "text": "command should be 1 byte: 1, 2 or 3"}
        if topic == rc.TOPIC_BARCODE:  # one per code the car reads: "A LEFT"
            code, direction = rc.describe_barcode(data)
            s["barcodes"] += 1
            return {"kind": "bc", "code": code, "dir": direction,
                    "text": f'barcode "{code}" decoded' + (f" -> {direction}" if direction else "")}
        if topic == rc.TOPIC_OBSTACLE:  # one per obstacle the car stops for
            s["obstacles"] += 1
            return {"kind": "obs", "text": rc.describe_obstacle(data)}
        if topic == rc.TOPIC_HUMP:  # one per hump a wheel rolls over: "h=12 s=R t=5.7"
            f = rc.parse_hump(data)
            s["humps"] += 1
            return {"kind": "hump", "h": f.get("h", "?"), "side": f.get("s", ""),
                    "text": rc.describe_hump(data)}
        if topic == rc.TOPIC_STATUS:  # retained: arrives at once when this page's reader connects
            online = data.strip() == b"online"
            s["car"], s["car_at"] = ("online" if online else "offline"), time.strftime("%H:%M:%S")
            return {"kind": "st", "text": "car CONNECTED to the broker" if online
                    else "car OFFLINE: the broker lost it (powered off or out of range)"}
        try:
            f = rc.unpack(data)
        except ValueError as e:
            s["bad"] += 1
            return {"kind": "bad", "text": str(e)}
        note = ""
        if self.last_seq is not None:
            missed = (f["seq"] - self.last_seq - 1) % 16
            if f["seq"] == self.last_seq:
                note = "  (repeat of the previous frame)"
            elif missed:
                s["missing"] += missed
                note = f"  ({missed} frame(s) missing)"
        self.last_seq = f["seq"]
        s["frames"] += 1
        text = rc.describe(f)
        return {"kind": "tel", "text": text + note, "fields": text.split(" | "), "run": rc.run_state(f)[0]}

    def subscribe(self):
        q = queue.Queue(maxsize=500)
        with self.lock:  # snapshot and register together, so no event is missed or doubled
            snapshot = list(self.backlog)
            self.clients.append(q)
        return q, snapshot

    def unsubscribe(self, q) -> None:
        with self.lock:
            self.clients.remove(q)


HUB = Hub()
ARGS = None  # set in main(); the Handler reads the MQTT host/port from it


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):  # keep the terminal quiet
        pass

    def _reply(self, code, body=b"", ctype="text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _host_ok(self) -> bool:
        # Blocks other websites from talking to this server through the browser (DNS rebinding).
        return ARGS.lan or self.headers.get("Host", "").split(":")[0] in ("localhost", "127.0.0.1")

    def do_GET(self):
        if not self._host_ok():
            return self._reply(403, b"forbidden")
        if self.path == "/":
            return self._reply(200, PAGE.encode(), "text/html; charset=utf-8")
        if self.path == "/events":
            return self._stream()
        self._reply(404, b"not found")

    def do_POST(self):
        m = re.fullmatch(r"/send/(start|stop|reset)", self.path)
        # The custom header can only be added by this page's own script, not by another website.
        if not m or not self._host_ok() or self.headers.get("X-Robocar") != "1":
            return self._reply(403, b"forbidden")
        try:
            code, err = rc.publish_command(ARGS, m.group(1))
        except SystemExit as e:  # robocar.find_tool exits when mosquitto_pub is not installed
            code, err = 1, str(e)
        self._reply(200, json.dumps({"ok": code == 0, "err": err}).encode(), "application/json")

    def _stream(self):
        q, snapshot = HUB.subscribe()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            first = json.dumps({"now": time.time(), "events": snapshot})
            self.wfile.write(f"event: backlog\ndata: {first}\n\n".encode())
            self.wfile.flush()
            while True:
                try:
                    out = f"data: {json.dumps(q.get(timeout=15))}\n\n"
                except queue.Empty:
                    out = ": keepalive\n\n"  # also notices a closed page
                self.wfile.write(out.encode())
                self.wfile.flush()
        except OSError:  # the page was closed
            pass
        finally:
            HUB.unsubscribe(q)


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robocar live</title>
<style>
:root{--bg:#f5f6f8;--card:#fff;--ink:#1c2230;--muted:#5d6678;--line:#dfe3ea;--ok:#1f7a3d;--okbg:#e3f4e8;--warn:#9a5b00;--warnbg:#fff1d6;
--bad:#b3261e;--badbg:#fde5e3;--idle:#4a5568;--idlebg:#e8ebf0;--accent:#1d5fd1;--cmd:#7a3db8;--mono:ui-monospace,Consolas,Menlo,monospace}
@media (prefers-color-scheme:dark){:root{--bg:#12151c;--card:#1b2029;--ink:#e7eaf0;--muted:#98a1b3;--line:#2b3240;--ok:#5fd18a;--okbg:#173325;
--warn:#f0b35a;--warnbg:#3a2c12;--bad:#ff8a80;--badbg:#3d1a18;--idle:#b4bdcc;--idlebg:#262c38;--accent:#7aa7ff;--cmd:#c79bff}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
.wrap{max-width:1180px;margin:0 auto;padding:16px}h1{font-size:20px;margin:0 0 12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center}.spacer{flex:1}
button{font:inherit;padding:7px 14px;border:1px solid var(--line);border-radius:6px;background:var(--card);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--accent)}button.go{background:var(--ok);border-color:var(--ok);color:#fff}
button.halt{background:var(--bad);border-color:var(--bad);color:#fff}
.pill{display:inline-block;padding:3px 12px;border-radius:999px;font-weight:700;font-size:14px}
.ok{background:var(--okbg);color:var(--ok)}.warn{background:var(--warnbg);color:var(--warn)}.bad{background:var(--badbg);color:var(--bad)}.idle{background:var(--idlebg);color:var(--idle)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin:12px 0}
.k{color:var(--muted);font-size:12px;text-transform:uppercase;letter-spacing:.04em}.v{font-size:22px;font-weight:650;font-variant-numeric:tabular-nums}.s{color:var(--muted);font-size:12px}
#frame{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:4px 18px;margin-top:8px;font:13.5px var(--mono)}
.tools{display:flex;flex-wrap:wrap;gap:12px;align-items:center;margin:14px 0 8px}.tools label{font-size:13px;color:var(--muted);display:flex;gap:4px;align-items:center}
#log{height:420px;overflow:auto;background:var(--card);border:1px solid var(--line);border-radius:10px;font:12.5px/1.5 var(--mono)}
.row{display:grid;grid-template-columns:70px 46px 1fr;gap:10px;padding:2px 10px;border-bottom:1px solid var(--line)}
.row .t{color:var(--muted)}.tel .g{color:var(--accent)}.cmd .g{color:var(--cmd);font-weight:700}.bad.row{background:var(--badbg)}.bad.row .g{color:var(--bad);font-weight:700}
.bc.row{background:var(--okbg)}.bc .g{color:var(--ok);font-weight:700}.obs.row{background:var(--warnbg)}.obs .g{color:var(--warn);font-weight:700}
.st .g{color:var(--accent);font-weight:700}.hump.row{background:var(--idlebg)}.hump .g{color:var(--cmd);font-weight:700}
.hex{color:var(--muted)}.m{white-space:pre-wrap;word-break:break-word}
@media (max-width:640px){.row{grid-template-columns:64px 1fr}.row .g{display:none}}
</style></head><body><div class="wrap">
<h1>Robocar live</h1>
<div class="card bar">
  <span id="conn" class="pill idle" title="from robocar/car1/status">CAR: no status yet</span>
  <span id="car" class="pill warn" title="from the telemetry frames">NO TELEMETRY</span>
  <button class="go" data-cmd="start">START</button><button class="halt" data-cmd="stop">STOP</button><button data-cmd="reset">RESET</button>
  <span id="sent" class="s">no command sent from this page yet</span>
  <span class="spacer"></span><span id="link" class="pill idle">connecting...</span>
</div>
<div class="grid">
  <div class="card"><div class="k">Last barcode</div><div class="v" id="bcCode">-</div><div class="s" id="bcInfo">none read yet · 0 in all</div></div>
  <div class="card"><div class="k">Last obstacle</div><div class="v" id="obsDist">-</div><div class="s" id="obsInfo">none yet · 0 in all</div></div>
  <div class="card"><div class="k">Last hump</div><div class="v" id="humpH">-</div><div class="s" id="humpInfo">none yet · 0 in all</div></div>
  <div class="card"><div class="k">Frames received</div><div class="v" id="nFrames">0</div><div class="s" id="rate">0.0 / s</div></div>
  <div class="card"><div class="k">Missing frames</div><div class="v" id="nMissing">0</div><div class="s">gaps in seq 0-15</div></div>
  <div class="card"><div class="k">Bad messages</div><div class="v" id="nBad">0</div><div class="s">wrong size, version or code</div></div>
  <div class="card"><div class="k">Commands seen</div><div class="v" id="nCmds">0</div><div class="s" id="cmdSplit"></div></div>
</div>
<div class="card"><b>Latest frame</b> <span class="s" id="lastAt">none yet</span><div id="frame"></div></div>
<div class="tools"><b>Log</b>
  <label><input type="checkbox" id="fTel" checked> telemetry</label><label><input type="checkbox" id="fCmd" checked> commands</label>
  <label><input type="checkbox" id="fBc" checked> barcodes</label><label><input type="checkbox" id="fObs" checked> obstacles</label>
  <label><input type="checkbox" id="fHump" checked> humps</label>
  <label><input type="checkbox" id="fScroll" checked> auto-scroll</label><span class="spacer"></span>
  <button id="pause">Pause</button><button id="clear">Clear</button><button id="save">Save log (.txt)</button></div>
<div id="log"></div>
</div>
<script>
"use strict";
const $ = id => document.getElementById(id), STALE_MS = 3000;
let lastTel = 0, rates = [], paused = false, lines = [], lastRun = "RUNNING";
const pill = (el, text, cls) => { el.textContent = text; el.className = "pill " + cls; };

function stats(s){
  $("nFrames").textContent = s.frames; $("nMissing").textContent = s.missing; $("nBad").textContent = s.bad;
  $("nCmds").textContent = s.START + s.STOP + s.RESET;
  $("cmdSplit").textContent = "START " + s.START + " · STOP " + s.STOP + " · RESET " + s.RESET;
  // robocar/car1/status: "online" from the car, "offline" from the broker (its Last Will)
  if (s.car === "online") pill($("conn"), "CAR CONNECTED", "ok");
  else if (s.car === "offline") pill($("conn"), "CAR OFFLINE", "bad");
  else pill($("conn"), "CAR: no status yet", "idle");
  $("conn").title = s.car_at ? "since " + s.car_at + " (robocar/car1/status)"
                             : "the car has not reported since this page's server started";
}
function frame(ev){
  const box = $("frame"); box.textContent = "";
  for (const f of ev.fields){ const d = document.createElement("div"); d.textContent = f; box.append(d); }
  $("lastAt").textContent = "received " + ev.t; lastRun = ev.run;
}
function row(ev){
  const text = ev.t + "  " + ev.kind.toUpperCase().padEnd(4) + " " + ev.hex + "  " + ev.text;
  lines.push(text); if (lines.length > 5000) lines.shift();
  const show = { tel: "fTel", cmd: "fCmd", bc: "fBc", obs: "fObs", hump: "fHump" }[ev.kind];
  if (show && !$(show).checked) return;
  const r = document.createElement("div"); r.className = "row " + ev.kind;
  const t = document.createElement("span"), g = document.createElement("span"), m = document.createElement("span"), h = document.createElement("span");
  t.className = "t"; t.textContent = ev.t; g.className = "g"; g.textContent = { tel: "TEL", cmd: "CMD", bc: "BC", obs: "OBS", st: "CAR", hump: "HUMP" }[ev.kind] || "BAD";
  m.className = "m"; h.className = "hex"; h.textContent = ev.hex + "  "; m.append(h, document.createTextNode(ev.text)); r.append(t, g, m);
  const log = $("log"); log.append(r);
  while (log.children.length > 500) log.removeChild(log.firstChild);
  if ($("fScroll").checked) log.scrollTop = log.scrollHeight;
}
function handle(ev, live, ageMs){
  stats(ev.stats);
  if (ev.kind === "tel"){ frame(ev); lastTel = Date.now() - ageMs; if (live) rates.push(Date.now()); }
  if (ev.kind === "bc"){
    $("bcCode").textContent = ev.code + (ev.dir ? " → " + ev.dir : "");
    $("bcInfo").textContent = "read at " + ev.t + " · " + (ev.stats.barcodes || 0) + " in all";
  }
  if (ev.kind === "hump"){
    const wheel = { R: "right wheel", L: "left wheel", C: "car pitched (rough)" }[ev.side] || "";
    $("humpH").textContent = ev.h + " mm";
    $("humpInfo").textContent = wheel + " · at " + ev.t + " · " + (ev.stats.humps || 0) + " in all";
  }
  if (ev.kind === "obs"){
    const d = /OBSTACLE (\d+) mm/.exec(ev.text);
    $("obsDist").textContent = d ? (d[1] / 10).toFixed(1) + " cm" : "seen";
    $("obsInfo").textContent = "at " + ev.t + " · " + (ev.stats.obstacles || 0) + " in all";
  }
  if (!paused || !live) row(ev);
}
const es = new EventSource("/events");
es.addEventListener("backlog", e => { const b = JSON.parse(e.data); b.events.forEach(ev => handle(ev, false, (b.now - ev.ts) * 1000)); });
es.onmessage = e => handle(JSON.parse(e.data), true, 0);
es.onopen = () => pill($("link"), "live", "ok");
es.onerror = () => pill($("link"), "server stopped - reconnecting", "bad");

setInterval(() => {
  const now = Date.now(); rates = rates.filter(t => now - t < 5000); $("rate").textContent = (rates.length / 5).toFixed(1) + " / s (last 5 s)";
  if (!lastTel || now - lastTel > STALE_MS) pill($("car"), "NO TELEMETRY", "idle");
  else pill($("car"), lastRun, lastRun === "RUNNING" ? "ok" : lastRun === "STOPPED" ? "idle" : "bad");
}, 500);

document.querySelectorAll("button[data-cmd]").forEach(b => b.onclick = async () => {
  const name = b.dataset.cmd.toUpperCase(); $("sent").textContent = "sending " + name + "...";
  try {
    const r = await (await fetch("/send/" + b.dataset.cmd, { method: "POST", headers: { "X-Robocar": "1" } })).json();
    $("sent").textContent = r.ok ? "sent " + name + " at " + new Date().toTimeString().slice(0, 8) : "FAILED to send " + name + ": " + (r.err || "mosquitto_pub error");
  } catch (e) { $("sent").textContent = "FAILED to send " + name + ": page cannot reach its server"; }
});
$("pause").onclick = () => { paused = !paused; $("pause").textContent = paused ? "Resume" : "Pause"; };
$("clear").onclick = () => { $("log").textContent = ""; lines = []; };
$("save").onclick = () => {
  const a = document.createElement("a"); a.href = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type: "text/plain" }));
  a.download = "robocar_log_" + new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-") + ".txt"; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};
</script></body></html>
"""


def main() -> None:
    global ARGS
    p = argparse.ArgumentParser(description="Live web page for the Robo Pico car")
    p.add_argument("--host", default=rc.DEFAULT_HOST, help="MQTT broker address (default %(default)s)")
    p.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    p.add_argument("--web-port", type=int, default=8080)
    p.add_argument("--lan", action="store_true", help="let other devices on your network open the page")
    p.add_argument("--no-browser", action="store_true")
    ARGS = p.parse_args()

    stop_reader = rc.start_reader(ARGS, HUB.on_message)
    server = ThreadingHTTPServer(("0.0.0.0" if ARGS.lan else "127.0.0.1", ARGS.web_port), Handler)
    server.daemon_threads = True
    url = f"http://localhost:{ARGS.web_port}"
    print(f"Robocar live page: {url}   (MQTT broker {ARGS.host}:{ARGS.port}, Ctrl+C to stop)")
    if not ARGS.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        stop_reader()


if __name__ == "__main__":
    main()