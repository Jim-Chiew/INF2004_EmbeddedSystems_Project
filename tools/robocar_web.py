#!/usr/bin/env python3
"""
robocar_web.py - a live web page for the car: what it is doing right now, every MQTT message as
it arrives, and the START / STOP / RESET buttons, in your browser.

  python tools/robocar_web.py --host localhost      then open http://localhost:8080

It reads the car's messages from the Mosquitto broker (the same way robocar.py does), decodes them
with robocar.py, and pushes each one to the page as it arrives. It needs no extra Python packages
(paho-mqtt is used if installed, like robocar.py gui). Keep this file next to robocar.py.

The page is in two halves:

  CURRENT CONDITIONS -- the newest telemetry frame as tiles: run state, speed, heading and
    compass point, line position, distance ahead, the measured object length, pitch and roll,
    hump jolt, the barcode direction and every status flag (error, barcode error, hump,
    dropped, Wi-Fi loss, low battery), plus the latest barcode / obstacle / hump message and
    a table of every field in the frame.
  RAW LOG -- every message in order with its hex, filters per topic, pause / clear / save.

The frames come from robocar/car1/telemetry, which the car publishes only while
S1_PUBLISH_TELEMETRY is 1 in app_program/subsystem1/subsystem1.h -- if the car is connected
but no frame arrives, the page says so.

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

BACKLOG = 600  # events kept so a page opened late (or refreshed) still shows recent history
               # (at 5 frames a second that is about two minutes)


class Hub:
    """Receives every MQTT message, turns it into an event, and hands it to all open pages."""

    def __init__(self):
        self.lock = threading.Lock()
        self.backlog = collections.deque(maxlen=BACKLOG)
        self.clients = []
        # Counters, the car's connection state, and the latest of each event kind. The latest
        # ones are kept here (not only in the backlog) so a page opened or refreshed much later
        # still shows the last barcode / obstacle / hump even after it has scrolled out.
        self.stats = {"frames": 0, "missing": 0, "bad": 0, "START": 0, "STOP": 0, "RESET": 0,
                      "barcodes": 0, "obstacles": 0, "humps": 0,
                      "car": "unknown", "car_at": "",            # from robocar/car1/status
                      "last_bc": None, "last_obs": None, "last_hump": None,
                      "jolt_max": 0.0}                           # biggest hump jolt seen, g
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
        now = time.strftime("%H:%M:%S")
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
            s["last_bc"] = {"code": code, "dir": direction, "t": now}
            return {"kind": "bc", "code": code, "dir": direction,
                    "text": f'barcode "{code}" decoded' + (f" -> {direction}" if direction else "")}
        if topic == rc.TOPIC_OBSTACLE:  # one per obstacle the car stops for
            f = rc.parse_fields(data)
            s["obstacles"] += 1
            text = rc.describe_obstacle(data)
            s["last_obs"] = {"t": now, "text": text, "d": f.get("d"), "w": f.get("w"),
                             "n": f.get("n"), "side": f.get("s", "")}
            return {"kind": "obs", "text": text}
        if topic == rc.TOPIC_HUMP:  # one per hump a wheel rolls over: "h=12 s=R t=5.7"
            f = rc.parse_fields(data)
            s["humps"] += 1
            text = rc.describe_hump(data)
            s["last_hump"] = {"t": now, "text": text, "h": f.get("h", "?"),
                              "side": f.get("s", ""), "tilt": f.get("t")}
            return {"kind": "hump", "h": f.get("h", "?"), "side": f.get("s", ""), "text": text}
        if topic == rc.TOPIC_STATUS:  # retained: arrives at once when this page's reader connects
            online = data.strip() == b"online"
            s["car"], s["car_at"] = ("online" if online else "offline"), now
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
        sm = rc.summary(f)
        if sm["jolt"] and sm["jolt"] > s["jolt_max"]:
            s["jolt_max"] = sm["jolt"]
        text = rc.describe(f)
        # "sum" is every field as a value the page can show directly (robocar.summary);
        # "raw" is the same frame as the bit fields the car actually sent.
        return {"kind": "tel", "text": text + note, "sum": sm, "raw": f, "run": sm["run"]}

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
/* Colours: the validated data-viz palette. Blue is the one data hue; the four status
   colours (good / warning / serious / critical) mean state only, and never carry a
   meaning on their own -- each one sits beside a word. */
:root{
  color-scheme:light;
  --plane:#f9f9f7; --surface:#fcfcfb; --ink:#0b0b0b; --ink2:#52514e; --muted:#898781;
  --grid:#e1e0d9; --axis:#c3c2b7; --ring:rgba(11,11,11,.10);
  --s1:#2a78d6; --track:#cde2fb;
  --good:#0ca30c; --warn:#fab219; --serious:#ec835a; --crit:#d03b3b;
  --t-good:#e4f6e4; --t-warn:#fef3d8; --t-serious:#fbeae1; --t-crit:#fbe7e7; --t-accent:#e8f1fd;
  --mono:ui-monospace,Consolas,Menlo,monospace;
  --sans:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
}
@media (prefers-color-scheme:dark){:root{
  color-scheme:dark;
  --plane:#0d0d0d; --surface:#1a1a19; --ink:#f0efec; --ink2:#c3c2b7; --muted:#898781;
  --grid:#2c2c2a; --axis:#383835; --ring:rgba(255,255,255,.10);
  --s1:#3987e5; --track:#184f95;
  --t-good:#142e14; --t-warn:#33280d; --t-serious:#30201a; --t-crit:#2e1716; --t-accent:#11263d;
}}
*{box-sizing:border-box}
body{margin:0;background:var(--plane);color:var(--ink);font:15px/1.45 var(--sans)}
.wrap{max-width:1240px;margin:0 auto;padding:16px 16px 44px}
h1{font-size:19px;font-weight:650;margin:0 0 12px}
h2{font-size:11.5px;letter-spacing:.07em;text-transform:uppercase;color:var(--muted);font-weight:650;margin:0}
.head{display:flex;flex-wrap:wrap;gap:10px;align-items:baseline;margin:22px 0 8px}
.card{background:var(--surface);border:1px solid var(--ring);border-radius:10px;padding:12px 14px}
.bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
.spacer{flex:1}
button{font:inherit;padding:7px 14px;border:1px solid var(--axis);border-radius:6px;background:var(--surface);color:var(--ink);cursor:pointer}
button:hover{border-color:var(--s1)}
button.go{background:var(--t-good);border-color:var(--good)}
button.halt{background:var(--t-crit);border-color:var(--crit)}
/* pills and chips: a coloured dot for the state, always beside its word */
.pill,.chip{display:inline-flex;gap:7px;align-items:center;border:1px solid var(--ring);border-radius:999px;background:var(--surface)}
.pill{padding:5px 13px;font-size:13.5px;font-weight:650}
.chip{padding:5px 11px;font-size:13px;font-weight:600;color:var(--ink2)}
.dot{width:9px;height:9px;border-radius:50%;background:var(--axis);flex:none}
.p-good,.c-good.on{background:var(--t-good);border-color:transparent}
.p-good .dot,.c-good.on .dot{background:var(--good)}
.p-warn,.c-warn.on{background:var(--t-warn);border-color:transparent}
.p-warn .dot,.c-warn.on .dot{background:var(--warn)}
.p-serious,.c-serious.on{background:var(--t-serious);border-color:transparent}
.p-serious .dot,.c-serious.on .dot{background:var(--serious)}
.p-crit,.c-crit.on{background:var(--t-crit);border-color:transparent}
.p-crit .dot,.c-crit.on .dot{background:var(--crit)}
.p-accent,.c-accent.on{background:var(--t-accent);border-color:transparent}
.p-accent .dot,.c-accent.on .dot{background:var(--s1)}
.chip.on{color:var(--ink)}
.chips{display:flex;flex-wrap:wrap;gap:8px;margin-top:9px}
/* tiles */
.tiles{display:grid;grid-template-columns:repeat(12,1fr);gap:10px}
.tile{grid-column:span 3;min-width:0}
.w6{grid-column:span 6}
.t2{grid-column:span 2}
@media (max-width:980px){.tile{grid-column:span 6}.w6{grid-column:span 12}.t2{grid-column:span 4}}
@media (max-width:560px){.tile,.w6{grid-column:span 12}.t2{grid-column:span 6}}
.k{color:var(--muted);font-size:11.5px;text-transform:uppercase;letter-spacing:.05em;font-weight:650}
.v{font-size:23px;font-weight:650;margin-top:3px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.v .u{font-size:13px;color:var(--ink2);font-weight:500;margin-left:4px}
.hero .v{font-size:52px;line-height:1.08;letter-spacing:-.015em}
.hero .v .u{font-size:15px}
.s{color:var(--ink2);font-size:12px;margin-top:3px}
.s.n{color:var(--muted)}
.sev{display:inline-flex;gap:5px;align-items:center;font-weight:600;color:var(--ink)}
#cond.stale{opacity:.48}
/* marks: 2px lines, >=8px end dot, a 10% area wash, hairline baseline */
.spark{width:100%;height:62px;display:block;margin-top:8px;touch-action:none;outline:none}
.hero .spark{height:92px}
.spark .ln{fill:none;stroke:var(--s1);stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
.spark .ar{fill:var(--s1);opacity:.10;stroke:none}
.spark .dt{fill:var(--s1);stroke:var(--surface);stroke-width:2}
.spark .bl{stroke:var(--grid);stroke-width:1}
.spark .cr{stroke:var(--axis);stroke-width:1;display:none}
.spark .mx{fill:var(--muted);font:10px var(--sans)}
.spark:focus-visible{outline:2px solid var(--s1);outline-offset:2px;border-radius:4px}
/* meter: the fill carries severity, the track is a lighter step of the same ramp */
.meter{height:8px;border-radius:999px;background:var(--track);margin-top:9px;overflow:hidden}
.meter>span{display:block;height:100%;width:0;border-radius:999px;background:var(--s1);transition:width .15s linear}
.dev{position:relative;height:8px;border-radius:999px;background:var(--track);margin-top:7px}
.dev:before{content:"";position:absolute;left:50%;top:-3px;bottom:-3px;width:1px;background:var(--axis)}
.dev>span{position:absolute;top:0;bottom:0;width:0;border-radius:999px;background:var(--s1)}
.lane{display:flex;gap:2px;margin-top:9px}
.lane>span{flex:1;height:8px;border-radius:3px;background:var(--track)}
.lane>span.on{background:var(--s1)}
.dialrow{display:flex;gap:12px;align-items:center}
.dialrow .num{min-width:0}
svg.dial{flex:none}
.dial .rim{fill:none;stroke:var(--axis);stroke-width:1}
.dial .tk{stroke:var(--axis);stroke-width:1}
.dial .lb{fill:var(--muted);font:9px var(--sans);text-anchor:middle}
.dial .nd{stroke:var(--s1);stroke-width:2;stroke-linecap:round}
.dial .hub{fill:var(--s1);stroke:var(--surface);stroke-width:2}
/* table view: every field of the newest frame */
table{width:100%;border-collapse:collapse;font-size:13px;margin-top:2px}
th,td{text-align:left;padding:4px 8px;border-bottom:1px solid var(--grid)}
th{color:var(--muted);font-size:10.5px;text-transform:uppercase;letter-spacing:.05em;font-weight:650}
td.rw{color:var(--muted);font-family:var(--mono);font-size:12px;font-variant-numeric:tabular-nums;text-align:right;width:70px}
/* log */
.tools{display:flex;flex-wrap:wrap;gap:12px;align-items:center;margin:8px 0}
.tools label{font-size:13px;color:var(--ink2);display:flex;gap:5px;align-items:center}
#log{height:360px;overflow:auto;background:var(--surface);border:1px solid var(--ring);border-radius:10px;font:12.5px/1.5 var(--mono)}
.row{display:grid;grid-template-columns:72px 58px 1fr;gap:10px;padding:2px 10px;border-bottom:1px solid var(--grid)}
.row .t{color:var(--muted)}
.row .g{color:var(--ink2);display:flex;gap:6px;align-items:center;font-family:var(--sans);font-size:11px;font-weight:650;letter-spacing:.04em}
.row .key{width:8px;height:8px;border-radius:2px;background:var(--s1);flex:none}
.bc .key{background:var(--good)}.obs .key{background:var(--serious)}
.hump .key{background:var(--warn)}.bad .key{background:var(--crit)}.st .key{background:var(--axis)}
.bc.row{background:var(--t-good)}.obs.row{background:var(--t-serious)}.hump.row{background:var(--t-warn)}
.cmd.row{background:var(--t-accent)}.bad.row{background:var(--t-crit)}
.hex{color:var(--muted)}.m{white-space:pre-wrap;word-break:break-word}
@media (max-width:640px){.row{grid-template-columns:64px 1fr}.row .g{display:none}}
#tip{position:fixed;z-index:9;display:none;pointer-events:none;background:var(--surface);color:var(--ink);
 border:1px solid var(--ring);border-radius:8px;padding:6px 9px;font-size:12.5px;box-shadow:0 3px 12px rgba(0,0,0,.2)}
#tip b{font-size:14px;font-weight:650}#tip span{color:var(--ink2);margin-left:6px}
</style></head><body><div class="wrap">

<h1>Robocar live</h1>

<div class="card bar">
  <span id="conn" class="pill p-idle" title="robocar/car1/status"><span class="dot"></span><span class="lab">No status yet</span></span>
  <span id="run" class="pill p-idle" title="from the telemetry frames"><span class="dot"></span><span class="lab">No telemetry</span></span>
  <button class="go" data-cmd="start">START</button><button class="halt" data-cmd="stop">STOP</button><button data-cmd="reset">RESET</button>
  <span id="sent" class="s">no command sent from this page yet</span>
  <span class="spacer"></span>
  <span id="link" class="pill p-idle"><span class="dot"></span><span class="lab">connecting...</span></span>
</div>

<div class="head"><h2>Current conditions</h2><span class="s n" id="telAt">no frame yet</span>
  <span class="spacer"></span><span class="s n" id="telHint"></span></div>

<div class="tiles" id="cond">
  <div class="card tile hero w6">
    <div class="k">Speed</div>
    <div class="v"><span id="vSpeed">&ndash;</span><span class="u" id="uSpeed">cm/s</span></div>
    <div class="s" id="sSpeed">from the wheel encoders</div>
    <svg class="spark" id="spSpeed" tabindex="0" role="img" aria-label="Speed over the last 12 seconds">
      <path class="ar"/><path class="ln"/><line class="bl"/><line class="cr"/><circle class="dt" r="4"/><text class="mx" x="3" y="11"></text></svg>
  </div>
  <div class="card tile">
    <div class="k">Heading</div>
    <div class="dialrow">
      <div class="num">
        <div class="v"><span id="vHeading">&ndash;</span></div>
        <div class="s" id="sHeading">compass heading from the IMU</div>
      </div>
      <svg class="dial" id="dial" viewBox="0 0 100 100" width="92" height="92" aria-hidden="true">
        <circle class="rim" cx="50" cy="50" r="33"/>
        <line class="tk" x1="50" y1="17" x2="50" y2="23"/><line class="tk" x1="83" y1="50" x2="77" y2="50"/>
        <line class="tk" x1="50" y1="83" x2="50" y2="77"/><line class="tk" x1="17" y1="50" x2="23" y2="50"/>
        <text class="lb" x="50" y="11">N</text><text class="lb" x="95" y="53">E</text>
        <text class="lb" x="50" y="98">S</text><text class="lb" x="5" y="53">W</text>
        <line class="nd" id="dialN" x1="50" y1="50" x2="50" y2="21"/><circle class="hub" cx="50" cy="50" r="4"/>
      </svg>
    </div>
  </div>
  <div class="card tile">
    <div class="k">Distance ahead</div>
    <div class="v"><span id="vDist">&ndash;</span></div>
    <div class="meter"><span id="mDist"></span></div>
    <div class="s"><span class="sev" id="dSev"><span class="dot"></span><span class="lab">no reading</span></span><span id="sDist"></span></div>
    <svg class="spark" id="spDist" tabindex="0" role="img" aria-label="Distance ahead over the last 12 seconds">
      <path class="ar"/><path class="ln"/><line class="bl"/><line class="cr"/><circle class="dt" r="4"/><text class="mx" x="3" y="11"></text></svg>
  </div>

  <div class="card tile">
    <div class="k">Navigation state</div>
    <div class="v" style="font-size:17px" id="vState">&ndash;</div>
    <div class="s" id="sState">the car's own state field</div>
  </div>
  <div class="card tile">
    <div class="k">Line position</div>
    <div class="v" style="font-size:19px" id="vLine">&ndash;</div>
    <div class="lane"><span id="laneL"></span><span id="laneC"></span><span id="laneR"></span></div>
    <div class="s">where the car sits: drifted left &middot; centred &middot; drifted right</div>
  </div>
  <div class="card tile">
    <div class="k">Pitch and roll</div>
    <div class="v" style="font-size:19px"><span id="vTilt">&ndash;</span><span class="u">pitch</span> <span id="vRoll">&ndash;</span><span class="u">roll</span></div>
    <div class="dev"><span id="bTilt"></span></div>
    <div class="dev"><span id="bRoll"></span></div>
    <div class="s">centre of each bar = level &middot; scale &plusmn;30&deg;</div>
  </div>
  <div class="card tile">
    <div class="k">Object length</div>
    <div class="v"><span id="vObj">&ndash;</span></div>
    <div class="meter"><span id="mObj"></span></div>
    <div class="s" id="sObj">measured by the servo scan, held until the next one</div>
  </div>

  <div class="card tile">
    <div class="k">Hump jolt</div>
    <div class="v"><span id="vJolt">&ndash;</span></div>
    <div class="meter"><span id="mJolt"></span></div>
    <div class="s" id="sJolt">peak since the previous frame &middot; 0 = no hump</div>
  </div>
  <div class="card tile">
    <div class="k">Barcode direction</div>
    <div class="v" style="font-size:19px" id="vBc">&ndash;</div>
    <div class="s" id="sBc">the last LEFT / RIGHT / FORWARD the car read</div>
  </div>
  <div class="card tile w6">
    <div class="k">Status flags</div>
    <div class="chips">
      <span class="chip c-crit" id="fError" title="the IMU is missing or its last read failed"><span class="dot"></span>Error / fault</span>
      <span class="chip c-serious" id="fBcErr" title="a barcode pass did not decode"><span class="dot"></span>Barcode error</span>
      <span class="chip c-accent" id="fHump" title="a hump ended since the previous frame"><span class="dot"></span>Hump</span>
      <span class="chip c-warn" id="fDrop" title="a message was dropped since the previous frame"><span class="dot"></span>Dropped</span>
      <span class="chip c-warn" id="fWifi" title="the car's Wi-Fi radio was down when it built the frame"><span class="dot"></span>Wi-Fi loss</span>
      <span class="chip c-warn" id="fBatt" title="the battery is not measured on this car, so this is always clear"><span class="dot"></span>Low battery</span>
    </div>
    <div class="s">lit = set in the newest frame. None of them stops the car.</div>
  </div>
</div>

<div class="head"><h2>Latest events</h2><span class="s n">one message each, as they happen</span></div>
<div class="tiles">
  <div class="card tile"><div class="k">Last barcode</div><div class="v" id="bcCode">&ndash;</div><div class="s" id="bcInfo">none read yet &middot; 0 in all</div></div>
  <div class="card tile w6"><div class="k">Last obstacle</div><div class="v" id="obsDist">&ndash;</div><div class="s" id="obsInfo">none yet &middot; 0 in all</div></div>
  <div class="card tile"><div class="k">Last hump</div><div class="v" id="humpH">&ndash;</div><div class="s" id="humpInfo">none yet &middot; 0 in all</div></div>
</div>

<div class="head"><h2>Link and stream</h2><span class="s n">how much of the telemetry is getting through</span></div>
<div class="tiles">
  <div class="card tile t2"><div class="k">Frames received</div><div class="v" id="nFrames">0</div><div class="s" id="rate">0.0 / s</div></div>
  <div class="card tile t2"><div class="k">Newest frame</div><div class="v" id="age">&ndash;</div><div class="s" id="seqInfo">seq &ndash;</div></div>
  <div class="card tile t2"><div class="k">Missing frames</div><div class="v" id="nMissing">0</div><div class="s">gaps in the sequence 0&ndash;15</div></div>
  <div class="card tile t2"><div class="k">Bad messages</div><div class="v" id="nBad">0</div><div class="s">wrong size, version or code</div></div>
  <div class="card tile t2"><div class="k">Commands seen</div><div class="v" id="nCmds">0</div><div class="s" id="cmdSplit">START 0 &middot; STOP 0 &middot; RESET 0</div></div>
  <div class="card tile t2"><div class="k">Biggest hump jolt</div><div class="v" id="joltMax">&ndash;</div><div class="s">largest in any frame so far</div></div>
</div>

<div class="head"><h2>Newest frame, every field</h2><span class="s n" id="frameHex">no frame yet</span></div>
<div class="card"><table><thead><tr><th>Field</th><th>Value</th><th class="rw">raw</th></tr></thead><tbody id="fieldRows"></tbody></table></div>

<div class="head"><h2>Raw log</h2><span class="s n">every message in order, with the bytes as they arrived</span></div>
<div class="tools">
  <label><input type="checkbox" id="fTel" checked> telemetry</label><label><input type="checkbox" id="fCmd" checked> commands</label>
  <label><input type="checkbox" id="fBc" checked> barcodes</label><label><input type="checkbox" id="fObs" checked> obstacles</label>
  <label><input type="checkbox" id="fHump" checked> humps</label>
  <label><input type="checkbox" id="fScroll" checked> auto-scroll</label><span class="spacer"></span>
  <button id="pause">Pause</button><button id="clear">Clear</button><button id="save">Save log (.txt)</button></div>
<div id="log"></div>

</div><div id="tip"></div>
<script>
"use strict";
const $ = id => document.getElementById(id);
const STALE_MS = 3000, WINDOW_MS = 12000, HIST_MAX = 400, STOP_CM = 15, JOLT_FULL = 2.56;
const S = {lastTel: 0, everTel: false, car: "unknown", hist: {speed: [], dist: []},
           rates: [], paused: false, lines: []};

function pill(el, text, cls){ el.className = "pill " + cls; el.querySelector(".lab").textContent = text; }
function sev(el, cls, word){ el.className = "sev " + cls; el.querySelector(".lab").textContent = word; }
function sign(v){ return (v > 0 ? "+" : "") + v; }
function yn(b){ return b ? "yes" : "no"; }
function deg(v){ return v == null ? "n/a" : sign(v) + "°"; }

/* ---------- counters, car state, latest events ---------- */
function stats(s){
  $("nFrames").textContent = s.frames;
  $("nMissing").textContent = s.missing;
  $("nBad").textContent = s.bad;
  $("nCmds").textContent = s.START + s.STOP + s.RESET;
  $("cmdSplit").textContent = "START " + s.START + " · STOP " + s.STOP + " · RESET " + s.RESET;
  $("joltMax").textContent = s.jolt_max ? s.jolt_max.toFixed(2) + " g" : "none";
  S.car = s.car;
  if (s.car === "online") pill($("conn"), "Car connected", "p-good");
  else if (s.car === "offline") pill($("conn"), "Car offline", "p-crit");
  else pill($("conn"), "No status yet", "p-idle");
  $("conn").title = s.car_at ? "robocar/car1/status, since " + s.car_at
                             : "the car has not reported since this page's server started";
  const bc = s.last_bc;
  $("bcCode").textContent = bc ? (bc.code + (bc.dir ? " → " + bc.dir : "")) : "–";
  $("bcInfo").textContent = bc ? "read at " + bc.t + " · " + s.barcodes + " in all"
                               : "none read yet · 0 in all";
  const ob = s.last_obs;
  $("obsDist").textContent = ob ? (ob.d ? (ob.d / 10).toFixed(1) + " cm ahead" : "seen") : "–";
  $("obsInfo").textContent = ob ? ob.text + " · at " + ob.t + " · " + s.obstacles + " in all"
                                : "none yet · 0 in all";
  const hp = s.last_hump;
  $("humpH").textContent = hp ? hp.h + " mm" : "–";
  $("humpInfo").textContent = hp
    ? ({R: "right wheel", L: "left wheel", C: "car pitched (rough)"}[hp.side] || "")
      + (hp.tilt ? " · peak tilt " + hp.tilt + "°" : "")
      + " · at " + hp.t + " · " + s.humps + " in all"
    : "none yet · 0 in all";
}

/* ---------- the newest frame ---------- */
const ROWS = [
  ["Run state",         m => m.run,                                                   null],
  ["Navigation state",  m => m.state,                                                 "state"],
  ["Speed",             m => m.speed == null ? "n/a" : m.speed + " cm/s",              "speed"],
  ["Line position",     m => m.line,                                                  "line"],
  ["Heading",           m => m.heading == null ? "n/a (IMU not calibrated)" : m.heading + "° " + m.compass, "dir"],
  ["Pitch",             m => deg(m.tilt),                                             "tilt"],
  ["Roll",              m => deg(m.roll),                                             "roll"],
  ["Distance ahead",    m => m.dist == null ? "nothing within 62 cm / no echo" : m.dist + " cm", "dist"],
  ["Object length",     m => m.obj_len == null ? "none measured yet" : m.obj_len + " cm", "obj_len"],
  ["Hump jolt",         m => m.jolt == null ? "n/a" : (m.jolt_capped ? "≥" : "") + m.jolt.toFixed(2) + " g", "hump_h"],
  ["Barcode direction", m => m.bc_dir,                                                "bc_dir"],
  ["Sequence",          m => "#" + m.seq,                                             "seq"],
  ["Error / fault",     m => yn(m.flags.error),                                       "error"],
  ["Barcode error",     m => yn(m.flags.bc_err),                                      "bc_err"],
  ["Hump this frame",   m => yn(m.flags.hump),                                        "hump"],
  ["Dropped",           m => yn(m.flags.drop),                                        "drop"],
  ["Wi-Fi loss",        m => yn(m.flags.wifi_loss),                                   "wifi_loss"],
  ["Low battery",       m => yn(m.flags.low_batt) + " (not measured on this car)",     "low_batt"],
];

function renderTel(ev){
  const m = ev.sum, raw = ev.raw;
  pill($("run"), m.run === "RUNNING" ? "Running" : m.run === "STOPPED" ? "Stopped" : "Fault stop",
       m.run === "RUNNING" ? "p-good" : m.run === "STOPPED" ? "p-idle" : "p-crit");
  $("telAt").textContent = "frame #" + m.seq + " at " + ev.t;
  $("seqInfo").textContent = "seq #" + m.seq + " of 0–15";

  $("vSpeed").textContent = m.speed == null ? "n/a" : m.speed;
  $("uSpeed").style.display = m.speed == null ? "none" : "";
  $("sSpeed").textContent = "wheel encoders · 0–" + m.speed_max + " cm/s · last 12 s below";

  $("vHeading").textContent = m.heading == null ? "n/a" : m.heading + "° " + m.compass;
  $("sHeading").textContent = m.heading == null ? "the IMU compass is not calibrated (GP21)"
                                                : "compass heading from the IMU";
  dial(m.heading);

  if (m.dist == null){
    $("vDist").textContent = "clear";
    bar($("mDist"), 100, "var(--s1)");
    sev($("dSev"), "", "no echo");
    $("sDist").textContent = " · nothing within 62 cm, or the sensor is quiet";
  } else {
    $("vDist").textContent = m.dist + " cm";
    const close = m.dist <= STOP_CM, near = m.dist <= 25;
    bar($("mDist"), 100 * m.dist / m.dist_max, close ? "var(--crit)" : near ? "var(--warn)" : "var(--s1)");
    sev($("dSev"), close ? "p-crit" : near ? "p-warn" : "p-good", close ? "stopping" : near ? "close" : "clear");
    $("sDist").textContent = " · the car stops at " + STOP_CM + " cm";
  }

  $("vState").textContent = m.state;
  $("sState").textContent = "the car's own state field, code " + m.state_code;

  $("vLine").textContent = m.line;
  const cross = m.line === "crossing";
  $("laneL").className = (cross || m.line === "drift-left") ? "on" : "";
  $("laneC").className = (cross || m.line === "centred") ? "on" : "";
  $("laneR").className = (cross || m.line === "drift-right") ? "on" : "";

  $("vTilt").textContent = deg(m.tilt);
  $("vRoll").textContent = deg(m.roll);
  dev($("bTilt"), m.tilt);
  dev($("bRoll"), m.roll);

  $("vObj").textContent = m.obj_len == null ? "none yet" : m.obj_len + " cm";
  bar($("mObj"), m.obj_len == null ? 0 : 100 * m.obj_len / m.obj_max, "var(--s1)");
  $("sObj").textContent = m.obj_len == null ? "no servo scan in this run yet"
                                            : "measured by the servo scan, held until the next one";

  $("vJolt").textContent = m.jolt == null ? "n/a" : (m.jolt_capped ? "≥" : "") + m.jolt.toFixed(2) + " g";
  bar($("mJolt"), m.jolt == null ? 0 : 100 * Math.min(1, m.jolt / JOLT_FULL), "var(--s1)");
  $("sJolt").textContent = m.jolt === 0 ? "no hump since the previous frame"
                                        : "peak since the previous frame · 0–2.56 g";

  $("vBc").textContent = m.bc_dir === "-" ? "none" : m.bc_dir;
  $("sBc").textContent = m.bc_dir === "-" ? "no LEFT / RIGHT / FORWARD code read yet"
                                          : "the last direction the car read (a U-turn reads as none)";

  for (const [id, key] of [["fError", "error"], ["fBcErr", "bc_err"], ["fHump", "hump"],
                           ["fDrop", "drop"], ["fWifi", "wifi_loss"], ["fBatt", "low_batt"]]){
    $(id).classList.toggle("on", m.flags[key]);
  }

  $("frameHex").textContent = ev.hex;
  const body = document.createDocumentFragment();
  for (const [label, val, key] of ROWS){
    const tr = document.createElement("tr"), a = document.createElement("td"),
          b = document.createElement("td"), c = document.createElement("td");
    a.textContent = label;
    b.textContent = val(m);
    c.className = "rw";
    c.textContent = key && raw[key] !== undefined ? raw[key] : "";
    tr.append(a, b, c);
    body.append(tr);
  }
  $("fieldRows").replaceChildren(body);
}

function bar(el, pct, colour){
  el.style.width = Math.max(0, Math.min(100, pct)) + "%";
  el.style.background = colour;
}
function dev(el, v){
  if (v == null){ el.style.width = "0"; return; }
  const f = Math.max(-1, Math.min(1, v / 30));
  if (f >= 0){ el.style.left = "50%"; el.style.width = (f * 50) + "%"; }
  else { el.style.left = (50 + f * 50) + "%"; el.style.width = (-f * 50) + "%"; }
}
function dial(d){
  const n = $("dialN");
  if (d == null){ n.style.display = "none"; return; }
  n.style.display = "";
  const a = d * Math.PI / 180;
  n.setAttribute("x2", (50 + 29 * Math.sin(a)).toFixed(1));
  n.setAttribute("y2", (50 - 29 * Math.cos(a)).toFixed(1));
}

/* ---------- sparklines: 2px line, 10% wash, 8px end dot, crosshair on hover ---------- */
const SPARKS = [
  {svg: "spSpeed", hist: "speed", unit: " cm/s", floor: 20},
  {svg: "spDist",  hist: "dist",  unit: " cm",   floor: 30},
];
function spark(cfg){
  const svg = $(cfg.svg), w = Math.max(80, svg.clientWidth), h = svg.clientHeight || 62, pad = 7;
  svg.setAttribute("viewBox", "0 0 " + w + " " + h);
  const t0 = Date.now() - WINDOW_MS;
  const pts = S.hist[cfg.hist].filter(p => p.t >= t0);
  let top = cfg.floor;
  for (const p of pts) if (p.v != null && p.v > top) top = p.v;
  const X = t => pad + (t - t0) / WINDOW_MS * (w - 2 * pad);
  const Y = v => h - pad - (v / top) * (h - 2 * pad - 10);
  let line = "", area = "", seg = [];
  const flush = () => {
    if (seg.length){
      const d = seg.map(p => X(p.t).toFixed(1) + " " + Y(p.v).toFixed(1)).join("L");
      line += "M" + d + " ";
      if (seg.length > 1){
        area += "M" + X(seg[0].t).toFixed(1) + " " + (h - pad) + "L" + d +
                "L" + X(seg[seg.length - 1].t).toFixed(1) + " " + (h - pad) + "Z ";
      }
    }
    seg = [];
  };
  for (const p of pts){ if (p.v == null) flush(); else seg.push(p); }
  flush();
  svg.querySelector(".ln").setAttribute("d", line);
  svg.querySelector(".ar").setAttribute("d", area);
  const bl = svg.querySelector(".bl");
  bl.setAttribute("x1", pad); bl.setAttribute("x2", w - pad);
  bl.setAttribute("y1", h - pad); bl.setAttribute("y2", h - pad);
  svg.querySelector(".mx").textContent = "0–" + top + cfg.unit + ", last 12 s";
  const last = pts.filter(p => p.v != null).pop(), dot = svg.querySelector(".dt");
  if (last){
    dot.setAttribute("cx", X(last.t).toFixed(1));
    dot.setAttribute("cy", Y(last.v).toFixed(1));
    dot.style.display = "";
  } else {
    dot.style.display = "none";
  }
  svg._pts = pts; svg._X = X; svg._cfg = cfg; svg._geom = {w: w, h: h, pad: pad};
}
function showTip(svg, clientX, clientY){
  const pts = (svg._pts || []).filter(p => p.v != null);
  if (!pts.length) return hideTip(svg);
  const r = svg.getBoundingClientRect(), g = svg._geom;
  const x = (clientX - r.left) * (g.w / r.width);
  let best = pts[0], bd = Infinity;
  for (const p of pts){ const d = Math.abs(svg._X(p.t) - x); if (d < bd){ bd = d; best = p; } }
  const cr = svg.querySelector(".cr"), bx = svg._X(best.t).toFixed(1);
  cr.setAttribute("x1", bx); cr.setAttribute("x2", bx);
  cr.setAttribute("y1", g.pad); cr.setAttribute("y2", g.h - g.pad);
  cr.style.display = "";
  const tip = $("tip"), b = document.createElement("b"), s = document.createElement("span");
  b.textContent = best.v + svg._cfg.unit;
  s.textContent = ((Date.now() - best.t) / 1000).toFixed(1) + " s ago";
  tip.replaceChildren(b, s);
  tip.style.display = "block";
  tip.style.left = Math.min(window.innerWidth - tip.offsetWidth - 8, clientX + 12) + "px";
  tip.style.top = Math.max(8, clientY - tip.offsetHeight - 10) + "px";
}
function hideTip(svg){ svg.querySelector(".cr").style.display = "none"; $("tip").style.display = "none"; }
for (const cfg of SPARKS){
  const svg = $(cfg.svg);
  svg.addEventListener("pointermove", e => showTip(svg, e.clientX, e.clientY));
  svg.addEventListener("pointerleave", () => hideTip(svg));
  svg.addEventListener("focus", () => {
    const r = svg.getBoundingClientRect();
    showTip(svg, r.right - 10, r.top + r.height / 2);
  });
  svg.addEventListener("blur", () => hideTip(svg));
}

/* ---------- raw log ---------- */
const TAG = {tel: "TEL", cmd: "CMD", bc: "BC", obs: "OBS", st: "CAR", hump: "HUMP"};
function row(ev){
  const text = ev.t + "  " + (TAG[ev.kind] || "BAD").padEnd(4) + " " + ev.hex + "  " + ev.text;
  S.lines.push(text);
  if (S.lines.length > 5000) S.lines.shift();
  const box = {tel: "fTel", cmd: "fCmd", bc: "fBc", obs: "fObs", hump: "fHump"}[ev.kind];
  if (box && !$(box).checked) return;
  const r = document.createElement("div");
  r.className = "row " + ev.kind;
  const t = document.createElement("span"), g = document.createElement("span"),
        key = document.createElement("span"), tag = document.createElement("span"),
        m = document.createElement("span"), h = document.createElement("span");
  t.className = "t"; t.textContent = ev.t;
  key.className = "key";
  tag.textContent = TAG[ev.kind] || "BAD";
  g.className = "g"; g.append(key, tag);
  h.className = "hex"; h.textContent = ev.hex + "  ";
  m.className = "m"; m.append(h, document.createTextNode(ev.text));
  r.append(t, g, m);
  const log = $("log");
  log.append(r);
  while (log.children.length > 500) log.removeChild(log.firstChild);
  if ($("fScroll").checked) log.scrollTop = log.scrollHeight;
}

/* ---------- events in ---------- */
function handle(ev, live, ageMs, quiet){
  const at = Date.now() - ageMs;
  if (ev.kind === "tel"){
    S.lastTel = at;
    S.everTel = true;
    for (const [name, v] of [["speed", ev.sum.speed], ["dist", ev.sum.dist]]){
      const hist = S.hist[name];
      hist.push({t: at, v: v});
      if (hist.length > HIST_MAX) hist.shift();
    }
    if (live) S.rates.push(Date.now());
  }
  if (!quiet){
    stats(ev.stats);
    if (ev.kind === "tel") renderTel(ev);
  }
  if (!S.paused || !live) row(ev);
}

function health(){
  const now = Date.now();
  S.rates = S.rates.filter(t => now - t < 5000);
  $("rate").textContent = (S.rates.length / 5).toFixed(1) + " / s (last 5 s)";
  const age = S.lastTel ? (now - S.lastTel) / 1000 : null;
  $("age").textContent = age == null ? "–" : age.toFixed(1) + " s ago";
  const stale = age == null || age * 1000 > STALE_MS;
  $("cond").classList.toggle("stale", stale);
  if (stale){
    pill($("run"), "No telemetry", "p-warn");
    $("telHint").textContent = !S.everTel
      ? (S.car === "online"
          ? "the car is connected but has sent no telemetry frame — check S1_PUBLISH_TELEMETRY in app_program/subsystem1/subsystem1.h"
          : "waiting for the first frame on robocar/car1/telemetry")
      : "frames stopped " + age.toFixed(0) + " s ago — the values below are the last ones that arrived";
  } else {
    $("telHint").textContent = "";
  }
  for (const cfg of SPARKS) spark(cfg);
}

const es = new EventSource("/events");
es.addEventListener("backlog", e => {
  const b = JSON.parse(e.data);
  b.events.forEach(ev => handle(ev, false, (b.now - ev.ts) * 1000, true));
  const last = b.events[b.events.length - 1];
  if (last) stats(last.stats);
  for (let i = b.events.length - 1; i >= 0; i--){
    if (b.events[i].kind === "tel"){ renderTel(b.events[i]); break; }
  }
  health();
});
es.onmessage = e => handle(JSON.parse(e.data), true, 0, false);
es.onopen = () => pill($("link"), "live", "p-good");
es.onerror = () => pill($("link"), "server stopped — reconnecting", "p-crit");

setInterval(health, 400);
health();

document.querySelectorAll("button[data-cmd]").forEach(b => b.onclick = async () => {
  const name = b.dataset.cmd.toUpperCase();
  $("sent").textContent = "sending " + name + "...";
  try {
    const r = await (await fetch("/send/" + b.dataset.cmd, {method: "POST", headers: {"X-Robocar": "1"}})).json();
    $("sent").textContent = r.ok ? "sent " + name + " at " + new Date().toTimeString().slice(0, 8)
                                 : "FAILED to send " + name + ": " + (r.err || "mosquitto_pub error");
  } catch (e) { $("sent").textContent = "FAILED to send " + name + ": page cannot reach its server"; }
});
$("pause").onclick = () => { S.paused = !S.paused; $("pause").textContent = S.paused ? "Resume" : "Pause"; };
$("clear").onclick = () => { $("log").textContent = ""; S.lines = []; };
$("save").onclick = () => {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([S.lines.join("\n") + "\n"], {type: "text/plain"}));
  a.download = "robocar_log_" + new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-") + ".txt";
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
};
window.addEventListener("resize", health);
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
