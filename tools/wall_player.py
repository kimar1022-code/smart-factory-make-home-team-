#!/usr/bin/env python3
"""벽 꽂기 독립 플레이어 (Claude 세션 불필요) — 9/3.
브라우저로 http://<PC>:8770/ 열고 색 버튼을 누르면 cycle_front_first 를 서버에서 돌린다.
색마다 앵커 코드 자동 전환(파랑=dots, 나머지=마커)은 run_sequence.sh 와 동일 규칙.
전제: 카메라(:8766/:8768)·브리지(:8765) 스택이 이미 떠 있어야 한다(start_cam.sh + start_console.sh + fr5_rescue.sh).

  python3 wall_player.py           # http://0.0.0.0:8770

안전:
 - 한 번에 하나만 실행(이미 돌면 409).
 - STOP = 실행 중 사이클 종료 + 브리지 stop(로봇 정지). 벽 회수는 사람이(반납 철칙).
 - 실패하면 그 색에서 멈추고 로그에 표시. 자동 반납/재시도 없음.
"""
import json, os, signal, subprocess, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

TOOLS = "/home/ar/bf2_console/tools"
LOGDIR = "/home/ar/bf2_console/logs"
DOTS = "place_front_first.py.0903_2130_dotsbak"
MARK = "place_front_first.py.0903_2110_0902markerbak"
BRIDGE = "http://127.0.0.1:8765"
COLORS = ["blue", "yellow", "red", "red_s"]
KOR = {"blue": "파랑", "yellow": "노랑", "red": "빨강(긴)", "red_s": "빨강(짧)", "green": "초록"}

JOB = {"proc": None, "seq": [], "idx": 0, "color": None, "log": None, "state": "idle", "results": [], "thread": None}
LOCK = threading.Lock()


def _swap_code(ck):
    src = DOTS if ck == "blue" else MARK
    subprocess.run(["cp", f"{TOOLS}/{src}", f"{TOOLS}/place_front_first.py"], check=True)


def _run_seq(colors):
    JOB["results"] = []
    for i, ck in enumerate(colors):
        with LOCK:
            if JOB["state"] == "stopping":
                break
            JOB["idx"] = i; JOB["color"] = ck
        _swap_code(ck)
        log = f"{LOGDIR}/player_{ck}.log"
        JOB["log"] = log
        with open(log, "w") as f:
            p = subprocess.Popen(["python3", "-u", "cycle_front_first.py", "1", ck],
                                 cwd=TOOLS, stdout=f, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            JOB["proc"] = p
            p.wait()
        try:
            txt = open(log).read()
        except Exception:
            txt = ""
        ok = "결과 1/1 성공" in txt
        JOB["results"].append({"color": ck, "ok": ok})
        if not ok:
            with LOCK:
                JOB["state"] = "failed"
            return
    with LOCK:
        JOB["state"] = "done" if JOB["state"] != "stopping" else "stopped"


def start(colors):
    with LOCK:
        if JOB["state"] == "running":
            return False, "이미 실행 중"
        JOB.update(proc=None, seq=colors, idx=0, color=colors[0] if colors else None, state="running", results=[])
    t = threading.Thread(target=_run_seq, args=(colors,), daemon=True)
    JOB["thread"] = t; t.start()
    return True, "시작"


def stop():
    with LOCK:
        JOB["state"] = "stopping"
        p = JOB["proc"]
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    try:
        urllib.request.urlopen(urllib.request.Request(BRIDGE + "/fr5/stop", data=b"{}",
                               headers={"Content-Type": "application/json"}), timeout=5).read()
    except Exception:
        pass
    with LOCK:
        JOB["state"] = "stopped"


def bridge_state():
    try:
        f = json.loads(urllib.request.urlopen(BRIDGE + "/status", timeout=4).read())["robots"]["fr5"]
        return {"connected": f.get("connected"), "busy": f.get("busy"), "grip": f.get("gripper"),
                "z": round(f["tcp"][2], 1) if f.get("tcp") else None, "frozen": f.get("frozen")}
    except Exception as e:
        return {"error": str(e)}


PAGE = """<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>벽 꽂기 플레이어</title><style>
body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
.wrap{max-width:760px;margin:0 auto;padding:16px}
h1{font-size:18px}
button{font-size:16px;padding:12px 16px;margin:4px;border:0;border-radius:8px;color:#fff;cursor:pointer}
.c{background:#2563eb}.all{background:#16a34a}.stop{background:#dc2626}
button:disabled{opacity:.4;cursor:default}
#log{white-space:pre-wrap;background:#000;border:1px solid #333;border-radius:8px;padding:10px;height:44vh;overflow:auto;font-size:12px;line-height:1.35}
.row{display:flex;flex-wrap:wrap;align-items:center}
#st{font-size:13px;color:#9ca3af;margin:8px 0}
.badge{display:inline-block;padding:2px 8px;border-radius:6px;margin-left:6px;font-size:12px}
</style></head><body><div class=wrap>
<h1>벽 꽂기 플레이어 <span id=run class=badge></span></h1>
<div class=row id=btns></div>
<div class=row><button class="c all" onclick="run(['blue','yellow','red','red_s'])">전체 4벽</button>
<button class=stop onclick="stopit()">■ STOP</button></div>
<div id=st></div>
<div id=log></div>
</div><script>
const COLORS=[["blue","파랑"],["yellow","노랑"],["red","빨강(긴)"],["red_s","빨강(짧)"]];
const b=document.getElementById('btns');
COLORS.forEach(([k,n])=>{let x=document.createElement('button');x.className='c';x.textContent=n;x.onclick=()=>run([k]);b.appendChild(x);});
function run(cs){fetch('/run?seq='+cs.join(','),{method:'POST'}).then(r=>r.json()).then(j=>{});}
function stopit(){fetch('/stop',{method:'POST'});}
async function tick(){
 try{let s=await (await fetch('/state')).json();
  let r=document.getElementById('run'); r.textContent=s.state; r.style.background={running:'#2563eb',done:'#16a34a',failed:'#dc2626',stopped:'#6b7280',idle:'#374151'}[s.state]||'#374151';
  let bs=s.bridge||{}; document.getElementById('st').textContent=
   `로봇: ${bs.connected?'연결':'끊김'} · busy ${bs.busy} · z ${bs.z} · grip ${bs.grip}` + (bs.frozen?' · ⚠동결':'') +
   (s.results&&s.results.length? '   |  '+s.results.map(x=>x.color+(x.ok?' ✅':' ❌')).join('  '):'');
  document.querySelectorAll('#btns button,.all').forEach(x=>x.disabled=(s.state==='running'));
  let lg=await (await fetch('/log')).text(); let el=document.getElementById('log');
  let atBottom=el.scrollHeight-el.scrollTop-el.clientHeight<40; el.textContent=lg; if(atBottom)el.scrollTop=el.scrollHeight;
 }catch(e){}
 setTimeout(tick,1000);
}
tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="text/plain; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

    def log_message(self, *a):
        pass

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/" or path == "/index.html":
            return self._send(200, PAGE, "text/html; charset=utf-8")
        if path == "/state":
            return self._send(200, json.dumps({"state": JOB["state"], "color": JOB["color"], "idx": JOB["idx"],
                                               "seq": JOB["seq"], "results": JOB["results"], "bridge": bridge_state()}),
                              "application/json")
        if path == "/log":
            try:
                txt = open(JOB["log"]).read() if JOB["log"] else "(로그 없음)"
            except Exception:
                txt = "(로그 없음)"
            return self._send(200, txt)
        return self._send(404, "not found")

    def do_POST(self):
        path = urlparse(self.path).path
        q = parse_qs(urlparse(self.path).query)
        if path == "/run":
            cs = [c for c in (q.get("seq", [""])[0].split(",")) if c in COLORS]
            if not cs:
                return self._send(400, json.dumps({"error": "색 없음"}), "application/json")
            ok, msg = start(cs)
            return self._send(200 if ok else 409, json.dumps({"ok": ok, "msg": msg}), "application/json")
        if path == "/stop":
            stop()
            return self._send(200, json.dumps({"ok": True}), "application/json")
        return self._send(404, "not found")


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", 8770), H).serve_forever()
