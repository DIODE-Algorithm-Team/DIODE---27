#!/usr/bin/env python3
"""DIODE-27 自瞄遥测看板（无 ROS、纯标准库）

把 tools::Plotter 通过 UDP 发出的 JSON 收下来，在浏览器里画实时曲线。

用法：
    python3 tools/debug/dashboard.py                 # 默认 UDP 9870 / HTTP 9871
    python3 tools/debug/dashboard.py --csv run.csv   # 同时落盘原始数据
    # 浏览器打开 http://<机器人IP>:9871/
    # 自瞄侧：./build/mt_auto_aim_debug configs/sentry.yaml

数据来源：任意调用 tools::Plotter::plot(json) 的程序（standard/uav/sentry/mt_*_debug/各 test）。
"""

import argparse
import json
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

MAX_SAMPLES = 4000

_samples = deque(maxlen=MAX_SAMPLES)
_lock = threading.Lock()
_seq = 0
_csv_path = None
_csv_file = None


def udp_loop(port: int):
    global _seq
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    print(f"[dashboard] UDP listening on 0.0.0.0:{port}")
    while True:
        try:
            raw, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError as exc:
            print(f"[dashboard] udp error: {exc}")
            break
        try:
            obj = json.loads(raw.decode("utf-8", "ignore"))
        except Exception:
            continue
        if not isinstance(obj, dict):
            continue
        obj["_t"] = time.time()
        with _lock:
            _seq += 1
            obj["_seq"] = _seq
            _samples.append(obj)
            if _csv_file is not None:
                _csv_file.write(json.dumps(obj, ensure_ascii=False) + "\n")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/data":
            qs = parse_qs(parsed.query)
            since = int(qs.get("since", ["0"])[0])
            with _lock:
                data = [s for s in _samples if s.get("_seq", 0) > since]
            body = json.dumps({"seq": _seq, "samples": data}, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)


PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>DIODE-27 自瞄遥测</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#0e1116; color:#d7dde5; font:13px/1.4 -apple-system,"Segoe UI",Roboto,"Noto Sans CJK SC",sans-serif; }
  header { padding:10px 14px; background:#151a21; border-bottom:1px solid #232a33; display:flex; gap:14px; align-items:center; flex-wrap:wrap; }
  header h1 { font-size:15px; margin:0; font-weight:600; color:#e8eef6; }
  .pill { padding:2px 8px; border-radius:10px; background:#1d2530; border:1px solid #2b3542; }
  .ok { color:#5ddb8a; } .warn { color:#f0c24b; } .bad { color:#ff6b6b; }
  main { display:grid; grid-template-columns: 1fr 300px; gap:12px; padding:12px; }
  #charts { display:flex; flex-direction:column; gap:12px; }
  .chart { background:#121720; border:1px solid #232a33; border-radius:8px; padding:8px; }
  .chart h2 { font-size:12px; margin:0 0 6px; font-weight:600; color:#9fb0c4; }
  canvas { width:100%; height:150px; display:block; }
  aside { background:#121720; border:1px solid #232a33; border-radius:8px; padding:10px; height:fit-content; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  td { padding:2px 4px; border-bottom:1px solid #1d2530; }
  td:last-child { text-align:right; font-variant-numeric:tabular-nums; }
  .keys label { display:inline-block; margin:2px 8px 2px 0; font-size:12px; color:#9fb0c4; }
  input[type=number] { width:64px; background:#0e1116; color:#d7dde5; border:1px solid #2b3542; border-radius:4px; }
</style>
</head>
<body>
<header>
  <h1>DIODE-27 自瞄遥测</h1>
  <span class="pill">UDP 9870 → HTTP 9871</span>
  <span class="pill" id="stat">等待数据…</span>
  <label class="pill">窗口 <input type="number" id="win" value="10" min="2" max="60"> s</label>
  <label class="pill"><input type="checkbox" id="pause"> 暂停</label>
  <span class="pill" id="fps">-</span>
</header>
<main>
  <div id="charts"></div>
  <aside>
    <div style="margin-bottom:8px;color:#9fb0c4">最新值</div>
    <table id="latest"></table>
  </aside>
</main>
<script>
const GROUPS = [
  { title: "NIS（4 自由度：0.711 ~ 9.488 为 90% 区间）", keys: ["nis", "nis_prefit"] },
  { title: "innovation（pre-fit 残差）", keys: ["innovation_yaw", "innovation_pitch", "innovation_distance", "innovation_angle"] },
  { title: "跟踪状态 / 帧率", keys: ["w", "fps", "armor_num", "recent_nis_failures"] },
  { title: "瞄准指令（度）", keys: ["cmd_yaw", "cmd_pitch", "gimbal_yaw", "gimbal_pitch"] },
];
const COLORS = ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8", "#4dd0e1", "#fff176"];
let since = 0, buffer = [], paused = false, charts = [];

function buildCharts() {
  const host = document.getElementById("charts");
  host.innerHTML = "";
  charts = GROUPS.map(g => {
    const div = document.createElement("div");
    div.className = "chart";
    div.innerHTML = `<h2>${g.title}</h2><canvas></canvas>`;
    host.appendChild(div);
    return { def: g, canvas: div.querySelector("canvas"), ctx: div.querySelector("canvas").getContext("2d") };
  });
}

function resize() {
  for (const c of charts) {
    const r = c.canvas.getBoundingClientRect();
    c.canvas.width = Math.max(320, Math.floor(r.width * devicePixelRatio));
    c.canvas.height = Math.floor(150 * devicePixelRatio);
  }
}

function draw() {
  const win = Math.max(2, +document.getElementById("win").value || 10);
  const now = buffer.length ? buffer[buffer.length - 1]._t : 0;
  const t0 = now - win;
  const visible = buffer.filter(s => s._t >= t0);
  for (const c of charts) {
    const ctx = c.ctx, W = c.canvas.width, H = c.canvas.height;
    ctx.clearRect(0, 0, W, H);
    const keys = c.def.keys.filter(k => visible.some(s => typeof s[k] === "number"));
    let min = 0, max = 0, has = false;
    for (const s of visible) for (const k of keys) {
      const v = s[k]; if (typeof v !== "number" || !isFinite(v)) continue;
      if (!has) { min = max = v; has = true; } else { min = Math.min(min, v); max = Math.max(max, v); }
    }
    if (!has) { ctx.fillStyle = "#41505f"; ctx.font = `${13*devicePixelRatio}px sans-serif`; ctx.fillText("无数据", 8, 20); continue; }
    if (max - min < 1e-9) { max = min + 1; }
    const pad = (max - min) * 0.1; min -= pad; max += pad;
    const X = t => (t - t0) / Math.max(1e-6, win) * W;
    const Y = v => H - (v - min) / (max - min) * H;
    // grid
    ctx.strokeStyle = "#1d2530"; ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = H * i / 4; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }
    // y labels
    ctx.fillStyle = "#5d6b7a"; ctx.font = `${10*devicePixelRatio}px monospace`;
    ctx.fillText(max.toFixed(2), 4, 12*devicePixelRatio);
    ctx.fillText(min.toFixed(2), 4, H - 4*devicePixelRatio);
    // series
    keys.forEach((k, idx) => {
      ctx.strokeStyle = COLORS[idx % COLORS.length]; ctx.lineWidth = 1.5*devicePixelRatio;
      ctx.beginPath(); let started = false;
      for (const s of visible) {
        const v = s[k]; if (typeof v !== "number" || !isFinite(v)) { started = false; continue; }
        const px = X(s._t), py = Y(v);
        if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
      }
      ctx.stroke();
      // legend
      ctx.fillStyle = COLORS[idx % COLORS.length];
      ctx.fillText(k, 34*devicePixelRatio + idx*90*devicePixelRatio, 12*devicePixelRatio);
    });
  }
}

function updateLatest() {
  const s = buffer.length ? buffer[buffer.length - 1] : null;
  const tbl = document.getElementById("latest");
  if (!s) { tbl.innerHTML = ""; return; }
  const keys = Object.keys(s).filter(k => !k.startsWith("_")).sort();
  tbl.innerHTML = keys.map(k => {
    const v = s[k];
    const txt = typeof v === "number" ? (Math.abs(v) >= 1000 ? v.toFixed(0) : v.toFixed(3)) : String(v);
    return `<tr><td>${k}</td><td>${txt}</td></tr>`;
  }).join("");
}

async function poll() {
  try {
    const r = await fetch(`/data?since=${since}`);
    const j = await r.json();
    since = j.seq;
    if (j.samples.length) {
      if (!paused) { buffer = buffer.concat(j.samples); if (buffer.length > 4000) buffer = buffer.slice(-4000); }
      const st = document.getElementById("stat");
      st.textContent = `样本 ${buffer.length}`; st.className = "pill ok";
      updateLatest();
    } else {
      const st = document.getElementById("stat");
      if (st.textContent.includes("等待")) st.textContent = "已连接，等待自瞄进程发数据…";
    }
    const fpsEl = document.getElementById("fps");
    const s = buffer[buffer.length - 1];
    if (s && typeof s["fps"] === "number") fpsEl.textContent = `fps ${s["fps"].toFixed(1)}`;
  } catch (e) {
    document.getElementById("stat").textContent = "连接断开";
    document.getElementById("stat").className = "pill bad";
  }
}

buildCharts(); resize();
addEventListener("resize", () => { resize(); draw(); });
document.getElementById("pause").onchange = e => paused = e.target.checked;
poll().then(() => setInterval(poll, 200));
setInterval(() => { draw(); }, 200);
</script>
</body>
</html>
"""


def main():
    global _csv_path, _csv_file
    ap = argparse.ArgumentParser()
    ap.add_argument("--udp-port", type=int, default=9870)
    ap.add_argument("--http-port", type=int, default=9871)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--csv", default=None, help="把原始 JSON 逐行落盘，便于离线分析")
    args = ap.parse_args()

    if args.csv:
        _csv_path = args.csv
        _csv_file = open(_csv_path, "w", encoding="utf-8")

    threading.Thread(target=udp_loop, args=(args.udp_port,), daemon=True).start()
    httpd = ThreadingHTTPServer((args.host, args.http_port), Handler)
    print(f"[dashboard] 打开 http://127.0.0.1:{args.http_port}/  （远程用机器人 IP）")
    if _csv_path:
        print(f"[dashboard] 原始数据 -> {_csv_path}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        if _csv_file:
            _csv_file.close()


if __name__ == "__main__":
    main()
