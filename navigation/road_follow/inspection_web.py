"""局域网侧视调试：共享帧 MJPEG、原图外层 ROI、持久化识别参数。"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from urllib.parse import urlparse

import cv2

from road_follow.inspection_io import crop


PAGE = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>涵洞侧视识别调试</title>
<style>body{background:#14202a;color:#edf5fa;font:16px system-ui;margin:0}main{max-width:1200px;margin:auto;padding:20px}.panel{background:#20313f;padding:16px;margin:12px 0;border-radius:8px}#view{position:relative;line-height:0;max-width:100%}#live{display:block;width:100%}#overlay{position:absolute;inset:0;width:100%;height:100%;touch-action:none}input{width:85px;padding:7px;margin:4px}label{display:inline-block;margin-right:12px}button{padding:10px 18px;margin:8px;font:inherit}#crop{max-width:100%;max-height:300px}pre{white-space:pre-wrap;overflow-wrap:anywhere}#message{color:#ffc26f}</style>
<main><h1>涵洞两侧识别</h1><p>在完整原图上拖拽外层 ROI，排除底部板卡。此裁剪同时用于人脸与刀具；刀具内部取景与前景提取保留。相似度不是成功概率。</p>
<div class="panel"><div id="view"><img id="live" src="/camera.mjpg" alt="侧视原图"><canvas id="overlay"></canvas></div><p id="camera">等待画面</p><p id="message"></p></div>
<div class="panel"><label>人脸相似度<input id="face_threshold" type="number" min="0.45" max="1" step="0.01"></label><label>刀具相似度<input id="knife_threshold" type="number" min="0" max="1" step="0.01"></label><label>连续新帧<input id="confirm_frames" type="number" min="1" max="100" step="1"></label><label>每侧超时秒<input id="side_timeout_s" type="number" min="1" max="600" step="1"></label><p>人脸服务身份门限为 0.45；刀具分差只显示。修改后重新累计，当前侧期限不重置；新超时值从下一侧生效。</p>
<label>X1<input id="x1" type="number" min="0" max="1" step="0.01"></label><label>Y1<input id="y1" type="number" min="0" max="1" step="0.01"></label><label>X2<input id="x2" type="number" min="0" max="1" step="0.01"></label><label>Y2<input id="y2" type="number" min="0" max="1" step="0.01"></label><p>ROI 坐标为原图的 0～1 比例，两个方向共用。</p><button id="save">应用并保存 JSON</button><button id="full">选整帧</button><p>当前已应用的裁剪预览：</p><img id="crop" alt="外层裁剪预览"></div>
<div class="panel"><h2>实时状态与地图记录</h2><p id="summary"></p><pre id="status"></pre><pre id="map"></pre></div></main>
<script>const $=x=>document.getElementById(x);const canvas=$('overlay'),ctx=canvas.getContext('2d');let initialized=false,start=null,candidate=null,revision=0;
function roi(){return ['x1','y1','x2','y2'].map(x=>Number($(x).value));}
function draw(status){const w=canvas.width=$('live').clientWidth,h=canvas.height=$('live').clientHeight;ctx.clearRect(0,0,w,h);const r=roi();ctx.strokeStyle='#ffc857';ctx.lineWidth=3;ctx.strokeRect(r[0]*w,r[1]*h,(r[2]-r[0])*w,(r[3]-r[1])*h);if(candidate?.bbox&&status?.camera?.image_size){const [iw,ih]=status.camera.image_size,b=candidate.bbox;ctx.strokeStyle='#73e0b0';ctx.strokeRect(b[0]/iw*w,b[1]/ih*h,(b[2]-b[0])/iw*w,(b[3]-b[1])/ih*h);}}
function point(e){const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))];}
canvas.onpointerdown=e=>{start=point(e);canvas.setPointerCapture(e.pointerId);};canvas.onpointermove=e=>{if(!start)return;const p=point(e);['x1','y1','x2','y2'].forEach((id,i)=>$(id).value=[Math.min(start[0],p[0]),Math.min(start[1],p[1]),Math.max(start[0],p[0]),Math.max(start[1],p[1])][i].toFixed(4));draw();};canvas.onpointerup=()=>start=null;canvas.onpointercancel=()=>start=null;
['x1','y1','x2','y2'].forEach(x=>$(x).oninput=()=>draw());$('full').onclick=()=>{['x1','y1','x2','y2'].forEach((x,i)=>$(x).value=[0,0,1,1][i]);draw();};
$('save').onclick=async()=>{try{const values={roi:roi()};['face_threshold','knife_threshold','confirm_frames','side_timeout_s'].forEach(k=>values[k]=Number($(k).value));const r=await fetch('/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({revision,values})});const d=await r.json();if(!r.ok)throw Error(d.error);revision=d.revision;$('message').textContent='已应用并保存；当前侧重新累计。';}catch(e){$('message').textContent=e.message;}};
async function poll(){try{const r=await fetch('/status'),s=await r.json();if(!initialized){['face_threshold','knife_threshold','confirm_frames','side_timeout_s'].forEach(k=>$(k).value=s.settings[k]);['x1','y1','x2','y2'].forEach((x,i)=>$(x).value=s.settings.roi[i]);revision=s.revision;initialized=true;}candidate=s.candidate;draw(s);$('camera').textContent=s.camera.error||(s.camera.frame_age_ms>1000?'侧视画面已过期，不能参与识别':'方向 '+s.camera.side+' · '+(s.camera.image_size||[]).join(' × ')+' · '+s.camera.frame_age_ms+' ms');$('summary').textContent='阶段 '+s.phase+' · 当前侧 '+s.side+' · 连续 '+s.valid_frames+' 帧 · 剩余 '+(s.remaining_s===null?'—':s.remaining_s.toFixed(1)+' 秒');$('status').textContent=JSON.stringify({candidate:s.candidate,reason:s.reason,failure:s.failure_reason,sides:s.sides,speech_enabled:s.speech_enabled},null,2);$('map').textContent=JSON.stringify(s.map||{},null,2);if(JSON.stringify(s.settings.roi)==='[0,0,1,1]'&&!$('message').textContent)$('message').textContent='当前 ROI 为整帧，尚未排除底部遮挡。';$('crop').src='/crop.jpg?t='+Date.now();}catch(e){$('camera').textContent='调试网页连接中断：'+e.message;}}
setInterval(poll,1000);poll();window.onresize=()=>draw();</script></html>'''


class InspectionWeb:
    def __init__(self, settings, hub, status, map_status=None, address=None):
        self.settings, self.hub, self.status_source = settings, hub, status
        self.map_status = map_status or (lambda: {})
        config, _ = settings.snapshot()
        self.condition = threading.Condition()
        self.stop = threading.Event()
        self.sequence, self.jpeg = 0, b""
        self.server = ThreadingHTTPServer(address or (config["web"]["host"], config["web"]["port"]), self._handler())
        self.server.daemon_threads = True
        self.server.timeout = .2
        self.serve_thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": .1}, daemon=True)
        self.encode_thread = threading.Thread(target=self._encode, args=(config["web"]["preview_fps"],), daemon=True)
        self.serve_thread.start()
        self.encode_thread.start()

    def _encode(self, fps):
        last = -1
        while not self.stop.is_set():
            frame = self.hub.latest()
            if frame is not None and frame.sequence != last:
                ok, encoded = cv2.imencode(".jpg", frame.image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self.condition:
                        self.sequence += 1
                        self.jpeg = encoded.tobytes()
                        self.condition.notify_all()
                    last = frame.sequence
            self.stop.wait(1/fps)

    def _handler(self):
        session = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def setup(self):
                super().setup()
                self.connection.settimeout(2)

            def send(self, code, body, content_type="application/json; charset=utf-8"):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def reply(self, code, value):
                self.send(code, json.dumps(value, ensure_ascii=False, allow_nan=False).encode())

            def do_GET(self):
                path = urlparse(self.path).path
                try:
                    if path == "/":
                        self.send(200, PAGE.encode(), "text/html; charset=utf-8")
                    elif path == "/status":
                        config, revision = session.settings.snapshot()
                        self.reply(200, {**session.status_source(), "camera": session.hub.info(),
                                         "settings": config["recognition"], "revision": revision, "map": session.map_status()})
                    elif path == "/crop.jpg":
                        frame = session.hub.latest()
                        if frame is None:
                            self.reply(503, {"error": "没有新鲜侧视画面"})
                            return
                        config, _ = session.settings.snapshot()
                        image, _ = crop(frame.image, config["recognition"]["roi"])
                        ok, encoded = cv2.imencode(".jpg", image)
                        if not ok:
                            raise ValueError("预览编码失败")
                        self.send(200, encoded.tobytes(), "image/jpeg")
                    elif path == "/camera.mjpg":
                        self.send_response(200)
                        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                        self.send_header("Cache-Control", "no-store")
                        self.end_headers()
                        sequence = -1
                        while not session.stop.is_set():
                            with session.condition:
                                session.condition.wait_for(lambda: session.stop.is_set() or session.sequence != sequence, timeout=1)
                                if sequence == session.sequence or not session.jpeg:
                                    continue
                                sequence, encoded = session.sequence, session.jpeg
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+str(len(encoded)).encode()+b"\r\n\r\n"+encoded+b"\r\n")
                            self.wfile.flush()
                    else:
                        self.reply(404, {"error": "页面不存在"})
                except (BrokenPipeError, ConnectionError, TimeoutError):
                    pass
                except (ValueError, OSError) as exc:
                    self.reply(400, {"error": str(exc)})

            def do_POST(self):
                if urlparse(self.path).path != "/settings":
                    self.reply(404, {"error": "接口不存在"})
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 8192:
                        raise ValueError("配置请求长度不合法")
                    payload = json.loads(self.rfile.read(size))
                    # 两个调试页面不能静默覆盖彼此已经保存的参数。
                    with session.settings._lock:
                        _, current = session.settings.snapshot()
                        if payload.get("revision") != current:
                            self.reply(409, {"error": "参数已由另一页面修改，请刷新后重试"})
                            return
                        values, revision = session.settings.update(payload["values"])
                    self.reply(200, {"settings": values, "revision": revision})
                except (KeyError, TypeError, ValueError, OSError) as exc:
                    self.reply(400, {"error": str(exc)})
        return Handler

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        self.server.shutdown()
        self.server.server_close()
        self.serve_thread.join(timeout=1)
        self.encode_thread.join(timeout=1)
