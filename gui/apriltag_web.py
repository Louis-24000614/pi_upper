"""局域网 AprilTag 四点标定：单路相机、冻结帧、独立保存，不应用导航配置。"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.parse import urlparse
import uuid

import cv2
import numpy as np

from apriltag_calibration import PREVIEW_BEV, ROOT, bev_size, detect_tag, save_calibration, solve_calibration


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AprilTag 四点标定</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101820;color:#edf3f7;font:16px system-ui,sans-serif}
main{max-width:1200px;margin:auto;padding:22px}h1{font-size:25px;margin:0 0 10px}
p{line-height:1.6;color:#b9ccd7}button,input{font:inherit}button{padding:10px 16px;margin:5px 8px 5px 0;border:0;border-radius:6px;cursor:pointer;background:#d2e4ed;color:#102430}
button.primary{background:#7ce1b7}button:disabled{opacity:.4;cursor:default}.panel{background:#1a2833;padding:16px;border-radius:10px;margin:15px 0}
.views{display:grid;grid-template-columns:3fr 1fr;gap:16px}.views img{width:100%;background:#05090c;object-fit:contain;border-radius:6px}
#live{max-height:70vh}#bev{image-rendering:pixelated}input{width:100px;padding:7px;border:1px solid #647a87;border-radius:5px;background:#101820;color:#fff}
input[type=checkbox]{width:auto}label{display:inline-block;margin:8px 12px 8px 0}pre{white-space:pre-wrap;overflow-wrap:anywhere;color:#b8d2e0;font-size:13px;max-height:300px;overflow:auto}
#status{min-height:26px;color:#7ce1b7}#error{color:#ffb9a9}@media(max-width:760px){.views{grid-template-columns:1fr}main{padding:12px}}
</style><main>
<h1>AprilTag 地面四点标定</h1>
<p>使用 tag36h11 · ID 0。<b>黑色外沿边长 13.4 cm，不含外侧白边</b>；标签平整贴在道路地面，四角完整可见，相机与车保持固定。</p>
<div class="panel"><div id="camera">正在连接相机…</div><img id="live" src="/camera.mjpg" alt="实时导航相机画面">
<button class="primary" id="capture">冻结当前帧并检测四角</button><button id="save" disabled>保存本次标定</button><button id="stop">停止推流并释放相机</button>
<div id="status">实时画面中的 0～3 是标签四角；冻结检测后才可保存。</div><div id="error"></div></div>
<div class="panel" id="frozen" hidden><div class="views"><div><p>冻结原图与四角</p><img id="source" alt="冻结四角画面"></div><div><p>标签坐标俯视预览</p><img id="bev" alt="标签坐标俯视预览"></div></div>
<p>原点是标签中心，X 向标签右、Y 向标签上。四点拟合不代表远处测距精度已通过验证。</p>
<details><summary>可选：记录标签相对于导航相机的位置</summary><p>先测量相机光心的地面垂足。仅当标签的 0→1 边在远端、X/Y 与车右/车前方向对齐时填写；不清楚可以留空，保存标签坐标结果。</p>
<label>中心横向 X（右为正，cm）<input id="x" type="number" step="0.1"></label>
<label>中心前向 Y（cm）<input id="y" type="number" step="0.1" min="0.1"></label>
<label><input id="aligned" type="checkbox">已确认标签方向与车辆对齐</label></details><pre id="record"></pre></div>
<p>结果独立保存在 data/calibration/apriltag/。导航与涵洞继续使用当前配置；此页面没有应用标定的操作。</p>
</main><script>
const $=id=>document.getElementById(id);let draft=null,busy=false,saved=false;
async function post(path,body={}){let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});let d=await r.json();if(!r.ok)throw Error(d.message||r.statusText);return d;}
function controls(){ $('capture').disabled=busy;$('save').disabled=busy||!draft||saved; }
$('capture').onclick=async()=>{busy=true;draft=null;saved=false;$('frozen').hidden=true;$('error').textContent='';$('status').textContent='正在冻结并检测…';controls();try{let d=await post('/capture');draft=d.draft_id;$('source').src='/frozen.jpg?draft='+draft;$('bev').src='/bev.jpg?draft='+draft;$('record').textContent=JSON.stringify(d.record,null,2);$('frozen').hidden=false;$('status').textContent='已冻结四角。确认标签贴地及角点位置后保存。';}catch(e){$('error').textContent=e.message;$('status').textContent='本次检测失败，不能保存旧结果。';}finally{busy=false;controls();}};
$('save').onclick=async()=>{busy=true;controls();$('error').textContent='';try{const x=$('x').value,y=$('y').value;let d=await post('/save',{draft_id:draft,tag_center_x_cm:x===''?null:Number(x),tag_center_y_cm:y===''?null:Number(y),axes_aligned:$('aligned').checked});saved=true;$('status').textContent='已独立保存：'+d.path;$('record').textContent=JSON.stringify(d.record,null,2);}catch(e){$('error').textContent=e.message;}finally{busy=false;controls();}};
$('stop').onclick=async()=>{if(!confirm('停止标定推流并释放相机？'))return;try{await post('/stop');$('live').removeAttribute('src');$('camera').textContent='推流已停止，相机正在释放';$('capture').disabled=true;$('save').disabled=true;}catch(e){$('error').textContent=e.message;}};
async function poll(){try{let r=await fetch('/status');let d=await r.json();$('camera').textContent=d.camera_error||((d.image_size?d.image_size.join(' × '):'等待画面')+' · '+d.detection+' · '+(d.frame_age_ms===null?'无新帧':d.frame_age_ms+' ms'));}catch(e){} }
setInterval(poll,1200);poll();
</script></html>"""


def jpeg(frame):
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise ValueError('JPEG 编码失败')
    return encoded.tobytes()


def annotated(frame, corners):
    result = frame.copy()
    cv2.polylines(result, [np.round(corners).astype(np.int32)], True, (0,255,0), 3)
    for i, point in enumerate(corners):
        x,y = np.round(point).astype(int)
        cv2.circle(result,(x,y),6,(0,0,255),-1)
        cv2.putText(result,str(i),(x+9,y-9),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,255),2)
    return result


def camera_reference(record, values):
    x,y = values.get('tag_center_x_cm'), values.get('tag_center_y_cm')
    aligned = values.get('axes_aligned',False)
    if x is None and y is None and aligned is False:
        return None
    if not isinstance(aligned,bool) or aligned is not True:
        raise ValueError('记录相机坐标前，需要确认标签方向与车辆对齐')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in (x,y)) or y <= 0:
        raise ValueError('必须填写有限的横向、前向实测距离，前向距离需大于 0')
    offset = np.array([x,y],dtype=float)/100
    return {'origin':'navigation_camera_ground_projection','axes':{'x':'right','y':'forward','unit':'m'},
            'tag_center_m':offset.tolist(), 'image_points':record['image_points'],
            'ground_points_m':(np.asarray(record['ground_points'])+offset).tolist(),
            'axes_aligned_by_user':True,'verified':False}


class CalibrationSession:
    def __init__(self, source, directory):
        self.source, self.directory = source, Path(directory)
        self._cond = threading.Condition()
        self._operation = threading.Lock()
        self.frame = None
        self.captured_s = None
        self._preview_s = -math.inf
        self._jpeg = b''
        self._sequence = 0
        self.detection = '等待完整标签'
        self.camera_error = ''
        self.draft = None

    def fail(self, message):
        with self._cond:
            self.camera_error = message
            self._cond.notify_all()

    def publish(self, frame, now=None):
        now = time.monotonic() if now is None else now
        with self._cond:
            self.frame = frame.copy()
            self.captured_s = now
            self.camera_error = ''
            if now - self._preview_s < .1:
                return
            self._preview_s = now
        try:
            corners = detect_tag(frame)
            preview = annotated(frame,corners)
            detection = '已识别唯一 ID 0，四角完整'
        except (ValueError,cv2.error) as exc:
            preview = frame
            detection = str(exc)
        encoded = jpeg(preview)
        with self._cond:
            self.detection = detection
            self._jpeg = encoded
            self._sequence += 1
            self._cond.notify_all()

    def _fresh_frame(self):
        with self._cond:
            if self.camera_error or self.frame is None or self.captured_s is None or time.monotonic()-self.captured_s > 1:
                raise ValueError(self.camera_error or '相机没有新鲜画面，请等待或检查相机')
            return self.frame.copy(), self.captured_s

    def status(self):
        with self._cond:
            return {'image_size':list(self.frame.shape[1::-1]) if self.frame is not None else None,
                    'frame_age_ms':round((time.monotonic()-self.captured_s)*1000) if self.captured_s is not None else None,
                    'camera_error':self.camera_error, 'detection':self.detection,
                    'draft_id':self.draft['id'] if self.draft else None,'auto_apply':False}

    def capture(self):
        with self._operation:
            # 新一次检测失败时不能保留可保存的旧草稿。
            with self._cond:
                self.draft = None
            frame,captured_s = self._fresh_frame()
            corners = detect_tag(frame)
            record = solve_calibration(corners,frame.shape[1::-1],PREVIEW_BEV,self.source)
            record.update(applied=False,captured_monotonic_s=captured_s,vehicle_reference=None)
            overlay = annotated(frame,corners)
            bev = cv2.warpPerspective(frame,np.asarray(record['H_img_to_bev']),bev_size(PREVIEW_BEV))
            draft = {'id':uuid.uuid4().hex,'record':record,'frame':frame,'overlay':overlay,'bev':bev,'path':None}
            with self._cond:
                self.draft = draft
            return {'draft_id':draft['id'],'record':copy.deepcopy(record)}

    def preview(self, kind, draft_id):
        with self._cond:
            if self.draft is None or self.draft['id'] != draft_id:
                raise ValueError('冻结结果已过期，请重新检测')
            frame = self.draft['overlay' if kind=='frozen' else 'bev'].copy()
        return jpeg(frame)

    def save(self, values):
        with self._operation:
            with self._cond:
                draft = self.draft
                if draft is None or values.get('draft_id') != draft['id']:
                    raise ValueError('冻结结果已过期或没有有效四角，请重新检测')
            current,_ = self._fresh_frame()
            if list(current.shape[1::-1]) != draft['record']['image_size']:
                raise ValueError('相机分辨率已改变，请重新检测')
            if draft['path']:
                return {'path':str(draft['path']),'record':copy.deepcopy(draft['record'])}
            record = copy.deepcopy(draft['record'])
            record['vehicle_reference'] = camera_reference(record,values)
            name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_'+draft['id'][:8]
            folder = self.directory/name
            folder.mkdir(parents=True,exist_ok=False)
            for filename,frame in [('raw.png',draft['frame']),('corners.png',draft['overlay']),('bev.png',draft['bev'])]:
                if not cv2.imwrite(str(folder/filename),frame):
                    raise OSError('无法保存 '+filename)
            path = folder/'calibration.json'
            save_calibration(record,path)
            with self._cond:
                draft['record'],draft['path'] = record,path
            print('标定结果独立保存：'+str(path),flush=True)
            return {'path':str(path),'record':copy.deepcopy(record)}

    def wait_jpeg(self, sequence):
        with self._cond:
            if self._sequence <= sequence:
                self._cond.wait(1)
            return (self._sequence,self._jpeg) if self._sequence>sequence and self._jpeg else None


def handler_factory(session, stop):
    class Handler(BaseHTTPRequestHandler):
        def send(self, status, content_type, body):
            self.send_response(status)
            self.send_header('Content-Type',content_type)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, payload, status=200):
            self.send(status,'application/json; charset=utf-8',json.dumps(payload,ensure_ascii=False,allow_nan=False).encode())

        def do_GET(self):
            request = urlparse(self.path)
            if request.path=='/':
                self.send(200,'text/html; charset=utf-8',PAGE.encode())
            elif request.path=='/status':
                self.send_json(session.status())
            elif request.path in ('/frozen.jpg','/bev.jpg'):
                from urllib.parse import parse_qs
                try:
                    draft_id = parse_qs(request.query).get('draft',[''])[0]
                    self.send(200,'image/jpeg',session.preview('frozen' if request.path=='/frozen.jpg' else 'bev',draft_id))
                except ValueError as exc:
                    self.send_json({'message':str(exc)},409)
            elif request.path=='/camera.mjpg':
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store')
                self.end_headers()
                sequence = 0
                try:
                    while not stop.is_set():
                        item = session.wait_jpeg(sequence)
                        if item is None:
                            continue
                        sequence,encoded = item
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(encoded)).encode()+b'\r\n\r\n'+encoded+b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError,ConnectionResetError,TimeoutError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.send_json({'message':'请从本机标定页面操作'},403)
                return
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0<=length<=4096:
                    raise ValueError('请求大小无效')
                values = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(values,dict):
                    raise ValueError('请求必须是 JSON 对象')
                path = urlparse(self.path).path
                if path=='/capture':
                    self.send_json(session.capture())
                elif path=='/save':
                    self.send_json(session.save(values))
                elif path=='/stop':
                    self.send_json({'message':'正在停止推流'})
                    stop.set()
                    threading.Thread(target=self.server.shutdown,daemon=True).start()
                else:
                    self.send_error(404)
            except (ValueError,KeyError,TypeError,cv2.error,OSError) as exc:
                self.send_json({'message':str(exc)},409)

        def log_message(self, *args):
            pass
    return Handler


def capture_loop(session, stop, width, height, fps):
    camera = cv2.VideoCapture(session.source,cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise RuntimeError('无法打开相机 '+session.source)
        camera.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH,width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        camera.set(cv2.CAP_PROP_FPS,fps)
        while not stop.is_set():
            ok,frame = camera.read()
            if not ok or frame is None:
                session.fail('相机读帧失败')
                stop.wait(.1)
                continue
            if frame.shape[1::-1] != (width,height):
                raise RuntimeError('实际分辨率不是请求的 '+str((width,height)))
            session.publish(frame)
    except Exception as exc:
        session.fail(str(exc))
        print('相机错误：'+str(exc),flush=True)
    finally:
        camera.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='/dev/video0')
    parser.add_argument('--bind',default='0.0.0.0')
    parser.add_argument('--port',type=int,default=8081)
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=720)
    parser.add_argument('--fps',type=int,default=15)
    parser.add_argument('--output',type=Path,default=ROOT/'data/calibration/apriltag')
    args = parser.parse_args(argv)
    if min(args.width,args.height,args.fps)<=0 or not 1<=args.port<=65535:
        parser.error('分辨率、帧率和端口无效')
    if not Path(args.device).exists():
        parser.error('相机不存在：'+args.device)
    check = subprocess.run(['fuser',args.device],capture_output=True,text=True)
    if check.returncode not in (0,1) or check.stdout.strip():
        parser.error('相机已被占用，请先停止导航或其他相机程序')
    session = CalibrationSession(args.device,args.output)
    stop = threading.Event()
    server = ThreadingHTTPServer((args.bind,args.port),handler_factory(session,stop))
    server.daemon_threads = True
    worker = threading.Thread(target=capture_loop,args=(session,stop,args.width,args.height,args.fps),daemon=True)
    worker.start()
    def request_stop(*_):
        stop.set()
        threading.Thread(target=server.shutdown,daemon=True).start()
    previous = {s:signal.signal(s,request_stop) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
    print(f'APRILTAG_WEB_READY port={args.port}; 标签 134 mm；独立保存，不应用配置，不调频、不打开 UART',flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        stop.set()
        server.server_close()
        worker.join(timeout=3)
        for s,handler in previous.items():
            signal.signal(s,handler)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
