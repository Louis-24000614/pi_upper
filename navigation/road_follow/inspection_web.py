"""局域网侧视调试：共享帧 MJPEG、原图外层 ROI、持久化识别参数。"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from urllib.parse import urlparse

import cv2

from road_follow.inspection_io import crop


PAGE = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>涵洞与导航调试</title>
<style>
body{background:#14202a;color:#edf5fa;font:16px system-ui;margin:0}main{max-width:1280px;margin:auto;padding:20px}.panel{background:#20313f;padding:18px;margin:14px 0;border-radius:10px}h1,h2,h3{margin:8px 0 16px}.muted{color:#b8cbd7}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:16px}.card{background:#172733;padding:16px;border-radius:8px}.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px}.metric strong{display:block;font-size:24px;margin-top:6px}.good{color:#73e0b0}.warn{color:#ffc26f}.bad{color:#ff9090}#view{position:relative;line-height:0;max-width:100%}#live{display:block;width:100%}#overlay{position:absolute;inset:0;width:100%;height:100%;touch-action:none}label{display:flex;flex-direction:column;gap:5px}input{width:100%;box-sizing:border-box;padding:9px;border:1px solid #536b7b;background:#14202a;color:#fff;border-radius:5px;font:inherit}button{padding:10px 18px;margin:12px 8px 4px 0;font:inherit;border:0;border-radius:6px;background:#79c9e8;color:#14202a;cursor:pointer}#crop{max-width:100%;max-height:300px}#topology{width:100%;max-height:650px;background:white;border-radius:7px}table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:9px 5px;border-bottom:1px solid #38505f;overflow-wrap:anywhere}td:first-child{color:#b8cbd7;width:45%}#message,#obstacle-message{color:#ffc26f}.scroll{overflow:auto}
</style>
<main><h1>涵洞与导航调试</h1><p class="muted">侧视画面、两侧识别、障碍判定与本次任务地图。相似度不是成功概率。</p>
<div class="panel"><h2>实时任务状态</h2><div id="summary" class="metrics"></div><p id="failure" class="bad"></p><div class="grid"><div class="card"><h3>当前候选目标</h3><table id="candidate"></table></div><div class="card"><h3>当前使用的识别参数</h3><table id="applied"></table></div></div></div>
<div class="panel"><h2>两侧检查结果</h2><div id="results" class="grid"></div></div>
<div class="panel"><h2>本次任务拓扑地图</h2><p id="route" class="muted"></p><img id="topology" src="/topology.svg" alt="本次任务拓扑地图"><p class="muted">每次启动任务清空动态标记。方框表示障碍：橙色为看见，红色为已确认封边。涵洞：黄色已发现、绿色两侧成功、紫色部分未确认、红色故障。蓝圈表示收到的标签号，蓝点表示车辆。位置按投影和拓扑里程估计。</p><div class="grid"><div class="card"><h3>障碍记录</h3><div id="obstacle-records"></div></div><div class="card"><h3>涵洞记录</h3><div id="culvert-records"></div></div><div class="card"><h3>标签记录</h3><div id="rfid-records"></div></div></div></div>
<div class="panel"><h2>障碍判断调参</h2><p class="muted">检测框底部用于估算距离。仅目标位于当前边、且没有进入终点余量时，才允许累计封边。前方其他路段的目标可先标记“看见”。这些值保存在涵洞识别 JSON 的障碍参数区；修改后从后续导航帧生效，并清空尚未确认的累计。</p><div class="grid">
<label>路段终点保留余量（米）<input id="ob-edge_end_margin_m" type="number" min="0" max="5" step="0.01"></label>
<label>投影距离修正（米，可为负）<input id="ob-distance_bias_m" type="number" min="-1" max="1" step="0.01"></label>
<label>最大判定距离（米）<input id="ob-max_distance_m" type="number" min="0.01" max="5" step="0.01"></label>
<label>相对道路中心的横向容差（米）<input id="ob-max_lateral_m" type="number" min="0.01" max="5" step="0.01"></label>
<label>障碍检测置信度下限<input id="ob-min_score" type="number" min="0" max="1" step="0.01"></label>
<label>框底部最低位置（原图高度比例）<input id="ob-min_bottom_ratio" type="number" min="0" max="1" step="0.01"></label>
<label>障碍连续确认帧数<input id="ob-confirm_frames" type="number" min="1" max="100" step="1"></label>
</div><button id="save-obstacle">应用并保存障碍参数</button><p id="obstacle-message"></p><h3>当前已应用的障碍参数</h3><table id="obstacle-applied"></table><h3>最近一帧的障碍判断</h3><div id="obstacle-live"></div></div>
<div class="panel"><h2>侧视画面与外层 ROI</h2><p>在完整原图上拖拽外层 ROI，排除底部板卡。此裁剪同时用于人脸与刀具；刀具内部取景与前景提取保留。</p><div id="view"><img id="live" src="/camera.mjpg" alt="侧视原图"><canvas id="overlay"></canvas></div><p id="camera">等待画面</p><p id="message"></p></div>
<div class="panel"><h2>识别参数与裁剪范围</h2><div class="grid"><label>人脸相似度下限<input id="face_threshold" type="number" min="0.45" max="1" step="0.01"></label><label>刀具相似度下限<input id="knife_threshold" type="number" min="0" max="1" step="0.01"></label><label>连续新帧数<input id="confirm_frames" type="number" min="1" max="100" step="1"></label><label>每侧识别期限（秒）<input id="side_timeout_s" type="number" min="1" max="600" step="1"></label></div><p class="muted">人脸服务身份门限为 0.45；刀具前两候选分差只显示。修改后重新累计，当前侧期限不重置；新期限从下一侧生效。</p><div class="grid"><label>左边界（比例）<input id="x1" type="number" min="0" max="1" step="0.01"></label><label>上边界（比例）<input id="y1" type="number" min="0" max="1" step="0.01"></label><label>右边界（比例）<input id="x2" type="number" min="0" max="1" step="0.01"></label><label>下边界（比例）<input id="y2" type="number" min="0" max="1" step="0.01"></label></div><p class="muted">边界使用原图的 0～1 比例，两个方向共用。</p><button id="save">应用并保存识别参数</button><button id="full">选整帧</button><p>当前已应用的裁剪预览：</p><img id="crop" alt="外层裁剪预览"></div></main>
<script>
const $=x=>document.getElementById(x),canvas=$('overlay'),ctx=canvas.getContext('2d');let initialized=false,start=null,candidate=null,revision=0,obstacleRevision=0,lastStatus=null;
const phaseNames={idle:'导航中，等待涵洞',recognizing:'识别当前侧',turning:'舵机转向',settling:'等待姿态稳定',done:'两侧检查结束',failed:'识别任务故障',cancelled:'任务已取消',closed:'任务已关闭'};
const reasonNames={waiting:'等待开始',waiting_fresh_frame:'等待新的侧视画面',camera_no_fresh_frame:'侧视相机没有新鲜帧',camera_failed:'侧视相机读取失败',accepted:'候选满足条件',confirmed:'连续新帧确认成功',no_face:'未检测到人脸',face_unknown_low_score_or_multiple:'人脸身份未知、相似度不足或检测到多人',face_low_score_or_unknown:'人脸相似度不足或身份未知',face_unknown_or_low_score:'人脸相似度不足或身份未知',multiple_faces:'检测到多张人脸',knife_low_score_or_unreliable_roi:'刀具相似度不足，或内部自动取景未通过',face_request_failed:'人脸服务请求失败',knife_request_failed:'刀具服务请求失败',request_failed:'识别服务请求失败',vision_unsafe:'导航画面未满足道路条件，已故障停车',odom_stale:'里程信息失效，已故障停车',stop_failed:'停车确认失败',task_failed:'识别执行器报告故障',hard_obstacle:'道路障碍接管，已取消涵洞任务',current_edge:'位于当前路段，可参与确认',beyond_current_edge:'位于当前路段以外，不封闭当前边',near_edge_boundary:'接近路段终点，暂不计入当前边',outside_distance_window:'超出配置的距离范围',outside_current_lane:'不在当前道路中心附近',unaligned_odom:'没有匹配采集时刻的里程',invalid_box:'检测框数据无效',invalid_ground_projection:'地面投影无效',clipped_ground_contact:'检测框被截断，底部接地点不可靠',no_detection:'未检测到障碍',low_score:'障碍置信度不足',too_small:'障碍检测框过小',too_far:'障碍框底部位置太远',outside_corridor:'障碍不在前方走廊',tracking:'正在连续确认',latched:'障碍已确认并锁存',duplicate_frame:'重复采集帧，不增加计数',settings_changed:'参数已修改，重新累计',unknown_near_target:'当前路段有未知障碍',stop_submit_failed:'停车请求发送失败',stop_ack_timeout:'停车回执超时',settle_timeout:'停稳确认超时',stop_position_error:'停车位置不符合涵洞要求',edge_changed:'识别时拓扑路段发生变化',reacquire_timeout:'重新确认道路超时'};
function reason(v){if(!v)return '暂无';if(v.startsWith('side_timeout:'))return '本侧识别期限已到：'+reason(v.slice(13));if(v.startsWith('pwm_')||v.startsWith('servo_'))return '舵机输出或回执异常';return reasonNames[v]||'检测未满足确认条件或服务响应异常';}
function identity(v){if(!v)return '尚未确认';if(v.startsWith('suspect_'))return '嫌疑人 '+Number(v.slice(8))+' 号';if(v.startsWith('knife_'))return '刀具 '+Number(v.slice(6))+' 号';return '未注册或未知目标';}
function num(v,d=3){return typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';}
function roiText(r){return r?'左 '+num(r[0]*100,1)+'% · 上 '+num(r[1]*100,1)+'% · 右 '+num(r[2]*100,1)+'% · 下 '+num(r[3]*100,1)+'%':'暂无';}
function table(target,rows){target.replaceChildren();for(const [label,value]of rows){const tr=document.createElement('tr');for(const text of [label,value]){const td=document.createElement('td');td.textContent=text;tr.append(td);}target.append(tr);}}
function parameterRows(p){return [['人脸相似度下限',num(p.face_threshold)],['刀具相似度下限',num(p.knife_threshold)],['连续确认帧数',p.confirm_frames+' 帧'],['每侧识别期限',p.side_timeout_s+' 秒'],['采样间隔',p.sample_interval_s+' 秒'],['服务请求期限',p.request_timeout_s+' 秒'],['外层裁剪范围',roiText(p.roi)]];}
function obstacleParameterRows(p){return [['路段终点保留余量',num(p.edge_end_margin_m,2)+' 米'],['投影距离修正',num(p.distance_bias_m,2)+' 米'],['最大判定距离',num(p.max_distance_m,2)+' 米'],['相对道路中心的横向容差',num(p.max_lateral_m,2)+' 米'],['障碍检测置信度下限',num(p.min_score)],['框底部最低位置',num(p.min_bottom_ratio*100,1)+'% 的原图高度'],['连续确认帧数',p.confirm_frames+' 帧']];}
function obstacleLabel(v){return v&&/[\u3400-\u9fff]/.test(v)?v:'道路障碍';}
function list(target,rows){const t=document.createElement('table');table(t,rows.length?rows:[['记录','本次任务尚无记录']]);target.replaceChildren(t);}
function metrics(s){$('summary').replaceChildren();for(const [label,value]of [['任务阶段',phaseNames[s.phase]||'等待状态'],['当前朝向',(s.side||'—')+' 侧'],['连续有效帧',s.valid_frames+' 帧'],['剩余识别时间',s.remaining_s==null?'—':num(s.remaining_s,1)+' 秒']]){const box=document.createElement('div');box.className='metric';const l=document.createElement('span');l.className='muted';l.textContent=label;const v=document.createElement('strong');v.textContent=value;box.append(l,v);$('summary').append(box);} $('failure').textContent=s.failure_reason?'故障原因：'+reason(s.failure_reason):'';}
function showResults(s){$('results').replaceChildren();const sides=s.sides||[];for(const direction of ['A','B']){const side=sides.find(x=>x.direction===direction),card=document.createElement('div');card.className='card';const title=document.createElement('h3');title.textContent=direction+' 侧检查结果';card.append(title);const t=document.createElement('table');if(side){table(t,[['检查状态',side.status==='confirmed'?'已确认':'未确认'],['朝向角度',side.angle_deg+'°'],['目标',identity(side.identity)],['确认相似度',num(side.score)],['有效确认帧',side.valid_frames+' 帧'],['结果原因',reason(side.reason)],...parameterRows(side.parameters||{})]);}else{table(t,[['检查状态',s.side===direction&&s.phase==='recognizing'?'正在检查':'尚未完成']]);}card.append(t);$('results').append(card);}}
function showMap(m){const v=m.vehicle;$('route').textContent=v?'当前路段 '+v.from_node+' → '+v.to_node+' · 已前进 '+num(v.progress_m,2)+' 米 · 仅本次任务的动态标记':'等待导航任务接入；独立侧视调试不生成实车地图。';list($('obstacle-records'),Object.values(m.obstacles||{}).map(x=>[x.from_node+' → '+x.to_node,(x.status==='confirmed'?'已确认封边':'看见，尚未封边')+' · '+obstacleLabel(x.label)+' · 置信度 '+num(x.score)]));const states={discovered:'已发现',done:'两侧识别成功',partial:'部分未确认',failed:'故障'};list($('culvert-records'),Object.values(m.culverts||{}).map(x=>[x.from_node+' → '+x.to_node,states[x.status]||'检查中']));list($('rfid-records'),(m.rfids||[]).map(x=>['标签 '+x.card_number+' 号',x.location?x.location.from_node+' → '+x.location.to_node+'，沿边 '+num(x.location.progress_m,2)+' 米（估计）':'位置尚未确定']));$('topology').src='/topology.svg?t='+Date.now();}
function showObstacles(items){list($('obstacle-live'),(items||[]).map(x=>[obstacleLabel(x.label),reason(x.reason)+'；投影距离 '+num(x.distance_m,2)+' 米，当前路段剩余 '+num(x.remaining_m,2)+' 米']));}
function roi(){return ['x1','y1','x2','y2'].map(x=>Number($(x).value));}
function draw(status){const w=canvas.width=$('live').clientWidth,h=canvas.height=$('live').clientHeight;ctx.clearRect(0,0,w,h);const r=roi();ctx.strokeStyle='#ffc857';ctx.lineWidth=3;ctx.strokeRect(r[0]*w,r[1]*h,(r[2]-r[0])*w,(r[3]-r[1])*h);if(candidate?.bbox&&status?.camera?.image_size){const [iw,ih]=status.camera.image_size,b=candidate.bbox;ctx.strokeStyle='#73e0b0';ctx.strokeRect(b[0]/iw*w,b[1]/ih*h,(b[2]-b[0])/iw*w,(b[3]-b[1])/ih*h);}}
function point(e){const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))];}
canvas.onpointerdown=e=>{start=point(e);canvas.setPointerCapture(e.pointerId);};canvas.onpointermove=e=>{if(!start)return;const p=point(e);['x1','y1','x2','y2'].forEach((id,i)=>$(id).value=[Math.min(start[0],p[0]),Math.min(start[1],p[1]),Math.max(start[0],p[0]),Math.max(start[1],p[1])][i].toFixed(4));draw(lastStatus);};canvas.onpointerup=()=>start=null;canvas.onpointercancel=()=>start=null;
['x1','y1','x2','y2'].forEach(x=>$(x).oninput=()=>draw(lastStatus));$('full').onclick=()=>{['x1','y1','x2','y2'].forEach((x,i)=>$(x).value=[0,0,1,1][i]);draw(lastStatus);};
async function save(path,payload){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});const d=await r.json();if(!r.ok)throw Error(d.error);return d;}
$('save').onclick=async()=>{try{const values={roi:roi()};['face_threshold','knife_threshold','confirm_frames','side_timeout_s'].forEach(k=>values[k]=Number($(k).value));const d=await save('/settings',{revision,values});revision=d.revision;$('message').textContent='识别参数已应用并保存，当前侧重新累计。';}catch(e){$('message').textContent=e.message;}};
$('save-obstacle').onclick=async()=>{try{const values={};Object.keys(lastStatus.obstacle.settings).forEach(k=>values[k]=Number($('ob-'+k).value));const d=await save('/obstacle-settings',{revision:obstacleRevision,values});obstacleRevision=d.revision;$('obstacle-message').textContent='障碍参数已保存，从后续帧生效。已封闭的边保持本次任务记录。';}catch(e){$('obstacle-message').textContent=e.message;}};
async function poll(){try{const r=await fetch('/status');if(!r.ok)throw Error('状态读取失败');const s=await r.json();lastStatus=s;if(!initialized){['face_threshold','knife_threshold','confirm_frames','side_timeout_s'].forEach(k=>$(k).value=s.settings[k]);['x1','y1','x2','y2'].forEach((x,i)=>$(x).value=s.settings.roi[i]);for(const [k,v]of Object.entries(s.obstacle.settings))$('ob-'+k).value=v;revision=s.revision;obstacleRevision=s.obstacle.revision;initialized=true;}candidate=s.candidate;metrics(s);table($('candidate'),[['目标',identity(candidate?.identity)],['目标类别',candidate?.category==='suspect'?'嫌疑人':candidate?.category==='knife'?'刀具':'待识别'],['相似度',num(candidate?.score)],['刀具前两候选分差',num(candidate?.margin)],['当前判定',reason(s.reason)]]);table($('applied'),parameterRows(s.settings));table($('obstacle-applied'),obstacleParameterRows(s.obstacle.settings));showResults(s);showMap(s.map||{});showObstacles(s.obstacle.details);draw(s);$('camera').textContent=!s.camera.image_size||s.camera.frame_age_ms==null?'等待侧视画面':s.camera.error?'侧视相机异常，等待恢复':s.camera.frame_age_ms>1000?'侧视画面没有及时更新，不能参与识别':'朝向 '+s.camera.side+' 侧 · '+(s.camera.image_size||[]).join(' × ')+' 像素 · 帧龄 '+s.camera.frame_age_ms+' 毫秒';if(s.settings.roi.every((v,i)=>v===[0,0,1,1][i])&&!$('message').textContent)$('message').textContent='当前 ROI 为整帧，尚未排除底部遮挡。';$('crop').src='/crop.jpg?t='+Date.now();}catch(e){$('camera').textContent='调试网页连接中断，等待重新连接。';}}
setInterval(poll,1000);poll();window.onresize=()=>draw(lastStatus);
</script></html>
'''


class InspectionWeb:
    def __init__(self, settings, hub, status, map_status=None, address=None, map_svg=None, obstacle_status=None):
        self.settings, self.hub, self.status_source = settings, hub, status
        self.map_status = map_status or (lambda: {})
        self.map_svg = map_svg or (lambda: '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 180"><rect width="720" height="180" fill="white"/><text x="30" y="90" font-size="22">等待导航任务地图接入</text></svg>'.encode())
        self.obstacle_status = obstacle_status or (lambda: [])
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
                        obstacle, obstacle_revision = session.settings.obstacle_snapshot()
                        self.reply(200, {**session.status_source(), "camera": session.hub.info(),
                                         "settings": config["recognition"], "revision": revision, "map": session.map_status(),
                                         "obstacle": {"settings":obstacle, "revision":obstacle_revision, "details":session.obstacle_status()}})
                    elif path == "/topology.svg":
                        self.send(200, session.map_svg(), "image/svg+xml; charset=utf-8")
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
                path = urlparse(self.path).path
                if path not in ("/settings", "/obstacle-settings"):
                    self.reply(404, {"error": "接口不存在"})
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if not 0 < size <= 8192:
                        raise ValueError("配置请求长度不合法")
                    payload = json.loads(self.rfile.read(size))
                    if not isinstance(payload, dict) or "values" not in payload:
                        raise ValueError("参数请求必须包含待保存的参数")
                    # 两个调试页面不能静默覆盖彼此已经保存的参数。
                    with session.settings._lock:
                        _, current = (session.settings.obstacle_snapshot() if path == "/obstacle-settings"
                                      else session.settings.snapshot())
                        if payload.get("revision") != current:
                            self.reply(409, {"error": "参数已由另一页面修改，请刷新后重试"})
                            return
                        update = session.settings.update_obstacle if path == "/obstacle-settings" else session.settings.update
                        values, revision = update(payload["values"])
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
