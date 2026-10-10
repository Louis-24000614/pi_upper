"""独立手动棋盘格网页：导航相机冻结/原图上传、四外角标定、独立保存。"""
from __future__ import annotations

import argparse
import copy
import hashlib
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse
import uuid

import cv2
import numpy as np

from manual_calibration import ROOT, UNITS, navigation_settings, number, render_images, solve_calibration, validate_record

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from vision.ipm_proto import manual_ground as ground


def encoded(frame, extension):
    ok, data = cv2.imencode(extension, frame)
    if not ok:
        raise OSError("图片编码失败，未保存标定")
    return data.tobytes()


class CalibrationSession:
    def __init__(self, settings, directory, *, offline=False):
        self.settings, self.directory = copy.deepcopy(settings), Path(directory)
        self.offline = offline
        self._operation = threading.RLock()
        self._condition = threading.Condition()
        self.frame = None
        self.captured_s = None
        self.camera_error = "离线模式：请上传原始图片" if offline else "等待导航相机画面"
        self._jpeg = b""
        self._sequence = 0
        self.frozen = None
        self.result = None
        self.revision = 0

    def publish(self, frame):
        if list(frame.shape[1::-1]) != self.settings["image_size"]:
            self.fail("相机实际分辨率与导航配置不同，请检查设备")
            return
        data = encoded(frame, ".jpg")
        with self._condition:
            self.frame, self.captured_s = frame.copy(), time.monotonic()
            self.camera_error = ""
            self._jpeg, self._sequence = data, self._sequence + 1
            self._condition.notify_all()

    def fail(self, message):
        with self._condition:
            self.camera_error = message
            self.frame = None
            self._condition.notify_all()

    def status(self):
        with self._operation, self._condition:
            try:
                active, application_error = ground.read_activation(self.directory), None
            except ValueError as exc:
                active, application_error = None, str(exc)
            active_path = self.directory / "active.json"
            restore_token = hashlib.sha256(active_path.read_bytes()).hexdigest() if active_path.exists() else None
            return {"offline": self.offline, "device": self.settings["device"],
                    "navigation_image_size": self.settings["image_size"], "camera_error": self.camera_error,
                    "can_capture": self.frame is not None and time.monotonic()-self.captured_s <= 1,
                    "frame_id": self.frozen["id"] if self.frozen else None, "revision": self.revision,
                    "frozen_image_size": list(self.frozen["frame"].shape[1::-1]) if self.frozen else None,
                    "frozen_source": copy.deepcopy(self.frozen["source"]) if self.frozen else None,
                    "baseline": self.settings["baseline"], "auto_apply": False,
                    "active_calibration": active, "application_error": application_error,
                    "restore_token": restore_token,
                    "can_apply": self.directory.resolve() == ground.DIRECTORY.resolve()}

    def _clear(self):
        self.frozen, self.result, self.revision = None, None, 0

    def _freeze(self, frame, source):
        self._clear()
        self.frozen = {"id": uuid.uuid4().hex, "frame": frame.copy(), "source": source,
                       "captured_at": datetime.now(timezone.utc).isoformat()}
        return {"frame_id": self.frozen["id"], "image_size": list(frame.shape[1::-1]),
                "source": copy.deepcopy(source), "baseline": self.settings["baseline"], "revision": 0}

    def capture(self):
        with self._operation:
            self._clear()
            with self._condition:
                if self.frame is None or self.camera_error or time.monotonic()-self.captured_s > 1:
                    raise ValueError(self.camera_error or "相机没有新鲜画面，请等待后重新冻结")
                frame = self.frame.copy()
            return self._freeze(frame, {"kind": "navigation_camera", "device": self.settings["device"],
                                      "requested_image_size": self.settings["image_size"]})

    def upload(self, data, filename):
        with self._operation:
            self._clear()
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION) if data else None
            if frame is None:
                raise ValueError("图片无法读取，请上传原始 PNG 或 JPEG 图片")
            if frame.shape[0] * frame.shape[1] > 20_000_000:
                raise ValueError("图片超过 2000 万像素，请使用原始导航相机分辨率")
            return self._freeze(frame, {"kind": "uploaded_image", "filename": filename.replace("\\", "/").split("/")[-1][:200],
                "navigation_device": self.settings["device"], "requested_image_size": self.settings["image_size"],
                "matches_navigation_resolution": list(frame.shape[1::-1]) == self.settings["image_size"],
                "exif_orientation_applied": False})

    def _current(self, values):
        if not self.frozen or values.get("frame_id") != self.frozen["id"]:
            raise ValueError("冻结图片已失效，请重新冻结或上传")
        revision = values.get("revision")
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < self.revision:
            raise ValueError("角点或输入已更新，本次结果已失效，请重新计算")
        return revision

    def invalidate(self, values):
        with self._operation:
            revision = self._current(values)
            self.revision, self.result = revision, None
            return {"revision": revision}

    def compute(self, values):
        with self._operation:
            revision = self._current(values)
            self.revision, self.result = revision, None
            parameters = values.get("parameters")
            record = solve_calibration(parameters, self.frozen["frame"].shape[1::-1],
                                       self.frozen["source"], self.settings["baseline"])
            record.update(frame_id=self.frozen["id"], captured_at=self.frozen["captured_at"], input_revision=revision)
            overlay, preview = render_images(self.frozen["frame"], record)
            self.result = {"id": uuid.uuid4().hex, "record": record, "parameters": copy.deepcopy(parameters),
                           "overlay": overlay, "preview": preview, "path": None, "checks": []}
            return {"result_id": self.result["id"], "revision": revision, "record": copy.deepcopy(record)}

    def image(self, kind, token):
        with self._operation:
            if kind == "frozen":
                if not self.frozen or token != self.frozen["id"]:
                    raise ValueError("冻结图片已失效")
                return encoded(self.frozen["frame"], ".png")
            if not self.result or token != self.result["id"]:
                raise ValueError("计算结果已失效，请重新计算")
            return encoded(self.result["preview" if kind == "bev" else "overlay"], ".png")

    def _valid_result(self, values):
        revision = self._current(values)
        result = self.result
        if not result or values.get("result_id") != result["id"] or revision != self.revision:
            raise ValueError("没有有效的当前计算结果，请重新计算")
        if values.get("parameters") != result["parameters"]:
            self.result = None
            raise ValueError("角点、尺寸或坐标设置已改变，请重新计算再保存")
        validate_record(result["record"])
        return result

    def check_point(self, values):
        with self._operation:
            result = self._valid_result(values)
            point = np.asarray(values.get("image_point"), np.float64)
            width, height = result["record"]["image_size"]
            if point.shape != (2,) or not np.isfinite(point).all() or np.any(point < 0) or point[0] >= width or point[1] >= height:
                raise ValueError("检查点必须位于原始图像内")
            record = result["record"]
            region = ground.project(record["H_img_to_region_ground_m"], [point])[0]
            candidate = record["vehicle_candidate"]
            vehicle = ground.project(candidate["H_img_to_vehicle_ground_m"], [point])[0] if candidate else None
            response = {"image_point": point.tolist(), "region_ground_m": region.tolist(),
                        "vehicle_ground_m": vehicle.tolist() if vehicle is not None else None}
            measured = values.get("measured")
            if measured is not None:
                if not candidate:
                    raise ValueError("请先确认车辆坐标方向并填写区域中心偏移，再记录实测检查点")
                if not isinstance(measured, dict) or measured.get("unit") not in UNITS:
                    raise ValueError("实测检查点单位请选择 mm 或 cm")
                actual = np.array([number(measured.get("x"), "检查点实测 X"),
                                   number(measured.get("y"), "检查点实测 Y")]) * UNITS[measured["unit"]]
                error = vehicle-actual
                check = {"image_point": point.tolist(), "measured_ground_m": actual.tolist(),
                         "predicted_ground_m": vehicle.tolist(), "error_xy_m": error.tolist(),
                         "error_m": float(np.linalg.norm(error)), "coordinate_reference": "navigation_camera_ground_projection",
                         "checked_at": datetime.now(timezone.utc).isoformat()}
                ground.validate_checks(candidate, [check])
                # 同一像素点修正实测值时替换该记录，不能靠重复点击增加检查点数。
                result["checks"] = [old for old in result["checks"] if np.linalg.norm(np.asarray(old["image_point"])-point) >= 5]
                result["checks"].append(check)
            response["checks"] = copy.deepcopy(result["checks"])
            return response

    def apply(self, values):
        with self._operation:
            if self.directory.resolve() != ground.DIRECTORY.resolve():
                raise ValueError("临时 --output 目录仅用于离线验证，不能应用到本项目导航")
            result = self._valid_result(values)
            candidate = result["record"]["vehicle_candidate"]
            if candidate is None:
                raise ValueError("请先生成车辆坐标候选，区域坐标不能直接应用到导航")
            if candidate["image_size"] != self.settings["image_size"]:
                raise ValueError("上传原图分辨率与导航不同，不能应用")
            current_settings = navigation_settings(self.settings["baseline"]["config_path"])
            if current_settings["baseline"]["config_sha256"] != self.settings["baseline"]["config_sha256"]:
                raise ValueError("导航配置在网页启动后已改变，请重启网页重新标定")
            if values.get("measurement_reviewed") is not True or values.get("image_source_confirmed") is not True:
                raise ValueError("请确认实测误差及导航相机原图来源")
            ground.validate_checks(candidate, result["checks"])
            current = ground.read_activation(self.directory)
            if values.get("expected_applied_at") != (current["applied_at"] if current else None):
                raise ValueError("已选标定发生变化，请刷新状态后再应用")
            path = self.save(values)["path"]
            active = ground.apply_activation(self.directory, path, candidate, copy.deepcopy(result["checks"]),
                device=self.settings["device"], measurement_reviewed=True, image_source_confirmed=True,
                ground_contact_verified=values.get("ground_contact_verified", False))
            return {"active_calibration": active, "message": "已选择标定；重启导航后生效，当前运行进程不切换矩阵"}

    def restore(self, values):
        with self._operation:
            if self.directory.resolve() != ground.DIRECTORY.resolve():
                raise ValueError("临时 --output 目录不能恢复本项目导航标定")
            active = ground.restore_activation(self.directory, values.get("expected_applied_at"), values.get("restore_token"))
            return {"active_calibration": active, "message": "已恢复上次设置；重启导航后生效"}

    def save(self, values):
        with self._operation:
            result = self._valid_result(values)
            record = copy.deepcopy(result["record"])
            validate_record(record)
            if list(self.frozen["frame"].shape[1::-1]) != record["image_size"]:
                raise ValueError("原始图片尺寸已改变，请重新计算")
            if result["path"]:
                return {"path": str(result["path"]), "record": record}
            self.directory.mkdir(parents=True, exist_ok=True)
            name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + "_" + uuid.uuid4().hex[:8]
            folder = self.directory / name
            pending = self.directory / (name + ".pending")
            pending.mkdir(exist_ok=False)
            try:
                for filename, frame in (("raw.png", self.frozen["frame"]), ("corners.png", result["overlay"]), ("bev.png", result["preview"])):
                    (pending / filename).write_bytes(encoded(frame, ".png"))
                candidate = record.pop("vehicle_candidate")
                record.update(saved_at=datetime.now(timezone.utc).isoformat(),
                              vehicle_candidate_file="vehicle_candidate.json" if candidate else None)
                (pending / "calibration.json").write_text(json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
                if candidate:
                    (pending / "vehicle_candidate.json").write_text(json.dumps(candidate, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
                pending.rename(folder)
            except Exception:
                # 只清理本次创建、且仍在结果目录内的临时目录，不覆盖已有结果。
                if pending.exists() and pending.resolve().parent == self.directory.resolve():
                    shutil.rmtree(pending)
                raise
            result["path"] = folder / "calibration.json"
            return {"path": str(result["path"]), "record": copy.deepcopy(result["record"])}

    def wait_jpeg(self, sequence):
        with self._condition:
            if self._sequence <= sequence:
                self._condition.wait(1)
            return (self._sequence, self._jpeg) if self._sequence > sequence and self._jpeg else None


def handler_factory(session, stop):
    class Handler(BaseHTTPRequestHandler):
        def send(self, status, kind, body):
            try:
                self.send_response(status)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            except ConnectionError:
                pass  # 切换图片或关闭页面会取消浏览器尚未完成的请求。

        def send_json(self, payload, status=200):
            self.send(status, "application/json; charset=utf-8", json.dumps(payload, ensure_ascii=False, allow_nan=False).encode())

        def do_GET(self):
            request = urlparse(self.path)
            assets = {"/": ("manual_calibration.html", "text/html; charset=utf-8"),
                      "/manual_calibration.js": ("manual_calibration.js", "text/javascript; charset=utf-8")}
            try:
                if request.path in assets:
                    filename, kind = assets[request.path]
                    self.send(200, kind, Path(__file__).with_name(filename).read_bytes())
                elif request.path == "/status":
                    self.send_json(session.status())
                elif request.path in ("/frozen.png", "/bev.png", "/corners.png"):
                    token = parse_qs(request.query).get("id", [""])[0]
                    self.send(200, "image/png", session.image(request.path.split(".")[0][1:], token))
                elif request.path == "/camera.mjpg" and not session.offline:
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    sequence = 0
                    while not stop.is_set():
                        item = session.wait_jpeg(sequence)
                        if item is None:
                            continue
                        sequence, data = item
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                        self.wfile.flush()
                else:
                    self.send_error(404)
            except ConnectionError:
                pass
            except (ValueError, OSError, cv2.error) as exc:
                self.send_json({"message": str(exc)}, 409)

        def do_POST(self):
            origin = self.headers.get("Origin")
            if origin and urlparse(origin).netloc != self.headers.get("Host"):
                self.send_json({"message": "请从本标定网页操作"}, 403)
                return
            request = urlparse(self.path)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                limit = 20*1024*1024 if request.path == "/upload" else 128*1024
                if not 0 < length <= limit:
                    raise ValueError("请求为空或过大，图片最多 20 MB")
                body = self.rfile.read(length)
                if request.path == "/upload":
                    self.send_json(session.upload(body, parse_qs(request.query).get("filename", ["uploaded-image"])[0]))
                    return
                values = json.loads(body)
                if not isinstance(values, dict):
                    raise ValueError("请求必须是 JSON 对象")
                operations = {"/capture": session.capture, "/compute": session.compute, "/invalidate": session.invalidate,
                              "/save": session.save, "/check-point": session.check_point,
                              "/apply": session.apply, "/restore": session.restore}
                if request.path in operations:
                    self.send_json(operations[request.path]() if request.path == "/capture" else operations[request.path](values))
                elif request.path == "/stop":
                    self.send_json({"message": "网页正在停止，相机将释放"})
                    stop.set()
                    threading.Thread(target=self.server.shutdown, daemon=True).start()
                else:
                    self.send_error(404)
            except (ValueError, TypeError, KeyError, OSError, cv2.error) as exc:
                self.send_json({"message": str(exc)}, 409)

        def log_message(self, *args):
            pass
    return Handler


def capture_loop(session, stop, fps):
    camera = cv2.VideoCapture(session.settings["device"], cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            raise RuntimeError("无法打开导航相机")
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, session.settings["image_size"][0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, session.settings["image_size"][1])
        camera.set(cv2.CAP_PROP_FPS, fps)
        while not stop.is_set():
            ok, frame = camera.read()
            if not ok or frame is None:
                session.fail("导航相机读帧失败")
                stop.wait(.1)
            else:
                session.publish(frame)
    except Exception as exc:
        session.fail(str(exc))
    finally:
        camera.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/nav_camera.yaml")
    parser.add_argument("--offline", action="store_true", help="仅上传图片，不打开相机")
    parser.add_argument("--image", type=Path, help="离线模式初始原始图片")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--output", type=Path, default=ROOT / "data/calibration/manual")
    args = parser.parse_args(argv)
    if args.fps <= 0 or not 0 <= args.port <= 65535:
        parser.error("帧率或端口无效")
    if args.image and not args.offline:
        parser.error("--image 必须与 --offline 一起使用")
    settings = navigation_settings(args.config)
    if not args.offline:
        if not Path(settings["device"]).exists():
            parser.error("导航相机不存在；离线标定请传 --offline")
        try:
            check = subprocess.run(["fuser", settings["device"]], capture_output=True, text=True)
        except OSError:
            parser.error("无法检查相机占用，请确认板端 fuser 可用")
        if check.returncode not in (0, 1) or check.stdout.strip():
            parser.error("导航相机已被占用，请先结束占用它的程序")
    session = CalibrationSession(settings, args.output, offline=args.offline)
    if args.image:
        session.upload(args.image.read_bytes(), args.image.name)
    stop = threading.Event()
    server = ThreadingHTTPServer((args.bind, args.port), handler_factory(session, stop))
    server.daemon_threads = True
    worker = None
    if not args.offline:
        worker = threading.Thread(target=capture_loop, args=(session, stop, args.fps), daemon=True)
        worker.start()
    def request_stop(*_):
        stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signals = [signal.SIGINT, signal.SIGTERM] + ([signal.SIGHUP] if hasattr(signal, "SIGHUP") else [])
    previous = {s: signal.signal(s, request_stop) for s in signals}
    print(f"MANUAL_CALIBRATION_READY port={server.server_port} offline={args.offline}; 保存不自动应用，明确应用后重启导航生效", flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        stop.set()
        server.server_close()
        if worker:
            worker.join(3)
        for s, handler in previous.items():
            signal.signal(s, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
