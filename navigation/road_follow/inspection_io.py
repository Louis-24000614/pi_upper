"""共享侧视帧、既有 HTTP 服务与常驻 PWM 桥；不依赖 Qt。"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import secrets
import subprocess
import threading
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np


@dataclass(frozen=True)
class SideFrame:
    image: np.ndarray
    sequence: int
    captured_s: float
    epoch: int
    side: str


class FrameHub:
    def __init__(self, side="A"):
        self.lock = threading.Lock()
        self.frame = None
        self.sequence, self.epoch, self.side = 0, 0, side
        self.error = "等待侧视相机画面"

    def generation(self):
        with self.lock:
            return self.epoch

    def invalidate(self, side=None):
        with self.lock:
            self.epoch += 1
            self.frame = None
            if side is not None:
                self.side = side
            return self.epoch

    def publish(self, image, captured_s, epoch=None):
        with self.lock:
            if epoch is not None and epoch != self.epoch:
                return False
            if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
                self.error = "侧视帧格式无效"
                return False
            self.sequence += 1
            self.frame = SideFrame(image.copy(), self.sequence, captured_s, self.epoch, self.side)
            self.error = ""
            return True

    def fail(self, reason):
        with self.lock:
            self.frame = None
            self.error = reason

    def latest(self, now=None, max_age_s=1):
        now = time.monotonic() if now is None else now
        with self.lock:
            f = self.frame
            if f is None or not 0 <= now-f.captured_s <= max_age_s:
                return None
            return f

    def info(self):
        with self.lock:
            return {"error": self.error, "side": self.side, "epoch": self.epoch,
                    "frame_age_ms": round((time.monotonic()-self.frame.captured_s)*1000) if self.frame else None,
                    "image_size": list(self.frame.image.shape[1::-1]) if self.frame else None}


class SideCamera:
    def __init__(self, config, hub, capture_factory=None):
        self.config, self.hub = config, hub
        self.factory = capture_factory or (lambda device: cv2.VideoCapture(device, cv2.CAP_V4L2))
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="culvert-side-camera", daemon=True)
        self.thread.start()

    def _run(self):
        capture = None
        try:
            capture = self.factory(self.config["device"])
            if not capture.isOpened():
                raise OSError("侧视相机打开失败")
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["width"])
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["height"])
            while not self.stop.is_set():
                epoch = self.hub.generation()
                ok, image = capture.read()
                captured_s = time.monotonic()
                if ok:
                    self.hub.publish(image, captured_s, epoch)
                else:
                    self.hub.fail("侧视相机读帧失败")
                    self.stop.wait(.05)
        except Exception as exc:
            self.hub.fail(str(exc))
        finally:
            if capture is not None:
                capture.release()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)


def crop(image, roi):
    h, w = image.shape[:2]
    x1, y1 = int(roi[0]*w), int(roi[1]*h)
    x2, y2 = int(roi[2]*w), int(roi[3]*h)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("外层 ROI 在实际分辨率下为空")
    return np.ascontiguousarray(image[y1:y2, x1:x2]), (x1, y1)


def offset_box(box, offset):
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return None
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in box):
        return None
    x, y = offset
    return [box[0]+x, box[1]+y, box[2]+x, box[3]+y]


def post_image(endpoint, image, fields, timeout):
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise ValueError("识别图像编码失败")
    boundary = "----culvert"+secrets.token_hex(12)
    chunks = []
    for name, value in fields.items():
        chunks.append((f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n').encode())
    chunks.extend([(f'--{boundary}\r\nContent-Disposition: form-data; name="image"; filename="frame.jpg"\r\nContent-Type: image/jpeg\r\n\r\n').encode(),
                   encoded.tobytes(), (f"\r\n--{boundary}--\r\n").encode()])
    request = Request(endpoint, data=b"".join(chunks), headers={"Content-Type": "multipart/form-data; boundary="+boundary}, method="POST")
    with urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read(2*1024*1024))
    if payload.get("status") != "success" or not isinstance(payload.get("data"), dict):
        raise ValueError("识别服务响应异常")
    return payload["data"]


class Recognizer:
    def __init__(self, services, request=post_image):
        self.services, self.request = services, request

    def recognize(self, frame, rec):
        image, offset = crop(frame.image, rec["roi"])
        timeout = rec["request_timeout_s"]
        data = self.request(self.services["face"], image, {}, timeout)
        faces = data.get("results")
        if not isinstance(faces, list):
            raise ValueError("人脸服务未返回明确 results")
        if faces:
            if any(not isinstance(face, dict) for face in faces):
                raise ValueError("人脸结果格式无效")
            candidate = max(faces, key=lambda f: float(f.get("score", -1)))
            label, score = candidate.get("name"), candidate.get("score")
            if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score):
                score = None
            valid = len(faces) == 1 and isinstance(label, str) and label.startswith("suspect_") and label in {f"suspect_{n:02d}" for n in range(1, 11)}
            valid = valid and isinstance(score, (int, float)) and not isinstance(score, bool) and math.isfinite(score) and score >= rec["face_threshold"]
            return {"accepted": bool(valid), "category": "suspect", "identity": label,
                    "score": score, "bbox": offset_box(candidate.get("bbox"), offset), "margin": None,
                    "reason": "accepted" if valid else "face_unknown_low_score_or_multiple"}
        fields = {"roi_selected": "false", "frame_id": frame.sequence,
                  "camera_epoch": frame.epoch, "captured_monotonic_ns": int(frame.captured_s*1e9),
                  "request_id": f"culvert-{frame.epoch}-{frame.sequence}"}
        data = self.request(self.services["knife"], image, fields, timeout)
        score, label = data.get("top1_score"), data.get("top1_class")
        if isinstance(score, bool) or not isinstance(score, (int, float)) or not math.isfinite(score):
            score = None
        margin = data.get("margin")
        if isinstance(margin, bool) or not isinstance(margin, (int, float)) or not math.isfinite(margin):
            margin = None
        box = offset_box(data.get("roi_xyxy"), offset)
        reliable = box is not None and box[2] > box[0] and box[3] > box[1] and data.get("roi_source") in ("light_surface", "foreground")
        valid = reliable and data.get("quality_ok") is True and label in {f"knife_{n:02d}" for n in range(1, 11)}
        valid = valid and isinstance(score, (int, float)) and not isinstance(score, bool) and math.isfinite(score) and score >= rec["knife_threshold"]
        return {"accepted": bool(valid), "category": "knife", "identity": label, "score": score,
                "margin": margin, "bbox": box, "roi_source": data.get("roi_source"),
                "recognition_mode": data.get("recognition_mode"), "reason": "accepted" if valid else "knife_low_score_or_unreliable_roi"}


class ServoBridge:
    """带请求 ID 的写入回执；回执不表示实际机械到位。"""
    def __init__(self, root, config):
        self.process = subprocess.Popen([str(Path(root)/config["binary"]), "--stdin", "--config", str(Path(root)/config["config"])],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.messages = queue.Queue(maxsize=32)
        self.closed = False
        self.reader = threading.Thread(target=self._read, name="culvert-pwm-receive", daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            try:
                self.messages.put_nowait(line.strip())
            except queue.Full:
                pass

    def request(self, angle):
        if self.closed or self.process.poll() is not None:
            raise OSError("PWM 常驻桥不可用")
        token = secrets.token_hex(8)
        self.process.stdin.write(f"angle {token} {angle}\n")
        self.process.stdin.flush()
        return token

    def alive(self):
        return not self.closed and self.process.poll() is None

    def poll(self, token):
        while True:
            try:
                line = self.messages.get_nowait()
            except queue.Empty:
                break
            if line == "SERVO_APPLIED "+token:
                return "done"
            if line.startswith("SERVO_FAIL "+token):
                return "failed"
        return "failed" if self.closed or self.process.poll() is not None else "running"

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.process.stdin.write("close\n")
            self.process.stdin.flush()
        except (OSError, ValueError):
            pass
        finally:
            try:
                self.process.stdin.close()
            except (OSError, ValueError):
                pass
        # 正常 close/EOF 和 TERM 均由 C++ 桥关闭 PWM；不使用 SIGKILL。
        try:
            self.process.wait(timeout=.5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
        self.reader.join(timeout=.2)
        if self.process.poll() is not None:
            self.process.stdout.close()
