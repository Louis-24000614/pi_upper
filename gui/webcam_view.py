"""在浏览器里看一路摄像头，并把原始帧存成道路样本。

独占这一路 V4L2。页面上的「拍照」保存的是采集到的 BGR 帧，
不是浏览器里缩小后的预览，路径规则与全屏拍照相同。
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np

from snap import open_recorder, recording_path, save_frame, snapshot_path


BOUNDARY = b"frame"
PREVIEW_INTERVAL_S = 1.0 / 15.0

PAGE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>摄像头</title>
<style>
  html, body { margin: 0; height: 100%; background: #111; color: #eee;
    font-family: sans-serif; }
  img { width: 100%; height: calc(100vh - 72px); object-fit: contain;
    background: #000; display: block; }
  .bar { display: flex; gap: 16px; align-items: center; height: 72px;
    padding: 0 16px; box-sizing: border-box; }
  button { font-size: 18px; padding: 8px 22px; cursor: pointer; }
  button.live { background: #c0392b; color: white; }
  #status { color: #9cf; }
  #status.live { color: #f5b4b4; }
</style>
</head>
<body>
<img src="/camera.mjpg" alt="摄像头画面">
<div class="bar">
  <button id="snap" type="button">拍照</button>
  <button id="record" type="button">录制</button>
  <span id="status">空格拍照，R 录制。保存的是原始画面，写到香橙派 data/road/。</span>
</div>
<script>
const status = document.getElementById("status");
const recordButton = document.getElementById("record");
let saving = false;
let recording = false;
function showRecord(data) {
  recording = !!data.recording;
  recordButton.textContent = recording ? "停止录制" : "录制";
  recordButton.classList.toggle("live", recording);
  status.classList.toggle("live", recording);
  if (data.message) status.textContent = data.message;
}
async function snap() {
  if (saving) return;
  saving = true;
  const previous = status.textContent;
  status.textContent = "正在保存…";
  try {
    const res = await fetch("/snap", {method: "POST"});
    const data = await res.json();
    status.textContent = data.ok ? ("已保存 " + data.message) : ("拍照失败：" + data.message);
  } catch (err) {
    status.textContent = "拍照失败：" + err;
    if (recording) status.textContent = previous;
  } finally {
    saving = false;
  }
}
async function toggleRecord() {
  const action = recording ? "stop" : "start";
  recordButton.disabled = true;
  try {
    const res = await fetch("/record/" + action, {method: "POST"});
    showRecord(await res.json());
  } catch (err) {
    status.textContent = "录制失败：" + err;
  } finally {
    recordButton.disabled = false;
  }
}
document.getElementById("snap").addEventListener("click", snap);
recordButton.addEventListener("click", toggleRecord);
document.addEventListener("keydown", (event) => {
  if (event.repeat) return;
  if (event.code === "Space" || event.code === "Enter") {
    event.preventDefault();
    snap();
  } else if (event.code === "KeyR" && !event.ctrlKey && !event.metaKey && !event.altKey) {
    event.preventDefault();
    toggleRecord();
  }
});
fetch("/record").then((res) => res.json()).then(showRecord).catch(() => {});
</script>
</body>
</html>
"""


class FrameHub:
    """采集线程写入最新原始帧；预览 JPEG 降到大约 15 帧。"""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frame: np.ndarray | None = None
        self._jpeg = b""
        self._jpeg_seq = 0
        self._jpeg_at = 0.0
        self.error = ""

    def publish(self, frame: np.ndarray, now: float | None = None) -> None:
        copied = np.ascontiguousarray(frame).copy()
        moment = time.monotonic() if now is None else now
        with self._cond:
            self._frame = copied
            self.error = ""
            encode = moment - self._jpeg_at >= PREVIEW_INTERVAL_S or not self._jpeg
            if encode:
                self._jpeg_at = moment
        if not encode:
            return
        ok, encoded = cv2.imencode(
            ".jpg",
            copied,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80],
        )
        if not ok:
            return
        with self._cond:
            self._jpeg = encoded.tobytes()
            self._jpeg_seq += 1
            self._cond.notify_all()

    def fail(self, message: str) -> None:
        with self._cond:
            self.error = message

    def copy_latest(self) -> np.ndarray | None:
        with self._cond:
            if self._frame is None:
                return None
            return self._frame.copy()

    def wait_jpeg(self, after_seq: int, timeout: float) -> tuple[int, bytes] | None:
        with self._cond:
            if self._jpeg_seq <= after_seq:
                self._cond.wait(timeout)
            if self._jpeg_seq <= after_seq or not self._jpeg:
                return None
            return self._jpeg_seq, self._jpeg


def page_html() -> str:
    return PAGE


def save_latest(
    hub: FrameHub,
    role: str,
    directory: Path | None = None,
) -> tuple[bool, str]:
    """把当前原始帧写成 PNG。没有画面时沿用 snap.save_frame 的失败原因。"""
    return save_frame(hub.copy_latest(), snapshot_path(role, directory=directory))


class Recorder:
    """采集线程写原始帧。网页只发出开始或停止，避免两个线程同时写文件。"""

    def __init__(self, role: str, fps: float, directory: Path | None = None) -> None:
        self._role = role
        self._fps = fps
        self._directory = directory
        self._lock = threading.Lock()
        self._pending_start = False
        self._pending_stop = False
        self._writer: cv2.VideoWriter | None = None
        self._path: Path | None = None
        self.recording = False
        self.message = ""

    def status(self) -> dict[str, object]:
        with self._lock:
            return {
                "recording": self.recording,
                "message": self.message,
                "path": "" if self._path is None else str(self._path),
            }

    def start(self, timeout: float = 1.5) -> dict[str, object]:
        with self._lock:
            self._pending_start = True
            self._pending_stop = False
            self.message = "正在开始录制…"
        result = self._wait(
            lambda item: bool(item["recording"]) or str(item["message"]).startswith("无法"),
            timeout,
        )
        result["ok"] = bool(result["recording"])
        if not result["ok"] and not str(result["message"]).startswith("无法"):
            result["message"] = "没有画面，录制没有开始"
        return result

    def stop(self, timeout: float = 1.5) -> dict[str, object]:
        with self._lock:
            self._pending_stop = True
            self._pending_start = False
        result = self._wait(lambda item: not item["recording"], timeout)
        result["ok"] = not bool(result["recording"])
        if result["ok"] and not result["message"]:
            result["message"] = "当前没有在录制"
        return result

    def apply(self, frame: np.ndarray) -> None:
        with self._lock:
            start = self._pending_start
            stop = self._pending_stop
            self._pending_start = False
            self._pending_stop = False
            writer = self._writer
        if stop or start:
            self._close()
            writer = None
        if start:
            self._open(frame)
            with self._lock:
                writer = self._writer
        if writer is not None:
            writer.write(frame)

    def close(self) -> None:
        self._close()

    def _open(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        path = recording_path(self._role, directory=self._directory)
        writer = open_recorder(path, width, height, self._fps)
        with self._lock:
            if writer is None:
                self._writer = None
                self._path = None
                self.recording = False
                self.message = f"无法开始录制：{path}"
                return
            self._writer = writer
            self._path = path
            self.recording = True
            self.message = f"正在录制 {path}"

    def _close(self) -> None:
        with self._lock:
            writer = self._writer
            path = self._path
            self._writer = None
            self.recording = False
            if writer is not None and path is not None:
                self.message = f"录制已保存 {path}"
        if writer is not None:
            writer.release()

    def _wait(self, ready, timeout: float) -> dict[str, object]:
        deadline = time.monotonic() + timeout
        item = self.status()
        while not ready(item) and time.monotonic() < deadline:
            time.sleep(0.02)
            item = self.status()
        return item


def jpeg_part(jpeg: bytes) -> bytes:
    header = (
        b"--" + BOUNDARY + b"\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n\r\n"
    )
    return header + jpeg + b"\r\n"


def handler_factory(
    hub: FrameHub,
    role: str,
    directory: Path | None = None,
    recorder: Recorder | None = None,
):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                body = page_html().encode("utf-8")
                self._send(200, "text/html; charset=utf-8", body)
                return
            if path == "/camera.mjpg":
                self._stream()
                return
            if path == "/record":
                self._record_status()
                return
            self.send_error(404)

        def do_POST(self) -> None:
            path = urlparse(self.path).path
            length = int(self.headers.get("Content-Length", "0") or 0)
            if length:
                self.rfile.read(length)
            if path == "/snap":
                ok, message = save_latest(hub, role, directory)
                self._json({"ok": ok, "message": message}, 200 if ok else 409)
                print(("已保存 " if ok else "拍照失败：") + message, flush=True)
                return
            if path == "/record/start":
                self._record_action("start")
                return
            if path == "/record/stop":
                self._record_action("stop")
                return
            self.send_error(404)

        def _record_status(self) -> None:
            if recorder is None:
                self._json({"ok": False, "recording": False, "message": "录制未启用"}, 409)
                return
            self._json({"ok": True, **recorder.status()})

        def _record_action(self, action: str) -> None:
            if recorder is None:
                self._json({"ok": False, "recording": False, "message": "录制未启用"}, 409)
                return
            result = recorder.start() if action == "start" else recorder.stop()
            self._json(result, 200 if result.get("ok") else 409)
            print(str(result.get("message", "")), flush=True)

        def _json(self, payload: dict, status: int = 200) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self._send(status, "application/json; charset=utf-8", body)

        def _send(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            seq = 0
            try:
                while True:
                    item = hub.wait_jpeg(seq, 1.0)
                    if item is None:
                        continue
                    seq, jpeg = item
                    self.wfile.write(jpeg_part(jpeg))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                return

        def log_message(self, fmt: str, *args) -> None:
            return

    return Handler


def capture_loop(
    device: str,
    width: int,
    height: int,
    fps: int,
    hub: FrameHub,
    stop: threading.Event,
    recorder: Recorder | None = None,
) -> None:
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        hub.fail(f"打不开 {device}。先关掉占用它的程序，例如全屏拍照或上位机。")
        print(hub.error, flush=True)
        return
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    actual_w = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"摄像头已打开 {device} {actual_w}×{actual_h}", flush=True)
    try:
        while not stop.is_set():
            ok, frame = capture.read()
            if not ok or frame is None:
                hub.fail("读取画面失败")
                time.sleep(0.03)
                continue
            hub.publish(frame)
            if recorder is not None:
                recorder.apply(frame)
    finally:
        if recorder is not None:
            recorder.close()
        capture.release()


def serve(
    bind: str,
    port: int,
    hub: FrameHub,
    role: str,
    recorder: Recorder | None = None,
) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((bind, port), handler_factory(hub, role, recorder=recorder))
    server.daemon_threads = True
    return server


def main() -> None:
    parser = argparse.ArgumentParser(description="浏览器查看摄像头、拍照并录制")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--role",
        default="recognition_camera",
        help="文件名前缀：recognition_camera → rec_，navigation_camera → nav_。与全屏拍照第一路相同。",
    )
    args = parser.parse_args()

    hub = FrameHub()
    recorder = Recorder(args.role, args.fps)
    stop = threading.Event()
    worker = threading.Thread(
        target=capture_loop,
        args=(args.device, args.width, args.height, args.fps, hub, stop, recorder),
        daemon=True,
    )
    worker.start()
    server = serve(args.bind, args.port, hub, args.role, recorder)
    print(f"在浏览器打开 http://<香橙派IP>:{args.port}/", flush=True)
    print("拍照或按空格存 PNG；「录制」或 R 存 MJPG/AVI。都写到 data/road/。Ctrl+C 停止。", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.shutdown()
        worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
