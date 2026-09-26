"""浏览器拍照和录制：页面、落盘与 MJPEG 分片（无硬件）。"""

import json
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.request import Request, urlopen

import numpy as np

from webcam_view import FrameHub, Recorder, handler_factory, jpeg_part, page_html, save_latest


def test_page_offers_snapshot_and_stream() -> None:
    html = page_html()
    assert "拍照" in html
    assert "录制" in html
    assert "/camera.mjpg" in html
    assert "/snap" in html
    assert "/record" in html


def test_save_latest_writes_navigation_png(tmp_path: Path) -> None:
    hub = FrameHub()
    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    frame[1, 2] = (0, 255, 0)
    hub.publish(frame, now=1.0)
    ok, message = save_latest(hub, "navigation_camera", tmp_path)
    assert ok is True
    dest = Path(message)
    assert dest.parent == tmp_path
    assert dest.name.startswith("nav_")
    assert dest.suffix == ".png"
    assert dest.is_file()


def test_save_latest_without_frame_fails(tmp_path: Path) -> None:
    ok, message = save_latest(FrameHub(), "navigation_camera", tmp_path)
    assert ok is False
    assert "没有可用画面" in message


def test_jpeg_part_is_one_multipart_chunk() -> None:
    part = jpeg_part(b"\xff\xd8\xff")
    assert part.startswith(b"--frame\r\n")
    assert b"Content-Type: image/jpeg\r\n" in part
    assert b"Content-Length: 3\r\n" in part
    assert part.endswith(b"\xff\xd8\xff\r\n")


def test_snap_endpoint_saves_current_frame(tmp_path: Path) -> None:
    hub = FrameHub()
    frame = np.zeros((6, 10, 3), dtype=np.uint8)
    hub.publish(frame, now=1.0)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(hub, "navigation_camera", tmp_path))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    port = server.server_address[1]
    try:
        page = urlopen(f"http://127.0.0.1:{port}/", timeout=2).read().decode("utf-8")
        assert "拍照" in page
        response = urlopen(
            Request(f"http://127.0.0.1:{port}/snap", data=b"", method="POST"),
            timeout=2,
        )
        payload = json.loads(response.read().decode("utf-8"))
        assert payload["ok"] is True
        dest = Path(payload["message"])
        assert dest.is_file()
        assert dest.parent == tmp_path
    finally:
        server.shutdown()
        thread.join(timeout=2.0)


def _pump(recorder: Recorder, frame: np.ndarray, stop: threading.Event) -> threading.Thread:
    def run() -> None:
        while not stop.is_set():
            recorder.apply(frame)
            time.sleep(0.01)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def test_record_without_frames_does_not_start(tmp_path: Path) -> None:
    result = Recorder("recognition_camera", 10, tmp_path).start(timeout=0.2)
    assert result["ok"] is False
    assert "没有画面" in str(result["message"])


def test_record_start_stop_writes_avi(tmp_path: Path) -> None:
    recorder = Recorder("recognition_camera", 10, tmp_path)
    frame = np.zeros((12, 16, 3), dtype=np.uint8)
    stop = threading.Event()
    thread = _pump(recorder, frame, stop)
    try:
        started = recorder.start(timeout=1.0)
        assert started["ok"] is True
        dest = Path(str(started["path"]))
        assert dest.name.startswith("rec_")
        assert dest.suffix == ".avi"
        stopped = recorder.stop(timeout=1.0)
        assert stopped["ok"] is True
        assert dest.is_file()
        assert dest.stat().st_size > 0
        assert "录制已保存" in str(stopped["message"])
    finally:
        stop.set()
        thread.join(timeout=1.0)
        recorder.close()


def test_record_endpoint_saves_avi(tmp_path: Path) -> None:
    hub = FrameHub()
    frame = np.zeros((12, 16, 3), dtype=np.uint8)
    hub.publish(frame, now=1.0)
    recorder = Recorder("recognition_camera", 10, tmp_path)
    stop = threading.Event()
    pump = _pump(recorder, frame, stop)
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0),
        handler_factory(hub, "recognition_camera", tmp_path, recorder),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    port = server.server_address[1]
    try:
        started = json.loads(
            urlopen(Request(f"http://127.0.0.1:{port}/record/start", data=b"", method="POST"), timeout=2).read()
        )
        assert started["ok"] is True
        dest = Path(started["path"])
        stopped = json.loads(
            urlopen(Request(f"http://127.0.0.1:{port}/record/stop", data=b"", method="POST"), timeout=2).read()
        )
        assert stopped["ok"] is True
        assert dest.is_file()
        assert dest.stat().st_size > 0
    finally:
        stop.set()
        pump.join(timeout=1.0)
        recorder.close()
        server.shutdown()
        thread.join(timeout=2.0)
