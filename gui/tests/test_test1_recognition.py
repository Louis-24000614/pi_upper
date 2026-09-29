"""无摄像头、无模型的测试1双服务接线回归。"""

from __future__ import annotations

import json
import os
import re
import sys
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread
from unittest.mock import patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QRect
from knife_roi import source_roi
from main_window import MainWindow


class _Handler(BaseHTTPRequestHandler):
    requests: list[tuple[str, str]] = []

    def do_POST(self) -> None:
        body = self.rfile.read(int(self.headers["Content-Length"]))
        self.requests.append((self.path, self.headers["Content-Type"]))
        if self.path.endswith("/knife/recognize"):
            def field(name: str) -> str:
                match = re.search(rb'name="' + name.encode() + rb'"\r\n\r\n([^\r]+)', body)
                return match.group(1).decode() if match else ""

            data = {"request_id": field("request_id"), "camera_epoch": int(field("camera_epoch")),
                    "top1_class": "knife_03", "top1_score": 0.82, "quality_ok": True}
        else:
            data = {"face_count": 1, "results": [
                {"name": "tester", "score": 0.71, "bbox": [10, 10, 40, 40]}
            ]}
        payload = json.dumps({"status": "success", "message": "ok", "data": data}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *_args) -> None:
        pass


class TestTest1Recognition(unittest.TestCase):
    def test_close_knife_candidates_are_visible(self) -> None:
        app = QApplication.instance() or QApplication([])
        with patch("main_window.list_cameras", return_value=[]):
            window = MainWindow()
        try:
            window.test1_running = True
            window.pending_knife_id = "field-knife"
            window._on_knife_result({
                "request_id": "field-knife",
                "camera_epoch": window.camera_epoch,
                "top1_class": "knife_01",
                "top1_score": 0.6918,
                "top2_class": "knife_02",
                "top2_score": 0.6627,
                "margin": 0.0291,
                "preprocess": {"foreground_ratio": 0.058},
            })
            self.assertIn("次选 knife_02", window.run_knife.text())
            self.assertIn("易混淆", window.run_knife.text())
            self.assertIn("可能漏掉刀刃", window.run_knife.text())
            window.pending_knife_id = "field-paper"
            window._on_knife_result({
                "request_id": "field-paper",
                "camera_epoch": window.camera_epoch,
                "top1_class": "knife_08",
                "top1_score": 0.612,
                "top2_class": "knife_07",
                "top2_score": 0.598,
                "margin": 0.014,
                "preprocess": {"foreground_ratio": 0.668},
            })
            self.assertIn("可能包含背景", window.run_knife.text())
            app.processEvents()
        finally:
            window.close()

    def test_roi_maps_letterboxed_preview_to_original_frame(self) -> None:
        image_rect = QRect(100, 50, 640, 360)
        selection = QRect(300, 150, 200, 100)
        self.assertEqual(source_roi(selection, image_rect, 1280, 720), (400, 200, 800, 400))

    def test_display_switch_keeps_roi_only_on_knife_input(self) -> None:
        app = QApplication.instance() or QApplication([])
        with patch("main_window.list_cameras", return_value=[]):
            window = MainWindow()
        try:
            window.role_sources.update(recognition_camera="rec", navigation_camera="nav")
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            window._on_frame_ready("rec", frame, 30.0)
            window._on_frame_ready("nav", np.zeros_like(frame), 30.0)
            window.knife_roi = (380, 130, 940, 570)
            window.test1_running = True
            window.test1_started_at = time.monotonic()
            window._switch_display()
            with patch.object(window.knife_client, "submit", return_value=True) as knife_submit, \
                    patch.object(window.face_client, "submit", return_value=True) as face_submit:
                window._recognition_tick()
            self.assertEqual(window.display_role, "navigation_camera")
            self.assertEqual(knife_submit.call_args.args[0].shape, (440, 560, 3))
            self.assertIs(face_submit.call_args.args[0], frame)
            app.processEvents()
        finally:
            window.close()

    def test_parallel_requests_and_stop_invalidates_results(self) -> None:
        app = QApplication.instance() or QApplication([])
        _Handler.requests = []
        server = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with patch("main_window.list_cameras", return_value=[]):
                window = MainWindow()
            base = f"http://127.0.0.1:{server.server_port}"
            window.knife_client.endpoint = base + "/api/v1/knife/recognize"
            window.face_client.endpoint = base + "/api/v1/recognize"
            window.role_sources["recognition_camera"] = "synthetic"
            frame = np.zeros((80, 80, 3), dtype=np.uint8)
            window._on_frame_ready("synthetic", frame, 30.0)
            window._start_test1()
            window._on_frame_ready("synthetic", frame, 30.0)
            window._recognition_tick()
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                app.processEvents()
                if "knife_03" in window.run_knife.text() and "tester" in window.run_face.text():
                    break
                time.sleep(0.02)
            self.assertIn("knife_03", window.run_knife.text())
            self.assertIn("tester", window.run_face.text())
            self.assertEqual(len(_Handler.requests), 2)
            types = {path: content_type for path, content_type in _Handler.requests}
            self.assertIn("multipart/form-data", types["/api/v1/knife/recognize"])
            self.assertIn("multipart/form-data", types["/api/v1/recognize"])
            window._stop_test1()
            window._on_face_result({"request_id": "old", "data": {"results": [{"name": "stale"}]}})
            self.assertEqual(window.run_face.text(), "未启动")
            window.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
