"""真实本机 HTTP 与合成相机；不打开任何硬件。"""
import json
from pathlib import Path
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from road_follow.inspection_config import Settings
from road_follow.inspection_io import FrameHub
from road_follow.inspection_web import InspectionWeb, PAGE


class WebTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path = Path(self.tmp.name)/"settings.json"
        path.write_bytes((Path(__file__).with_name("fixtures")/"culvert_inspection.json").read_bytes())
        self.settings, self.hub = Settings(path), FrameHub()
        self.hub.publish(np.full((100, 200, 3), 120, np.uint8), time.monotonic())
        state = {"phase": "recognizing", "side": "A", "candidate": None, "sides": [],
                 "valid_frames": 1, "remaining_s": 40, "speech_enabled": False}
        self.web = InspectionWeb(self.settings, self.hub, lambda: state,
                                 map_status=lambda: {"edge": {"status": "partial"}}, address=("127.0.0.1", 0))
        self.addCleanup(self.web.close)
        self.url = "http://127.0.0.1:"+str(self.web.server.server_port)

    def get(self, path):
        return urlopen(self.url+path, timeout=2)

    def post(self, payload, path="/settings"):
        request = Request(self.url+path, data=json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json"}, method="POST")
        return urlopen(request, timeout=2)

    def test_page_has_outer_roi_and_independent_thresholds(self):
        with self.get("/") as response:
            page = response.read().decode()
        self.assertIn("底部板卡", page)
        self.assertIn('id="face_threshold"', page)
        self.assertIn('id="knife_threshold"', page)
        self.assertIn("getBoundingClientRect", page)
        self.assertNotIn("前景提取阈值", page)

    def test_apply_save_restart_and_crop_preview(self):
        with self.post({"revision": 0, "values": {"roi": [.1, .2, .9, .8], "knife_threshold": .6}}) as response:
            self.assertEqual(json.load(response)["revision"], 1)
        with self.get("/crop.jpg") as response:
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape[:2], (60, 160))
        saved, _ = Settings(self.settings.path).snapshot()
        self.assertEqual(saved["recognition"]["knife_threshold"], .6)

    def test_invalid_input_and_stale_revision_do_not_overwrite(self):
        with self.assertRaises(HTTPError) as error:
            self.post({"revision": 0, "values": {"face_threshold": .1}})
        self.assertEqual(error.exception.code, 400)
        error.exception.close()
        with self.post({"revision": 0, "values": {"confirm_frames": 4}}):
            pass
        with self.assertRaises(HTTPError) as error:
            self.post({"revision": 0, "values": {"confirm_frames": 5}})
        self.assertEqual(error.exception.code, 409)
        error.exception.close()
        config, revision = self.settings.snapshot()
        self.assertEqual(config["recognition"]["confirm_frames"], 4)
        self.assertEqual(revision, 1)

    def test_idle_stream_client_does_not_block_status_or_settings(self):
        stream = self.get("/camera.mjpg")
        self.addCleanup(stream.close)
        started = time.monotonic()
        with self.get("/status") as response:
            payload = json.load(response)
        self.assertLess(time.monotonic()-started, 1)
        self.assertEqual(payload["map"]["edge"]["status"], "partial")
        with self.post({"revision": 0, "values": {"knife_threshold": .7}}) as response:
            self.assertEqual(json.load(response)["revision"], 1)

    def test_old_camera_frame_not_used_for_crop(self):
        self.hub.fail("camera failed")
        with self.assertRaises(HTTPError) as error:
            self.get("/crop.jpg")
        self.assertEqual(error.exception.code, 503)
        error.exception.close()

    def test_page_uses_chinese_cards_tables_and_map_instead_of_json_dump(self):
        with self.get("/") as response:page=response.read().decode()
        for text in ("当前候选目标","当前使用的识别参数","两侧检查结果","障碍判断调参","本次任务拓扑地图","侧视相机没有新鲜帧"):
            self.assertIn(text,page)
        self.assertNotIn("<pre",page)
        self.assertNotIn("JSON.stringify(s",page)
        with self.get("/topology.svg") as response:
            self.assertIn("image/svg+xml",response.headers["Content-Type"])
            self.assertIn("导航任务",response.read().decode())

    def test_obstacle_settings_save_and_conflict_are_separate_from_recognition(self):
        with self.post({"revision":0,"values":{"edge_end_margin_m":.12}},"/obstacle-settings") as response:
            self.assertEqual(json.load(response)["revision"],1)
        with self.get("/status") as response:state=json.load(response)
        self.assertEqual(state["obstacle"]["settings"]["edge_end_margin_m"],.12)
        self.assertEqual(state["revision"],0)
        with self.assertRaises(HTTPError) as error:
            self.post({"revision":0,"values":{"edge_end_margin_m":.2}},"/obstacle-settings")
        self.assertEqual(error.exception.code,409)
        error.exception.close()
        self.assertEqual(Settings(self.settings.path).obstacle_snapshot()[0]["edge_end_margin_m"],.12)


if __name__ == "__main__":
    unittest.main()
