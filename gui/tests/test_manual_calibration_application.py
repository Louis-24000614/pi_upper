"""应用/恢复与真实导航投影入口的离线集成测试；写入仅在系统临时目录。"""
import copy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
for folder in (ROOT, ROOT / "gui", ROOT / "navigation", ROOT / "vision"):
    sys.path.insert(0, str(folder))
from manual_calibration import navigation_settings
from manual_calibration_web import CalibrationSession, encoded, handler_factory
from test_manual_calibration import checker_frame, parameters
from vision.ipm_proto import manual_ground as ground
from road_follow.pipeline import make_ipm
from road_follow.culvert_perception import CulvertCalibration
from road_follow.culvert_runtime import validate_culvert_config


class ApplicationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="manual_application_")
        self.directory = Path(self.temp.name) / "results"
        self.patch_directory = patch.object(ground, "DIRECTORY", self.directory)
        self.patch_directory.start(); ground.runtime_activation.cache_clear()
        self.settings = navigation_settings(ROOT / "config/nav_camera.yaml")
        self.cfg = yaml.safe_load((ROOT / "config/nav_camera.yaml").read_bytes())
        self.session = CalibrationSession(self.settings, self.directory, offline=True)
        self.frame, self.board_to_image, _ = checker_frame()
        self.values = None

    def tearDown(self):
        ground.runtime_activation.cache_clear(); self.patch_directory.stop(); self.temp.cleanup()

    def compute(self, *, vehicle=True, width=1280, height=720):
        frame = self.frame if (width, height) == (1280, 720) else cv2.resize(self.frame, (width, height))
        captured = self.session.upload(encoded(frame, ".png"), "navigation_raw.png")
        params = parameters()
        if vehicle:
            params["vehicle"] = {"axes_aligned": True, "unit": "cm", "x": 3, "y": 40}
        if width != 1280:
            params["image_points"] = (np.array(params["image_points"])*[width/1280, height/720]).tolist()
        values = {"frame_id": captured["frame_id"], "revision": 4, "parameters": params}
        result = self.session.compute(values)
        self.values = {**values, "result_id": result["result_id"]}
        return self.values

    def check(self, board=(350, 300), *, error=(0, 0)):
        point = cv2.perspectiveTransform(np.array([[board]], np.float64), self.board_to_image)[0, 0]
        # 由合成板的已知 30 mm/100 px 尺度得到实测值，不从拟合 H 反求期望值。
        actual = np.array([.03+(board[0]-350)*.0003, .4-(board[1]-300)*.0003])
        values = {**self.values, "image_point": point.tolist(),
                  "measured": {"unit": "cm", "x": (actual[0]+error[0])*100, "y": (actual[1]+error[1])*100}}
        return self.session.check_point(values), point, actual

    def apply_values(self, **changes):
        active = ground.read_activation()
        return {**self.values, "measurement_reviewed": True, "image_source_confirmed": True,
                "ground_contact_verified": True, "expected_applied_at": active["applied_at"] if active else None, **changes}

    def test_actual_navigation_and_culvert_share_vehicle_matrix_and_check_errors(self):
        self.compute()
        response, point, actual = self.check(board=(360, 320), error=(.01, -.02))
        np.testing.assert_allclose(response["vehicle_ground_m"], actual, atol=1e-8)
        np.testing.assert_allclose(response["checks"][0]["error_xy_m"], [-.01, .02], atol=1e-8)
        self.assertAlmostEqual(response["checks"][0]["error_m"], np.hypot(.01, .02))
        config_bytes = {p: (ROOT / p).read_bytes() for p in ("config/nav_camera.yaml", "config/culvert.yaml")}
        selected = self.session.apply(self.apply_values())["active_calibration"]
        ipm = make_ipm(self.cfg, self.frame.shape)
        culvert = CulvertCalibration.from_manual_activation(ground.runtime_activation(), self.cfg, (1280, 720))
        np.testing.assert_allclose(ipm.image_points_to_ground([point]), [actual], atol=1e-8)
        np.testing.assert_allclose(culvert.ipm.H_img_to_bev, ipm.H_img_to_bev, atol=1e-10)
        np.testing.assert_allclose(culvert.ipm.image_points_to_ground([point]), [actual], atol=1e-8)
        self.assertEqual(culvert.mode, "manual_vehicle_ground")
        self.assertEqual(culvert.metadata["calibration_file"], selected["calibration_file"])
        for p, data in config_bytes.items(): self.assertEqual((ROOT / p).read_bytes(), data)
        # 原始独立保存仍保持候选身份；应用事实在 active.json 中记录。
        candidate = json.loads((self.directory / selected["candidate_file"]).read_text())
        self.assertFalse(candidate["applied"]); self.assertFalse(candidate["verified"])

    def test_no_selected_result_preserves_legacy_and_running_snapshot_does_not_switch(self):
        before = make_ipm(self.cfg, self.frame.shape).H_img_to_bev.copy()
        self.assertIsNone(ground.runtime_activation())
        self.compute(); self.check(); first = self.session.apply(self.apply_values())["active_calibration"]
        np.testing.assert_allclose(make_ipm(self.cfg, self.frame.shape).H_img_to_bev, before)
        ground.runtime_activation.cache_clear()  # 模拟重启导航。
        active_matrix = make_ipm(self.cfg, self.frame.shape).H_img_to_bev.copy()
        self.assertFalse(np.allclose(active_matrix, before))
        self.session.restore({"expected_applied_at": first["applied_at"]})
        self.assertFalse((self.directory / "active.json").exists())
        np.testing.assert_allclose(make_ipm(self.cfg, self.frame.shape).H_img_to_bev, active_matrix)
        ground.runtime_activation.cache_clear()
        np.testing.assert_allclose(make_ipm(self.cfg, self.frame.shape).H_img_to_bev, before)

    def test_apply_and_restore_chain_preserves_every_saved_result(self):
        self.compute(); self.check(); first = self.session.apply(self.apply_values())["active_calibration"]
        first_bytes = (self.directory / "active.json").read_bytes()
        candidate_bytes = (self.directory / first["candidate_file"]).read_bytes()
        self.compute(); self.check(board=(380, 260)); second = self.session.apply(self.apply_values())["active_calibration"]
        with self.assertRaises(ValueError): self.session.restore({"expected_applied_at": first["applied_at"]})
        restored = self.session.restore({"expected_applied_at": second["applied_at"]})["active_calibration"]
        self.assertEqual(restored, first); self.assertEqual((self.directory / "active.json").read_bytes(), first_bytes)
        self.assertEqual((self.directory / first["candidate_file"]).read_bytes(), candidate_bytes)
        self.session.restore({"expected_applied_at": first["applied_at"]})
        self.assertIsNone(ground.read_activation()); self.assertGreaterEqual(len(list((self.directory / "activation_history").iterdir())), 3)

    def test_region_only_unchecked_missing_checks_wrong_resolution_and_stale_rejected(self):
        self.compute(vehicle=False)
        with self.assertRaisesRegex(ValueError, "车辆坐标"): self.session.apply(self.apply_values())
        self.compute()
        with self.assertRaisesRegex(ValueError, "至少"): self.session.apply(self.apply_values())
        self.check()
        for changes in ({"measurement_reviewed": False}, {"image_source_confirmed": False}, {"expected_applied_at": "old"}):
            with self.assertRaises(ValueError): self.session.apply(self.apply_values(**changes))
        stale = self.apply_values(); self.session.invalidate({"frame_id": self.values["frame_id"], "revision": 5})
        with self.assertRaises(ValueError): self.session.apply(stale)
        self.compute(width=640, height=360)
        with self.assertRaisesRegex(ValueError, "分辨率"): self.session.apply(self.apply_values())
        self.assertFalse((self.directory / "active.json").exists())

    def test_contact_gate_size_device_and_corrupt_files_fail_without_fallback(self):
        self.compute(); self.check(); self.session.apply(self.apply_values(ground_contact_verified=False))
        active = ground.runtime_activation()
        with self.assertRaisesRegex(ValueError, "底边"): CulvertCalibration.from_manual_activation(active, self.cfg, (1280, 720))
        with self.assertRaises(ValueError): make_ipm(self.cfg, (480, 640))
        changed = copy.deepcopy(self.cfg); changed["capture"]["device"] = "/dev/other"
        with self.assertRaises(ValueError): make_ipm(changed, self.frame.shape)
        path = self.directory / active["candidate_file"]; path.write_text("{}")
        ground.runtime_activation.cache_clear()
        with self.assertRaisesRegex(ValueError, "变化"): make_ipm(self.cfg, self.frame.shape)

    def test_fitting_corners_invalid_checks_and_temporary_output_cannot_apply(self):
        self.compute()
        for point in ([-1, 5], [1280, 1], [float("nan"), 1], [4], self.values["parameters"]["image_points"][0]):
            with self.assertRaises(ValueError): self.session.check_point({**self.values, "image_point": point, "measured": {"unit": "cm", "x": 0, "y": 40}})
        response, _, _ = self.check(); self.check()
        self.assertEqual(len(self.session.result["checks"]), 1)
        self.session.directory = self.directory.parent / "custom_output"
        self.assertFalse(self.session.status()["can_apply"])
        with self.assertRaisesRegex(ValueError, "临时"): self.session.apply(self.apply_values())

    def test_config_changed_after_freeze_and_failed_atomic_publish_preserve_old_selection(self):
        self.compute(); self.check(); first = self.session.apply(self.apply_values())["active_calibration"]
        original = (self.directory / "active.json").read_bytes()
        altered = copy.deepcopy(self.settings); altered["baseline"]["config_sha256"] = "changed"
        with patch("manual_calibration_web.navigation_settings", return_value=altered):
            with self.assertRaisesRegex(ValueError, "配置"): self.session.apply(self.apply_values())
        with patch.object(ground.os, "replace", side_effect=OSError("模拟发布失败")):
            with self.assertRaises(OSError): self.session.apply(self.apply_values())
        self.assertEqual((self.directory / "active.json").read_bytes(), original)
        self.assertEqual(ground.read_activation(), first)
        self.assertEqual(list(self.directory.glob("*.pending")), [])

    def test_culvert_preflight_prioritizes_active_even_when_estimated_flag_is_present(self):
        self.compute(); self.check(); self.session.apply(self.apply_values())
        with tempfile.TemporaryDirectory(prefix="culvert_model_stub_") as folder:
            root = Path(folder); (root / "model.rknn").write_bytes(b"test model only")
            import hashlib
            fingerprint = hashlib.sha256(b"test model only").hexdigest()
            config = {"model": {"path": "model.rknn", "sha256": fingerprint, "verify_sha256": True, "classes": {"count": 4}},
                      "class_mapping": {"verified": True, "model_sha256": fingerprint, "culvert_class_id": 3, "hard_block_class_ids": [0, 1, 2]},
                      "detect": {"class_aware_nms": True}, "calibration": {"verified": False}}
            path = root / "culvert.yaml"; path.write_text(yaml.safe_dump(config), encoding="utf-8")
            for estimated in (False, True):
                setup = validate_culvert_config(path, root, drive=True, image_size=(1280, 720), near_speed=.05,
                                                estimated_camera=estimated, nav_config=self.cfg)
                self.assertEqual(setup[2].mode, "manual_vehicle_ground")

    def test_web_can_restore_corrupt_current_selection_with_optimistic_token(self):
        self.compute(); self.check(); self.session.apply(self.apply_values())
        active_path = self.directory / "active.json"
        active_path.write_text("broken JSON", encoding="utf-8")
        state = self.session.status()
        self.assertIsNotNone(state["application_error"]); self.assertIsNotNone(state["restore_token"])
        with self.assertRaises(ValueError): self.session.restore({"restore_token": "stale"})
        self.assertIsNone(self.session.restore({"restore_token": state["restore_token"]})["active_calibration"])
        self.assertFalse(active_path.exists())

    def test_navigation_preflight_rejects_corrupt_selection_before_any_hardware(self):
        from road_follow import __main__ as entry
        import contextlib
        import io
        self.directory.mkdir(); (self.directory / "active.json").write_text("broken JSON")
        with patch.object(entry, "RoadSegmenter", side_effect=AssertionError("不得加载模型")), \
             patch.object(entry.cv2, "VideoCapture", side_effect=AssertionError("不得打开相机")), \
             patch.object(entry, "needs_frequency_guard", side_effect=AssertionError("不得改动设备频率")), \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
            entry.main(["--heap-trim-interval", "0"])
        self.assertEqual(raised.exception.code, 2)

    def test_http_check_apply_restore_and_stale_check_never_open_camera(self):
        self.compute()
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(self.session, threading.Event()))
        worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
        def post(path, values):
            request = Request(f"http://127.0.0.1:{server.server_port}"+path, data=json.dumps(values).encode(), headers={"Content-Type": "application/json"})
            with urlopen(request, timeout=3) as response: return json.load(response)
        try:
            with patch("manual_calibration_web.cv2.VideoCapture", side_effect=AssertionError("不得打开相机")):
                _, point, actual = self.check()
                response = post("/check-point", {**self.values, "image_point": point.tolist(), "measured": {"unit": "mm", "x": actual[0]*1000, "y": actual[1]*1000}})
                self.assertEqual(len(response["checks"]), 1)
                selected = post("/apply", self.apply_values())["active_calibration"]
                self.assertIsNone(post("/restore", {"expected_applied_at": selected["applied_at"]})["active_calibration"])
                self.session.invalidate({"frame_id": self.values["frame_id"], "revision": 5})
                with self.assertRaises(HTTPError) as raised: post("/check-point", {**self.values, "image_point": point.tolist()})
                self.assertEqual(raised.exception.code, 409); raised.exception.close()
        finally:
            server.shutdown(); server.server_close(); worker.join(2)
