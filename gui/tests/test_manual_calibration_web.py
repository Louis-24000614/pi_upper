"""本机 HTTP、草稿时序、独立保存；输出仅位于系统临时目录。"""
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual_calibration import navigation_settings, validate_record
from manual_calibration_web import CalibrationSession, encoded, handler_factory, main
from test_manual_calibration import checker_frame, parameters


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="manual_calibration_test_")
        self.root = Path(self.temp.name)
        settings = navigation_settings(Path(__file__).resolve().parents[2] / "config/nav_camera.yaml")
        self.session = CalibrationSession(settings, self.root / "results", offline=True)
        self.frame = checker_frame()[0]

    def tearDown(self):
        self.temp.cleanup()

    def upload(self):
        return self.session.upload(encoded(self.frame, ".png"), "raw.png")

    def compute(self, revision=4, params=None):
        capture = self.upload()
        values = {"frame_id": capture["frame_id"], "revision": revision, "parameters": params or parameters()}
        result = self.session.compute(values)
        return {**values, "result_id": result["result_id"]}, result

    def test_save_complete_artifacts_unique_directories_and_idempotent_current_result(self):
        values, result = self.compute()
        first = self.session.save(values)
        self.assertEqual(first, self.session.save(values))
        path = Path(first["path"])
        self.assertEqual(set(p.name for p in path.parent.iterdir()), {"raw.png", "corners.png", "bev.png", "calibration.json"})
        np.testing.assert_array_equal(cv2.imread(str(path.parent / "raw.png")), self.frame)
        record = json.loads(path.read_text(encoding="utf-8")); validate_record(record)
        self.assertEqual(record["corner_order"][0], "region_top_left")
        self.assertIsNone(record["vehicle_candidate_file"])
        self.assertFalse(record["applied"])
        original = path.read_bytes()
        values, _ = self.compute(); second = self.session.save(values)
        self.assertNotEqual(first["path"], second["path"])
        self.assertEqual(path.read_bytes(), original)

    def test_vehicle_coordinates_saved_separately_not_as_region_origin(self):
        params = parameters(); params["vehicle"] = {"axes_aligned": True, "unit": "cm", "x": 3, "y": 40}
        values, _ = self.compute(params=params)
        saved = Path(self.session.save(values)["path"])
        record = json.loads(saved.read_text(encoding="utf-8"))
        candidate = json.loads((saved.parent / "vehicle_candidate.json").read_text(encoding="utf-8"))
        self.assertEqual(record["coordinate_reference"], "selected_region_center")
        self.assertEqual(candidate["region_center_m"], [.03, .4])
        self.assertFalse(candidate["verified"])

    def test_upload_and_failed_recapture_invalidate_old_tokens(self):
        values, _ = self.compute()
        self.upload()
        for call in (lambda: self.session.save(values), lambda: self.session.image("bev", values["result_id"]), lambda: self.session.compute(values)):
            with self.assertRaises(ValueError): call()
        values, _ = self.compute()
        with self.assertRaises(ValueError): self.session.capture()
        self.assertIsNone(self.session.frozen)
        with self.assertRaises(ValueError): self.session.save(values)
        values, _ = self.compute()
        with self.assertRaises(ValueError): self.session.upload(b"not an image", "bad.png")
        with self.assertRaises(ValueError): self.session.save(values)

    def test_revision_reordering_invalid_input_and_parameter_changes_reject_stale_save(self):
        values, _ = self.compute()
        self.session.invalidate({"frame_id": values["frame_id"], "revision": 5})
        with self.assertRaises(ValueError): self.session.compute(values)
        with self.assertRaises(ValueError): self.session.save(values)
        values["revision"] = 6; result = self.session.compute(values); values["result_id"] = result["result_id"]
        with self.assertRaises(ValueError): self.session.invalidate({"frame_id": values["frame_id"], "revision": 3})
        self.assertEqual(self.session.result["id"], values["result_id"])
        altered = copy.deepcopy(values); altered["parameters"]["dimensions"]["cell_size"] = 31
        with self.assertRaises(ValueError): self.session.save(altered)
        self.assertIsNone(self.session.result)
        values, _ = self.compute(); bad = copy.deepcopy(values); bad["parameters"]["image_points"] = []
        with self.assertRaises(ValueError): self.session.compute(bad)
        with self.assertRaises(ValueError): self.session.save(values)

    def test_tampered_result_and_save_failure_do_not_leave_successful_records(self):
        values, _ = self.compute(); self.session.result["record"]["H_img_to_region_ground_m"][0][0] += 1
        with self.assertRaises(ValueError): self.session.save(values)
        self.assertFalse(self.session.directory.exists())
        values, _ = self.compute()
        with patch("manual_calibration_web.encoded", side_effect=OSError("模拟磁盘写入失败")):
            with self.assertRaises(OSError): self.session.save(values)
        self.assertIsNone(self.session.result["path"])
        self.assertEqual(list(self.session.directory.iterdir()), [])

    def test_live_freeze_retains_original_resolution_and_pixels(self):
        self.session.offline = False; self.session.publish(self.frame)
        frozen = self.session.capture()
        np.testing.assert_array_equal(self.session.frozen["frame"], self.frame)
        self.assertEqual(frozen["image_size"], [1280, 720])
        self.assertEqual(frozen["source"]["device"], self.session.settings["device"])
        self.session.publish(cv2.resize(self.frame, (640, 360)))
        with self.assertRaises(ValueError): self.session.capture()

    def test_upload_other_resolution_keeps_native_pixels_and_marks_mismatch(self):
        small = cv2.resize(self.frame, (640, 360))
        result = self.session.upload(encoded(small, ".png"), "C:/photos/raw.png")
        self.assertEqual(result["image_size"], [640, 360])
        self.assertEqual(result["source"]["filename"], "raw.png")
        self.assertFalse(result["source"]["matches_navigation_resolution"])
        np.testing.assert_array_equal(self.session.frozen["frame"], small)


class HttpTest(unittest.TestCase):
    def setUp(self):
        SessionTest.setUp(self)
        self.stop = threading.Event()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler_factory(self.session, self.stop))
        self.server.daemon_threads = True
        self.worker = threading.Thread(target=self.server.serve_forever, daemon=True); self.worker.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.stop.set(); self.server.shutdown(); self.server.server_close(); self.worker.join(2)
        SessionTest.tearDown(self)

    def request(self, path, values=None, raw=None, origin=None):
        headers = {"Content-Type": "application/json"}
        if origin: headers["Origin"] = origin
        body = raw if raw is not None else json.dumps(values).encode() if values is not None else None
        return urlopen(Request(self.url+path, data=body, headers=headers), timeout=3)

    def test_http_offline_upload_compute_preview_save_and_stale_rejection(self):
        with patch("manual_calibration_web.cv2.VideoCapture", side_effect=AssertionError("不得打开相机")):
            with self.request("/") as r: self.assertIn("不是内角点数量", r.read().decode())
            with self.request("/status") as r: self.assertTrue(json.load(r)["offline"])
            with self.request("/upload?filename=raw.png", raw=encoded(self.frame, ".png")) as r: freeze = json.load(r)
            values = {"frame_id": freeze["frame_id"], "revision": 4, "parameters": parameters()}
            with self.request("/compute", values) as r: result = json.load(r)
            with self.request("/bev.png?id="+result["result_id"]) as r: self.assertEqual(r.read(8), b"\x89PNG\r\n\x1a\n")
            with self.request("/save", {**values, "result_id": result["result_id"]}) as r: self.assertTrue(Path(json.load(r)["path"]).is_file())
            with self.request("/invalidate", {"frame_id": freeze["frame_id"], "revision": 5}): pass
            with self.assertRaises(HTTPError) as raised: self.request("/save", {**values, "result_id": result["result_id"]})
            self.assertEqual(raised.exception.code, 409)
            raised.exception.close()

    def test_cross_origin_malformed_and_unknown_requests(self):
        with self.assertRaises(HTTPError) as raised: self.request("/capture", {}, origin="http://unrelated.invalid")
        self.assertEqual(raised.exception.code, 403)
        raised.exception.close()
        for path, raw in (("/compute", b"not JSON"), ("/camera.mjpg", None), ("/missing", None)):
            with self.assertRaises(HTTPError) as raised: self.request(path, raw=raw)
            raised.exception.close()

    def test_synthetic_live_freeze_replaces_previous_frame_and_computation(self):
        self.session.offline = False
        self.session.publish(self.frame)
        with self.request("/capture", {}) as r: first = json.load(r)
        values = {"frame_id": first["frame_id"], "revision": 4, "parameters": parameters()}
        with self.request("/compute", values) as r: result = json.load(r)
        with self.request("/capture", {}) as r: second = json.load(r)
        self.assertNotEqual(first["frame_id"], second["frame_id"])
        with self.request("/frozen.png?id="+second["frame_id"]) as r:
            decoded = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(decoded, self.frame)
        with self.assertRaises(HTTPError) as raised: self.request("/save", {**values, "result_id": result["result_id"]})
        raised.exception.close()


class OfflineEntryTest(unittest.TestCase):
    def test_offline_entry_never_opens_camera_or_starts_capture_worker(self):
        with tempfile.TemporaryDirectory(prefix="manual_offline_entry_") as directory:
            with patch("manual_calibration_web.cv2.VideoCapture", side_effect=AssertionError("不得打开相机")), \
                 patch("manual_calibration_web.capture_loop", side_effect=AssertionError("不得启动采集")), \
                 patch("manual_calibration_web.ThreadingHTTPServer") as server, \
                 patch("manual_calibration_web.signal.signal"):
                server.return_value.server_port = 8082
                self.assertEqual(main(["--offline", "--output", directory+"/results"]), 0)
                server.return_value.serve_forever.assert_called_once()
            self.assertFalse((Path(directory)/"results").exists())


if __name__ == "__main__":
    unittest.main()
