"""无硬件验证：透视图像、标签旋转、持久化及测试2相机接线。"""

from __future__ import annotations

import copy
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from apriltag_calibration import (PREVIEW_BEV, detect_tag, load_calibration,
                                 save_calibration, solve_calibration, validate_calibration)


def marker_frame():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 0, 160)
    frame = np.full((480, 640), 255, dtype=np.uint8)
    frame[160:320, 240:400] = marker
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


class TestCalibration(unittest.TestCase):
    def record(self):
        return solve_calibration([[220, 170], [410, 185], [390, 350], [240, 330]],
                                 (640, 480), PREVIEW_BEV, "synthetic-nav")

    def test_known_square_has_metric_scale_and_forward_axis(self):
        record = self.record()
        matrix = np.asarray(record["H_img_to_bev"])
        warped = cv2.perspectiveTransform(np.asarray(record["image_points"], dtype=float).reshape(-1, 1, 2), matrix).reshape(4, 2)
        np.testing.assert_allclose(warped, [[43.3, 33.3], [56.7, 33.3], [56.7, 46.7], [43.3, 46.7]], atol=1e-5)
        self.assertEqual(record["bev_size"], [100, 80])
        self.assertEqual(record["coordinate_reference"], "tag_center")
        self.assertEqual(record["image_space"], "raw_distorted")

    def test_rotated_marker_keeps_decoded_corner_order(self):
        frame = marker_frame()
        original = detect_tag(frame)
        for code in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE):
            detected = detect_tag(cv2.rotate(frame, code))
            if code == cv2.ROTATE_90_CLOCKWISE:
                expected = np.column_stack((479 - original[:, 1], original[:, 0]))
            elif code == cv2.ROTATE_180:
                expected = np.column_stack((639 - original[:, 0], 479 - original[:, 1]))
            else:
                expected = np.column_stack((original[:, 1], 639 - original[:, 0]))
            np.testing.assert_allclose(detected, expected, atol=0.5)

    def test_perspective_marker_recovers_known_corners(self):
        frame = marker_frame()
        original = detect_tag(frame)
        wanted = np.array([[200, 120], [405, 145], [380, 350], [230, 310]], dtype=np.float32)
        warp = cv2.getPerspectiveTransform(original.astype(np.float32), wanted)
        distorted = cv2.warpPerspective(frame, warp, (640, 480), borderValue=(255, 255, 255))
        detected = detect_tag(distorted)
        np.testing.assert_allclose(detected, wanted, atol=1.5)
        record = solve_calibration(detected, (640, 480), PREVIEW_BEV, "synthetic-nav")
        validate_calibration(record)

    def test_missing_and_duplicate_id_are_rejected(self):
        with self.assertRaises(ValueError):
            detect_tag(np.full((480, 640, 3), 255, dtype=np.uint8))
        with self.assertRaises(ValueError):
            detect_tag(np.concatenate((marker_frame(), marker_frame()), axis=1))

    def test_degenerate_and_outside_corners_are_rejected(self):
        for points in ([[10, 10], [20, 20], [30, 30], [40, 40]],
                       [[10, 10], [650, 10], [650, 100], [10, 100]]):
            with self.assertRaises(ValueError):
                solve_calibration(points, (640, 480), PREVIEW_BEV, "nav")

    def test_round_trip_retains_full_precision_and_previous_record(self):
        with tempfile.TemporaryDirectory(prefix="pi_upper_test2_") as directory:
            path = Path(directory) / "bev_calibration.json"
            record = self.record()
            save_calibration(record, path)
            before = path.read_bytes()
            self.assertEqual(load_calibration(path), record)
            record["created_at"] = "2026-10-06T00:00:00+00:00"
            save_calibration(record, path)
            archives = list(path.parent.glob("bev_calibration_*.json"))
            self.assertEqual(len(archives), 1)
            self.assertEqual(archives[0].read_bytes(), before)
            self.assertEqual(load_calibration(path), record)

    def test_corrupt_record_cannot_replace_existing_result(self):
        with tempfile.TemporaryDirectory(prefix="pi_upper_test2_") as directory:
            path = Path(directory) / "bev_calibration.json"
            record = self.record()
            save_calibration(record, path)
            before = path.read_bytes()
            for bad_matrix in (np.zeros((3, 3)).tolist(), np.full((3, 3), np.nan).tolist(), np.eye(3).tolist()):
                invalid = copy.deepcopy(record)
                invalid["H_img_to_bev"] = bad_matrix
                with self.assertRaises(ValueError):
                    save_calibration(invalid, path)
                self.assertEqual(path.read_bytes(), before)
            record["H_img_to_bev"] = (np.asarray(record["H_img_to_bev"]) * 2).tolist()
            validate_calibration(record)


class TestTest2Gui(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_navigation_mapping_and_expired_frame(self):
        from main_window import MainWindow
        with patch("main_window.list_cameras", return_value=[]):
            window = MainWindow()
        try:
            window.role_sources.update(recognition_camera="rec", navigation_camera="nav")
            rec = np.zeros((480, 640, 3), dtype=np.uint8)
            nav = marker_frame()
            window._on_frame_ready("rec", rec, 30)
            window._on_frame_ready("nav", nav, 30)
            window._select_task("测试2")
            frame, source = window._test2_snapshot()
            self.assertIs(frame, nav)
            self.assertEqual(source, "nav")
            self.assertFalse(window.test2_panel.isHidden())
            self.assertEqual(window.pages.currentIndex(), 3)
            window.navigation_frame_at = time.monotonic() - 3
            with self.assertRaises(ValueError):
                window._test2_snapshot()
            window._select_task("测试1")
            self.assertTrue(window.test2_panel.isHidden())
        finally:
            window.close()

    def test_dialog_saves_and_rejects_changed_camera(self):
        from apriltag_dialog import ApriltagCalibrationDialog
        frame = marker_frame()
        state = {"source": "nav"}
        with tempfile.TemporaryDirectory(prefix="pi_upper_test2_") as directory:
            path = Path(directory) / "record.json"
            dialog = ApriltagCalibrationDialog(lambda: (frame, state["source"]), path=path)
            try:
                self.assertTrue(dialog.save_button.isEnabled())
                state["source"] = "other-camera"
                dialog.save()
                self.assertFalse(path.exists())
                self.assertIn("来源或分辨率", dialog.status.text())
                state["source"] = "nav"
                dialog.save()
                self.assertEqual(load_calibration(path)["image_size"], [640, 480])
                dialog.read_saved()
                self.assertIn("H_img_to_bev", dialog.details.toPlainText())
                np.testing.assert_array_equal(frame, marker_frame())
            finally:
                dialog.close()

    def test_failed_recapture_disables_save_and_preserves_existing_file(self):
        from apriltag_dialog import ApriltagCalibrationDialog
        state = {"frame": marker_frame()}
        with tempfile.TemporaryDirectory(prefix="pi_upper_test2_") as directory:
            path = Path(directory) / "record.json"
            dialog = ApriltagCalibrationDialog(lambda: (state["frame"], "nav"), path=path)
            try:
                dialog.save()
                before = path.read_bytes()
                state["frame"] = np.full_like(state["frame"], 255)
                dialog.capture()
                self.assertFalse(dialog.save_button.isEnabled())
                self.assertIsNone(dialog.record)
                dialog.save()
                self.assertEqual(path.read_bytes(), before)
            finally:
                dialog.close()


if __name__ == "__main__":
    unittest.main()
