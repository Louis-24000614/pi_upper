"""合成透视棋盘与纯逻辑验证；不连接相机或串口。"""
import copy
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manual_calibration import corners, dimensions, homography, navigation_settings, render_images, solve_calibration, validate_record


POINTS = [[270., 150.], [900., 180.], [1080., 590.], [160., 560.]]


def parameters():
    return {"columns": 3, "rows": 2, "dimensions": {"mode": "cell", "unit": "mm", "cell_size": 30},
            "image_points": copy.deepcopy(POINTS), "vehicle": {"axes_aligned": False}}


def checker_frame():
    # 整块 8×6 棋盘；只有中间 3×2 格的四外角用于标定，整板可以在画面外。
    board = np.zeros((600, 800, 3), np.uint8)
    for row in range(6):
        for column in range(8):
            board[row*100:(row+1)*100, column*100:(column+1)*100] = 240 if (row+column)%2 else 25
    selected = np.array([[200, 200], [500, 200], [500, 400], [200, 400]], np.float32)
    matrix = cv2.getPerspectiveTransform(selected, np.array(POINTS, np.float32))
    return cv2.warpPerspective(board, matrix, (1280, 720)), matrix, selected


class GeometryTest(unittest.TestCase):
    def solve(self, params=None):
        return solve_calibration(params or parameters(), (1280, 720), {"kind": "synthetic_partial_board"}, {})

    def test_partial_non_square_board_four_corners_and_independent_interior_point(self):
        frame, board_to_image, selected = checker_frame()
        record = self.solve()
        self.assertAlmostEqual(record["actual_width_m"], .09)
        self.assertAlmostEqual(record["actual_height_m"], .06)
        matrix = np.array(record["H_img_to_region_ground_m"])
        mapped = cv2.perspectiveTransform(np.array(POINTS).reshape(-1, 1, 2), matrix).reshape(-1, 2)
        np.testing.assert_allclose(mapped, [[-.045, .03], [.045, .03], [.045, -.03], [-.045, -.03]], atol=1e-8)
        independent_board_point = np.array([[[350., 300.]]])
        image_point = cv2.perspectiveTransform(independent_board_point, board_to_image)
        ground_point = cv2.perspectiveTransform(image_point, matrix)
        np.testing.assert_allclose(ground_point, [[[0., 0.]]], atol=1e-8)
        overlay, preview = render_images(frame, record)
        self.assertEqual(preview.shape[1::-1], tuple(record["preview"]["size"]))
        self.assertFalse(np.array_equal(overlay, frame))
        self.assertEqual(record["image_space"], "raw_distorted")
        self.assertIsNone(record["vehicle_candidate"])
        validate_record(record)

    def test_units_and_exclusive_dimensions(self):
        self.assertEqual(dimensions(3, 2, {"mode": "cell", "unit": "mm", "cell_size": 30})[:2], (.09, .06))
        self.assertEqual(dimensions(3, 2, {"mode": "cell", "unit": "cm", "cell_size": 3})[:2], (.09, .06))
        for unit, w, h in (("mm", 90, 60), ("cm", 9, 6)):
            width, height, _ = dimensions(3, 2, {"mode": "total", "unit": unit, "width": w, "height": h})
            self.assertAlmostEqual(width, .09); self.assertAlmostEqual(height, .06)
        # 总宽、高分别实测，无需强行推定相等单格边长。
        self.assertEqual(dimensions(3, 2, {"mode": "total", "unit": "cm", "width": 12, "height": 5})[:2], (.12, .05))
        for values in ({"mode": "cell", "unit": "mm", "cell_size": 30, "width": 90},
                       {"mode": "total", "unit": "cm", "width": 9, "height": 6, "cell_size": 3}):
            with self.assertRaisesRegex(ValueError, "不能同时"): dimensions(3, 2, values)

    def test_invalid_counts_sizes_units_and_nonfinite_values(self):
        for n, m in ((0, 2), (-1, 2), (3, 0), (3.5, 2), (True, 2), (3, None)):
            with self.assertRaises(ValueError): dimensions(n, m, parameters()["dimensions"])
        for cell in (0, -1, float("inf"), float("nan"), True, None, "30"):
            with self.assertRaises(ValueError): dimensions(3, 2, {"mode": "cell", "unit": "mm", "cell_size": cell})
        for values in (None, {}, {"mode": "cell", "unit": "m", "cell_size": .03},
                       {"mode": "total", "unit": "mm", "width": 90}, {"mode": "other", "unit": "mm"}):
            with self.assertRaises(ValueError): dimensions(3, 2, values)

    def test_out_of_range_wrong_order_concave_small_and_degenerate_points(self):
        invalid = [POINTS[:3], [[-1, 0], *POINTS[1:]], [[1280, 150], *POINTS[1:]],
                   [POINTS[i] for i in (0, 3, 2, 1)], [POINTS[i] for i in (0, 2, 1, 3)],
                   [[10, 10], [30, 10], [15, 15], [10, 30]], [[10, 10], [12, 10], [12, 12], [10, 12]],
                   [[10, 10], [400, 11], [800, 12.001], [390, 11.01]],
                   [[float("nan"), 150], *POINTS[1:]], [[True, 150], *POINTS[1:]],
                   [["270", 150], *POINTS[1:]], [POINTS[0], POINTS[0], *POINTS[2:]]]
        for points in invalid:
            with self.subTest(points=points), self.assertRaises(ValueError): corners(points, (1280, 720))

    def test_do_not_sort_or_change_region_direction(self):
        # 旋转的区域也保留用户指定的自身左上起点，不按屏幕 X/Y 重排。
        params = parameters(); params["image_points"] = [POINTS[i] for i in (1, 2, 3, 0)]
        record = self.solve(params)
        self.assertEqual(record["image_points"], params["image_points"])
        np.testing.assert_allclose(record["ground_points_m"][0], [-.045, .03])

    def test_vehicle_candidate_requires_alignment_and_measured_offset(self):
        params = parameters(); params["vehicle"] = {"axes_aligned": False, "unit": "cm", "x": 3, "y": 40}
        self.assertIsNone(self.solve(params)["vehicle_candidate"])
        params["vehicle"]["axes_aligned"] = True
        record = self.solve(params); candidate = record["vehicle_candidate"]
        self.assertEqual(candidate["region_center_m"], [.03, .4])
        np.testing.assert_allclose(candidate["ground_points_m"], np.array(record["ground_points_m"])+[.03, .4])
        self.assertEqual(record["coordinate_reference"], "selected_region_center")
        self.assertEqual(candidate["coordinate_reference"], "navigation_camera_ground_projection")
        self.assertFalse(candidate["verified"]); self.assertFalse(candidate["applied"])
        validate_record(record)
        params["vehicle"] = {"axes_aligned": True, "unit": "mm", "x": 30, "y": 400}
        np.testing.assert_allclose(self.solve(params)["vehicle_candidate"]["ground_points_m"], candidate["ground_points_m"])
        for values in ({"axes_aligned": True}, {"axes_aligned": "yes"}, {"axes_aligned": True, "unit": "cm", "x": 0, "y": float("nan")}):
            params["vehicle"] = values
            with self.assertRaises(ValueError): self.solve(params)

    def test_tampered_matrices_and_ground_points_are_rejected(self):
        for key in ("H_img_to_region_ground_m", "H_img_to_preview_px", "ground_points_m"):
            record = self.solve(); record[key][0][0] += 1
            with self.assertRaises(ValueError): validate_record(record)
        record = self.solve(); record["axes"]["y"] = "vehicle_forward"
        with self.assertRaises(ValueError): validate_record(record)

    def test_current_navigation_baseline_is_read_only_and_matches_ipm_points(self):
        path = Path(__file__).resolve().parents[2] / "config/nav_camera.yaml"
        before = path.read_bytes(); settings = navigation_settings(path)
        self.assertEqual(path.read_bytes(), before)
        baseline = settings["baseline"]
        self.assertIsNone(baseline["error"])
        self.assertEqual(baseline["camera"]["height_m"], yaml.safe_load(before)["camera"]["height_m"])
        ground = np.array(baseline["H_img_to_vehicle_ground_m"])
        bev = np.array(baseline["H_img_to_navigation_bev_px"])
        pixels = np.array([[[640., 500.]], [[780., 550.]]])
        ground_points = cv2.perspectiveTransform(pixels, ground)
        bev_points = cv2.perspectiveTransform(pixels, bev)
        values = baseline["bev"]
        expected = bev_points.copy(); expected[:, :, 0] = bev_points[:, :, 0]*values["m_per_px"]+values["x_min"]
        expected[:, :, 1] = values["y_max"]-bev_points[:, :, 1]*values["m_per_px"]
        np.testing.assert_allclose(ground_points, expected, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
