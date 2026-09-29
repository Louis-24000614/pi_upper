"""赛道尺度的路口开口：直路、丁字、十字、拐角、截断。"""

from __future__ import annotations

import unittest

import numpy as np

from ipm_proto.ipm import BevConfig
from ipm_proto.junction import (
    KIND_BLOCKED,
    KIND_CORNER,
    KIND_CROSS,
    KIND_STRAIGHT,
    KIND_T,
    JunctionTracker,
    classify_junction,
)


def _field() -> BevConfig:
    return BevConfig(y_min=0.20, y_max=1.00, x_min=-0.5, x_max=0.5, m_per_px=0.01)


def _fill(mask: np.ndarray, bev: BevConfig, x0: float, x1: float, y0: float, y1: float) -> None:
    u0, v_near = bev.ground_to_bev_px(min(x0, x1), min(y0, y1))
    u1, v_far = bev.ground_to_bev_px(max(x0, x1), max(y0, y1))
    c0 = max(0, int(np.floor(min(u0, u1))))
    c1 = min(mask.shape[1], int(np.ceil(max(u0, u1))) + 1)
    r0 = max(0, int(np.floor(min(v_far, v_near))))
    r1 = min(mask.shape[0], int(np.ceil(max(v_far, v_near))) + 1)
    mask[r0:r1, c0:c1] = 255


def _blank(bev: BevConfig) -> np.ndarray:
    return np.zeros((bev.height_px, bev.width_px), dtype=np.uint8)


class JunctionTest(unittest.TestCase):
    def test_wide_practice_lane_stays_straight(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.40, 0.40, 0.20, 1.00)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_STRAIGHT)
        self.assertFalse(reading.left or reading.right)

    def test_straight_lane(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 1.00)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_STRAIGHT)
        self.assertTrue(reading.forward)
        self.assertFalse(reading.left)
        self.assertFalse(reading.right)
        self.assertAlmostEqual(reading.corridor_end_y_m or 0.0, 1.00, delta=0.02)

    def test_cross_keeps_the_far_arm(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 1.00)
        _fill(mask, bev, -0.50, 0.50, 0.62, 0.82)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_CROSS)
        self.assertTrue(reading.forward and reading.left and reading.right)
        self.assertIsNotNone(reading.junction_y_m)
        self.assertAlmostEqual(reading.junction_y_m, 0.72, delta=0.03)

    def test_t_from_the_stem_has_no_far_arm(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 0.62)
        _fill(mask, bev, -0.50, 0.50, 0.62, 0.82)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_T)
        self.assertFalse(reading.forward)
        self.assertTrue(reading.left and reading.right)
        # 横路穿过中央走廊，最远边比 0.72 m 路口中心远半个道宽。
        self.assertAlmostEqual(reading.corridor_end_y_m or 0.0, 0.82, delta=0.03)

    def test_side_opening_is_a_t(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 1.00)
        _fill(mask, bev, 0.10, 0.45, 0.62, 0.82)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_T)
        self.assertTrue(reading.forward and reading.right)
        self.assertFalse(reading.left)
        self.assertAlmostEqual(reading.junction_y_m or 0.0, 0.72, delta=0.03)

    def test_corner_opens_one_side(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 0.62)
        _fill(mask, bev, 0.10, 0.45, 0.62, 0.82)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_CORNER)
        self.assertFalse(reading.forward)
        self.assertTrue(reading.right)
        self.assertFalse(reading.left)

    def test_blocked_lane_is_not_a_junction(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 0.50)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_BLOCKED)
        self.assertFalse(reading.forward or reading.left or reading.right)
        self.assertAlmostEqual(reading.corridor_end_y_m or 0.0, 0.50, delta=0.02)

    def test_corridor_end_ignores_a_disconnected_far_blob(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 0.50)
        _fill(mask, bev, -0.03, 0.03, 0.90, 0.94)
        reading = classify_junction(mask, bev)
        self.assertEqual(reading.kind, KIND_BLOCKED)
        self.assertAlmostEqual(reading.corridor_end_y_m or 0.0, 0.50, delta=0.02)

    def test_tracker_holds_until_three_frames(self) -> None:
        bev = _field()
        mask = _blank(bev)
        _fill(mask, bev, -0.10, 0.10, 0.20, 1.00)
        reading = classify_junction(mask, bev)
        tracker = JunctionTracker(hold=3)
        self.assertEqual(tracker.update(reading), "unknown")
        self.assertEqual(tracker.update(reading), "unknown")
        self.assertEqual(tracker.update(reading), KIND_STRAIGHT)


if __name__ == "__main__":
    unittest.main()
