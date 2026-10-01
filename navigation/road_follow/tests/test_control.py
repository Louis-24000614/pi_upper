"""纯跟踪：右偏要右转（负角速度），丢线要停车。"""

from __future__ import annotations

import math
import unittest

from road_follow.control import FollowConfig, command_from_centerline


class ControlTest(unittest.TestCase):
    def test_centered_line_goes_straight(self) -> None:
        points = [(0.0, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        cmd = command_from_centerline(points, road_pixels=2000, cfg=FollowConfig())
        self.assertEqual(cmd.reason, "follow")
        self.assertAlmostEqual(cmd.v_mps, 0.10, places=3)
        self.assertAlmostEqual(cmd.omega_radps, 0.0, places=3)

    def test_right_offset_turns_right(self) -> None:
        # 预瞄点在车体右侧。串口正角速度是左转，所以这里必须为负。
        points = [(0.20, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        cmd = command_from_centerline(points, road_pixels=2000, cfg=FollowConfig())
        self.assertEqual(cmd.reason, "follow")
        self.assertAlmostEqual(cmd.v_mps, 0.06, places=3)
        self.assertLess(cmd.omega_radps, -0.05)
        x, y = 0.20, 0.45
        expect = -0.06 * (2.0 * x) / (x * x + y * y)
        self.assertAlmostEqual(cmd.omega_radps, expect, places=3)

    def test_left_offset_turns_left(self) -> None:
        points = [(-0.05, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        cmd = command_from_centerline(points, road_pixels=2000, cfg=FollowConfig())
        self.assertEqual(cmd.reason, "follow")
        self.assertAlmostEqual(cmd.v_mps, 0.10, places=3)
        self.assertGreater(cmd.omega_radps, 0.0)

    def test_short_centerline_stops(self) -> None:
        cmd = command_from_centerline([(0.0, 0.3), (0.0, 0.4)], road_pixels=2000, cfg=FollowConfig())
        self.assertEqual(cmd.v_mps, 0.0)
        self.assertEqual(cmd.omega_radps, 0.0)
        self.assertEqual(cmd.reason, "stop_centerline")

    def test_too_few_road_pixels_stops(self) -> None:
        points = [(0.0, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        cmd = command_from_centerline(points, road_pixels=10, cfg=FollowConfig())
        self.assertEqual(cmd.reason, "stop_road")
        self.assertEqual(cmd.v_mps, 0.0)
        self.assertEqual(cmd.omega_radps, 0.0)

    def test_left_reading_bias_holds_a_straight_line(self) -> None:
        points = [(-0.03, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        cmd = command_from_centerline(
            points, road_pixels=2000, cfg=FollowConfig(x_bias_m=0.03)
        )
        self.assertEqual(cmd.reason, "follow")
        self.assertAlmostEqual(cmd.omega_radps, 0.0, places=3)

    def test_omega_is_clamped(self) -> None:
        points = [(0.20, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        cfg = FollowConfig(max_abs_omega=0.05)
        cmd = command_from_centerline(points, road_pixels=2000, cfg=cfg)
        self.assertAlmostEqual(cmd.omega_radps, -0.05, places=3)
        self.assertTrue(math.isfinite(cmd.omega_radps))

    def test_steering_gain_strengthens_correction_without_changing_direction(self) -> None:
        points = [(-0.05, y) for y in (0.25, 0.35, 0.45, 0.55, 0.70, 0.85, 0.95, 1.0)]
        normal = command_from_centerline(points, road_pixels=2000, cfg=FollowConfig())
        stronger = command_from_centerline(
            points, road_pixels=2000, cfg=FollowConfig(steering_gain=1.7)
        )
        self.assertGreater(normal.omega_radps, 0.0)
        self.assertAlmostEqual(stronger.omega_radps, normal.omega_radps * 1.7, places=6)

    def test_short_visible_road_uses_slow_near_lookahead(self) -> None:
        points = [(0.08, y) for y in (0.25, 0.28, 0.31, 0.34, 0.37, 0.40, 0.42, 0.43)]
        cfg = FollowConfig(min_lookahead_m=0.28, near_mps=0.05)
        cmd = command_from_centerline(points, road_pixels=2000, cfg=cfg)
        self.assertEqual(cmd.reason, "follow_near")
        self.assertAlmostEqual(cmd.v_mps, 0.05, places=3)
        self.assertLess(cmd.omega_radps, 0.0)


if __name__ == "__main__":
    unittest.main()
