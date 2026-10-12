"""路宽先验与中心线时间平滑。"""

from __future__ import annotations

import unittest

import numpy as np

from ipm_proto.centerline import centerline_lateral_error, extract_centerline
from ipm_proto.ipm import BevConfig
from ipm_proto.prior import RoadWidthPrior, extract_centerline_with_width_prior
from ipm_proto.synth import make_straight_road_bev
from ipm_proto.temporal import CenterlineSmoother


class WidthPriorTest(unittest.TestCase):
    def _lane(self, center, *, branch=0, width=0.20):
        """生成主路和单侧连接开口，用真实几何验证支路不会污染中心线。"""
        bev = BevConfig(y_min=0.12, y_max=1.0)
        mask = np.zeros((bev.height_px, bev.width_px), np.uint8)
        for v in range(bev.height_px):
            _, y = bev.bev_px_to_ground(0, v)
            x = center(y)
            left, right = x - width / 2, x + width / 2
            if 0.26 <= y <= 0.55:
                left -= max(0, -branch) * 0.10
                right += max(0, branch) * 0.10
            u0, _ = bev.ground_to_bev_px(left, y)
            u1, _ = bev.ground_to_bev_px(right, y)
            mask[v, round(u0):round(u1)] = 255
        return bev, mask

    def test_connected_side_branch_below_max_width_keeps_main_lane(self):
        for side in (-1, 1):
            with self.subTest(side=side):
                bev, mask = self._lane(lambda y: 0.06, branch=side)
                points = extract_centerline_with_width_prior(
                    mask, bev, RoadWidthPrior(0.20, 0.14, 0.32)
                )
                opening = [x for x, y in points if 0.28 <= y <= 0.50]
                self.assertTrue(opening)
                self.assertLess(max(abs(x - 0.055) for x in opening), 0.015)

    def test_slanted_and_curved_main_lane_are_not_locked_to_vehicle_axis(self):
        for center in (lambda y: 0.04 + 0.18*y, lambda y: 0.03 + 0.20*y*y):
            for side in (0, -1, 1):
                with self.subTest(side=side, center=center):
                    bev, mask = self._lane(center, branch=side)
                    points = extract_centerline_with_width_prior(
                        mask, bev, RoadWidthPrior(0.20, 0.14, 0.32)
                    )
                    self.assertLess(max(abs(x-center(y)) for x,y in points), 0.02)

    def test_wide_band_window_cannot_cross_gap_to_other_component(self):
        bev = BevConfig(y_min=0.12, y_max=1.0)
        mask = np.zeros((bev.height_px, bev.width_px), np.uint8)
        mask[:, 5:9] = 255  # 独立噪声不能作为所选道路段的边界。
        mask[:, 58:99] = 255
        points = extract_centerline_with_width_prior(
            mask, bev, RoadWidthPrior(0.20, 0.14, 0.32)
        )
        self.assertTrue(points)
        self.assertGreaterEqual(min(x for x, _ in points), 0.17)

    def test_gradually_widening_branch_and_mask_cracks_keep_main_lane(self):
        for side in (-1, 1):
            with self.subTest(side=side):
                bev, mask = self._lane(lambda y: 0.0)
                for v in range(bev.height_px):
                    _, y = bev.bev_px_to_ground(0, v)
                    if 0.24 <= y <= 0.55:
                        extension = min(10, round((y - 0.24) * 100))
                        if side < 0:
                            mask[v, 40-extension:40] = 255
                        else:
                            mask[v, 60:60+extension] = 255
                        mask[v, 49:51] = 0  # 两像素裂缝仍属于同一条路。
                before = mask.copy()
                points = extract_centerline_with_width_prior(
                    mask, bev, RoadWidthPrior(0.20, 0.14, 0.32)
                )
                self.assertLess(max(abs(x) for x, _ in points), 0.02)
                np.testing.assert_array_equal(mask, before)

    def test_disconnected_nominal_width_side_lane_does_not_replace_main_lane(self):
        bev, mask = self._lane(lambda y: 0.0, branch=1)
        mask[:60, 5:25] = 255
        points = extract_centerline_with_width_prior(
            mask, bev, RoadWidthPrior(0.20, 0.14, 0.32)
        )
        self.assertLess(max(abs(x) for x, _ in points), 0.015)

    def test_nominal_matches_plain_extract(self) -> None:
        bev = BevConfig()
        mask = make_straight_road_bev(bev, road_width_m=0.8)
        plain = extract_centerline(mask, bev)
        prior = extract_centerline_with_width_prior(mask, bev, RoadWidthPrior())
        self.assertGreater(len(prior), 20)
        err = centerline_lateral_error(prior, near_y_max=1.3)
        self.assertLess(err, 0.05)
        # 与无先验中心线近端应一致
        self.assertLess(abs(prior[0][0] - plain[0][0]), 0.03)

    def test_flooded_white_mask_recovers_corridor(self) -> None:
        """整幅 BEV 为「路」时，先验应裁回约 0.8 m 并居中。"""
        bev = BevConfig()
        flooded = np.full((bev.height_px, bev.width_px), 255, dtype=np.uint8)
        points = extract_centerline_with_width_prior(
            flooded, bev, RoadWidthPrior(expected_width_m=0.8, max_width_m=1.0)
        )
        self.assertGreater(len(points), 20)
        err = centerline_lateral_error(points, near_y_max=1.3)
        self.assertLess(err, 0.06)

    def test_prefers_corridor_near_center_over_side_blob(self) -> None:
        bev = BevConfig()
        mask = np.zeros((bev.height_px, bev.width_px), dtype=np.uint8)
        # 左侧窄噪声带 + 居中 0.8 m 走廊
        half = 0.4
        for v in range(bev.height_px):
            u0, _ = bev.ground_to_bev_px(-half, bev.y_min)
            u1, _ = bev.ground_to_bev_px(half, bev.y_min)
            mask[v, int(u0) : int(u1)] = 255
            # 左侧 0.15 m 噪声
            ul0, _ = bev.ground_to_bev_px(-0.48, bev.y_min)
            ul1, _ = bev.ground_to_bev_px(-0.33, bev.y_min)
            mask[v, max(0, int(ul0)) : max(0, int(ul1))] = 255
        points = extract_centerline_with_width_prior(mask, bev, RoadWidthPrior())
        self.assertGreater(len(points), 10)
        self.assertLess(centerline_lateral_error(points), 0.08)


class TemporalTest(unittest.TestCase):
    def test_ema_pulls_toward_previous(self) -> None:
        sm = CenterlineSmoother(alpha=0.5, y_match_tol_m=0.05)
        a = [(0.0, 0.5), (0.0, 1.0)]
        b = [(0.2, 0.5), (0.2, 1.0)]
        out1 = sm.update(a)
        self.assertEqual(out1[0][0], 0.0)
        out2 = sm.update(b)
        self.assertAlmostEqual(out2[0][0], 0.1, places=5)

    def test_empty_resets(self) -> None:
        sm = CenterlineSmoother()
        sm.update([(0.1, 0.5)])
        self.assertEqual(sm.update([]), [])
        out = sm.update([(0.0, 0.5)])
        self.assertEqual(out[0][0], 0.0)


if __name__ == "__main__":
    unittest.main()
