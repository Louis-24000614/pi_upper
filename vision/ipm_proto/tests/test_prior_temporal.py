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
