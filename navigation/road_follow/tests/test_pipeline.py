"""前视 mask 经鸟瞰和路宽先验后，直道巡航、右偏右转、空 mask 停车。"""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from ipm_proto.synth import make_straight_road_bev
from ipm_proto.temporal import CenterlineSmoother
from road_follow.pipeline import command_from_mask, make_ipm


def _cfg() -> dict:
    return {
        "camera": {"height_m": 0.16, "pitch_deg": 28.0},
        "bev": {
            "y_min": 0.20,
            "y_max": 1.00,
            "x_min": -0.5,
            "x_max": 0.5,
            "m_per_px": 0.01,
        },
        "road_prior": {
            "expected_width_m": 0.8,
            "min_width_m": 0.35,
            "max_width_m": 1.15,
            "min_pixels": 3,
        },
        "temporal": {"smooth_alpha": 0.45, "y_match_tol_m": 0.04},
        "follow": {
            "cruise_mps": 0.10,
            "turn_mps": 0.06,
            "lookahead_m": 0.45,
            "turn_abs_x_m": 0.08,
        },
    }


def _image_from_bev(bev_mask: np.ndarray, cfg: dict) -> np.ndarray:
    ipm = make_ipm(cfg, (720, 1280))
    return cv2.warpPerspective(
        bev_mask,
        ipm.H_bev_to_img,
        (1280, 720),
        flags=cv2.INTER_NEAREST,
    )


class PipelineTest(unittest.TestCase):
    def test_straight_road_cruises(self) -> None:
        cfg = _cfg()
        ipm = make_ipm(cfg, (720, 1280))
        image = _image_from_bev(make_straight_road_bev(ipm.bev, 0.8), cfg)
        cmd = command_from_mask(image, cfg, CenterlineSmoother())
        self.assertEqual(cmd.reason, "follow")
        self.assertAlmostEqual(cmd.v_mps, 0.10, places=2)
        self.assertLess(abs(cmd.omega_radps), 0.08)

    def test_right_shifted_road_turns_right(self) -> None:
        cfg = _cfg()
        ipm = make_ipm(cfg, (720, 1280))
        # 0.8 m 路几乎铺满 16 cm 相机的画面，小偏移看不出边。用更窄的一条，边还在画面里。
        bev = make_straight_road_bev(ipm.bev, 0.4)
        shifted = np.zeros_like(bev)
        shifted[:, 15:] = bev[:, :-15]
        image = _image_from_bev(shifted, cfg)
        cmd = command_from_mask(image, cfg, CenterlineSmoother())
        self.assertEqual(cmd.reason, "follow")
        self.assertLess(cmd.omega_radps, -0.05)

    def test_thin_road_falls_back_to_centerline(self) -> None:
        cfg = _cfg()
        ipm = make_ipm(cfg, (720, 1280))
        image = _image_from_bev(make_straight_road_bev(ipm.bev, 0.2), cfg)
        cmd = command_from_mask(image, cfg, CenterlineSmoother())
        self.assertEqual(cmd.reason, "follow")
        self.assertAlmostEqual(cmd.v_mps, 0.10, places=2)
        self.assertLess(abs(cmd.omega_radps), 0.08)

    def test_empty_mask_stops(self) -> None:
        cmd = command_from_mask(np.zeros((720, 1280), np.uint8), _cfg(), CenterlineSmoother())
        self.assertEqual(cmd.reason, "stop_road")
        self.assertEqual(cmd.v_mps, 0.0)
        self.assertEqual(cmd.omega_radps, 0.0)


if __name__ == "__main__":
    unittest.main()
