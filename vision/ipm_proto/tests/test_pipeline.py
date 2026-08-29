"""合成直道：IPM 往返后路宽与中心线几何自检。"""

from __future__ import annotations

import unittest

import cv2
import numpy as np

from ipm_proto.centerline import centerline_lateral_error, extract_centerline
from ipm_proto.ipm import BevConfig, Ipm
from ipm_proto.synth import (
    make_straight_road_bev,
    measure_bev_road_width_m,
    synthesize_straight_road,
)


class PipelineTest(unittest.TestCase):
    def test_bev_road_width_nominal(self) -> None:
        bev = BevConfig()
        mask = make_straight_road_bev(bev, road_width_m=0.8)
        w = measure_bev_road_width_m(mask, bev)
        self.assertAlmostEqual(w, 0.8, delta=0.02)

    def test_synth_roundtrip_centerline(self) -> None:
        scene = synthesize_straight_road()
        bev_mask = scene.ipm.warp_to_bev(scene.mask, flags=cv2.INTER_NEAREST)
        # 在视场未裁切的中部 Y 量路宽（近端 0.8 m 常超出水平 FOV）
        width = measure_bev_road_width_m(bev_mask, scene.ipm.bev, at_y_m=1.0)
        self.assertAlmostEqual(width, scene.road_width_m, delta=0.06)

        points = extract_centerline(bev_mask, scene.ipm.bev)
        self.assertGreater(len(points), 20)
        # 用中远端点（已完整入画）检查居中；近端可能因 FOV 裁切偏置
        mid = [p for p in points if 0.8 <= p[1] <= 1.3]
        err = centerline_lateral_error(mid if mid else points, near_y_max=1.3)
        self.assertLess(err, 0.05)

    def test_four_point_homography_matches_extrinsics(self) -> None:
        scene = synthesize_straight_road()
        bev = scene.ipm.bev
        gnd = bev.corner_grounds()
        # 用已知 H 把地面角点映到图像（经 BEV）
        bev_corners = bev.corner_bev_px().astype(np.float32).reshape(-1, 1, 2)
        img_pts = cv2.perspectiveTransform(bev_corners, scene.ipm.H_bev_to_img).reshape(4, 2)
        ipm2 = Ipm.from_image_ground_points(img_pts, gnd, bev)
        # 同一 mask 两种 H 得到的中心线近端应接近
        bev_a = scene.ipm.warp_to_bev(scene.mask, flags=cv2.INTER_NEAREST)
        bev_b = ipm2.warp_to_bev(scene.mask, flags=cv2.INTER_NEAREST)
        pa = extract_centerline(bev_a, bev)
        pb = extract_centerline(bev_b, bev)
        self.assertLess(centerline_lateral_error(pa), 0.05)
        self.assertLess(centerline_lateral_error(pb), 0.05)


if __name__ == "__main__":
    unittest.main()
