"""验证优化的数值、内存生命周期及拒绝旧帧规则，不依赖真实 NPU/UART。"""
import unittest
from dataclasses import replace
from unittest.mock import patch
import cv2
import numpy as np
from road_follow.segment import letterbox, decode_road_mask
from road_follow.segment_buffers import SegmentBuffers
from road_follow.pipeline import BevProjector, make_ipm, command_from_mask_with_diagnostics
from ipm_proto.temporal import CenterlineSmoother
from experiments.seg_fps.frames import Frame, ResultGate


class OptimizationTest(unittest.TestCase):
    def test_nms_uses_width_height_for_disjoint_boxes(self):
        pred = np.zeros((37, 100), np.float32)
        pred[:5, :2] = np.array([[100, 120], [100, 100], [20, 20], [20, 20], [.9, .8]])
        pred[5, :2] = 1
        proto = np.ones((32, 160, 160), np.float32)
        good = decode_road_mask(pred, proto, 1, 0, 0, (640, 640), correct_nms=True)
        bad = decode_road_mask(pred, proto, 1, 0, 0, (640, 640), correct_nms=False)
        self.assertEqual(good[100, 120], 255)
        self.assertEqual(bad[100, 120], 0)
        # 完全重叠的低分框必须被抑制，不能把修正实现成完全取消 NMS。
        pred[0, 1] = 100
        pred[5, 1] = -1
        same = decode_road_mask(pred, proto, 1, 0, 0, (640, 640), correct_nms=True)
        self.assertEqual(same[100, 100], 255)

    def test_buffers_match_across_aspect_ratios_and_dark_bright_frames(self):
        buffers = SegmentBuffers()
        rng = np.random.default_rng(20261002)
        for shape in [(720, 1280, 3), (300, 150, 3), (640, 640, 3)] * 2:
            for image in [np.zeros(shape, np.uint8), np.full(shape, 255, np.uint8),
                          rng.integers(0, 256, shape, dtype=np.uint8)]:
                original = letterbox(image)
                reused = buffers.letterbox(image)
                np.testing.assert_array_equal(original[0], reused[0])
                self.assertEqual(original[1:], reused[1:])

    def test_cached_projection_matches_and_invalidates(self):
        cfg = {"camera": {"height_m": .16, "pitch_deg": 28}}
        projector = BevProjector()
        rng = np.random.default_rng(12)
        for shape in [(720, 1280), (360, 640)]:
            mask = rng.integers(0, 2, shape, dtype=np.uint8)*255
            first, bev = projector.project(mask, cfg)
            original = make_ipm(cfg, shape)
            np.testing.assert_array_equal(original.warp_to_bev(mask, cv2.INTER_NEAREST), bev)
            a = command_from_mask_with_diagnostics(mask, cfg, CenterlineSmoother())
            b = command_from_mask_with_diagnostics(mask, cfg, CenterlineSmoother(), projection=(first, bev))
            self.assertEqual(a, b)
            cfg["camera"]["pitch_deg"] += 1
            changed, _ = projector.project(mask, cfg)
            self.assertIsNot(first, changed)

    def test_result_gate_rejects_duplicate_late_expired_and_old_source(self):
        gate = ResultGate(source=1, max_age_s=.2)
        frame = Frame(10, 1, 100, 100, 1, np.zeros((1, 1, 3), np.uint8))
        self.assertTrue(gate.accept(frame, 100.1))
        self.assertFalse(gate.accept(frame, 100.1))
        self.assertFalse(gate.accept(replace(frame, sequence=9), 100.1))
        self.assertFalse(gate.accept(replace(frame, sequence=11), 101))
        self.assertFalse(gate.accept(replace(frame, sequence=12, source=0), 100.1))
        self.assertEqual(gate.last, 10)
        self.assertTrue(gate.accept(replace(frame, sequence=13), 100.15))
        gate.switch_source(2)
        self.assertFalse(gate.accept(replace(frame, sequence=14), 100.15))
        self.assertTrue(gate.accept(replace(frame, sequence=0, source=2), 100.15))

    def test_lazy_raw_preserves_diagnostics_for_prior_and_fallback(self):
        from road_follow.tests.test_pipeline import _cfg, _image_from_bev
        from ipm_proto.synth import make_straight_road_bev
        cfg = _cfg()
        ipm = make_ipm(cfg, (720,1280))
        for width in [.8, .2]:
            mask = _image_from_bev(make_straight_road_bev(ipm.bev, width), cfg)
            normal = command_from_mask_with_diagnostics(mask,cfg,CenterlineSmoother())
            lazy = command_from_mask_with_diagnostics(mask,cfg,CenterlineSmoother(),lazy_raw=True)
            self.assertEqual(normal, lazy)
