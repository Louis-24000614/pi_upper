"""检查停用 GrabCut 后的前景分割及保留的辅助函数，不依赖模型或相机。"""

from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from vision.knife import preprocessing


def knife_scene() -> np.ndarray:
    """固定背景、深色刀柄和浅色刀刃，用于复现低前景场景。"""
    image = np.full((180, 300, 3), 250, dtype=np.uint8)
    image[80:105, 60:140] = 45
    image[83:102, 140:200] = 220
    return image


class GrabCutPreprocessingTests(unittest.TestCase):
    def setUp(self):
        # GrabCut 的 GMM 初始化使用 OpenCV RNG，固定种子使回归测试可复现。
        cv2.setRNGSeed(12)

    def test_low_contrast_scene_keeps_legacy_mask_without_recovery(self):
        image = knife_scene()
        with patch.object(preprocessing, "_scene_grabcut", side_effect=AssertionError("GrabCut 已停用")) as recovery:
            mask = preprocessing.segment_foreground(image)
        recovery.assert_not_called()

        # 保持旧颜色分割结果，不再用 GrabCut 扩大浅色刀刃区域。
        blade = np.s_[86:99, 145:195]
        handle = np.s_[85:100, 65:135]
        self.assertEqual(np.count_nonzero(mask[blade]), 0)
        self.assertGreater(float(np.mean(mask[handle] > 0)), 0.95)
        self.assertEqual(np.count_nonzero(mask[:50]), 0)
        self.assertLess(float(np.mean(mask > 0)), 0.10)
        self.assertEqual(mask.shape, image.shape[:2])
        self.assertEqual(mask.dtype, np.uint8)

    def test_legacy_mask_reaches_existing_alignment_pipeline(self):
        with patch.object(preprocessing, "_scene_grabcut", side_effect=AssertionError("GrabCut 已停用")):
            rgb, mask, metadata = preprocessing.preprocess_bgr(knife_scene())
        self.assertEqual(rgb.shape, (448, 448, 3))
        self.assertEqual(mask.shape, (448, 448))
        self.assertGreater(metadata.foreground_pixels, 1500)
        self.assertLess(metadata.foreground_pixels, 2500)
        self.assertLess(metadata.foreground_ratio, 0.10)
        self.assertTrue(np.isfinite(preprocessing.normalize_rgb(rgb)).all())

    def test_alpha_foreground_keeps_original_path(self):
        image = np.zeros((100, 140, 4), dtype=np.uint8)
        image[35:65, 30:110, :3] = 220
        image[35:65, 30:110, 3] = 255
        expected = np.where(image[:, :, 3] > 16, 255, 0).astype(np.uint8)
        with patch.object(preprocessing, "_scene_grabcut") as recovery:
            mask = preprocessing.segment_foreground(image)
        recovery.assert_not_called()
        np.testing.assert_array_equal(mask, expected)

    def test_regular_foreground_does_not_run_recovery(self):
        image = np.full((180, 300, 3), 250, dtype=np.uint8)
        image[60:120, 60:230] = 45
        with patch.object(preprocessing, "_scene_grabcut") as recovery:
            mask = preprocessing.segment_foreground(image)
        recovery.assert_not_called()
        self.assertGreater(float(np.mean(mask > 0)), 0.10)
        self.assertLess(float(np.mean(mask > 0)), 0.50)

    def test_uniform_background_remains_empty(self):
        image = np.full((180, 300, 3), 250, dtype=np.uint8)
        self.assertIsNone(preprocessing._scene_grabcut(image))
        self.assertEqual(np.count_nonzero(preprocessing.segment_foreground(image)), 0)
        with self.assertRaisesRegex(ValueError, "前景不足"):
            preprocessing.preprocess_bgr(image)

    def test_tiny_image_skips_grabcut(self):
        with patch.object(cv2, "grabCut") as grabcut:
            self.assertIsNone(preprocessing._scene_grabcut(np.zeros((20, 40, 3), np.uint8)))
        grabcut.assert_not_called()

    def test_segmentation_does_not_reach_opencv_grabcut(self):
        image = knife_scene()
        with patch.object(cv2, "grabCut", side_effect=cv2.error("synthetic failure")) as grabcut:
            mask = preprocessing.segment_foreground(image)
        grabcut.assert_not_called()
        self.assertGreater(np.count_nonzero(mask[85:100, 65:135]), 0)
        self.assertEqual(np.count_nonzero(mask[86:99, 145:195]), 0)

    def test_implausibly_large_recovery_is_rejected(self):
        # 验证质量门槛：背景被 GrabCut 全部判为前景时应保留原流程结果。
        def fill_foreground(image, mask, *args):
            mask[:] = cv2.GC_FGD

        with patch.object(cv2, "grabCut", side_effect=fill_foreground):
            self.assertIsNone(preprocessing._scene_grabcut(knife_scene()))


if __name__ == "__main__":
    unittest.main()
