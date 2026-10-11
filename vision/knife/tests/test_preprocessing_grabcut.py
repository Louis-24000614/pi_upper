"""用合成刀具和退化输入检查实拍 GrabCut 补偿，不依赖模型或相机。"""

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
    """固定背景、深色刀柄和浅色刀刃，刀刃不应被颜色阈值漏掉。"""
    image = np.full((180, 300, 3), 250, dtype=np.uint8)
    image[80:105, 60:140] = 45
    image[83:102, 140:200] = 220
    return image


class GrabCutPreprocessingTests(unittest.TestCase):
    def setUp(self):
        # GrabCut 的 GMM 初始化使用 OpenCV RNG，固定种子使回归测试可复现。
        cv2.setRNGSeed(12)

    def test_low_contrast_blade_is_recovered_without_background(self):
        image = knife_scene()
        with patch.object(preprocessing, "_scene_grabcut", return_value=None):
            legacy = preprocessing.segment_foreground(image)
        mask = preprocessing.segment_foreground(image)

        # 独立已知几何区域检查真实恢复效果，不能仅检查补偿函数被调用。
        blade = np.s_[86:99, 145:195]
        self.assertEqual(np.count_nonzero(legacy[blade]), 0)
        self.assertGreater(float(np.mean(mask[blade] > 0)), 0.95)
        self.assertEqual(np.count_nonzero(mask[:50]), 0)
        self.assertGreater(np.count_nonzero(mask), np.count_nonzero(legacy))
        self.assertEqual(mask.shape, image.shape[:2])
        self.assertEqual(mask.dtype, np.uint8)

    def test_recovered_mask_reaches_existing_alignment_pipeline(self):
        rgb, mask, metadata = preprocessing.preprocess_bgr(knife_scene())
        self.assertEqual(rgb.shape, (448, 448, 3))
        self.assertEqual(mask.shape, (448, 448))
        self.assertGreater(metadata.foreground_pixels, 2800)
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

    def test_opencv_failure_falls_back_to_legacy_mask(self):
        image = knife_scene()
        with patch.object(preprocessing, "_scene_grabcut", return_value=None):
            expected = preprocessing.segment_foreground(image)
        with patch.object(cv2, "grabCut", side_effect=cv2.error("synthetic failure")) as grabcut:
            mask = preprocessing.segment_foreground(image)
        grabcut.assert_called_once()
        np.testing.assert_array_equal(mask, expected)

    def test_implausibly_large_recovery_is_rejected(self):
        # 验证质量门槛：背景被 GrabCut 全部判为前景时应保留原流程结果。
        def fill_foreground(image, mask, *args):
            mask[:] = cv2.GC_FGD

        with patch.object(cv2, "grabCut", side_effect=fill_foreground):
            self.assertIsNone(preprocessing._scene_grabcut(knife_scene()))


if __name__ == "__main__":
    unittest.main()
