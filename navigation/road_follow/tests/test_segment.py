"""1280×720 进 640 时，上下留灰边，水平不留。"""

from __future__ import annotations

import unittest

import numpy as np

from road_follow.segment import letterbox


class LetterboxTest(unittest.TestCase):
    def test_720p_pads_vertically(self) -> None:
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        image[:, :] = (10, 20, 30)
        canvas, ratio, left, top = letterbox(image, size=640)
        self.assertEqual(canvas.shape, (640, 640, 3))
        self.assertAlmostEqual(ratio, 0.5, places=3)
        self.assertEqual(left, 0)
        self.assertEqual(top, 140)
        self.assertEqual(tuple(int(v) for v in canvas[0, 0]), (114, 114, 114))
        # BGR (10,20,30) → RGB，内容区不被灰边盖住。
        pixel = canvas[top + 20, left + 20]
        self.assertEqual(tuple(int(v) for v in pixel), (30, 20, 10))


if __name__ == "__main__":
    unittest.main()
