"""故障视频不得伪装正常 EOF，线程异常必须到达有序消费者。"""
import unittest
from unittest.mock import patch
from pathlib import Path
import cv2
import numpy as np
from experiments.seg_fps.frames import VideoFrames


class BrokenCapture:
    def __init__(self, path):
        self.calls = 0
        self.released = False

    def isOpened(self):
        return True

    def get(self, key):
        return 10 if key == cv2.CAP_PROP_FPS else 3

    def read(self):
        self.calls += 1
        return (True, np.full((8,8,3), self.calls, np.uint8)) if self.calls == 1 else (False, None)

    def release(self):
        self.released = True


class FrameFaultTest(unittest.TestCase):
    def test_early_read_failure_reaches_both_consumers(self):
        for threaded in [False, True]:
            with self.subTest(threaded=threaded), patch(
                    "experiments.seg_fps.frames.cv2.VideoCapture", BrokenCapture):
                frames = VideoFrames(Path("broken.avi"), threaded=threaded)
                try:
                    iterator = iter(frames)
                    first = next(iterator)
                    self.assertEqual(first.sequence, 0)
                    self.assertEqual(int(first.image[0,0,0]), 1)
                    with self.assertRaisesRegex(RuntimeError, "中途读取失败"):
                        next(iterator)
                finally:
                    frames.close()


if __name__ == "__main__":
    unittest.main()
