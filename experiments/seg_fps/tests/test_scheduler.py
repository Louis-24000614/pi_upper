"""验证实时帧能及时调度到空闲 context，以及长稳采样不改变 FPS 分母。"""
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import cv2
import numpy as np


@unittest.skipUnless(os.name == "posix", "测速入口使用板端 Linux resource 模块")
class SchedulerTest(unittest.TestCase):
    def run_fake(self, fail=False):
        from experiments.seg_fps import benchmark
        calls, closed = [], []
        lock = threading.Lock()

        class Segment:
            def __init__(self, *args, **kwargs):
                self._session = SimpleNamespace(inference=lambda **kwargs: [0, 0])
                self.last_timings = {}

            def _load(self):
                return self._session

            def mask(self, image):
                began = time.monotonic()
                with lock:
                    calls.append(began)
                if fail:
                    raise RuntimeError("injected inference failure")
                time.sleep(.08)
                self.last_timings = {"rknn_ms": 80}
                return np.zeros(image.shape[:2], np.uint8)

            def close(self):
                closed.append(self)

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            video, config = root/"frames.avi", root/"config.yaml"
            writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 50, (32,32))
            if not writer.isOpened():
                self.fail("测试录像编码器不可用")
            for i in range(10):
                writer.write(np.full((32,32,3), i*20, np.uint8))
            writer.release()
            config.write_text("{}\n")
            args = SimpleNamespace(video=video, config=config, model=root/"model.rknn",
                output=root/"result.json", variant="buffers", cores="1,2,4", mode="seg",
                backend="lite", realtime=True, speed=1, max_age=1, seconds=.4,
                retained_rows=2, record="none", mixed_load=False)
            with patch.object(benchmark, "RoadSegmenter", Segment), patch.object(
                    benchmark, "snapshot", return_value={}):
                try:
                    result = benchmark.run(args)
                finally:
                    self.assertEqual(len(closed), 3)
        return calls, result

    def test_realtime_dispatch_and_bounded_diagnostics(self):
        calls, result = self.run_fake()
        self.assertGreaterEqual(len(calls), 3)
        # 发布间隔 20ms，推理 80ms；前三帧应在第一帧完成前启动。
        self.assertLess(calls[2]-calls[0], .075)
        self.assertEqual(len(result["raw"]), 2)
        self.assertGreater(result["completed"], 2)
        self.assertAlmostEqual(result["fps"], result["completed"]/result["seconds"])
        self.assertEqual(result["completed"], result["consumed"])

    def test_worker_exception_propagates_and_closes_contexts(self):
        with self.assertRaisesRegex(RuntimeError, "injected inference failure"):
            self.run_fake(fail=True)


if __name__ == "__main__":
    unittest.main()
