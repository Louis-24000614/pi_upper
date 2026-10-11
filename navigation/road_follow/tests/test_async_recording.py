"""验证异步录像时间轴、输入所有权和 worker 异常能传播到调用者。"""
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from road_follow.async_recording import AsyncVideoRecorder
from experiments.seg_fps.frames import VideoFrames


class AsyncRecordingTest(unittest.TestCase):
    def test_timeline_and_fifo_decode_match(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"drive.avi"
            recorder = AsyncVideoRecorder(path, capacity=8)
            frame = np.full((48, 64, 3), 100, np.uint8)
            recorder.write(frame, 100)
            recorder.write(frame, 100.05)
            recorder.write(frame, 100.2)
            recorder.close(100.31)
            self.assertEqual(recorder.frames_written, 4)
            self.assertEqual(recorder.dropped, 0)
            direct, fifo = VideoFrames(path), VideoFrames(path, threaded=True, capacity=1)
            try:
                a, b = list(direct), list(fifo)
                self.assertEqual([f.sequence for f in a], [f.sequence for f in b])
                for x,y in zip(a,b):
                    np.testing.assert_array_equal(x.image, y.image)
                self.assertEqual(fifo.dropped, 0)
            finally:
                direct.close()
                fifo.close()

    def test_pending_frame_is_owned_and_queue_is_bounded(self):
        began, allow = threading.Event(), threading.Event()
        saved = []
        class Writer:
            frames_written = 0
            def __init__(self, *args): pass
            def write(self, frame, timestamp):
                began.set()
                if not allow.wait(2): raise RuntimeError("test timeout")
                saved.append((frame.copy(), timestamp))
            def close(self, *args): pass
        with patch("road_follow.async_recording.VideoRecorder", Writer):
            recorder = AsyncVideoRecorder(Path("unused.avi"), capacity=1)
            frame = np.full((2,2,3), 10, np.uint8)
            recorder.write(frame, 0)
            self.assertTrue(began.wait(2))
            frame.fill(20)
            recorder.write(frame, .1)
            frame.fill(30)
            recorder.write(frame, .2)
            frame.fill(255)
            allow.set()
            recorder.close()
            self.assertEqual(recorder.dropped, 1)
            self.assertEqual(saved[0][0][0,0,0], 10)
            self.assertEqual(saved[-1][0][0,0,0], 30)

    def test_worker_error_is_reported_on_close(self):
        class Writer:
            frames_written = 0
            def __init__(self, *args): pass
            def write(self, *args): raise OSError("disk full")
            def close(self, *args): pass
        with patch("road_follow.async_recording.VideoRecorder", Writer):
            recorder = AsyncVideoRecorder(Path("unused.avi"))
            recorder.write(np.zeros((2,2,3),np.uint8), 0)
            with self.assertRaisesRegex(RuntimeError,"异步录像失败"):
                recorder.close()
