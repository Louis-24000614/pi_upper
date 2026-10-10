"""录像默认关闭；启用后保留实际时间且不覆盖已有文件。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from road_follow.recording import VideoRecorder


class VideoRecorderTest(unittest.TestCase):
    def test_record_video_uses_elapsed_time(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "drive.avi"
            recorder = VideoRecorder(path, fps=10)
            frame = np.full((48, 64, 3), 100, dtype=np.uint8)
            recorder.write(frame, 100.0)
            recorder.write(frame, 100.05)
            recorder.write(frame, 100.20)
            recorder.close(100.31)

            self.assertEqual(recorder.frames_written, 4)
            capture = cv2.VideoCapture(str(path))
            try:
                self.assertTrue(capture.isOpened())
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 4)
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FPS)), 10)
                ok, recorded = capture.read()
                self.assertTrue(ok)
                self.assertEqual(recorded.shape, frame.shape)
            finally:
                capture.release()


    def test_long_idle_before_close_does_not_extend_video(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "drive.avi"
            recorder = VideoRecorder(path, fps=10)
            frame = np.full((48, 64, 3), 100, dtype=np.uint8)
            recorder.write(frame, 100.0)
            recorder.write(frame, 100.20)
            recorder.close(1000.0)

            self.assertEqual(recorder.frames_written, 4)
            capture = cv2.VideoCapture(str(path))
            try:
                self.assertTrue(capture.isOpened())
                self.assertEqual(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), 4)
                decoded = 0
                while capture.read()[0]:
                    decoded += 1
                self.assertEqual(decoded, 4)
            finally:
                capture.release()

    def test_tail_write_failure_still_releases_writer(self) -> None:
        class Writer:
            released = False
            writes = 0
            def isOpened(self):
                return True
            def write(self, frame):
                self.writes += 1
                if self.writes > 1:
                    raise OSError("disk full")
            def release(self):
                self.released = True

        writer = Writer()
        with tempfile.TemporaryDirectory() as directory:
            with patch("road_follow.recording.cv2.VideoWriter", return_value=writer):
                recorder = VideoRecorder(Path(directory) / "drive.avi")
                recorder.write(np.zeros((2, 2, 3), np.uint8), 100.0)
                with self.assertRaisesRegex(OSError, "disk full"):
                    recorder.close(100.2)
                self.assertTrue(writer.released)
                recorder.close(100.2)

    def test_existing_video_is_not_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "drive.avi"
            path.write_bytes(b"existing")
            with self.assertRaises(FileExistsError):
                VideoRecorder(path)
            self.assertEqual(path.read_bytes(), b"existing")

    def test_requires_avi_path(self) -> None:
        with self.assertRaises(ValueError):
            VideoRecorder(Path("drive.mp4"))


if __name__ == "__main__":
    unittest.main()
