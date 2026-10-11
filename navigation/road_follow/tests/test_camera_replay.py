"""核验 A/B 的独立发布钟及驱动丢帧计数，防止读取完成时刻遮住积压。"""
import unittest
from unittest.mock import patch
import numpy as np
from experiments.seg_fps.camera_replay import ClockedCapture


class Clock:
    def __init__(self):
        self.now = 0.0
    def monotonic(self):
        return self.now


class Stop:
    def __init__(self, clock):
        self.clock, self.stopped = clock, False
    def wait(self, delay):
        if not self.stopped:
            self.clock.now += delay
        return self.stopped
    def is_set(self):
        return self.stopped
    def set(self):
        self.stopped = True


class Video:
    def __init__(self, clock, delay=0):
        self.clock, self.delay, self.index = clock, delay, 0
    def isOpened(self):
        return True
    def get(self, key):
        return 10
    def read(self):
        self.clock.now += self.delay
        image = np.full((2,2,3), self.index, np.uint8)
        self.index += 1
        return True, image
    def grab(self):
        self.index += 1
        return True
    def set(self, key, value):
        self.index = int(value)
        return True
    def release(self):
        pass


class CameraReplayTest(unittest.TestCase):
    def test_slow_reader_keeps_published_age_and_counts_driver_drops(self):
        clock = Clock()
        with patch('experiments.seg_fps.camera_replay.cv2.VideoCapture', return_value=Video(clock)), \
                patch('experiments.seg_fps.camera_replay.time.monotonic', side_effect=clock.monotonic):
            capture = ClockedCapture('fake.avi',10,3)
            capture.stop = Stop(clock)
            self.assertEqual(int(capture.read()[1][0,0,0]),0)
            clock.now = 1
            self.assertEqual(int(capture.read()[1][0,0,0]),8)
            self.assertAlmostEqual(capture.captured_s,.8)
            self.assertEqual(capture.camera_dropped,7)
            self.assertEqual(int(capture.read()[1][0,0,0]),9)
            self.assertEqual(int(capture.read()[1][0,0,0]),0)
            self.assertAlmostEqual(capture.captured_s,1)

    def test_decode_time_is_not_reset_as_capture_time_and_stop_wakes_read(self):
        clock = Clock()
        with patch('experiments.seg_fps.camera_replay.cv2.VideoCapture', return_value=Video(clock,.2)), \
                patch('experiments.seg_fps.camera_replay.time.monotonic', side_effect=clock.monotonic):
            capture = ClockedCapture('fake.avi',10,3)
            capture.stop = Stop(clock)
            self.assertTrue(capture.read()[0])
            self.assertEqual(capture.captured_s,0)
            self.assertAlmostEqual(clock.now,.2)
            capture.stop_reading()
            self.assertEqual(capture.read(),(False,None))


if __name__ == '__main__':
    unittest.main()
