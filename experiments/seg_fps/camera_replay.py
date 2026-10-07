"""独立发布钟的虚拟相机：明确模拟驱动 FIFO，避免误把文件 FPS 当相机 FPS。"""
import math
import threading
import time
import cv2


class ClockedCapture:
    """以指定发布率逐帧解码，读取迟到时仅保留 driver_capacity 个历史画面。"""

    def __init__(self, path, fps=30.0, driver_capacity=4):
        if fps < 0 or driver_capacity < 1:
            raise ValueError('发布率不能为负，驱动容量至少为 1')
        self.cap = cv2.VideoCapture(str(path))
        self.expected = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not self.cap.isOpened() or self.expected <= 0:
            self.cap.release()
            raise RuntimeError('虚拟相机无法读取有效录像')
        self.fps, self.driver_capacity = fps, driver_capacity
        self.stop = threading.Event()
        self.start_s = None
        self.sequence = self.decoded = self.reads = self.camera_dropped = 0
        self.captured_s = 0.0
        self.decode_ms = []

    def _next(self, skip=False):
        if self.decoded == self.expected:
            if not self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0):
                raise RuntimeError('录像循环定位失败')
            self.decoded = 0
        if skip:
            ok, image = self.cap.grab(), None
        else:
            ok, image = self.cap.read()
        if not ok:
            raise RuntimeError(f'录像中途读取失败: {self.decoded}/{self.expected}')
        self.decoded += 1
        return image

    def read(self):
        now = time.monotonic()
        if self.start_s is None:
            self.start_s = now
        if self.fps:
            published = math.floor((now-self.start_s)*self.fps)
            oldest = max(self.sequence, published-self.driver_capacity+1)
            # 驱动主动丢弃超出 FIFO 的画面；此计数与应用单帧槽替换分开报告。
            while self.sequence < oldest:
                self._next(skip=True)
                self.sequence += 1
                self.camera_dropped += 1
            due = self.start_s+self.sequence/self.fps
            if self.stop.wait(max(0, due-time.monotonic())):
                return False, None
        else:
            due = time.monotonic()
        if self.stop.is_set():
            return False, None
        began = time.monotonic()
        image = self._next()
        self.decode_ms.append((time.monotonic()-began)*1000)
        self.captured_s = due
        self.sequence += 1
        self.reads += 1
        return True, image

    def stop_reading(self):
        self.stop.set()

    def release(self):
        self.cap.release()
