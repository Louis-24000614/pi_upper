"""录像编码移到单独线程，有界队列避免写盘积压拖慢视觉消费者。"""
from pathlib import Path
import queue
import threading
import numpy as np
from road_follow.recording import VideoRecorder


class AsyncVideoRecorder:
    """仅 worker 访问 VideoWriter；保留固定时间轴，明确统计丢弃的输入帧。"""

    def __init__(self, path: Path, fps: int = 10, capacity: int = 2):
        if capacity <= 0:
            raise ValueError("录像队列容量必须大于零")
        self._recorder = VideoRecorder(path, fps)
        self.path = path
        self._queue = queue.Queue(maxsize=capacity)
        self._error = None
        self._closed = False
        self._shape = None
        self.dropped = 0
        self.accepted = 0
        self._thread = threading.Thread(target=self._run, name="road-video-writer")
        self._thread.start()

    @property
    def frames_written(self):
        return self._recorder.frames_written

    def _check_error(self):
        if self._error is not None:
            raise RuntimeError("异步录像失败") from self._error

    def write(self, frame: np.ndarray, captured_s: float):
        self._check_error()
        if self._closed:
            raise RuntimeError("录像已经关闭")
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("录像只接受 uint8 BGR 三通道相机帧")
        if self._shape is not None and frame.shape != self._shape:
            raise ValueError("相机分辨率在录制中改变")
        self._shape = frame.shape
        # 拷贝后移交所有权，避免采集或推理重用数组时污染尚未编码的帧。
        item = (frame.copy(), captured_s)
        while True:
            try:
                self._queue.put_nowait(item)
                self.accepted += 1
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass

    def _run(self):
        try:
            while True:
                frame, timestamp = self._queue.get()
                if frame is None:
                    self._recorder.close(timestamp)
                    return
                self._recorder.write(frame, timestamp)
        except BaseException as exc:
            self._error = exc
            self._recorder.close()

    def close(self, stopped_s=None):
        if self._closed:
            self._check_error()
            return
        self._closed = True
        # 先通知在途补帧结束；调用线程只设置 Event，不访问编码器。
        self._recorder.request_close()
        # 保存剩余真实帧，结束时间交给编码线程做有界补帧和封尾。
        while self._thread.is_alive():
            try:
                self._queue.put((None, stopped_s), timeout=.05)
                break
            except queue.Full:
                self._check_error()
        self._thread.join()
        self._check_error()
