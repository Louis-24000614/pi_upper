"""有界、有序的相机分割流水线；每个 context 私有模型与可写内存。"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
import time
import numpy as np
from road_follow.segment import RoadSegmenter


class CameraReadError(RuntimeError):
    """采集失败与模型异常分开：前者允许清空旧画面后按原逻辑重试。"""


@dataclass(frozen=True)
class SegmentedFrame:
    sequence: int
    source: int
    captured_s: float
    image: np.ndarray
    mask: np.ndarray
    inference_s: float
    started_s: float = 0.0
    finished_s: float = 0.0


class OrderedSegmentStream:
    """每个 context 最多一个在途帧，按采集序消费，不能让快结果覆盖慢结果。"""

    def __init__(self, model: Path, cores=(1,2,4), *, correct_nms=False,
                 reuse_buffers=False, backend="lite", factory=RoadSegmenter,
                 capture_timestamp=None):
        if not cores:
            raise ValueError("至少需要一个 NPU context")
        self.segments = [factory(model, core_mask=core, correct_nms=correct_nms,
                                 reuse_buffers=reuse_buffers, backend=backend) for core in cores]
        self.workers = [ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"road-npu-{core}")
                        for core in cores]
        self.pending = deque()
        self.sequence, self.source = 0, 0
        self.closed = False
        # 回放可传入独立发布钟；生产默认 read 完成时刻，保持原入口兼容。
        self._capture_timestamp = capture_timestamp

    def warmup(self, image):
        try:
            for segment, worker in zip(self.segments, self.workers):
                worker.submit(segment.mask, image).result()
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _work(segment, sequence, source, captured_s, image):
        began = time.monotonic()
        mask = segment.mask(image)
        finished = time.monotonic()
        return SegmentedFrame(sequence, source, captured_s, image, mask,
                              finished-began, began, finished)

    def read(self, capture, on_capture=None):
        if self.closed:
            raise RuntimeError("分割流水线已经关闭")
        while len(self.pending) < len(self.workers):
            # 优先消费完成的队首，避免阻塞等下一次相机发布而增加结果年龄。
            if self.pending and self.pending[0].done():
                break
            ok, image = capture.read()
            captured_s = self._capture_timestamp() if self._capture_timestamp else time.monotonic()
            if not ok:
                raise CameraReadError("相机读取失败")
            # 采集缓冲可能被驱动重用；进入异步 worker 前移交独立图像所有权。
            image = image.copy()
            if on_capture is not None:
                on_capture(image, captured_s)
            index = self.sequence % len(self.workers)
            self.pending.append(self.workers[index].submit(
                self._work, self.segments[index], self.sequence, self.source, captured_s, image))
            self.sequence += 1
        result = self.pending.popleft().result()
        if result.source != self.source:
            raise RuntimeError("旧来源结果进入消费者")
        return result

    def invalidate(self):
        """转弯后的 EMA 重置同时清空旧画面，禁止它们再次污染新方向状态。"""
        self.source += 1
        for future in self.pending:
            future.cancel()
        self.pending.clear()
        # 已开始的同步推理无法中断，但其结果不再消费。每个 worker 串行执行，
        # 下一代任务必须等待旧任务结束，不能同时复用同一个模型的输入缓冲。

    def close(self):
        if self.closed:
            return
        self.closed = True
        for worker in self.workers:
            worker.shutdown(wait=True, cancel_futures=True)
        for segment in self.segments:
            segment.close()
