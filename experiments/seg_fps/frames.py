"""有界解码队列与结果时效检查；帧元数据和图像始终作为同一对象传递。"""
from dataclasses import dataclass
from pathlib import Path
import queue
import threading
import time
import cv2
import numpy as np


@dataclass(frozen=True)
class Frame:
    sequence: int
    video_s: float
    published_s: float
    decoded_s: float
    source: int
    image: np.ndarray


class ResultGate:
    """有序消费者拒绝旧来源、旧编号与超时结果，拒绝项不能推进状态机。"""

    def __init__(self, source: int = 0, max_age_s: float | None = None):
        self.source, self.max_age_s = source, max_age_s
        self.last = -1
        self.counts = {"old_source": 0, "out_of_order": 0, "expired": 0}

    def switch_source(self, source: int):
        """切换源版本时重置编号；旧源在途结果仍由 source 字段拒绝。"""
        self.source = source
        self.last = -1

    def accept(self, frame: Frame, now_s: float) -> bool:
        if frame.source != self.source:
            self.counts["old_source"] += 1
            return False
        if frame.sequence <= self.last:
            self.counts["out_of_order"] += 1
            return False
        if self.max_age_s is not None and now_s-frame.published_s > self.max_age_s:
            self.counts["expired"] += 1
            return False
        self.last = frame.sequence
        return True


class VideoFrames:
    """FIFO 用背压保证不丢帧；latest 仅替换尚未开始的帧，队列大小恒定。"""

    def __init__(self, path: Path, *, threaded=False, latest=False,
                 realtime=False, speed=1.0, loop=False, capacity=2):
        if speed <= 0 or capacity <= 0:
            raise ValueError("播放速度和队列容量必须大于零")
        self.path, self.latest, self.realtime = path, latest, realtime
        self.speed, self.loop = speed, loop
        # 实时回放的发布钟独立于消费者；否则等待下一帧会遮住已完成结果。
        self.threaded = threaded or latest or realtime
        self.queue = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.dropped = 0
        self.error = None
        self.thread = None
        self.decode_ms = []

    def _iterate(self):
        cap = cv2.VideoCapture(str(self.path))
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"无法读取录像: {self.path}")
        fps = cap.get(cv2.CAP_PROP_FPS)
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if fps <= 0 or count <= 0:
            cap.release()
            raise ValueError("录像缺少有效 FPS/帧数")
        start, sequence, decoded = time.monotonic(), 0, 0
        try:
            while not self.stop.is_set():
                t0 = time.monotonic()
                ok, image = cap.read()
                t1 = time.monotonic()
                if not ok:
                    # 已冻结素材声明帧数与实解帧数一致；中途损坏不能当正常 EOF。
                    if decoded < count:
                        raise RuntimeError(f"录像中途读取失败: {self.path}, {decoded}/{count}")
                    if not self.loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    decoded = 0
                    continue
                decoded += 1
                self.decode_ms.append((t1-t0)*1000)
                due = start + sequence / fps / self.speed
                if self.realtime and self.stop.wait(max(0, due-time.monotonic())):
                    break
                yield Frame(sequence, sequence/fps, due if self.realtime else t0,
                            t1, 0, image)
                sequence += 1
        finally:
            cap.release()

    def _produce(self):
        try:
            for frame in self._iterate():
                while not self.stop.is_set():
                    try:
                        # latest 满队列时立即替换；等待 50ms 会人为增加帧龄。
                        self.queue.put(frame, timeout=0 if self.latest else 0.05)
                        break
                    except queue.Full:
                        if self.latest:
                            try:
                                self.queue.get_nowait()
                                self.dropped += 1
                            except queue.Empty:
                                pass
        except BaseException as exc:
            self.error = exc
        finally:
            while not self.stop.is_set():
                try:
                    self.queue.put(None, timeout=0.05)
                    break
                except queue.Full:
                    pass

    def __iter__(self):
        if not self.threaded:
            yield from self._iterate()
            return
        self.start()
        while True:
            value = self.queue.get()
            if value is None:
                if self.error:
                    raise self.error
                return
            yield value

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._produce, name="seg-video-decode")
            self.thread.start()

    def read_nowait(self):
        """消费者有在途结果时不阻塞等下一次视频发布，避免人为增加结果年龄。"""
        self.start()
        value = self.queue.get_nowait()
        if value is None and self.error:
            raise self.error
        return value

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("解码线程未退出")
