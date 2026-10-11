"""独立读取、单帧替换及空闲 context 取帧；消费者仍独占导航状态。"""
from collections import deque
import threading
import time

from road_follow.parallel_segment import CameraReadError, OrderedSegmentStream


class LatestSegmentStream(OrderedSegmentStream):
    """只替换未开工的图像，已开工的推理完成后交给有序/时效检查。"""

    def __init__(self, *args, capture_timestamp=None, result_order="completion", **kwargs):
        if result_order not in ("completion", "capture"):
            raise ValueError("结果顺序必须为 completion 或 capture")
        super().__init__(*args, **kwargs)
        self._condition = threading.Condition()
        self._latest = None
        self._ready = deque()
        self._result_order = result_order
        # 必须在取走输入的同一个锁内登记在途序号。否则快结果先完成时，
        # 消费者可能误以为没有更早的帧，仍然造成导航时间顺序倒退。
        self._inflight = {}
        self._reader = None
        self._capture = None
        self._on_capture = None
        self._capture_timestamp = capture_timestamp
        self._camera_error = None
        self._fatal_error = None
        self._last_returned = -1
        self.stats = dict(captured=0, replaced_input=0, started=0, completed=0,
                          old_source=0, out_of_order=0, invalidated_input=0,
                          max_waiting=0, max_results=0)

    def start(self, capture, on_capture=None):
        """预热后启动；采集句柄及录像回调此后只由读取线程访问。"""
        with self._condition:
            if self.closed:
                raise RuntimeError("分割流水线已经关闭")
            if self._reader is not None:
                if capture is not self._capture:
                    raise ValueError("切换采集句柄前必须关闭旧流水线")
                return
            self._capture, self._on_capture = capture, on_capture
            self._reader = threading.Thread(target=self._read_camera,
                                           name="road-latest-reader", daemon=True)
            for index, worker in enumerate(self.workers):
                worker.submit(self._infer, index)
            self._reader.start()

    def _read_camera(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self.closed or self._fatal_error is not None
                                             or self._camera_error is None)
                    if self.closed or self._fatal_error is not None:
                        return
                    generation = self.source
                ok, image = self._capture.read()
                # 默认记录 read 完成时刻。录像模拟器可以提供独立发布钟，避免把
                # 驱动中已经积压的画面误算为刚拍摄；真实硬件需另测曝光到消费延迟。
                captured_s = (self._capture_timestamp() if self._capture_timestamp
                              else time.monotonic())
                if not ok:
                    with self._condition:
                        if not self.closed:
                            self._camera_error = CameraReadError("相机读取失败")
                            self._condition.notify_all()
                    continue
                # 驱动可能反复写同一缓冲；复制后才发布，推理中的图像不能被新帧覆盖。
                image = image.copy()
                if self._on_capture is not None:
                    self._on_capture(image, captured_s)
                with self._condition:
                    if self.closed:
                        return
                    sequence = self.sequence
                    self.sequence += 1
                    self.stats["captured"] += 1
                    if generation != self.source:
                        # TURN_DONE 发生在阻塞 read 期间时，也不能把旧方向帧标成新一代。
                        self.stats["old_source"] += 1
                        continue
                    if self._latest is not None:
                        self.stats["replaced_input"] += 1
                    self._latest = (sequence, generation, captured_s, image)
                    self.stats["max_waiting"] = 1
                    self._condition.notify_all()
        except BaseException as exc:
            with self._condition:
                self._fatal_error = exc
                self._condition.notify_all()
        finally:
            # 仅读取线程释放句柄，避免另一线程 release 与 V4L2/OpenCV read 并发。
            try:
                self._capture.release()
            except BaseException as exc:
                with self._condition:
                    self._fatal_error = exc
                    self._condition.notify_all()

    def _infer(self, index):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self.closed or self._fatal_error is not None
                                             or (self._latest is not None
                                                 and self._camera_error is None))
                    if self.closed or self._fatal_error is not None:
                        return
                    packet, self._latest = self._latest, None
                    self._inflight[index] = (packet[0], packet[1])
                    self.stats["started"] += 1
                # 每个线程始终使用自己的模型/输入工作区，不排队提交第二个推理任务。
                result = self._work(self.segments[index], *packet)
                with self._condition:
                    self._inflight.pop(index, None)
                    self.stats["completed"] += 1
                    if self.closed:
                        return
                    if result.source != self.source:
                        self.stats["old_source"] += 1
                        continue
                    self._ready.append((index, result))
                    self.stats["max_results"] = max(self.stats["max_results"], len(self._ready))
                    self._condition.notify_all()
                    # 每个 context 最多保留一个未消费结果。下游变慢时背压结果端，
                    # 输入端仍持续替换最新帧，不能用无限结果队列造成堆积。
                    self._condition.wait_for(lambda: self.closed or self._fatal_error is not None
                        or not any(worker == index for worker, _ in self._ready))
                    if self.closed or self._fatal_error is not None:
                        return
        except BaseException as exc:
            with self._condition:
                self._fatal_error = exc
                self._condition.notify_all()

    def _next_ready_index(self):
        """调用方持有条件锁；仅等待已开工的更早帧，不等待被替换的序号。"""
        if not self._ready:
            return None
        if self._result_order == "completion":
            return 0
        index = min(range(len(self._ready)), key=lambda i: self._ready[i][1].sequence)
        result = self._ready[index][1]
        if any(source == self.source and sequence < result.sequence
               for sequence, source in self._inflight.values()):
            return None
        # 结果端仍受每个 context 一份结果的背压约束，排序不能引入无界缓存。
        # 等待可能增加帧龄；导航没有固定帧龄上限，代际和结果顺序检查仍保留。
        return index

    def read(self, capture, on_capture=None):
        self.start(capture, on_capture)
        with self._condition:
            while True:
                self._condition.wait_for(lambda: self.closed or self._fatal_error is not None
                                         or self._camera_error is not None
                                         or self._next_ready_index() is not None)
                if self._fatal_error is not None:
                    raise self._fatal_error
                if self.closed:
                    raise RuntimeError("分割流水线已经关闭")
                if self._camera_error is not None:
                    raise self._camera_error
                index = self._next_ready_index()
                _, result = self._ready[index]
                del self._ready[index]
                self._condition.notify_all()
                if result.source != self.source:
                    self.stats["old_source"] += 1
                    continue
                if result.sequence <= self._last_returned:
                    # 不等待慢 context 阻塞更新；其晚到结果不能倒序推进 EMA/路口状态。
                    self.stats["out_of_order"] += 1
                    continue
                self._last_returned = result.sequence
                return result

    def invalidate(self):
        with self._condition:
            self.source += 1
            self.stats["invalidated_input"] += self._latest is not None
            self.stats["old_source"] += len(self._ready)
            self._latest = None
            self._ready.clear()
            self._camera_error = None
            # 已开始的 RKNN 调用不取消；结束后按 source 拒绝旧结果。
            self._condition.notify_all()

    def close(self):
        with self._condition:
            if self.closed:
                return
            self.closed = True
            self._latest = None
            self._ready.clear()
            self._condition.notify_all()
        if self._capture is not None and hasattr(self._capture, "stop_reading"):
            # 可中断的回放源用事件唤醒；V4L2 由 read 自身返回，不并发 release。
            self._capture.stop_reading()
        if self._reader is not None:
            self._reader.join(timeout=5)
        for worker in self.workers:
            worker.shutdown(wait=True, cancel_futures=True)
        for segment in self.segments:
            segment.close()
        if self._reader is not None and self._reader.is_alive():
            raise RuntimeError("相机读取线程 5 秒内未退出，采集句柄等待该线程自行释放")

    @property
    def owns_capture(self):
        return self._reader is not None
