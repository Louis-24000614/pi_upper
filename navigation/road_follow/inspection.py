"""涵洞两侧任务：独立新帧确认、一次翻转、部分未确认也允许结束检查。"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict
import threading
import time

from road_follow.inspection_io import FrameHub, Recognizer, ServoBridge, SideCamera


class SideInspection:
    def __init__(self, settings, hub, servo, recognizer, *, event=None, notify=None, pool=None):
        self.settings, self.hub, self.servo, self.recognizer = settings, hub, servo, recognizer
        self.event = event or (lambda *args, **kwargs: None)
        # 预留结果事件；默认不发送任何 UART/语音消息。
        self.notify = notify or (lambda result: None)
        config, self.revision = settings.snapshot()
        self.config = config
        self.side = config["servo"]["initial_side"]
        self.pool = pool or ThreadPoolExecutor(max_workers=1, thread_name_prefix="culvert-recognize")
        self.lock = threading.RLock()
        self.phase, self.closed = "idle", False
        self.future, self.job = None, None
        self.generation = 0
        self.camera = self.web = None
        self.target = None
        self.sides = []
        self.deadline_s = self.next_sample_s = 0.0
        self.count, self.identity, self.last_sequence = 0, None, -1
        self.candidate = None
        self.last_reason = "waiting"
        self.token = None
        self.result = None
        self.failure_reason = None

    def _emit(self, kind, **values):
        self.event(kind, side=self.side, phase=self.phase, **values)

    def _reset_count(self):
        self.count, self.identity, self.candidate = 0, None, None

    def _begin_side(self, now):
        self.hub.invalidate(self.side)
        self.phase = "recognizing"
        self.deadline_s = now+self.config["recognition"]["side_timeout_s"]
        self.next_sample_s = now
        self.last_sequence = -1
        self.last_reason = "waiting_fresh_frame"
        self._reset_count()
        self._emit("inspection_side_started", deadline_s=self.deadline_s)

    def start(self, now_s, target):
        with self.lock:
            if self.closed or getattr(self.servo, "closed", False):
                raise RuntimeError("侧视任务或 PWM 桥已关闭")
            self.generation += 1
            self.config, self.revision = self.settings.snapshot()
            self.target, self.sides, self.result = target, [], None
            self.failure_reason = None
            self._begin_side(now_s)

    def _finish_side(self, now, confirmed):
        record = {"direction": self.side, "angle_deg": self.config["servo"]["angles"][self.side],
                  "status": "confirmed" if confirmed else "unconfirmed", "confirmed_s": now,
                  "category": self.candidate.get("category") if confirmed else None,
                  "identity": self.candidate.get("identity") if confirmed else None,
                  "score": self.candidate.get("score") if confirmed else None,
                  "valid_frames": self.count if confirmed else 0,
                  "settings_revision": self.revision,
                  "parameters": deepcopy(self.config["recognition"]),
                  "reason": "confirmed" if confirmed else "side_timeout:"+self.last_reason}
        self.sides.append(record)
        self._emit("inspection_side_result", result=record)
        self.notify(deepcopy(record))
        self.generation += 1  # 第一侧仍在途的 HTTP 结果不能进入第二侧。
        if len(self.sides) == 2:
            self.phase = "done"
            self.result = {"status": "done" if all(s["status"] == "confirmed" for s in self.sides) else "partial",
                           "sides": deepcopy(self.sides), "speech_enabled": False}
            self._emit("inspection_finished", result=self.result)
            return
        self.next_side = "B" if self.side == "A" else "A"
        self.hub.invalidate(self.next_side)
        self.phase = "turning"
        self.token = self.servo.request(self.config["servo"]["angles"][self.next_side])
        self.turn_deadline_s = now+self.config["servo"]["ack_timeout_s"]
        self._reset_count()
        self._emit("inspection_servo_requested", next_side=self.next_side)

    def _fail(self, reason):
        self.failure_reason = reason
        self.phase = "failed"
        self.generation += 1
        self._reset_count()
        self.servo.close()
        self._emit("inspection_failed", reason=reason)
        return "failed"

    def step(self, now_s):
        with self.lock:
            try:
                return self._step(now_s)
            except Exception as exc:
                return self._fail("inspection_error:"+str(exc))

    def _step(self, now):
        if self.phase == "done":
            return "done"
        if self.phase in ("failed", "cancelled") or self.closed:
            return "failed"
        if self.phase == "idle":
            return "running"
        if hasattr(self.servo, "alive") and not self.servo.alive():
            return self._fail("servo_bridge_exited")
        # 在当前侧保留既有 deadline；超时参数修改从下一侧开始使用。
        config, revision = self.settings.snapshot()
        if revision != self.revision:
            self.config, self.revision = config, revision
            self.generation += 1
            self.hub.invalidate(self.side)
            self._reset_count()
            self.last_sequence = -1
            self._emit("inspection_settings_changed", revision=revision)
        if self.phase == "turning":
            status = self.servo.poll(self.token)
            if status == "failed" or (status != "done" and now >= self.turn_deadline_s):
                return self._fail("servo_write_failed_or_timeout")
            if status == "done":
                self.side = self.next_side
                self.phase = "settling"
                self.settle_until_s = now+self.config["servo"]["settle_s"]
                self._emit("inspection_servo_applied", settle_until_s=self.settle_until_s)
            return "running"
        if self.phase == "settling":
            if now >= self.settle_until_s:
                self._begin_side(now)
            return "running"
        if self.phase != "recognizing":
            return "failed"
        if now >= self.deadline_s:
            self._finish_side(now, False)
            return "done" if self.phase == "done" else "running"
        if self.future is not None and self.future.done():
            future, job = self.future, self.job
            self.future = self.job = None
            if job["generation"] == self.generation and job["revision"] == self.revision and job["epoch"] == self.hub.generation():
                try:
                    candidate = future.result()
                except Exception as exc:
                    candidate = {"accepted": False, "reason": "request_failed:"+str(exc)}
                self.candidate = candidate
                self.last_reason = candidate["reason"]
                if candidate.get("accepted"):
                    identity = (candidate["category"], candidate["identity"])
                    self.count = self.count+1 if identity == self.identity else 1
                    self.identity = identity
                else:
                    self.count, self.identity = 0, None
                self._emit("inspection_sample", frame_id=job["frame_id"], candidate=candidate, valid_frames=self.count)
                if self.count >= self.config["recognition"]["confirm_frames"]:
                    self._finish_side(now, True)
                    return "done" if self.phase == "done" else "running"
        frame = self.hub.latest(now)
        if frame is None:
            self.count, self.identity = 0, None
            self.last_reason = "camera_no_fresh_frame"
        elif self.future is None and now >= self.next_sample_s and frame.sequence != self.last_sequence and frame.side == self.side:
            self.last_sequence = frame.sequence
            self.next_sample_s = now+self.config["recognition"]["sample_interval_s"]
            self.job = {"generation": self.generation, "revision": self.revision, "epoch": frame.epoch, "frame_id": frame.sequence}
            self.future = self.pool.submit(self.recognizer.recognize, frame, deepcopy(self.config["recognition"]))
        return "running"

    def cancel(self, reason="cancelled"):
        with self.lock:
            self.generation += 1
            self.phase, self.failure_reason = "cancelled", reason
            self._reset_count()
            if self.future is not None:
                self.future.cancel()
            self.servo.close()

    def status(self):
        with self.lock:
            return {"phase": self.phase, "side": self.side, "valid_frames": self.count,
                    "candidate": deepcopy(self.candidate), "sides": deepcopy(self.sides),
                    "remaining_s": max(0, self.deadline_s-time.monotonic()) if self.phase == "recognizing" else None,
                    "failure_reason": self.failure_reason, "reason": self.last_reason,
                    "result": deepcopy(self.result), "speech_enabled": False,
                    "target": asdict(self.target) if self.target is not None else None}

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.cancel("closed")
        try:
            if self.web is not None:
                self.web.close()
        finally:
            try:
                if self.camera is not None:
                    self.camera.close()
            finally:
                self.pool.shutdown(wait=False, cancel_futures=True)


def create_inspection(settings, root, *, web_enabled=False, event=None):
    """显式启用后的初始化；导航开始前确认首次 PWM 写入和姿态等待。"""
    config, _ = settings.snapshot()
    hub = FrameHub(config["servo"]["initial_side"])
    servo = ServoBridge(root, config["servo"])
    executor = SideInspection(settings, hub, servo, Recognizer(config["services"]), event=event)
    try:
        executor.camera = SideCamera(config["camera"], hub)
        token = servo.request(config["servo"]["angles"][executor.side])
        limit = time.monotonic()+config["servo"]["ack_timeout_s"]
        while True:
            state = servo.poll(token)
            if state == "done":
                break
            if state == "failed" or time.monotonic() >= limit:
                raise OSError("首次 PWM 写入失败；不会开始导航")
            time.sleep(.02)
        time.sleep(config["servo"]["settle_s"])
        if hub.latest() is None:
            raise OSError("侧视相机没有新鲜画面；不会开始导航")
        hub.invalidate(executor.side)
        if web_enabled:
            from road_follow.inspection_web import InspectionWeb
            executor.web = InspectionWeb(settings, hub, executor.status)
        return executor
    except BaseException:
        executor.close()
        raise
