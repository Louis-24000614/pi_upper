"""涵洞两侧任务：独立新帧确认、一次翻转、部分未确认也允许结束检查。"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict
import threading
import time

from road_follow.inspection_io import FrameHub, Recognizer, ServoBridge, SideCamera
from road_follow.inspection_pairs import paired_knife_identity, PAIR_SPEECH_DELAY_S


class SideInspection:
    def __init__(self, settings, hub, servo, recognizer, *, event=None, notify=None, pool=None,
                 speech_enabled=False, pair_fallback=False):
        self.settings, self.hub, self.servo, self.recognizer = settings, hub, servo, recognizer
        self.event = event or (lambda *args, **kwargs: None)
        # 默认保持只输出结果；正式导航由 runtime 注入共用 UART 桥的语音请求。
        self.notify = notify or (lambda result: None)
        self.speech_enabled = speech_enabled
        self.pair_fallback = pair_fallback
        self.knife_observations = {}
        self.pending_announcement = None
        self.inferred_announcements = []
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

    def _parameters(self):
        return deepcopy(self.config["recognition"])

    def _expected_knife(self):
        for record in self.sides:
            if record["status"] == "confirmed" and record["category"] == "suspect":
                return paired_knife_identity(record["identity"])
        return None

    def _begin_side(self, now):
        self.hub.invalidate(self.side)
        self.phase = "recognizing"
        self.deadline_s = now+self._parameters()["side_timeout_s"]
        self.next_sample_s = now
        self.last_sequence = -1
        self.last_reason = "waiting_fresh_frame"
        self._reset_count()
        self._emit("inspection_side_started", deadline_s=self.deadline_s)

    def start(self, now_s, target, *, initial_turn=False):
        with self.lock:
            if self.closed or getattr(self.servo, "closed", False):
                raise RuntimeError("侧视任务或 PWM 桥已关闭")
            self.generation += 1
            self.config, self.revision = self.settings.snapshot()
            self.target, self.sides, self.result = target, [], None
            self.failure_reason = None
            self.pending_announcement = None
            self.inferred_announcements = []
            self.knife_observations = {}
            if self.future is not None and self.future.cancel():
                self.future = self.job = None
            if initial_turn:
                self._turn_to(self.config["servo"]["initial_side"], now_s)
            else:
                self._begin_side(now_s)

    def _record(self, now, confirmed):
        return {"direction": self.side, "angle_deg": self.config["servo"]["angles"][self.side],
                  "status": "confirmed" if confirmed else "unconfirmed", "confirmed_s": now,
                  "category": self.candidate.get("category") if confirmed else None,
                  "identity": self.candidate.get("identity") if confirmed else None,
                  "score": self.candidate.get("score") if confirmed else None,
                  "valid_frames": self.count if confirmed else 0,
                  "settings_revision": self.revision,
                  "parameters": self._parameters(),
                  "reason": "confirmed" if confirmed else "side_timeout:"+self.last_reason}

    def _finish_side(self, now, confirmed):
        record = self._record(now, confirmed)
        if self.pair_fallback and not self.sides and not confirmed and self.knife_observations:
            record = deepcopy(max(self.knife_observations.values(), key=lambda item: item["score"]))
        self.sides.append(record)
        self._emit("inspection_side_result", result=record)
        deferred = self.pair_fallback and len(self.sides) == 1 and record["category"] == "knife"
        if not deferred:
            self.notify({**deepcopy(record), "edge_id": self.target.edge_id})
        self.generation += 1  # 第一侧仍在途的 HTTP 结果不能进入第二侧。
        if len(self.sides) == 2:
            if self.pair_fallback:
                self._resolve_pair(now)
            if self.pending_announcement is not None:
                self.phase = "announcing"
            else:
                self._complete()
            return
        self._turn_to("B" if self.side == "A" else "A", now)

    def _resolve_pair(self, now):
        first, second = self.sides
        expected = self._expected_knife()
        if expected is None:
            if first["status"] == "confirmed" and first["category"] == "knife":
                if second["status"] == "confirmed":
                    self.pending_announcement = (now+PAIR_SPEECH_DELAY_S, deepcopy(first))
                else:
                    self.notify({**deepcopy(first), "edge_id": self.target.edge_id})
            return
        face_first = first["status"] == "confirmed" and first["category"] == "suspect"
        face, knife = (first, second) if face_first else (second, first)
        if not face_first and expected in self.knife_observations:
            knife = deepcopy(self.knife_observations[expected])
            self.sides[0] = knife
            self._emit("inspection_pair_verified", result=knife)
        if knife["status"] == "confirmed" and knife["identity"] == expected:
            if not face_first:
                self.pending_announcement = (now+PAIR_SPEECH_DELAY_S, deepcopy(knife))
            return
        if knife["status"] == "confirmed":
            knife["pair_match"] = False
        inferred = {"direction": knife["direction"], "angle_deg": knife["angle_deg"],
                    "status": "inferred", "category": "knife", "identity": expected,
                    "source": "face_pair", "based_on": face["identity"],
                    "score": None, "valid_frames": 0, "reason": "paired_knife_timeout",
                    "settings_revision": self.revision, "parameters": self._parameters()}
        if face_first:
            self._announce(inferred, now)
        else:
            self.pending_announcement = (now+PAIR_SPEECH_DELAY_S, inferred)

    def _announce(self, record, now):
        record = {**deepcopy(record), "announced_s": now}
        if record["status"] == "inferred":
            self.inferred_announcements.append(record)
            self._emit("inspection_pair_fallback", result=record)
        self.notify({**record, "edge_id": self.target.edge_id})

    def _complete(self):
        confirmed = all(s["status"] == "confirmed" and s.get("pair_match") is not False
                        for s in self.sides)
        self.phase = "done"
        self.result = {"status": "done" if confirmed and not self.inferred_announcements else "partial",
                       "sides": deepcopy(self.sides), "speech_enabled": self.speech_enabled}
        if self.pair_fallback:
            self.result.update(pair_fallback=True,
                               inferred_announcements=deepcopy(self.inferred_announcements))
        self._emit("inspection_finished", result=self.result)

    def _turn_to(self, side, now):
        self.next_side = side
        self.hub.invalidate(self.next_side)
        self.phase = "turning"
        self.token = self.servo.request(self.config["servo"]["angles"][self.next_side])
        self.turn_deadline_s = now+self.config["servo"]["ack_timeout_s"]
        self._reset_count()
        self._emit("inspection_servo_requested", next_side=self.next_side)

    def _fail(self, reason):
        self.failure_reason = reason
        self.phase = "failed"
        self.pending_announcement = None
        self.generation += 1
        self._reset_count()
        if self.servo is not None:
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
        if self.phase == "announcing":
            due, record = self.pending_announcement
            if now >= due:
                self.pending_announcement = None
                self._announce(record, now)
                self._complete()
                return "done"
            return "running"
        config, revision = self.settings.snapshot()
        if revision != self.revision:
            self.config, self.revision = config, revision
            self.generation += 1
            self.hub.invalidate(self.side)
            self._reset_count()
            self.last_sequence = -1
            if not self.sides:
                self.knife_observations.clear()
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
                expected = self._expected_knife() if self.pair_fallback else None
                if expected and candidate.get("accepted") and (
                        candidate.get("category") != "knife" or candidate.get("identity") != expected):
                    candidate = {**candidate, "accepted": False, "reason": "pair_mismatch"}
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
                    if self.pair_fallback and not self.sides and candidate["category"] == "knife":
                        self.knife_observations[candidate["identity"]] = self._record(now, True)
                    else:
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
            self.pending_announcement = None
            self._reset_count()
            if self.future is not None:
                self.future.cancel()
            if self.servo is not None:
                self.servo.close()

    def status(self):
        with self.lock:
            return {"phase": self.phase, "side": self.side, "valid_frames": self.count,
                    "candidate": deepcopy(self.candidate), "sides": deepcopy(self.sides),
                    "remaining_s": max(0, self.deadline_s-time.monotonic()) if self.phase == "recognizing" else None,
                    "failure_reason": self.failure_reason, "reason": self.last_reason,
                    "result": deepcopy(self.result), "speech_enabled": self.speech_enabled,
                    "pair_fallback": self.pair_fallback,
                    "effective_side_timeout_s": self._parameters()["side_timeout_s"],
                    "pending_announcement": ({"due_s": self.pending_announcement[0],
                                               "record": deepcopy(self.pending_announcement[1])}
                                              if self.pending_announcement else None),
                    "inferred_announcements": deepcopy(self.inferred_announcements),
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
                if isinstance(self.recognizer, Recognizer):
                    self.recognizer.close()


def create_inspection(settings, root, *, web_enabled=False, event=None, notify=None,
                      speech_enabled=False, pair_fallback=False):
    """显式启用后的初始化；导航开始前确认首次 PWM 写入和姿态等待。"""
    config, _ = settings.snapshot()
    hub = FrameHub(config["servo"]["initial_side"])
    servo = ServoBridge(root, config["servo"])
    executor = SideInspection(settings, hub, servo, Recognizer(config["services"]), event=event,
                              notify=notify, speech_enabled=speech_enabled, pair_fallback=pair_fallback)
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
