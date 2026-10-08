"""涵洞任务的纯状态机：沿边里程不清零，不直接操作设备。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from road_follow.control import VelocityCommand


@dataclass(frozen=True)
class CulvertConfig:
    length_m: float = 0.27
    min_distance_m: float = 0.20
    max_distance_m: float = 0.60
    max_lateral_m: float = 0.10
    max_heading_rad: float = 0.10
    confirm_frames: int = 3
    position_spread_m: float = 0.05
    max_frame_age_s: float = 0.20
    odom_timeout_s: float = 0.50
    stop_margin_m: float = 0.01
    stop_error_m: float = 0.05
    stop_ack_timeout_s: float = 2.0
    settle_s: float = 2.0
    settle_timeout_s: float = 5.0
    stable_position_m: float = 0.01
    stable_yaw_rad: float = 0.03
    task_s: float = 5.0
    enter_extra_s: float = 2.0
    reacquire_frames: int = 3
    reacquire_timeout_s: float = 5.0
    entry_distance_bias_m: float = 0.0

    @classmethod
    def from_mapping(cls, mapping):
        allowed = cls.__dataclass_fields__
        unknown = set(mapping) - set(allowed)
        if unknown:
            raise ValueError(f"未知涵洞参数: {sorted(unknown)}")
        config = cls(**mapping)
        for name in allowed:
            value = getattr(config, name)
            if name == "entry_distance_bias_m":
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("涵洞参数 entry_distance_bias_m 必须是有限数")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"涵洞参数 {name} 必须是有限正数")
        for name in ("confirm_frames", "reacquire_frames"):
            if not isinstance(getattr(config, name), int):
                raise ValueError(f"{name} 必须是整数")
        if config.min_distance_m >= config.max_distance_m or config.stop_error_m >= config.length_m / 2:
            raise ValueError("涵洞距离窗或停车误差不合法")
        if config.settle_timeout_s < config.settle_s:
            raise ValueError("停稳超时不能短于停稳窗口")
        return config


class OdomHistory:
    """同一条有向边的接收时钟历史，禁止外推或跨边插值。"""
    def __init__(self, config):
        self.config = config
        self.samples = deque(maxlen=256)
        self.key = None

    def reset(self, key):
        self.key = key
        self.samples.clear()

    def add(self, key, received_s, progress_m, x, y, yaw):
        if self.key != key:
            self.reset(key)
        if not all(math.isfinite(v) for v in (received_s, progress_m, x, y, yaw)):
            self.samples.clear()
            return
        if self.samples and received_s <= self.samples[-1][0]:
            return
        self.samples.append((received_s, progress_m, x, y, yaw))
        while len(self.samples) > 1 and received_s - self.samples[0][0] > 3:
            self.samples.popleft()

    def fresh(self, now):
        return bool(self.samples) and 0 <= now - self.samples[-1][0] <= self.config.odom_timeout_s

    def progress_at(self, captured_s):
        for a, b in zip(self.samples, list(self.samples)[1:]):
            if a[0] <= captured_s <= b[0] and b[0] - a[0] <= self.config.odom_timeout_s:
                ratio = (captured_s - a[0]) / (b[0] - a[0])
                return a[1] + ratio * (b[1] - a[1])
        if self.samples and self.samples[-1][0] == captured_s:
            return self.samples[-1][1]
        return None

    def stable(self, now, after_s):
        if not self.fresh(now):
            return False
        latest = self.samples[-1][0]
        points = [p for p in self.samples if p[0] >= after_s and p[0] >= latest - self.config.settle_s - .1]
        if len(points) < 2 or latest - points[0][0] < self.config.settle_s:
            return False
        if any(b[0] - a[0] > self.config.odom_timeout_s for a, b in zip(points, points[1:])):
            return False
        span = max(math.hypot(a[2] - b[2], a[3] - b[3]) for a in points for b in points)
        angles = [points[0][4]]
        for a, b in zip(points, points[1:]):
            delta = math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))
            angles.append(angles[-1] + delta)
        return span <= self.config.stable_position_m and max(angles) - min(angles) <= self.config.stable_yaw_rad


@dataclass(frozen=True)
class CulvertTarget:
    edge_id: str
    from_node: str
    to_node: str
    entrance_s_m: float
    target_s_m: float
    exit_s_m: float
    canonical_offset_m: float
    captured_s: float


class PauseTask:
    """可替换的任务执行器接口：start / step -> running, done, failed。"""
    def __init__(self, duration_s):
        self.duration_s = duration_s
        self.started_s = None

    def start(self, now_s, target):
        self.started_s = now_s

    def step(self, now_s):
        if self.started_s is None:
            return "failed"
        return "done" if now_s - self.started_s >= self.duration_s else "running"


@dataclass(frozen=True)
class CulvertOutcome:
    command: VelocityCommand
    owns: bool
    send_velocity: bool = True
    resumed: bool = False


class CulvertController:
    def __init__(self, config, history, *, send, event, records, executor=None):
        self.config, self.history = config, history
        self.send, self.event, self.records = send, event, records
        self.executor = executor or PauseTask(config.task_s)
        self.phase = "idle"
        self.target = None
        self.started_s = 0.0
        self.deadline_s = 0.0
        self.stop_requested_s = None
        self.stop_ack_s = None
        self.clear = 0
        self.last_reacquire_s = None
        self.fault_reason = None
        self.task_pose = None

    @property
    def owns(self):
        return self.phase != "idle"

    def _transition(self, phase, now, **details):
        before, self.phase = self.phase, phase
        self.event("culvert_phase", phase=phase, before=before, now_s=now, **details)

    def begin(self, target, now, current_s, near_speed):
        if self.owns or getattr(self.records, "handled", self.records.done)(target.edge_id):
            return False
        if not math.isfinite(near_speed) or near_speed <= 0:
            raise ValueError("涵洞进入速度必须大于零")
        self.target = target
        self.started_s = now
        self.deadline_s = now + max(0, target.target_s_m - current_s) / near_speed + self.config.enter_extra_s
        self.stop_requested_s = self.stop_ack_s = None
        self.clear = 0
        self.records.discover(target)
        self._transition("entering", now, target_s_m=target.target_s_m)
        return True

    def fault(self, reason, now):
        if self.phase != "fault":
            # STOP is allowed regardless of the former control mode.
            self.send("stop")
            self.fault_reason = reason
            if hasattr(self.executor, "cancel"):
                self.executor.cancel(reason)
            if self.target:
                self.records.fail(self.target.edge_id, reason)
            self._transition("fault", now, reason=reason)

    def cancel_for_obstacle(self, now):
        if hasattr(self.executor, "cancel"):
            self.executor.cancel("hard_obstacle")
        if self.target:
            self.records.fail(self.target.edge_id, "hard_obstacle")
        self.target = None
        self._transition("idle", now, reason="hard_obstacle")

    def step(self, *, now, current_s, edge_key, visual_ok, frame_s, command, near_speed, notes=()):
        cfg = self.config
        if not self.owns:
            return CulvertOutcome(command, False)
        if self.phase != "fault":
            if self.target and edge_key != (self.target.edge_id, self.target.from_node, self.target.to_node):
                self.fault("edge_changed", now)
            elif not self.history.fresh(now):
                self.fault("odom_stale", now)
            elif not visual_ok:
                self.fault("vision_unsafe", now)
            elif "STOP_FAIL" in notes:
                self.fault("stop_failed", now)
        stop = VelocityCommand(0.0, 0.0, "stop_culvert_" + self.phase)
        if self.phase == "fault":
            return CulvertOutcome(stop, True, False)
        target = self.target
        if self.phase == "entering":
            remaining = target.target_s_m - current_s
            if now >= self.deadline_s:
                self.fault("enter_timeout", now)
            elif remaining <= cfg.stop_margin_m:
                self.stop_requested_s = now
                if self.send("stop"):
                    self._transition("stopping", now, remaining_m=remaining)
                else:
                    self.fault("stop_submit_failed", now)
            else:
                # Lower speed and steering together to retain the same curvature.
                speed = min(command.v_mps, near_speed)
                if speed <= 0:
                    self.fault("road_lost", now)
                else:
                    omega = command.omega_radps * speed / command.v_mps
                    return CulvertOutcome(VelocityCommand(speed, omega, "culvert_enter"), True)
        elif self.phase == "stopping":
            if "STOP_DONE" in notes:
                self.stop_ack_s = now
                self._transition("settling", now)
            elif now - self.stop_requested_s >= cfg.stop_ack_timeout_s:
                self.fault("stop_ack_timeout", now)
        elif self.phase == "settling":
            if self.history.stable(now, self.stop_ack_s):
                if not target.entrance_s_m < current_s < target.exit_s_m or abs(current_s - target.target_s_m) > cfg.stop_error_m:
                    self.fault("stop_position_error", now)
                else:
                    self.task_pose = self.history.samples[-1][2:5]
                    try:
                        self.executor.start(now, target)
                    except Exception as exc:
                        self.fault("task_start_failed", now)
                        self.event("culvert_task_error", error=str(exc))
                    else:
                        self._transition("task", now)
            elif now - self.stop_ack_s >= cfg.settle_timeout_s:
                self.fault("settle_timeout", now)
        elif self.phase == "task":
            x, y, yaw = self.history.samples[-1][2:5]
            ox, oy, oyaw = self.task_pose
            yaw_delta = math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))
            if math.hypot(x - ox, y - oy) > cfg.stable_position_m or abs(yaw_delta) > cfg.stable_yaw_rad:
                self.fault("moved_during_task", now)
            else:
                try:
                    result = self.executor.step(now)
                except Exception as exc:
                    self.event("culvert_task_error", error=str(exc))
                    result = "failed"
                if result == "failed":
                    self.fault("task_failed", now)
                elif result == "done":
                    if abs(current_s - target.target_s_m) > cfg.stop_error_m:
                        self.fault("stop_position_error", now)
                    else:
                        inspection = getattr(self.executor, "result", None)
                        if inspection is not None:
                            self.records.complete_inspection(target.edge_id, current_s-target.target_s_m, inspection)
                        else:
                            self.records.complete(target.edge_id, current_s-target.target_s_m)
                        self.deadline_s = now + cfg.reacquire_timeout_s
                        self.last_reacquire_s = frame_s
                        self._transition("reacquire", now)
                elif result != "running":
                    self.fault("invalid_task_result", now)
        elif self.phase == "reacquire":
            if frame_s is not None and (self.last_reacquire_s is None or frame_s > self.last_reacquire_s):
                self.last_reacquire_s = frame_s
                self.clear = self.clear + 1 if command.v_mps > 0 else 0
            if self.clear >= cfg.reacquire_frames:
                self._transition("idle", now)
                self.target = None
                return CulvertOutcome(VelocityCommand(0, 0, "culvert_resume"), True, True, True)
            if now >= self.deadline_s:
                self.fault("reacquire_timeout", now)
        return CulvertOutcome(VelocityCommand(0, 0, "stop_culvert_" + self.phase), True,
                              self.phase == "reacquire")
