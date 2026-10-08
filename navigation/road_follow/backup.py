"""遇障后使用视觉闭环负速度，沿原路倒回上一个拓扑节点。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from road_follow.control import VelocityCommand

Point2D = tuple[float, float]


@dataclass(frozen=True)
class BackupConfig:
    """视觉倒车速度、纠偏限制和安全停止条件。"""

    reverse_speed_mps: float = 0.08
    lookahead_y_m: float = 0.28
    max_abs_omega: float = 0.4
    near_y_min_m: float = 0.20
    near_y_max_m: float = 0.35
    done_progress_m: float = 0.0
    max_distance_m: float = 1.10
    max_duration_s: float = 16.0
    max_missing_frames: int = 3


@dataclass
class EdgeProgress:
    """离开上一个路口之后的沿边进度。清零的是这个标量，不是下位机里程计。"""

    s_m: float = 0.0
    yaw_rad: float | None = None
    _x: float | None = None
    _y: float | None = None
    last_sample_s: float | None = None

    def reset(self) -> None:
        self.s_m = 0.0
        self.yaw_rad = None
        self._x = None
        self._y = None
        self.last_sample_s = None

    def update(self, x_m: float, y_m: float, yaw_rad: float, received_s: float | None = None) -> float:
        """用本帧航向把平面位移投到走廊上。倒车时 s 减小，但不小于 0。"""
        if not all(math.isfinite(value) for value in (x_m, y_m, yaw_rad)):
            self.last_sample_s = None
            return self.s_m
        self.last_sample_s = time.monotonic() if received_s is None else received_s
        self.yaw_rad = float(yaw_rad)
        if self._x is not None and self._y is not None:
            ds = (x_m - self._x) * math.cos(yaw_rad) + (y_m - self._y) * math.sin(yaw_rad)
            self.s_m = max(0.0, self.s_m + ds)
        self._x = x_m
        self._y = y_m
        return self.s_m

    def is_fresh(self, now_s: float, timeout_s: float = 0.5) -> bool:
        return self.last_sample_s is not None and 0 <= now_s - self.last_sample_s <= timeout_s


@dataclass
class Backup:
    """idle → backing → done；视觉或里程异常进入 fault。"""

    phase: str = "idle"
    started_s: float = 0.0
    start_progress_m: float = 0.0
    reverse_distance_m: float = 0.0
    missing_frames: int = 0


def near_lane_x(
    points: list[Point2D] | tuple[Point2D, ...],
    y_min_m: float = 0.20,
    y_max_m: float = 0.35,
) -> float | None:
    """近处中心线平均横向位置，供倒车期间持续纠偏。"""
    band = [float(x) for x, y in points if y_min_m <= float(y) <= y_max_m]
    if len(band) < 2:
        return None
    return sum(band) / len(band)


def near_lane_heading(
    points: list[Point2D] | tuple[Point2D, ...],
    y_min_m: float = 0.20,
    y_max_m: float = 0.35,
) -> float | None:
    """近处中心线相对车头的方向。向右偏为正，平行偏移仍是 0。"""
    band = [(float(x), float(y)) for x, y in points if y_min_m <= float(y) <= y_max_m]
    if len(band) < 2:
        return None
    y_mean = sum(y for _, y in band) / len(band)
    x_mean = sum(x for x, _ in band) / len(band)
    var_y = sum((y - y_mean) ** 2 for _, y in band)
    if var_y < 1e-6:
        return None
    cov = sum((y - y_mean) * (x - x_mean) for x, y in band)
    return math.atan2(cov, var_y)


def reverse_omega(near_x_m: float, cfg: BackupConfig) -> float:
    """负线速度下的 Pure Pursuit 修正；符号与正向循迹相反。"""
    y = max(1e-3, abs(cfg.lookahead_y_m))
    denom = near_x_m * near_x_m + y * y
    speed = -abs(cfg.reverse_speed_mps)
    omega = -speed * (2.0 * near_x_m) / denom
    limit = abs(cfg.max_abs_omega)
    return max(-limit, min(limit, omega))


def _stop(reason: str) -> VelocityCommand:
    return VelocityCommand(0.0, 0.0, reason)


def step_backup(
    state: Backup,
    triggered: bool,
    near_x_m: float | None,
    progress_s_m: float,
    command: VelocityCommand,
    now_s: float,
    cfg: BackupConfig | None = None,
) -> tuple[Backup, VelocityCommand]:
    """硬堵塞后以负 `CMD_VEL` 视觉倒车，沿边进度回到 0 才算回到路口中心。"""
    cfg = cfg or BackupConfig()

    if state.phase == "fault":
        return state, _stop("stop_backup_fault")
    if state.phase == "done":
        return state, _stop("backup_done")

    if state.phase == "idle":
        if not triggered:
            return state, command
        if progress_s_m <= cfg.done_progress_m:
            state.phase = "done"
            return state, _stop("backup_done_at_entry")
        state.phase = "backing"
        state.started_s = now_s
        state.start_progress_m = max(0.0, progress_s_m)
        state.reverse_distance_m = 0.0
        state.missing_frames = 0
        # 触发帧先发零速；下一帧才开始负速度，避免从正向速度直接跳成倒车。
        return state, _stop("stop_backup")

    state.reverse_distance_m = max(
        state.reverse_distance_m,
        max(0.0, state.start_progress_m - progress_s_m),
    )
    if progress_s_m <= cfg.done_progress_m:
        state.phase = "done"
        return state, _stop("backup_done")
    if state.reverse_distance_m >= cfg.max_distance_m:
        state.phase = "fault"
        return state, _stop("stop_backup_distance_limit")
    if now_s - state.started_s >= cfg.max_duration_s:
        state.phase = "fault"
        return state, _stop("stop_backup_timeout")

    if near_x_m is None:
        state.missing_frames += 1
        if state.missing_frames > cfg.max_missing_frames:
            state.phase = "fault"
            return state, _stop("stop_backup_road_lost")
        return state, _stop("stop_backup_no_vision")

    state.missing_frames = 0
    return state, VelocityCommand(
        -abs(cfg.reverse_speed_mps),
        reverse_omega(near_x_m, cfg),
        "visual_backup",
    )
