"""遇障后使用视觉闭环负速度，沿原路倒回上一个拓扑节点。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from road_follow.control import VelocityCommand, heading_hold_command

Point2D = tuple[float, float]


@dataclass(frozen=True)
class BackupConfig:
    """视觉倒车速度、纠偏限制和安全停止条件。"""

    reverse_speed_mps: float = 0.08
    lookahead_y_m: float = 0.28
    max_abs_omega: float = 0.4
    # 死区和帧间变化率限制：压住倒车时近带中心线抖动直接打轮的蛇形。
    omega_deadband_x_m: float = 0.03
    max_omega_step_radps: float = 0.15
    near_y_min_m: float = 0.20
    near_y_max_m: float = 0.35
    done_progress_m: float = 0.0
    max_distance_m: float = 1.10
    max_duration_s: float = 16.0
    max_missing_frames: int = 3
    max_no_vision_distance_m: float = 0.15
    max_no_vision_duration_s: float = 2.0
    max_no_vision_heading_rad: float = 0.10
    heading_hold_gain: float = 4.0
    reacquire_frames: int = 3


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
    hold_yaw_rad: float | None = None
    no_vision_started_s: float | None = None
    no_vision_start_m: float = 0.0
    no_vision_segment_m: float = 0.0
    no_vision_distance_m: float = 0.0
    clear_frames: int = 0
    last_frame_s: float | None = None
    last_omega_radps: float | None = None


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


def _lane_heading_fit(band: list[Point2D]) -> float | None:
    y_mean = sum(y for _, y in band) / len(band)
    x_mean = sum(x for x, _ in band) / len(band)
    var_y = sum((y - y_mean) ** 2 for _, y in band)
    if var_y < 1e-6:
        return None
    cov = sum((y - y_mean) * (x - x_mean) for x, y in band)
    return math.atan2(cov, var_y)


def forward_lane_heading(
    points: list[Point2D] | tuple[Point2D, ...],
    *,
    strip_x_m: float = 0.09,
    y_min_m: float = 0.14,
    y_max_m: float = 0.28,
    min_points: int = 3,
    base_y_max_m: float = 0.19,
    extend_max_dev_m: float = 0.02,
) -> float | None:
    """正前方近距窄条内的中心线方向，墙端摆正的抗开口污染参考。

    侧向开口的点横向超出窄条被剔除；墙角处混进窄条的行用"基带回归
    +逐行一致性延伸"截住：先只用近处基带定方向，再向深处逐行外推，
    偏差超限即停止延伸。条内点不足时返回 None：没有测量就不摆。
    """
    strip = sorted(
        (float(x), float(y))
        for x, y in points
        if y_min_m <= float(y) <= y_max_m + 1e-9 and abs(float(x)) <= abs(strip_x_m)
    )
    base = [p for p in strip if p[1] <= base_y_max_m + 1e-9]
    if len(base) < max(2, int(min_points)):
        return None
    heading = _lane_heading_fit(base)
    if heading is None:
        return None
    accepted = list(base)
    for x, y in strip:
        if y <= base_y_max_m + 1e-9:
            continue
        last_x, last_y = accepted[-1]
        predicted = last_x + math.tan(heading) * (y - last_y)
        if abs(x - predicted) > abs(extend_max_dev_m):
            break
        accepted.append((x, y))
        extended = _lane_heading_fit(accepted)
        if extended is not None:
            heading = extended
    return heading


def reverse_omega(near_x_m: float, cfg: BackupConfig) -> float:
    """负线速度下的 Pure Pursuit 修正；符号与正向循迹相反。

    横向偏差小于死区时不修正：白底白沿下中心线几厘米的抖动不值得打轮。
    """
    if abs(near_x_m) < max(0.0, cfg.omega_deadband_x_m):
        return 0.0
    y = max(1e-3, abs(cfg.lookahead_y_m))
    denom = near_x_m * near_x_m + y * y
    speed = -abs(cfg.reverse_speed_mps)
    omega = -speed * (2.0 * near_x_m) / denom
    limit = abs(cfg.max_abs_omega)
    return max(-limit, min(limit, omega))


def _rate_limited_omega(state: Backup, omega: float, cfg: BackupConfig) -> float:
    """限制相邻两帧 ω 的跳变幅度，倒车指令只能渐变不能甩。"""
    step = abs(cfg.max_omega_step_radps)
    prev = 0.0 if state.last_omega_radps is None else state.last_omega_radps
    if step > 0.0:
        omega = max(prev - step, min(prev + step, omega))
    state.last_omega_radps = omega
    return omega


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
    *,
    yaw_rad: float | None = None,
    odom_valid: bool = True,
    frame_captured_s: float | None = None,
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
        if not odom_valid or not math.isfinite(progress_s_m):
            state.phase = "fault"
            return state, _stop("stop_backup_odom_stale")
        if progress_s_m <= cfg.done_progress_m:
            state.phase = "done"
            return state, _stop("backup_done_at_entry")
        state.phase = "backing"
        state.started_s = now_s
        state.start_progress_m = max(0.0, progress_s_m)
        state.reverse_distance_m = 0.0
        state.missing_frames = 0
        state.hold_yaw_rad = yaw_rad if yaw_rad is not None and math.isfinite(yaw_rad) else None
        state.no_vision_started_s = None
        state.no_vision_segment_m = state.no_vision_distance_m = 0.0
        state.clear_frames = 0
        state.last_frame_s = None
        # 触发帧发零速，ω 变化率也从 0 起步。
        state.last_omega_radps = 0.0
        # 触发帧先发零速；下一帧才开始负速度，避免从正向速度直接跳成倒车。
        return state, _stop("stop_backup")

    if not odom_valid or not math.isfinite(progress_s_m):
        state.phase = "fault"
        return state, _stop("stop_backup_odom_stale")

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

    vision_ok = near_x_m is not None and math.isfinite(near_x_m)
    if not vision_ok and state.no_vision_started_s is None and state.hold_yaw_rad is not None:
        state.no_vision_started_s = now_s
        state.no_vision_start_m = progress_s_m
        state.no_vision_segment_m = 0.0
        state.clear_frames = 0
        state.last_frame_s = None

    if state.no_vision_started_s is not None:
        # A new loss of vision keeps the distance already used in this recovery.
        state.no_vision_segment_m = max(
            state.no_vision_segment_m, state.no_vision_start_m - progress_s_m
        )
        frame_fresh = (
            frame_captured_s is not None and math.isfinite(frame_captured_s)
            and 0.0 <= now_s - frame_captured_s <= 0.20
        )
        if not vision_ok or not frame_fresh:
            state.clear_frames = 0
        elif state.last_frame_s is None or frame_captured_s > state.last_frame_s:
            state.clear_frames += 1
            state.last_frame_s = frame_captured_s

        if state.clear_frames >= cfg.reacquire_frames:
            state.no_vision_distance_m += state.no_vision_segment_m
            state.no_vision_segment_m = 0.0
            state.no_vision_started_s = None
        else:
            used_m = state.no_vision_distance_m + state.no_vision_segment_m
            if used_m >= cfg.max_no_vision_distance_m - 1e-9:
                state.phase = "fault"
                return state, _stop("stop_backup_no_vision_distance")
            if now_s - state.no_vision_started_s >= cfg.max_no_vision_duration_s:
                state.phase = "fault"
                return state, _stop("stop_backup_no_vision_timeout")
            if yaw_rad is None or not math.isfinite(yaw_rad):
                state.phase = "fault"
                return state, _stop("stop_backup_heading_invalid")
            error = math.atan2(math.sin(state.hold_yaw_rad - yaw_rad),
                               math.cos(state.hold_yaw_rad - yaw_rad))
            if abs(error) > cfg.max_no_vision_heading_rad:
                state.phase = "fault"
                return state, _stop("stop_backup_heading_error")
            hold = heading_hold_command(-abs(cfg.reverse_speed_mps), yaw_rad,
                                        state.hold_yaw_rad, cfg.heading_hold_gain,
                                        cfg.max_abs_omega)
            omega = _rate_limited_omega(state, hold.omega_radps, cfg)
            return state, VelocityCommand(hold.v_mps, omega, "heading_backup")

    if not vision_ok:
        state.missing_frames += 1
        if state.missing_frames > cfg.max_missing_frames:
            state.phase = "fault"
            return state, _stop("stop_backup_road_lost")
        # 停车帧 ω 实发 0；视觉恢复后从 0 重新渐变。
        state.last_omega_radps = 0.0
        return state, _stop("stop_backup_no_vision")

    state.missing_frames = 0
    omega = _rate_limited_omega(state, reverse_omega(near_x_m, cfg), cfg)
    return state, VelocityCommand(
        -abs(cfg.reverse_speed_mps),
        omega,
        "visual_backup",
    )
