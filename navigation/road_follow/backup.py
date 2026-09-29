"""遇障后用近处路面摆正，再按沿边里程计距离倒回上一个路口。"""

from __future__ import annotations

import math
import queue
from dataclasses import dataclass

from road_follow.control import VelocityCommand

Point2D = tuple[float, float]


@dataclass(frozen=True)
class BackupConfig:
    """摆正窗口和倒车距离帽。距离传给下位机的 kBackward。"""

    align_x_m: float = 0.02
    align_frames: int = 3
    align_timeout_s: float = 3.0
    align_y_m: float = 0.28
    align_speed_mps: float = 0.06
    max_abs_omega: float = 0.4
    near_y_min_m: float = 0.20
    near_y_max_m: float = 0.35
    max_distance_m: float = 0.70
    speed_mmps: int = 100


@dataclass
class EdgeProgress:
    """离开上一个路口之后的沿边进度。清零的是这个标量，不是下位机里程计。"""

    s_m: float = 0.0
    _x: float | None = None
    _y: float | None = None

    def reset(self) -> None:
        self.s_m = 0.0
        self._x = None
        self._y = None

    def update(self, x_m: float, y_m: float, yaw_rad: float) -> float:
        """用本帧航向把平面位移投到走廊上。倒车时 s 减小，但不小于 0。"""
        if self._x is not None and self._y is not None:
            ds = (x_m - self._x) * math.cos(yaw_rad) + (y_m - self._y) * math.sin(yaw_rad)
            self.s_m = max(0.0, self.s_m + ds)
        self._x = x_m
        self._y = y_m
        return self.s_m


@dataclass
class Backup:
    """idle → align → backing → done。倒车期间不再发视觉速度。"""

    phase: str = "idle"
    distance_mm: int = 0
    started_s: float = 0.0
    clear: int = 0


def near_lane_x(
    points: list[Point2D] | tuple[Point2D, ...],
    y_min_m: float = 0.20,
    y_max_m: float = 0.35,
) -> float | None:
    """近处中心线的平均横向位置。牌子在更远处，不参与摆正。"""
    band = [float(x) for x, y in points if y_min_m <= float(y) <= y_max_m]
    if len(band) < 2:
        return None
    return sum(band) / len(band)


def backup_distance_mm(s_m: float, cfg: BackupConfig) -> int:
    """沿边进度换成倒车毫米数，不超过 700 mm，也不超过协议的 1000 mm。"""
    capped = min(max(0.0, s_m), cfg.max_distance_m, 1.0)
    return int(round(capped * 1000.0))


def align_omega(near_x_m: float, cfg: BackupConfig) -> float:
    """中心线在右侧时向右转。角速度符号与前进寻线相同，线速度保持 0。"""
    denom = near_x_m * near_x_m + cfg.align_y_m * cfg.align_y_m
    if denom <= 1e-8:
        return 0.0
    omega = -cfg.align_speed_mps * (2.0 * near_x_m) / denom
    limit = abs(cfg.max_abs_omega)
    return max(-limit, min(limit, omega))


def _start_reverse(state: Backup, send, cfg: BackupConfig) -> VelocityCommand:
    """近处路能摆正就摆正；摆不正时用触发时记下的距离直接倒。"""
    if state.distance_mm < 1:
        state.phase = "fault"
        return VelocityCommand(0.0, 0.0, "stop_backup_no_distance")
    request = f"backward {state.distance_mm} {cfg.speed_mmps}"
    if not send(request):
        state.phase = "fault"
        return VelocityCommand(0.0, 0.0, "stop_backup_send_fail")
    state.phase = "backing"
    return VelocityCommand(0.0, 0.0, "backup_backing")


def _drain(notes: queue.Queue[str]) -> list[str]:
    found: list[str] = []
    while True:
        try:
            found.append(notes.get_nowait())
        except queue.Empty:
            return found


def step_backup(
    state: Backup,
    triggered: bool,
    near_x_m: float | None,
    progress_s_m: float,
    command: VelocityCommand,
    notes: queue.Queue[str],
    send,
    now_s: float,
    cfg: BackupConfig | None = None,
) -> tuple[Backup, VelocityCommand]:
    """障碍连续出现后摆正并定距倒车。idle 时原样返回寻线指令。"""
    cfg = cfg or BackupConfig()
    received = _drain(notes)

    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_backup_fail")
    if state.phase == "done":
        return state, VelocityCommand(0.0, 0.0, "backup_done")

    if state.phase == "idle":
        if not triggered:
            return state, command
        state.phase = "align"
        state.started_s = now_s
        state.clear = 0
        state.distance_mm = backup_distance_mm(progress_s_m, cfg)
        return state, VelocityCommand(0.0, 0.0, "stop_backup")

    if state.phase == "align":
        if now_s - state.started_s > cfg.align_timeout_s or near_x_m is None:
            return state, _start_reverse(state, send, cfg)
        if abs(near_x_m) <= cfg.align_x_m:
            state.clear += 1
        else:
            state.clear = 0
        if state.clear >= cfg.align_frames:
            return state, _start_reverse(state, send, cfg)
        return state, VelocityCommand(0.0, align_omega(near_x_m, cfg), "backup_align")

    if any(note.startswith("BACKWARD_FAIL") for note in received):
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_backup_fail")
    if any(note.startswith("BACKWARD_DONE") for note in received):
        state.phase = "done"
        return state, VelocityCommand(0.0, 0.0, "backup_done")
    return state, VelocityCommand(0.0, 0.0, "backup_backing")
