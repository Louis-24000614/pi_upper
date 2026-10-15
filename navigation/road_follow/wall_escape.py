"""视觉卡墙后的局部脱困，不修改拓扑路线或封闭当前道路。"""

from __future__ import annotations

import math
import queue
from dataclasses import dataclass

from road_follow.control import VelocityCommand, heading_hold_command, is_visual_follow


@dataclass(frozen=True)
class WallEscapeConfig:
    """卡墙确认、短距离倒车和重新识路参数。"""

    enabled: bool = True
    confirm_s: float = 0.8
    min_lateral_m: float = 0.12
    severe_lateral_m: float = 0.18
    min_abs_omega: float = 0.35
    severe_abs_omega: float = 0.45
    lateral_growth_m: float = 0.05
    max_forward_depth_m: float = 0.22
    reverse_distance_m: float = 0.12
    reverse_speed_mps: float = 0.05
    reverse_max_s: float = 4.0
    stop_settle_s: float = 0.5
    reacquire_frames: int = 3
    reacquire_timeout_s: float = 3.0
    max_attempts_per_edge: int = 1
    heading_hold_gain: float = 2.0
    max_reverse_omega: float = 0.25


def wall_escape_config_from_mapping(cfg: dict) -> WallEscapeConfig:
    raw = cfg.get("wall_escape", {}) or {}
    return WallEscapeConfig(
        enabled=bool(raw.get("enabled", True)),
        confirm_s=max(0.1, float(raw.get("confirm_s", 0.8))),
        min_lateral_m=max(0.0, float(raw.get("min_lateral_m", 0.12))),
        severe_lateral_m=max(0.0, float(raw.get("severe_lateral_m", 0.18))),
        min_abs_omega=max(0.0, float(raw.get("min_abs_omega", 0.35))),
        severe_abs_omega=max(0.0, float(raw.get("severe_abs_omega", 0.45))),
        lateral_growth_m=max(0.0, float(raw.get("lateral_growth_m", 0.05))),
        max_forward_depth_m=max(0.0, float(raw.get("max_forward_depth_m", 0.22))),
        reverse_distance_m=max(0.01, float(raw.get("reverse_distance_m", 0.12))),
        reverse_speed_mps=max(0.01, float(raw.get("reverse_speed_mps", 0.05))),
        reverse_max_s=max(0.5, float(raw.get("reverse_max_s", 4.0))),
        stop_settle_s=max(0.0, float(raw.get("stop_settle_ms", 500)) / 1000.0),
        reacquire_frames=max(1, int(raw.get("reacquire_frames", 3))),
        reacquire_timeout_s=max(0.5, float(raw.get("reacquire_timeout_s", 3.0))),
        max_attempts_per_edge=max(1, int(raw.get("max_attempts_per_edge", 1))),
        heading_hold_gain=max(0.0, float(raw.get("heading_hold_gain", 2.0))),
        max_reverse_omega=max(0.0, float(raw.get("max_reverse_omega", 0.25))),
    )


@dataclass
class WallEscape:
    """idle → stopping → reversing → turn_wait → turning → reacquire → idle。"""

    phase: str = "idle"
    edge_key: tuple[str | None, str | None, str | None] | None = None
    attempts: int = 0
    candidate_started_s: float | None = None
    candidate_progress_m: float = 0.0
    candidate_abs_x_m: float = 0.0
    trusted_progress_m: float = 0.0
    hold_yaw_rad: float | None = None
    side: str = "none"
    stop_started_s: float | None = None
    reverse_started_s: float | None = None
    reacquire_started_s: float | None = None
    turn_clear_frames: int = 0
    failure_reason: str = ""


@dataclass(frozen=True)
class WallEscapeOutcome:
    command: VelocityCommand
    owns: bool = False
    restore_progress: bool = False
    reset_smoother: bool = False


def _stop(reason: str) -> WallEscapeOutcome:
    return WallEscapeOutcome(VelocityCommand(0.0, 0.0, reason), owns=True)


def _reset_candidate(state: WallEscape) -> None:
    state.candidate_started_s = None
    state.candidate_progress_m = 0.0
    state.candidate_abs_x_m = 0.0


def _away_from_wall(near_x_m: float | None) -> str:
    """中心线在左侧时向左脱离右墙，反之向右脱离左墙。"""
    if near_x_m is None or not math.isfinite(near_x_m):
        return "left"
    return "left" if near_x_m < 0.0 else "right"


def _fail(state: WallEscape, reason: str) -> WallEscapeOutcome:
    state.phase = "fault"
    state.failure_reason = reason
    return _stop(reason)


def _consume_notes(state: WallEscape, notes: queue.Queue[str], now_s: float) -> None:
    while True:
        try:
            note = notes.get_nowait()
        except queue.Empty:
            return
        if state.phase in ("stopping", "turn_wait") and note == "STOP_DONE":
            state.stop_started_s = now_s
        elif state.phase in ("stopping", "turn_wait") and note == "STOP_FAIL":
            state.phase = "fault"
            state.failure_reason = "stop_wall_escape"
        elif state.phase == "turning" and note == "TURN_DONE":
            state.phase = "reacquire"
            state.reacquire_started_s = None
            state.turn_clear_frames = 0
        elif state.phase == "turning" and note == "TURN_FAIL":
            state.phase = "fault"
            state.failure_reason = "turn_wall_escape"


def step_wall_escape(
    state: WallEscape,
    *,
    cfg: WallEscapeConfig,
    armed: bool,
    edge_key: tuple[str | None, str | None, str | None] | None,
    command: VelocityCommand,
    near_x_m: float | None,
    forward_depth_m: float | None,
    progress_m: float,
    now_s: float,
    yaw_rad: float | None,
    visual_reacquired: bool,
    notes: queue.Queue[str],
    send,
) -> tuple[WallEscape, WallEscapeOutcome]:
    """更新卡墙检测和局部脱困状态机。"""
    if edge_key != state.edge_key and state.phase == "idle":
        state.edge_key = edge_key
        state.attempts = 0
        _reset_candidate(state)

    if state.phase == "fault":
        return state, _stop(state.failure_reason or "stop_wall_escape")

    if state.phase == "idle":
        if not cfg.enabled or not armed or state.attempts >= cfg.max_attempts_per_edge:
            _reset_candidate(state)
            return state, WallEscapeOutcome(command)
        if command.v_mps <= 0.0 or not math.isfinite(progress_m):
            _reset_candidate(state)
            return state, WallEscapeOutcome(command)
        if near_x_m is None or not math.isfinite(near_x_m):
            _reset_candidate(state)
            return state, WallEscapeOutcome(command)

        abs_x = abs(float(near_x_m))
        abs_omega = abs(float(command.omega_radps))
        bad = abs_x >= cfg.min_lateral_m and abs_omega >= cfg.min_abs_omega
        if not bad:
            _reset_candidate(state)
            return state, WallEscapeOutcome(command)
        if state.candidate_started_s is None:
            state.candidate_started_s = now_s
            state.candidate_progress_m = progress_m
            state.candidate_abs_x_m = abs_x
        elapsed = now_s - state.candidate_started_s
        growth = abs_x - state.candidate_abs_x_m
        severe = abs_x >= cfg.severe_lateral_m and abs_omega >= cfg.severe_abs_omega
        close_wall = forward_depth_m is not None and forward_depth_m <= cfg.max_forward_depth_m
        confirmed = elapsed >= cfg.confirm_s and close_wall and (
            severe or growth >= cfg.lateral_growth_m
        )
        if not confirmed:
            return state, WallEscapeOutcome(command)

        state.attempts += 1
        state.trusted_progress_m = state.candidate_progress_m
        state.hold_yaw_rad = (
            float(yaw_rad) if yaw_rad is not None and math.isfinite(yaw_rad) else None
        )
        state.side = _away_from_wall(near_x_m)
        _reset_candidate(state)
        if not send("stop"):
            return state, _fail(state, "stop_wall_escape")
        state.phase = "stopping"
        state.stop_started_s = None
        return state, _stop("wall_escape_trigger")

    _consume_notes(state, notes, now_s)
    if state.phase == "fault":
        return state, _stop(state.failure_reason or "stop_wall_escape")

    if state.phase == "stopping":
        if state.stop_started_s is None:
            return state, _stop("wall_escape_stop_wait")
        if now_s - state.stop_started_s < cfg.stop_settle_s:
            return state, _stop("wall_escape_stop_settle")
        state.phase = "reversing"
        state.reverse_started_s = now_s

    if state.phase == "reversing":
        started = state.reverse_started_s if state.reverse_started_s is not None else now_s
        reverse_duration = min(cfg.reverse_max_s, cfg.reverse_distance_m / cfg.reverse_speed_mps)
        if now_s - started < reverse_duration:
            if state.hold_yaw_rad is not None and yaw_rad is not None:
                hold = heading_hold_command(
                    -cfg.reverse_speed_mps,
                    yaw_rad,
                    state.hold_yaw_rad,
                    cfg.heading_hold_gain,
                    cfg.max_reverse_omega,
                )
                return state, WallEscapeOutcome(
                    VelocityCommand(hold.v_mps, hold.omega_radps, "wall_escape_reverse"),
                    owns=True,
                )
            return state, WallEscapeOutcome(
                VelocityCommand(-cfg.reverse_speed_mps, 0.0, "wall_escape_reverse"),
                owns=True,
            )
        if not send("stop"):
            return state, _fail(state, "stop_wall_escape")
        state.phase = "turn_wait"
        state.stop_started_s = None
        return state, _stop("wall_escape_reverse_done")

    if state.phase == "turn_wait":
        if state.stop_started_s is None:
            return state, _stop("wall_escape_turn_wait")
        if now_s - state.stop_started_s < cfg.stop_settle_s:
            return state, _stop("wall_escape_turn_settle")
        if not send(f"turn {state.side}"):
            return state, _fail(state, "turn_wall_escape")
        state.phase = "turning"
        return state, _stop("wall_escape_turning")

    if state.phase == "turning":
        return state, _stop("wall_escape_turning")

    if state.phase == "reacquire":
        if state.reacquire_started_s is None:
            state.reacquire_started_s = now_s
        if now_s - state.reacquire_started_s > cfg.reacquire_timeout_s:
            return state, _fail(state, "stop_wall_escape_reacquire")
        if visual_reacquired and is_visual_follow(command):
            state.turn_clear_frames += 1
        else:
            state.turn_clear_frames = 0
        if state.turn_clear_frames >= cfg.reacquire_frames:
            state.phase = "idle"
            state.side = "none"
            state.hold_yaw_rad = None
            state.reverse_started_s = None
            state.stop_started_s = None
            state.reacquire_started_s = None
            state.turn_clear_frames = 0
            return state, WallEscapeOutcome(
                VelocityCommand(0.0, 0.0, "wall_escape_reacquired"),
                owns=True,
                restore_progress=True,
                reset_smoother=True,
            )
        return state, _stop("wall_escape_reacquire")

    return state, _fail(state, "stop_wall_escape")
