"""视觉路口到下位机定距直行/90°转弯的状态机。"""

from __future__ import annotations

import queue
from dataclasses import dataclass

from road_follow.control import VelocityCommand


@dataclass(frozen=True)
class JunctionTurnConfig:
    stable_frames: int = 2
    handoff_min_distance_m: float = 0.18
    handoff_max_distance_m: float = 0.55
    camera_ahead_of_turn_center_m: float = 0.0
    stop_before_center_m: float = 0.0
    forward_speed_mmps: int = 50
    min_forward_mm: int = 50
    max_forward_mm: int = 650
    reacquire_frames: int = 3
    road_end_enabled: bool = True
    road_end_min_y_m: float = 0.48
    road_end_max_y_m: float = 0.56
    road_end_beyond_turn_center_m: float = 0.10
    road_end_max_abs_lane_x_m: float = 0.08
    road_end_min_lane_width_m: float = 0.14
    road_end_max_lane_width_m: float = 0.32
    branch_observe_min_distance_m: float = 0.55
    branch_observe_max_distance_m: float = 0.95
    branch_latch_max_frames: int = 200


@dataclass(frozen=True)
class JunctionCue:
    """分割几何给出的待转方向和路口中心距离。"""

    detected: bool
    side: str = "none"
    distance_m: float | None = None
    source: str = "none"


@dataclass
class JunctionTurn:
    """follow → approach → forward → turning → reacquire。"""

    phase: str = "follow"
    side: str = "none"
    arm: int = 0
    clear: int = 0
    forward_mm: int = 0
    branch_latched: bool = False
    approach_age: int = 0


def junction_turn_config_from_mapping(cfg: dict) -> JunctionTurnConfig:
    raw = cfg.get("junction_turn", {}) or {}
    return JunctionTurnConfig(
        stable_frames=max(1, int(raw.get("stable_frames", 2))),
        handoff_min_distance_m=float(raw.get("handoff_min_distance_m", 0.18)),
        handoff_max_distance_m=float(raw.get("handoff_max_distance_m", 0.55)),
        camera_ahead_of_turn_center_m=float(
            raw.get("camera_ahead_of_turn_center_m", 0.0)
        ),
        stop_before_center_m=float(raw.get("stop_before_center_m", 0.0)),
        forward_speed_mmps=max(1, int(raw.get("forward_speed_mmps", 50))),
        min_forward_mm=max(1, int(raw.get("min_forward_mm", 50))),
        max_forward_mm=max(1, int(raw.get("max_forward_mm", 650))),
        reacquire_frames=max(1, int(raw.get("reacquire_frames", 3))),
        road_end_enabled=bool(raw.get("road_end_enabled", True)),
        road_end_min_y_m=float(raw.get("road_end_min_y_m", 0.48)),
        road_end_max_y_m=float(raw.get("road_end_max_y_m", 0.56)),
        road_end_beyond_turn_center_m=float(
            raw.get("road_end_beyond_turn_center_m", 0.10)
        ),
        road_end_max_abs_lane_x_m=float(
            raw.get("road_end_max_abs_lane_x_m", 0.08)
        ),
        road_end_min_lane_width_m=float(
            raw.get("road_end_min_lane_width_m", 0.14)
        ),
        road_end_max_lane_width_m=float(
            raw.get("road_end_max_lane_width_m", 0.32)
        ),
        branch_observe_min_distance_m=float(
            raw.get("branch_observe_min_distance_m", 0.55)
        ),
        branch_observe_max_distance_m=float(
            raw.get("branch_observe_max_distance_m", 0.95)
        ),
        branch_latch_max_frames=max(
            1, int(raw.get("branch_latch_max_frames", 200))
        ),
    )


def road_end_turn_cue(
    *,
    side: str,
    stable_blocked: bool,
    raw_blocked: bool,
    command: VelocityCommand,
    road_end_y_m: float | None,
    lane_x_m: float,
    lane_width_m: float,
    cfg: JunctionTurnConfig,
    approach_latched: bool = False,
) -> JunctionCue:
    """侧路被墙遮住时，用仍可见的道路末端提前交接。"""
    detected = (
        cfg.road_end_enabled
        and side in ("left", "right")
        and (stable_blocked or approach_latched)
        and raw_blocked
        and command.reason == "follow"
        and road_end_y_m is not None
        and cfg.road_end_min_y_m <= road_end_y_m <= cfg.road_end_max_y_m
        and abs(lane_x_m) <= cfg.road_end_max_abs_lane_x_m
        and cfg.road_end_min_lane_width_m
        <= lane_width_m
        <= cfg.road_end_max_lane_width_m
    )
    if not detected:
        return JunctionCue(False)
    turn_center_m = road_end_y_m - cfg.road_end_beyond_turn_center_m
    return JunctionCue(True, side, turn_center_m, "road_end")


def step_junction_turn(
    state: JunctionTurn,
    cue: JunctionCue,
    command: VelocityCommand,
    notes: queue.Queue[str],
    send,
    cfg: JunctionTurnConfig,
) -> tuple[JunctionTurn, VelocityCommand]:
    """在视觉仍可靠时交接；有限动作期间不再发送 ``CMD_VEL``。"""
    _consume_notes(state, notes, send)

    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_action_fail")

    if state.phase in ("follow", "approach"):
        if state.phase == "approach":
            state.approach_age += 1
            if state.approach_age > cfg.branch_latch_max_frames:
                state.phase = "follow"
                state.side = "none"
                state.branch_latched = False
                state.approach_age = 0
                state.arm = 0

        branch_observed = (
            cue.detected
            and cue.source == "side_branch"
            and cue.side in ("left", "right")
            and cue.distance_m is not None
            and cfg.branch_observe_min_distance_m
            <= cue.distance_m
            <= cfg.branch_observe_max_distance_m
        )
        if branch_observed:
            state.phase = "approach"
            state.side = cue.side
            state.branch_latched = True
            state.approach_age = 0

        in_window = (
            cue.detected
            and cue.side in ("left", "right")
            and cue.distance_m is not None
            and command.reason == "follow"
            and cfg.handoff_min_distance_m
            <= cue.distance_m
            <= cfg.handoff_max_distance_m
        )
        state.arm = state.arm + 1 if in_window else 0
        required_frames = (
            1
            if state.branch_latched and cue.source == "road_end"
            else cfg.stable_frames
        )
        if state.arm < required_frames:
            return state, command

        distance_m = (
            float(cue.distance_m)
            + cfg.camera_ahead_of_turn_center_m
            - cfg.stop_before_center_m
        )
        forward_mm = int(round(distance_m * 1000.0))
        if not (cfg.min_forward_mm <= forward_mm <= cfg.max_forward_mm):
            state.arm = 0
            return state, VelocityCommand(0.0, 0.0, "stop_bad_turn_distance")

        state.side = cue.side
        state.forward_mm = forward_mm
        state.arm = 0
        if send(f"forward {forward_mm} {cfg.forward_speed_mmps}"):
            state.phase = "forward"
            return state, VelocityCommand(0.0, 0.0, "blind_forward")
        return state, VelocityCommand(0.0, 0.0, "forward_wait")

    if state.phase in ("forward", "turning"):
        return state, VelocityCommand(0.0, 0.0, state.phase)

    # 转完后先原地观察；新方向中心线连续稳定才重新交给视觉速度环。
    if command.reason == "follow":
        state.clear += 1
    else:
        state.clear = 0
    if state.clear >= cfg.reacquire_frames:
        state.phase = "follow"
        state.side = "none"
        state.forward_mm = 0
        state.clear = 0
        state.branch_latched = False
        state.approach_age = 0
        return state, command
    return state, VelocityCommand(0.0, 0.0, "reacquire")


def _consume_notes(state: JunctionTurn, notes: queue.Queue[str], send) -> None:
    while True:
        try:
            note = notes.get_nowait()
        except queue.Empty:
            return
        if state.phase == "forward":
            if note == "FORWARD_DONE":
                if send(f"turn {state.side}"):
                    state.phase = "turning"
                else:
                    state.phase = "fault"
            elif note == "FORWARD_FAIL":
                state.phase = "fault"
        elif state.phase == "turning":
            if note == "TURN_DONE":
                state.phase = "reacquire"
                state.clear = 0
            elif note == "TURN_FAIL":
                state.phase = "fault"
