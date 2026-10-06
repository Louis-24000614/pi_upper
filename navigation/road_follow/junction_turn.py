"""视觉路口到下位机定距直行/90°转弯的状态机。"""

from __future__ import annotations

import queue
import time
from dataclasses import dataclass, field

from road_follow.control import VelocityCommand, is_visual_follow


@dataclass(frozen=True)
class JunctionTurnConfig:
    stable_frames: int = 2
    handoff_min_distance_m: float = 0.18
    handoff_max_distance_m: float = 0.35
    camera_ahead_of_turn_center_m: float = 0.0
    stop_before_center_m: float = 0.0
    forward_speed_mmps: int = 50
    turn_forward_m: float = 0.15
    stop_settle_s: float = 2.0
    min_forward_mm: int = 50
    max_forward_mm: int = 650
    reacquire_frames: int = 3
    road_end_enabled: bool = True
    # 85° 镜头下，角掉出画面时可见末端贴近鸟瞰近边，而不是旧的 0.48–0.56 m。
    road_end_min_y_m: float = 0.20
    road_end_max_y_m: float = 0.40
    road_end_stopped_max_y_m: float = 0.45
    road_end_band_max_ratio: float = 0.10
    road_end_missing_frames: int = 3
    # 角看不见之后，IMU 定距再走这段到路口中心。
    blind_forward_m: float = 0.15
    road_end_max_abs_lane_x_m: float = 0.08
    road_end_min_lane_width_m: float = 0.14
    road_end_max_lane_width_m: float = 0.32
    branch_observe_min_distance_m: float = 0.45
    branch_observe_max_distance_m: float = 0.95
    branch_latch_max_frames: int = 200
    branch_vote_window: int = 8
    branch_vote_min: int = 2
    odom_stop_margin_m: float = 0.05


@dataclass(frozen=True)
class JunctionCue:
    """分割几何给出的待转方向和路口中心距离。"""

    detected: bool
    side: str = "none"
    distance_m: float | None = None
    source: str = "none"


@dataclass
class JunctionTurn:
    """follow → approach → forward → arrived → stopping/stopped → departure。

    ``arrived`` 只表示定距走完。左转、右转、直行或倒车要等调用方另行提交。
    """

    phase: str = "follow"
    side: str = "none"
    arm: int = 0
    clear: int = 0
    forward_mm: int = 0
    branch_latched: bool = False
    approach_age: int = 0
    departure: str = "none"
    suppress_cue: bool = False
    stop_started_s: float = 0.0
    stop_settle_s: float = 2.0
    branch_votes: list[bool] = field(default_factory=list)


def junction_turn_config_from_mapping(cfg: dict) -> JunctionTurnConfig:
    raw = cfg.get("junction_turn", {}) or {}
    return JunctionTurnConfig(
        stable_frames=max(1, int(raw.get("stable_frames", 2))),
        handoff_min_distance_m=float(raw.get("handoff_min_distance_m", 0.18)),
        handoff_max_distance_m=float(raw.get("handoff_max_distance_m", 0.35)),
        camera_ahead_of_turn_center_m=float(
            raw.get("camera_ahead_of_turn_center_m", 0.0)
        ),
        stop_before_center_m=float(raw.get("stop_before_center_m", 0.0)),
        forward_speed_mmps=max(1, int(raw.get("forward_speed_mmps", 50))),
        turn_forward_m=max(0.001, float(raw.get("turn_forward_m", 0.15))),
        stop_settle_s=max(
            0.0, float(raw.get("stop_settle_ms", 2000)) / 1000.0
        ),
        min_forward_mm=max(1, int(raw.get("min_forward_mm", 50))),
        max_forward_mm=max(1, int(raw.get("max_forward_mm", 650))),
        reacquire_frames=max(1, int(raw.get("reacquire_frames", 3))),
        road_end_enabled=bool(raw.get("road_end_enabled", True)),
        road_end_min_y_m=float(raw.get("road_end_min_y_m", 0.20)),
        road_end_max_y_m=float(raw.get("road_end_max_y_m", 0.40)),
        road_end_stopped_max_y_m=float(
            raw.get("road_end_stopped_max_y_m", 0.45)
        ),
        road_end_band_max_ratio=max(
            0.0, min(1.0, float(raw.get("road_end_band_max_ratio", 0.10)))
        ),
        road_end_missing_frames=max(
            1, int(raw.get("road_end_missing_frames", 3))
        ),
        blind_forward_m=float(raw.get("blind_forward_m", 0.15)),
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
            raw.get("branch_observe_min_distance_m", 0.45)
        ),
        branch_observe_max_distance_m=float(
            raw.get("branch_observe_max_distance_m", 0.95)
        ),
        branch_latch_max_frames=max(
            1, int(raw.get("branch_latch_max_frames", 200))
        ),
        branch_vote_window=max(1, int(raw.get("branch_vote_window", 8))),
        branch_vote_min=max(1, int(raw.get("branch_vote_min", 2))),
        odom_stop_margin_m=max(
            0.0, float(raw.get("odom_stop_margin_m", 0.05))
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
    forward_band_ratio: float = 0.0,
) -> JunctionCue:
    """支路已锁存且正前方检测带的 road mask 消失时，才允许最后一次定距。"""
    stopped_after_latch = approach_latched and command.reason == "stop_lookahead"
    max_end_y = (
        cfg.road_end_stopped_max_y_m
        if stopped_after_latch
        else cfg.road_end_max_y_m
    )
    detected = (
        cfg.road_end_enabled
        and side in ("left", "right")
        and approach_latched
        and forward_band_ratio <= cfg.road_end_band_max_ratio
        and (is_visual_follow(command) or stopped_after_latch)
        and road_end_y_m is not None
        and cfg.road_end_min_y_m <= road_end_y_m <= max_end_y
        and abs(lane_x_m) <= cfg.road_end_max_abs_lane_x_m
        and cfg.road_end_min_lane_width_m
        <= lane_width_m
        <= cfg.road_end_max_lane_width_m
    )
    if not detected:
        return JunctionCue(False)
    return JunctionCue(True, side, cfg.blind_forward_m, "road_end")


def odom_handoff_turn_cue(
    *,
    side: str,
    progress_m: float,
    edge_length_m: float,
    target_role: str,
    state: JunctionTurn,
    command: VelocityCommand,
    cfg: JunctionTurnConfig,
) -> JunctionCue:
    """十字路口前方道路不会消失，用边末端 ODOM 触发最后定距交接。"""
    handoff_distance_m = cfg.turn_forward_m
    trigger_m = max(0.0, edge_length_m - handoff_distance_m)
    detected = (
        target_role == "junction"
        and state.phase in ("follow", "approach")
        and state.branch_latched
        and side in ("left", "right")
        and is_visual_follow(command)
        and progress_m + 1e-6 >= trigger_m
        and progress_m < edge_length_m
    )
    if not detected:
        return JunctionCue(False)
    return JunctionCue(True, side, handoff_distance_m, "odom_handoff")


def step_junction_turn(
    state: JunctionTurn,
    cue: JunctionCue,
    command: VelocityCommand,
    notes: queue.Queue[str],
    send,
    cfg: JunctionTurnConfig,
    now_s: float | None = None,
) -> tuple[JunctionTurn, VelocityCommand]:
    """在视觉仍可靠时交接；有限动作期间不再发送 ``CMD_VEL``。"""
    current_s = time.monotonic() if now_s is None else now_s
    _consume_notes(state, notes, send, current_s)

    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_action_fail")

    if state.phase == "arrived":
        return state, VelocityCommand(0.0, 0.0, "arrived")

    if state.phase == "backup":
        return state, VelocityCommand(0.0, 0.0, "backup")

    if state.phase == "done":
        return state, VelocityCommand(0.0, 0.0, "done")

    if state.phase == "odom_wait":
        return state, VelocityCommand(0.0, 0.0, "stop_odom_junction_wait")

    if state.phase == "stopped":
        if current_s - state.stop_started_s < state.stop_settle_s:
            return state, VelocityCommand(0.0, 0.0, "junction_stop_settle")
        _resume_departure(state, send)
        if state.phase == "fault":
            return state, VelocityCommand(0.0, 0.0, "stop_action_fail")
        if state.phase == "follow":
            return state, command
        return state, VelocityCommand(0.0, 0.0, state.phase)

    if state.phase in ("follow", "approach"):
        if state.suppress_cue:
            if cue.detected:
                return state, command
            state.suppress_cue = False
        branch_observed = (
            cue.detected
            and cue.source == "side_branch"
            and cue.side in ("left", "right")
            and cue.distance_m is not None
            and cfg.branch_observe_min_distance_m
            <= cue.distance_m
            <= cfg.branch_observe_max_distance_m
        )
        state.branch_votes.append(branch_observed)
        del state.branch_votes[: -cfg.branch_vote_window]
        if (
            not state.branch_latched
            and branch_observed
            and sum(state.branch_votes) >= cfg.branch_vote_min
        ):
            state.phase = "approach"
            state.side = cue.side
            state.branch_latched = True
            state.approach_age = 0
        elif state.phase == "approach":
            # 一旦确认过支路就保持到本节点动作结束；墙体遮挡不能清掉锁存。
            state.approach_age += 1

        if cue.source in ("road_end", "odom_handoff"):
            distance_ok = cue.distance_m is not None and cue.distance_m > 0.0
        elif cue.source == "side_branch":
            # 侧边角只用来确认「这是路口」，不再直接触发最后一次定距。
            distance_ok = False
        else:
            distance_ok = (
                cue.distance_m is not None
                and cfg.handoff_min_distance_m
                <= cue.distance_m
                <= cfg.handoff_max_distance_m
            )
        in_window = (
            cue.detected
            and cue.side in ("left", "right")
            and (
                is_visual_follow(command)
                or (
                    state.branch_latched
                    and cue.source == "road_end"
                    and command.reason == "stop_lookahead"
                )
            )
            and distance_ok
        )
        state.arm = state.arm + 1 if in_window else 0
        required_frames = (
            cfg.road_end_missing_frames
            if cue.source == "road_end"
            else cfg.stable_frames
        )
        if state.arm < required_frames:
            return state, command

        # 路口交接后统一只前进配置的定距，避免视觉距离抖动改变转弯起点。
        forward_mm = int(round(cfg.turn_forward_m * 1000.0))
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

    if state.phase in ("forward", "stopping", "turning"):
        return state, VelocityCommand(0.0, 0.0, state.phase)

    # 转完后先原地观察；新方向中心线连续稳定才重新交给视觉速度环。
    if is_visual_follow(command):
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
        state.branch_votes.clear()
        return state, command
    return state, VelocityCommand(0.0, 0.0, "reacquire")


def _consume_notes(
    state: JunctionTurn, notes: queue.Queue[str], send, now_s: float
) -> None:
    while True:
        try:
            note = notes.get_nowait()
        except queue.Empty:
            return
        if state.phase == "forward":
            if note == "FORWARD_DONE":
                state.phase = "arrived"
                state.departure = "none"
            elif note == "FORWARD_FAIL":
                state.phase = "fault"
        elif state.phase == "turning":
            if note == "TURN_DONE":
                state.phase = "reacquire"
                state.clear = 0
            elif note == "TURN_FAIL":
                state.phase = "fault"
        elif state.phase == "stopping":
            if note == "STOP_DONE":
                state.phase = "stopped"
                state.stop_started_s = now_s
            elif note == "STOP_FAIL":
                state.phase = "fault"


def _resume_departure(state: JunctionTurn, send) -> None:
    """停稳 2 秒后才执行规划动作。"""
    if state.departure in ("left", "right"):
        if send(f"turn {state.departure}"):
            state.phase = "turning"
            state.side = state.departure
        else:
            state.phase = "fault"
        return
    if state.departure == "straight":
        state.phase = "follow"
        state.side = "none"
        state.forward_mm = 0
        state.clear = 0
        state.arm = 0
        state.branch_latched = False
        state.approach_age = 0
        state.suppress_cue = True
        return
    if state.departure == "backup":
        state.phase = "backup"
        state.side = "none"
        return
    state.phase = "fault"


def should_stop_at_expected_junction(
    progress_m: float,
    edge_length_m: float,
    target_role: str,
    state: JunctionTurn,
    cfg: JunctionTurnConfig,
) -> bool:
    """视觉全程没确认路口时，ODOM 到拓扑节点只停车，不允许盲转。"""
    if target_role != "junction" or state.phase not in ("follow", "approach"):
        return False
    threshold = max(0.0, edge_length_m - cfg.odom_stop_margin_m)
    return progress_m >= threshold
