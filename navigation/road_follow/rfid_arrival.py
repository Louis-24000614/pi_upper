"""保留原 patrol_slot 视觉接近状态机；当前主循环不向它提供 UID。"""

from __future__ import annotations

import math
import queue
import time
from dataclasses import dataclass, field

from road_follow.backup import near_lane_heading
from road_follow.control import (
    FollowConfig,
    HeadingAnchorGate,
    VelocityCommand,
    align_ready_to_creep,
    align_settle_command,
    alignment_command,
    command_from_forward_strip,
    forward_strip_points,
    heading_hold_command,
    heading_needs_align,
    heading_anchor_enabled_from_mapping,
    step_heading_anchor,
)


@dataclass(frozen=True)
class RfidArrivalConfig:
    step_distance_mm: int = 200
    search_speed_mmps: int = 50
    edge_visible_frames: int = 2
    road_end_missing_frames: int = 3
    road_end_band_max_ratio: float = 0.10
    align_max_abs_x_m: float = 0.02
    align_max_abs_heading_rad: float = 0.05
    align_stable_frames: int = 5
    align_timeout_s: float = 1.5
    align_gain: float = 4.0
    align_max_abs_omega: float = 0.5
    forward_strip_abs_x_m: float = 0.12
    heading_anchor_enabled: bool = True


@dataclass
class RfidArrival:
    """视觉循迹直到交接，再按锁存的直行航向用里程走完最后一段。"""

    phase: str = "follow"
    searched_mm: int = 0
    active_step_mm: int = 0
    card_number: int = 0
    generation: int = 0
    edge_seen_frames: int = 0
    road_end_missing_frames: int = 0
    edge_latched: bool = False
    # 保存本次接近过程中实际见过的侧边方向；标签号码不参与位置判断。
    edge_left_seen: bool = False
    edge_right_seen: bool = False
    odom_handoff_frames: int = 0
    align_started_s: float = 0.0
    align_stable: int = 0
    hold_start_m: float | None = None
    hold_yaw_rad: float | None = None
    anchor: HeadingAnchorGate = field(default_factory=HeadingAnchorGate)


def rfid_arrival_config_from_mapping(cfg: dict) -> RfidArrivalConfig:
    raw = cfg.get("rfid_turn", {}) or {}
    return RfidArrivalConfig(
        step_distance_mm=min(
            300, max(1, int(raw.get("search_step_distance_mm", 200)))
        ),
        search_speed_mmps=min(
            400, max(20, int(raw.get("search_speed_mmps", 50)))
        ),
        edge_visible_frames=max(1, int(raw.get("edge_visible_frames", 2))),
        road_end_missing_frames=max(
            1, int(raw.get("road_end_missing_frames", 3))
        ),
        road_end_band_max_ratio=max(
            0.0, min(1.0, float(raw.get("road_end_band_max_ratio", 0.10)))
        ),
        heading_anchor_enabled=heading_anchor_enabled_from_mapping(cfg),
    )


def step_rfid_arrival(
    state: RfidArrival,
    detection: tuple[int, int] | None,
    visual: VelocityCommand,
    notes: queue.Queue[str],
    send,
    cfg: RfidArrivalConfig,
    *,
    edge_visible: bool = False,
    edge_left_visible: bool = False,
    edge_right_visible: bool = False,
    forward_band_ratio: float = 1.0,
    visual_safe: bool = True,
    arrival_mode: str = "visual_end",
    odom_handoff: bool = False,
    odom_stable_frames: int = 2,
    near_x_m: float | None = None,
    lane_heading_rad: float | None = None,
    yaw_rad: float | None = None,
    now_s: float | None = None,
    progress_m: float | None = None,
    odom_valid: bool = True,
    centerline_points=None,
    road_pixels: int = 800,
    follow: FollowConfig | None = None,
    frame_captured_s: float | None = None,
    odom_received_s: float | None = None,
    anchor_notes=None,
) -> tuple[RfidArrival, VelocityCommand]:
    """侧边端头只负责锁存；正前方检测带稳定无 road mask 后才直走一次。"""
    received = _drain_notes(notes)
    current_s = time.monotonic() if now_s is None else now_s

    if state.phase == "fault":
        reason = "stop_heading_anchor_" + state.anchor.failure if state.anchor.failure else "stop_rfid_not_found"
        return state, VelocityCommand(0.0, 0.0, reason)
    if state.phase == "odom_wait":
        return state, VelocityCommand(0.0, 0.0, "stop_arrival_guard")
    if state.phase == "arrived":
        return state, VelocityCommand(0.0, 0.0, "rfid_arrived")

    if _valid_detection(detection):
        assert detection is not None
        state.card_number, state.generation = detection
        if send("stop"):
            state.phase = "stopping_wait"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_action")
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_action_fail")

    if state.phase == "stopping_wait":
        if "STOP_DONE" in received:
            state.phase = "arrived"
            return state, VelocityCommand(0.0, 0.0, "rfid_arrived")
        if "STOP_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_action_fail")
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_action")

    if state.phase == "heading_hold":
        return _step_heading_hold(state, progress_m, odom_valid, cfg, yaw_rad)

    def advance_anchor():
        _, heading = _forward_strip_view(
            state, cfg, visual, lane_heading_rad, arrival_mode,
            centerline_points, road_pixels, follow,
        )
        state.phase, result, ready = step_heading_anchor(
            state.anchor, state.phase, lane_heading_rad=heading,
            frame_captured_s=frame_captured_s, visual_safe=visual_safe, now_s=current_s,
            odom_valid=odom_valid, odom_received_s=odom_received_s, yaw_rad=yaw_rad,
            notes=anchor_notes, send=send, max_abs_heading_rad=cfg.align_max_abs_heading_rad,
            stable_frames=cfg.align_stable_frames, align_timeout_s=cfg.align_timeout_s,
            align_gain=_visual_end_align_gain(cfg, follow, arrival_mode),
            max_abs_omega=cfg.align_max_abs_omega,
        )
        if ready:
            return _begin_heading_hold(state, progress_m, odom_valid, cfg, yaw_rad)
        return state, result

    if cfg.heading_anchor_enabled and state.phase in ("align", "anchor_wait"):
        return advance_anchor()

    if state.phase == "align":
        if current_s - state.align_started_s >= cfg.align_timeout_s:
            state.phase = "fault"
            state.anchor.failure = "align_timeout"
            return state, VelocityCommand(0.0, 0.0, "stop_heading_anchor_align_timeout")
        # 摆正开始后做完。检测带回升只说明车头转进了旁边的路，不能退回循迹。
        _, lane_heading_rad = _forward_strip_view(
            state, cfg, visual, lane_heading_rad, arrival_mode,
            centerline_points, road_pixels, follow,
        )
        ready, state.align_stable = align_ready_to_creep(
            lane_heading_rad,
            state.align_started_s,
            state.align_stable,
            current_s,
            max_abs_x_m=cfg.align_max_abs_heading_rad,
            required_frames=cfg.align_stable_frames,
            timeout_s=cfg.align_timeout_s,
        )
        if not ready:
            return state, align_settle_command(
                lane_heading_rad,
                _visual_end_align_gain(cfg, follow, arrival_mode),
                cfg.align_max_abs_omega,
                cfg.align_max_abs_heading_rad,
            )
        return _begin_heading_hold(state, progress_m, odom_valid, cfg, yaw_rad)

    any_edge_visible = edge_visible or edge_left_visible or edge_right_visible
    if any_edge_visible:
        state.edge_seen_frames += 1
        state.edge_left_seen = state.edge_left_seen or edge_left_visible
        state.edge_right_seen = state.edge_right_seen or edge_right_visible
        if state.edge_seen_frames >= cfg.edge_visible_frames:
            state.edge_latched = True
    else:
        if not state.edge_latched:
            state.edge_seen_frames = 0
            state.edge_left_seen = False
            state.edge_right_seen = False

    visual, lane_heading_rad = _forward_strip_view(
        state, cfg, visual, lane_heading_rad, arrival_mode,
        centerline_points, road_pixels, follow,
    )
    if state.edge_latched and arrival_mode == "visual_end":
        if forward_band_ratio <= cfg.road_end_band_max_ratio:
            state.road_end_missing_frames += 1
        else:
            state.road_end_missing_frames = 0

    if arrival_mode == "visual_odom":
        state.odom_handoff_frames = (
            state.odom_handoff_frames + 1 if state.edge_latched and odom_handoff else 0
        )
        ready = state.odom_handoff_frames >= max(1, odom_stable_frames)
    else:
        ready = state.road_end_missing_frames >= cfg.road_end_missing_frames
    if not state.edge_latched or not ready:
        return state, visual
    if not visual_safe:
        # 整幅路消失不能当成到墙，也不锁故障；画面恢复后继续循迹。
        state.road_end_missing_frames = 0
        return state, visual
    if cfg.heading_anchor_enabled:
        state.phase = "align"
        state.align_started_s = current_s
        state.anchor = HeadingAnchorGate(started_s=current_s)
        return advance_anchor()
    if heading_needs_align(lane_heading_rad, cfg.align_max_abs_heading_rad):
        state.phase = "align"
        state.align_started_s = current_s
        state.align_stable = 0
        return state, alignment_command(
            float(lane_heading_rad),
            _visual_end_align_gain(cfg, follow, arrival_mode),
            cfg.align_max_abs_omega,
        )
    return _begin_heading_hold(state, progress_m, odom_valid, cfg, yaw_rad)


def _visual_end_align_gain(
    cfg: RfidArrivalConfig,
    follow: FollowConfig | None,
    arrival_mode: str,
) -> float:
    """尽头点摆正用循迹增益的四分之一；其它交接仍用原来的摆正增益。"""
    if (
        arrival_mode == "visual_end"
        and follow is not None
        and math.isfinite(follow.steering_gain)
    ):
        return max(0.0, float(follow.steering_gain)) / 4.0
    return cfg.align_gain


def _forward_strip_view(
    state: RfidArrival,
    cfg: RfidArrivalConfig,
    visual: VelocityCommand,
    lane_heading_rad: float | None,
    arrival_mode: str,
    centerline_points,
    road_pixels: int,
    follow: FollowConfig | None,
) -> tuple[VelocityCommand, float | None]:
    """尽头点锁住侧边后，方向和循迹都只看车头正前方窄带。"""
    if (
        arrival_mode != "visual_end"
        or not state.edge_latched
        or centerline_points is None
    ):
        return visual, lane_heading_rad
    follow_cfg = follow or FollowConfig()
    strip_command = command_from_forward_strip(
        centerline_points, road_pixels, follow_cfg, cfg.forward_strip_abs_x_m
    )
    heading = near_lane_heading(
        forward_strip_points(centerline_points, cfg.forward_strip_abs_x_m)
    )
    if state.phase != "follow":
        return visual, heading
    return strip_command, heading


def _measured_progress(progress_m: float | None, odom_valid: bool) -> float | None:
    if not odom_valid or progress_m is None:
        return None
    value = float(progress_m)
    if not math.isfinite(value):
        return None
    return value


def _remember_hold_yaw(state: RfidArrival, yaw_rad: float | None) -> None:
    if state.hold_yaw_rad is not None or yaw_rad is None or not math.isfinite(yaw_rad):
        return
    state.hold_yaw_rad = float(yaw_rad)


def _step_heading_hold(
    state: RfidArrival,
    progress_m: float | None,
    odom_valid: bool,
    cfg: RfidArrivalConfig,
    yaw_rad: float | None = None,
) -> tuple[RfidArrival, VelocityCommand]:
    """锁存摆正后的航向。里程中断只停车，恢复后仍从原来的起点和航向计量。"""
    measured = _measured_progress(progress_m, odom_valid)
    if measured is None:
        return state, VelocityCommand(0.0, 0.0, "stop_odom_stale")
    if state.hold_start_m is None:
        state.hold_start_m = measured
    _remember_hold_yaw(state, yaw_rad)
    if measured + 1e-6 >= state.hold_start_m + cfg.step_distance_mm / 1000.0:
        state.searched_mm += state.active_step_mm
        state.active_step_mm = 0
        state.phase = "arrived"
        return state, VelocityCommand(0.0, 0.0, "rfid_arrived")
    return state, heading_hold_command(
        cfg.search_speed_mmps / 1000.0,
        yaw_rad,
        state.hold_yaw_rad,
        cfg.align_gain,
        cfg.align_max_abs_omega,
    )


def _begin_heading_hold(
    state: RfidArrival,
    progress_m: float | None,
    odom_valid: bool,
    cfg: RfidArrivalConfig,
    yaw_rad: float | None = None,
) -> tuple[RfidArrival, VelocityCommand]:
    state.active_step_mm = cfg.step_distance_mm
    state.phase = "heading_hold"
    state.hold_start_m = None
    state.hold_yaw_rad = None
    return _step_heading_hold(state, progress_m, odom_valid, cfg, yaw_rad)


def _valid_detection(detection: tuple[int, int] | None) -> bool:
    return detection is not None and 1 <= detection[0] <= 12


def _drain_notes(notes: queue.Queue[str]) -> set[str]:
    received: set[str] = set()
    while True:
        try:
            received.add(notes.get_nowait())
        except queue.Empty:
            return received
