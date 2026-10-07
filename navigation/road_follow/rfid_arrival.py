"""保留原 patrol_slot 视觉接近状态机；当前主循环不向它提供 UID。"""

from __future__ import annotations

import math
import queue
import time
from dataclasses import dataclass

from road_follow.control import (
    VelocityCommand,
    align_ready_to_creep,
    align_settle_command,
    alignment_command,
    heading_hold_command,
    heading_needs_align,
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
) -> tuple[RfidArrival, VelocityCommand]:
    """侧边端头只负责锁存；正前方检测带稳定无 road mask 后才直走一次。"""
    received = _drain_notes(notes)
    current_s = time.monotonic() if now_s is None else now_s

    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_not_found")
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

    if state.phase == "align":
        if (
            arrival_mode == "visual_end"
            and forward_band_ratio > cfg.road_end_band_max_ratio
        ):
            state.phase = "follow"
            state.road_end_missing_frames = 0
            state.align_stable = 0
            return state, visual
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
                cfg.align_gain,
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
    if heading_needs_align(lane_heading_rad, cfg.align_max_abs_heading_rad):
        state.phase = "align"
        state.align_started_s = current_s
        state.align_stable = 0
        return state, alignment_command(
            float(lane_heading_rad), cfg.align_gain, cfg.align_max_abs_omega
        )
    return _begin_heading_hold(state, progress_m, odom_valid, cfg, yaw_rad)


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
