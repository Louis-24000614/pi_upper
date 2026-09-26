"""Agent 路线中 patrol_slot 的 RFID 到点状态机。"""

from __future__ import annotations

import queue
from dataclasses import dataclass

from road_follow.control import VelocityCommand


@dataclass(frozen=True)
class RfidArrivalConfig:
    step_distance_mm: int = 200
    search_speed_mmps: int = 50
    edge_visible_frames: int = 2
    road_end_missing_frames: int = 3
    road_end_band_max_ratio: float = 0.10


@dataclass
class RfidArrival:
    """视觉循迹直到矮沿端头消失，再仅盲走一次固定距离。"""

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
) -> tuple[RfidArrival, VelocityCommand]:
    """侧边端头只负责锁存；正前方检测带稳定无 road mask 后才直走一次。"""
    received = _drain_notes(notes)

    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_not_found")
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

    if state.phase == "blind_forward":
        if "FORWARD_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_search_fail")
        if "FORWARD_DONE" not in received:
            return state, VelocityCommand(0.0, 0.0, "rfid_searching")
        state.searched_mm += state.active_step_mm
        state.active_step_mm = 0
        # 检测带消失后只允许这一段 20 cm；仍未读到卡就停车，禁止连续盲走。
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_not_found")

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

    if state.edge_latched:
        if forward_band_ratio <= cfg.road_end_band_max_ratio:
            state.road_end_missing_frames += 1
        else:
            state.road_end_missing_frames = 0

    if (
        not state.edge_latched
        or state.road_end_missing_frames < cfg.road_end_missing_frames
    ):
        return state, visual
    if not visual_safe:
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_unsafe")
    if _send_next_step(state, send, cfg):
        state.phase = "blind_forward"
        return state, VelocityCommand(0.0, 0.0, "rfid_searching")
    state.phase = "fault"
    return state, VelocityCommand(0.0, 0.0, "stop_rfid_search_fail")


def _send_next_step(state: RfidArrival, send, cfg: RfidArrivalConfig) -> bool:
    distance = cfg.step_distance_mm
    if distance <= 0:
        return False
    if not send(f"forward {distance} {cfg.search_speed_mmps}"):
        return False
    state.active_step_mm = distance
    state.phase = "blind_forward"
    return True


def _valid_detection(detection: tuple[int, int] | None) -> bool:
    return detection is not None and 1 <= detection[0] <= 12


def _drain_notes(notes: queue.Queue[str]) -> set[str]:
    received: set[str] = set()
    while True:
        try:
            received.add(notes.get_nowait())
        except queue.Empty:
            return received
