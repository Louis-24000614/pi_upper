"""RFID 到点停车、IMU 90°转向并重新交回视觉循迹。"""

from __future__ import annotations

import queue
from dataclasses import dataclass

from road_follow.control import VelocityCommand, is_visual_follow


@dataclass(frozen=True)
class RfidTurnConfig:
    stop_settle_s: float = 0.30
    reacquire_frames: int = 3
    search_distance_mm: int = 350
    search_speed_mmps: int = 50


@dataclass
class RfidTurn:
    """一次性测试状态：follow → searching/stopping → turning → complete。"""

    phase: str = "follow"
    side: str = "none"
    card_number: int = 0
    generation: int = 0
    stop_started_s: float = 0.0
    clear: int = 0


def rfid_turn_config_from_mapping(cfg: dict) -> RfidTurnConfig:
    raw = cfg.get("rfid_turn", {}) or {}
    return RfidTurnConfig(
        stop_settle_s=max(0.0, float(raw.get("stop_settle_ms", 300)) / 1000.0),
        reacquire_frames=max(1, int(raw.get("reacquire_frames", 3))),
        search_distance_mm=min(
            1000, max(1, int(raw.get("search_distance_mm", 350)))
        ),
        search_speed_mmps=min(
            400, max(20, int(raw.get("search_speed_mmps", 50)))
        ),
    )


def step_rfid_turn(
    state: RfidTurn,
    detection: tuple[int, int] | None,
    command: VelocityCommand,
    notes: queue.Queue[str],
    send,
    now_s: float,
    cfg: RfidTurnConfig,
) -> tuple[RfidTurn, VelocityCommand]:
    """处理一次 RFID 到点；有限转向期间不再发送视觉速度。"""
    received = _drain_notes(notes)

    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_turn_fail")

    if state.phase == "follow":
        if detection is not None:
            card_number, generation = detection
            if 1 <= card_number <= 12:
                state.card_number = card_number
                state.generation = generation
                state.stop_started_s = now_s
                state.phase = "stopping"
                return state, VelocityCommand(0.0, 0.0, "stop_rfid")
        if command.reason == "stop_lookahead":
            request = f"forward {cfg.search_distance_mm} {cfg.search_speed_mmps}"
            if send(request):
                state.phase = "searching"
                return state, VelocityCommand(0.0, 0.0, "rfid_searching")
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_search_fail")
        return state, command

    if state.phase == "searching":
        if detection is not None:
            card_number, generation = detection
            if 1 <= card_number <= 12:
                state.card_number = card_number
                state.generation = generation
                if send("stop"):
                    state.phase = "stopping_wait"
                    return state, VelocityCommand(0.0, 0.0, "stop_rfid_action")
                state.phase = "fault"
                return state, VelocityCommand(0.0, 0.0, "stop_rfid_turn_fail")
        if "FORWARD_DONE" in received or "FORWARD_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_not_found")
        return state, VelocityCommand(0.0, 0.0, "rfid_searching")

    if state.phase == "stopping_wait":
        if "STOP_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_turn_fail")
        if "STOP_DONE" in received:
            state.stop_started_s = now_s
            state.phase = "stopping"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_settle")
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_action")

    if state.phase == "stopping":
        if now_s - state.stop_started_s < cfg.stop_settle_s:
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_settle")
        if send(f"turn {state.side}"):
            state.phase = "turning"
            return state, VelocityCommand(0.0, 0.0, "rfid_turning")
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_rfid_turn_fail")

    if state.phase == "turning":
        if "TURN_DONE" in received:
            state.phase = "reacquire"
            state.clear = 0
        elif "TURN_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_rfid_turn_fail")
        else:
            return state, VelocityCommand(0.0, 0.0, "rfid_turning")

    if state.phase == "reacquire":
        if is_visual_follow(command):
            state.clear += 1
        else:
            state.clear = 0
        if state.clear >= cfg.reacquire_frames:
            state.phase = "complete"
            state.clear = 0
            return state, command
        return state, VelocityCommand(0.0, 0.0, "rfid_reacquire")

    # 本局部测试只允许 RFID 触发一次；完成后持续交给视觉循迹。
    return state, command


def _drain_notes(notes: queue.Queue[str]) -> set[str]:
    received: set[str] = set()
    while True:
        try:
            note = notes.get_nowait()
        except queue.Empty:
            return received
        received.add(note)
