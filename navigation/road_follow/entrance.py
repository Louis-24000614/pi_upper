"""出发区起步：定距前进到 0_J，再按 Agent 路线转向并交回视觉贴线。"""

from __future__ import annotations

import queue
import time
from dataclasses import dataclass

from road_follow.control import VelocityCommand, is_visual_follow


@dataclass(frozen=True)
class EntranceConfig:
    forward_mm: int = 150
    forward_speed_mmps: int = 50
    stop_settle_s: float = 2.0
    reacquire_frames: int = 3
    recovery_frames: int = 10
    recovery_mps: float = 0.06


@dataclass
class EntranceDeparture:
    """pending → forward → stopping → turning → reacquire → recovery → done。"""

    phase: str = "pending"
    turn_side: str = "none"
    stop_started_s: float = 0.0
    clear: int = 0
    recovery: int = 0


def entrance_config_from_mapping(cfg: dict) -> EntranceConfig:
    raw = cfg.get("entrance", {}) or {}
    return EntranceConfig(
        forward_mm=max(1, int(raw.get("forward_mm", 150))),
        forward_speed_mmps=max(1, int(raw.get("forward_speed_mmps", 50))),
        stop_settle_s=max(0.0, float(raw.get("stop_settle_ms", 2000)) / 1000.0),
        reacquire_frames=max(1, int(raw.get("reacquire_frames", 3))),
        recovery_frames=max(1, int(raw.get("recovery_frames", 10))),
        recovery_mps=max(0.0, float(raw.get("recovery_mps", 0.06))),
    )


def step_entrance_departure(
    state: EntranceDeparture,
    notes: queue.Queue[str],
    send,
    command: VelocityCommand,
    cfg: EntranceConfig | None = None,
    now_s: float | None = None,
) -> tuple[EntranceDeparture, VelocityCommand]:
    """起步和重新锁定期间停车，然后先低速、再完全交回视觉速度。"""
    cfg = cfg or EntranceConfig()
    current_s = time.monotonic() if now_s is None else now_s
    received = _drain_notes(notes)

    if state.phase == "done":
        return state, command
    if state.phase == "fault":
        return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")

    if state.phase == "pending":
        if send(f"forward {cfg.forward_mm} {cfg.forward_speed_mmps}"):
            state.phase = "forward"
            return state, VelocityCommand(0.0, 0.0, "entrance_forward")
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")

    if state.phase == "forward":
        if "FORWARD_DONE" in received:
            if send("stop"):
                state.phase = "stopping_wait"
                return state, VelocityCommand(0.0, 0.0, "entrance_stop")
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")
        if "FORWARD_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")
        return state, VelocityCommand(0.0, 0.0, "entrance_forward")

    if state.phase == "stopping_wait":
        if "STOP_DONE" in received:
            state.phase = "stopping"
            state.stop_started_s = current_s
            return state, VelocityCommand(0.0, 0.0, "entrance_stop_settle")
        if "STOP_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")
        return state, VelocityCommand(0.0, 0.0, "entrance_stop")

    if state.phase == "stopping":
        if current_s - state.stop_started_s < cfg.stop_settle_s:
            return state, VelocityCommand(0.0, 0.0, "entrance_stop_settle")
        if state.turn_side not in ("left", "right"):
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")
        if send(f"turn {state.turn_side}"):
            state.phase = "turning"
            return state, VelocityCommand(0.0, 0.0, "entrance_turn")
        state.phase = "fault"
        return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")

    if state.phase == "turning":
        if "TURN_DONE" in received:
            state.phase = "reacquire"
            state.clear = 0
            state.recovery = 0
            return state, VelocityCommand(0.0, 0.0, "entrance_reacquire")
        if "TURN_FAIL" in received:
            state.phase = "fault"
            return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")
        return state, VelocityCommand(0.0, 0.0, "entrance_turn")

    if state.phase == "reacquire":
        state.clear = state.clear + 1 if is_visual_follow(command) else 0
        if state.clear < cfg.reacquire_frames:
            return state, VelocityCommand(0.0, 0.0, "entrance_reacquire")
        state.phase = "recovery"
        state.recovery = 0

    if state.phase == "recovery":
        if not is_visual_follow(command):
            state.phase = "reacquire"
            state.clear = 0
            state.recovery = 0
            return state, VelocityCommand(0.0, 0.0, "entrance_reacquire")
        state.recovery += 1
        slow = VelocityCommand(
            min(command.v_mps, cfg.recovery_mps),
            command.omega_radps,
            "entrance_recovery",
        )
        if state.recovery >= cfg.recovery_frames:
            state.phase = "done"
        return state, slow

    return state, VelocityCommand(0.0, 0.0, "stop_entrance_fail")


def _drain_notes(notes: queue.Queue[str]) -> list[str]:
    received: list[str] = []
    while True:
        try:
            received.append(notes.get_nowait())
        except queue.Empty:
            return received
