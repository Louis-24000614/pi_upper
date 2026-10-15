"""转弯结束后短时原地摆正，再交回原有导航流程。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from road_follow.control import VelocityCommand, align_settle_command


@dataclass(frozen=True)
class PostTurnConfig:
    duration_s: float = 1.0
    gain: float = 2.0
    max_abs_omega: float = 0.5
    max_abs_heading_rad: float = 0.05


def post_turn_config_from_mapping(cfg: dict) -> PostTurnConfig:
    raw = cfg.get("post_turn", {}) or {}
    return PostTurnConfig(
        duration_s=max(0.0, float(raw.get("duration_s", 1.0))),
        gain=float(raw.get("gain", 2.0)),
        max_abs_omega=abs(float(raw.get("max_abs_omega", 0.5))),
        max_abs_heading_rad=abs(float(raw.get("max_abs_heading_rad", 0.05))),
    )


def turn_completed(before, states) -> bool:
    """只在最后一次转弯完成时触发，故障或连续掉头的中间阶段不触发。"""
    return any(
        phase == "turning" and state.phase in ("reacquire", "recovery", "follow", "done", "idle")
        for phase, state in zip(before, states)
    )


@dataclass
class PostTurnAlignment:
    started_s: float | None = None

    def start(self, now_s: float) -> None:
        self.started_s = now_s

    def step(
        self,
        *,
        now_s: float,
        frame_captured_s: float,
        lane_heading_rad: float | None,
        visual_safe: bool,
        cfg: PostTurnConfig,
    ) -> VelocityCommand | None:
        if self.started_s is None:
            return None
        if now_s - self.started_s >= cfg.duration_s:
            self.started_s = None
            return None
        # 不使用转弯前的在途帧或携带航向；缺少当前测量时原地等待。
        fresh = (
            math.isfinite(frame_captured_s)
            and frame_captured_s > self.started_s
            and 0.0 <= now_s - frame_captured_s <= 0.20
        )
        heading = lane_heading_rad if fresh and visual_safe else None
        command = align_settle_command(
            heading, cfg.gain, cfg.max_abs_omega, cfg.max_abs_heading_rad
        )
        return VelocityCommand(0.0, command.omega_radps, "post_turn_align")
