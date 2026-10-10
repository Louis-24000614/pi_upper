"""中心线纯跟踪。地面 X 向右、Y 向前；串口正角速度是左转。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

Point2D = Tuple[float, float]


@dataclass(frozen=True)
class FollowConfig:
    """寻线速度。弯道用更低的线速度，角速度有上限。"""

    cruise_mps: float = 0.10
    turn_mps: float = 0.06
    lookahead_m: float = 0.45
    turn_abs_x_m: float = 0.08
    min_points: int = 8
    min_road_pixels: int = 200
    max_abs_omega: float = 1.0
    # 纯跟踪角速度增益。用于补偿底盘左右轮差异造成的转向响应不足。
    steering_gain: float = 1.0
    # 加到预瞄点 X（向右为正）。直道被看成偏左时用正值，避免一直左转。
    x_bias_m: float = 0.0
    # 仅在需要时启用：固定预瞄点已越过可见路面时，仍用近处中心线低速循迹。
    min_lookahead_m: float = 0.0
    near_mps: float = 0.05


@dataclass(frozen=True)
class VelocityCommand:
    v_mps: float
    omega_radps: float
    reason: str


def near_centerline_needs_align(near_x_m: float | None, max_abs_x_m: float) -> bool:
    """近处中心线缺失或已经在阈值内时，不进入原地摆正。"""
    return near_x_m is not None and abs(near_x_m) > max_abs_x_m


def heading_needs_align(lane_heading_rad: float | None, max_abs_heading_rad: float) -> bool:
    """道路方向未知或已经对齐时，不原地转车头。横向偏差不在这里处理。"""
    return (
        lane_heading_rad is not None
        and math.isfinite(lane_heading_rad)
        and abs(lane_heading_rad) > max_abs_heading_rad
    )


def heading_hold_command(
    v_mps: float,
    yaw_rad: float | None,
    target_yaw_rad: float | None,
    gain: float,
    max_abs_omega: float,
) -> VelocityCommand:
    """按锁存航向修正。目标在当前航向左侧时角速度为正。"""
    omega = 0.0
    if (
        yaw_rad is not None
        and target_yaw_rad is not None
        and math.isfinite(yaw_rad)
        and math.isfinite(target_yaw_rad)
        and math.isfinite(gain)
    ):
        error = math.atan2(
            math.sin(target_yaw_rad - yaw_rad),
            math.cos(target_yaw_rad - yaw_rad),
        )
        omega = float(gain) * error
        limit = abs(float(max_abs_omega))
        if omega > limit:
            omega = limit
        elif omega < -limit:
            omega = -limit
    return VelocityCommand(v_mps, omega, "heading_hold")


def align_ready_to_creep(
    near_x_m: float | None,
    started_s: float,
    stable_frames: int,
    now_s: float,
    *,
    max_abs_x_m: float,
    required_frames: int,
    timeout_s: float,
) -> tuple[bool, int]:
    """摆正结束条件。测量消失或超时都不能冒充稳定成功。"""
    if now_s - started_s >= timeout_s:
        return False, 0
    if near_x_m is None or not math.isfinite(near_x_m):
        return False, 0
    if abs(near_x_m) <= max_abs_x_m:
        stable_frames += 1
        return stable_frames >= required_frames, stable_frames
    return False, 0


@dataclass
class HeadingAnchorGate:
    """只在新鲜视觉稳定后请求 MCU 重锚动作参考，不改变原始 yaw。"""

    started_s: float = 0.0
    last_frame_s: float | None = None
    stable_frames: int = 0
    requested_s: float | None = None
    acknowledged_s: float | None = None
    failure: str = ""


def heading_anchor_enabled_from_mapping(cfg: dict) -> bool:
    enabled = (cfg.get("heading_anchor", {}) or {}).get("enabled", True)
    if not isinstance(enabled, bool):
        raise ValueError("heading_anchor.enabled 必须是 true 或 false")
    return enabled


def step_heading_anchor(
    gate: HeadingAnchorGate,
    phase: str,
    *,
    lane_heading_rad: float | None,
    frame_captured_s: float | None,
    visual_safe: bool,
    now_s: float,
    odom_valid: bool,
    odom_received_s: float | None,
    yaw_rad: float | None,
    notes,
    send,
    max_abs_heading_rad: float,
    stable_frames: int,
    align_timeout_s: float,
    align_gain: float,
    max_abs_omega: float,
    anchor_timeout_s: float = 3.0,
) -> tuple[str, VelocityCommand, bool]:
    """MCU ACK 证明已重锚；之后还须等一帧新的 ODOM 才允许短程前进。"""
    stopped = VelocityCommand(0.0, 0.0, "anchor_wait" if phase == "anchor_wait" else "align")

    def fail(reason):
        gate.failure = reason
        send("stop")
        return "fault", VelocityCommand(0.0, 0.0, "stop_heading_anchor_" + reason), False

    frame_fresh = (
        frame_captured_s is not None
        and math.isfinite(frame_captured_s)
        and 0.0 <= now_s - frame_captured_s <= 0.20
    )
    heading_known = lane_heading_rad is not None and math.isfinite(lane_heading_rad)
    evidence_valid = visual_safe and frame_fresh and heading_known and odom_valid

    if phase == "align":
        if now_s - gate.started_s >= align_timeout_s:
            return fail("align_timeout")
        if not evidence_valid:
            gate.stable_frames = 0
            return phase, stopped, False
        if gate.last_frame_s is not None and frame_captured_s <= gate.last_frame_s:
            return phase, stopped, False
        gate.last_frame_s = frame_captured_s
        ready, gate.stable_frames = align_ready_to_creep(
            lane_heading_rad, gate.started_s, gate.stable_frames, now_s,
            max_abs_x_m=max_abs_heading_rad, required_frames=stable_frames,
            timeout_s=align_timeout_s,
        )
        if not ready:
            return phase, align_settle_command(
                lane_heading_rad, align_gain, max_abs_omega, max_abs_heading_rad
            ), False
        # 发请求前清除无主的旧回执；本流程同时只允许一个参考请求。
        while notes is not None and not notes.empty():
            notes.get_nowait()
        if not send("0 0") or not send("anchor_heading"):
            return fail("submit_failed")
        gate.requested_s = now_s
        return "anchor_wait", VelocityCommand(0.0, 0.0, "anchor_wait"), False

    if gate.requested_s is None:
        return fail("invalid_state")
    if now_s - gate.requested_s >= anchor_timeout_s:
        return fail("timeout")
    if not evidence_valid or abs(lane_heading_rad) > max_abs_heading_rad:
        return fail("evidence_lost")
    while notes is not None and not notes.empty():
        text, received_s = notes.get_nowait()
        if received_s < gate.requested_s:
            continue
        if text == "ANCHOR_DONE":
            gate.acknowledged_s = received_s
        elif text.startswith("ANCHOR_FAIL"):
            reason = text.partition(" ")[2] or "rejected"
            return fail(reason.lower())
    if (
        gate.acknowledged_s is not None
        and odom_received_s is not None
        and odom_received_s > gate.acknowledged_s
        and 0.0 <= now_s - odom_received_s <= 0.5
        and yaw_rad is not None
        and math.isfinite(yaw_rad)
    ):
        return "heading_hold", stopped, True
    return "anchor_wait", stopped, False


def align_settle_command(
    error: float | None,
    gain: float,
    max_abs_omega: float,
    max_abs_error: float,
) -> VelocityCommand:
    """误差进带或测量消失时停车等待，不再边数稳定帧边转。"""
    if error is None or not math.isfinite(error) or abs(error) <= max_abs_error:
        return VelocityCommand(0.0, 0.0, "align")
    return alignment_command(error, gain, max_abs_omega)


def forward_strip_points(
    points: Sequence[Point2D],
    max_abs_x_m: float,
) -> list[Point2D]:
    """只保留车头正前方窄带内的中心线，侧面开口里的点不算。"""
    kept: list[Point2D] = []
    limit = abs(float(max_abs_x_m))
    for x, y in points:
        xf, yf = float(x), float(y)
        if math.isfinite(xf) and math.isfinite(yf) and abs(xf) <= limit:
            kept.append((xf, yf))
    return kept


def command_from_forward_strip(
    points: Sequence[Point2D],
    road_pixels: int,
    cfg: FollowConfig,
    max_abs_x_m: float,
) -> VelocityCommand:
    """窄带里没有足够的点就停车，不再朝侧面开口转向。"""
    kept = forward_strip_points(points, max_abs_x_m)
    if len(kept) < cfg.min_points:
        return VelocityCommand(0.0, 0.0, "stop_forward_strip")
    return command_from_centerline(kept, road_pixels, cfg)


def alignment_command(near_x_m: float, gain: float, max_abs_omega: float) -> VelocityCommand:
    """原地摆正：线速度为 0，近处中心线在右侧时角速度为负。"""
    omega = -float(gain) * float(near_x_m)
    limit = abs(float(max_abs_omega))
    if omega > limit:
        omega = limit
    elif omega < -limit:
        omega = -limit
    return VelocityCommand(0.0, omega, "align")


def is_visual_follow(command: VelocityCommand) -> bool:
    """固定预瞄和近距离预瞄都属于视觉闭环循迹。"""
    return command.reason in ("follow", "follow_near")


def follow_config_from_mapping(cfg: dict) -> FollowConfig:
    f = cfg.get("follow", {}) or {}
    return FollowConfig(
        cruise_mps=float(f.get("cruise_mps", 0.10)),
        turn_mps=float(f.get("turn_mps", 0.06)),
        lookahead_m=float(f.get("lookahead_m", 0.45)),
        turn_abs_x_m=float(f.get("turn_abs_x_m", 0.08)),
        min_points=int(f.get("min_points", 8)),
        min_road_pixels=int(f.get("min_road_pixels", 200)),
        max_abs_omega=float(f.get("max_abs_omega", 1.0)),
        steering_gain=float(f.get("steering_gain", 1.0)),
        x_bias_m=float(f.get("x_bias_m", 0.0)),
        min_lookahead_m=float(f.get("min_lookahead_m", 0.0)),
        near_mps=float(f.get("near_mps", 0.05)),
    )


def _point_at_lookahead(points: Sequence[Point2D], lookahead_m: float) -> Optional[Point2D]:
    ordered = sorted(((float(x), float(y)) for x, y in points), key=lambda p: p[1])
    if len(ordered) < 2:
        return None
    for (x0, y0), (x1, y1) in zip(ordered, ordered[1:]):
        if y1 < y0:
            continue
        if y0 <= lookahead_m <= y1:
            span = y1 - y0
            if span <= 1e-6:
                return (x0, lookahead_m)
            t = (lookahead_m - y0) / span
            return (x0 + t * (x1 - x0), lookahead_m)
    return None


def command_from_centerline(
    points: Sequence[Point2D],
    road_pixels: int,
    cfg: FollowConfig,
) -> VelocityCommand:
    """由近到远的中心线给出 (v, ω)。看不见路或中心线不够就停车。"""
    if road_pixels < cfg.min_road_pixels:
        return VelocityCommand(0.0, 0.0, "stop_road")
    if len(points) < cfg.min_points:
        return VelocityCommand(0.0, 0.0, "stop_centerline")

    target = _point_at_lookahead(points, cfg.lookahead_m)
    near_mode = False
    if target is None and 0.0 < cfg.min_lookahead_m < cfg.lookahead_m:
        visible_end = max(float(y) for _, y in points)
        near_y = min(cfg.lookahead_m, visible_end - 0.02)
        if near_y >= cfg.min_lookahead_m:
            target = _point_at_lookahead(points, near_y)
            near_mode = target is not None
    if target is None:
        return VelocityCommand(0.0, 0.0, "stop_lookahead")

    x_right, y_forward = target
    x_right += cfg.x_bias_m
    v = cfg.turn_mps if abs(x_right) >= cfg.turn_abs_x_m else cfg.cruise_mps
    if near_mode:
        v = min(v, cfg.near_mps)
    denom = x_right * x_right + y_forward * y_forward
    if denom <= 1e-8 or y_forward <= 1e-3:
        return VelocityCommand(0.0, 0.0, "stop_lookahead")

    # κ = 2 x / (x²+y²)，x 向右。正 ω 是左转，所以取负。
    omega = -v * (2.0 * x_right) / denom * max(0.0, cfg.steering_gain)
    limit = abs(cfg.max_abs_omega)
    if omega > limit:
        omega = limit
    elif omega < -limit:
        omega = -limit
    return VelocityCommand(v, omega, "follow_near" if near_mode else "follow")
