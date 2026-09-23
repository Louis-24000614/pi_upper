"""中心线纯跟踪。地面 X 向右、Y 向前；串口正角速度是左转。"""

from __future__ import annotations

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
    min_road_pixels: int = 400
    max_abs_omega: float = 1.0


@dataclass(frozen=True)
class VelocityCommand:
    v_mps: float
    omega_radps: float
    reason: str


def follow_config_from_mapping(cfg: dict) -> FollowConfig:
    f = cfg.get("follow", {}) or {}
    return FollowConfig(
        cruise_mps=float(f.get("cruise_mps", 0.10)),
        turn_mps=float(f.get("turn_mps", 0.06)),
        lookahead_m=float(f.get("lookahead_m", 0.45)),
        turn_abs_x_m=float(f.get("turn_abs_x_m", 0.08)),
        min_points=int(f.get("min_points", 8)),
        min_road_pixels=int(f.get("min_road_pixels", 400)),
        max_abs_omega=float(f.get("max_abs_omega", 1.0)),
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
    if target is None:
        return VelocityCommand(0.0, 0.0, "stop_lookahead")

    x_right, y_forward = target
    v = cfg.turn_mps if abs(x_right) >= cfg.turn_abs_x_m else cfg.cruise_mps
    denom = x_right * x_right + y_forward * y_forward
    if denom <= 1e-8 or y_forward <= 1e-3:
        return VelocityCommand(0.0, 0.0, "stop_lookahead")

    # κ = 2 x / (x²+y²)，x 向右。正 ω 是左转，所以取负。
    omega = -v * (2.0 * x_right) / denom
    limit = abs(cfg.max_abs_omega)
    if omega > limit:
        omega = limit
    elif omega < -limit:
        omega = -limit
    return VelocityCommand(v, omega, "follow")
