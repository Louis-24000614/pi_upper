"""在鸟瞰路面上判断前方是直路、丁字、十字、拐角还是被截断。

只报告开着的方向，不发转弯，也不改寻线速度。车道宽度从近处估计；
近处不像一条路时，用赛道大约 0.2 m 的宽度。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ipm import BevConfig

KIND_STRAIGHT = "straight"
KIND_T = "t_junction"
KIND_CROSS = "cross"
KIND_CORNER = "corner"
KIND_BLOCKED = "blocked"
KIND_UNKNOWN = "unknown"


@dataclass(frozen=True)
class JunctionRead:
    """一帧鸟瞰的开口。X 向右、Y 向前。"""

    kind: str
    forward: bool
    left: bool
    right: bool
    lane_x_m: float
    lane_width_m: float
    # 左/右支路在车体前方的中心距离；没有侧向支路时为 None。
    junction_y_m: float | None = None
    # 当前车道中央走廊在原始 BEV road mask 中的最远距离。
    corridor_end_y_m: float | None = None


class JunctionTracker:
    """同一种判断连续若干帧才切换，避免一帧误分割改口。"""

    def __init__(self, hold: int = 3) -> None:
        self._hold = max(1, hold)
        self._shown = KIND_UNKNOWN
        self._pending = KIND_UNKNOWN
        self._count = 0

    def update(self, reading: JunctionRead) -> str:
        if reading.kind == self._pending:
            self._count += 1
        else:
            self._pending = reading.kind
            self._count = 1
        if self._count >= self._hold:
            self._shown = reading.kind
        return self._shown


def classify_junction(
    bev_mask: np.ndarray,
    bev: BevConfig,
    *,
    lane_width_m: float = 0.20,
    binary_thresh: int = 127,
) -> JunctionRead:
    """数前方 0.5–0.9 m 左、右、前是否还有路。

    近处 0.2 m 用来量本车道。左右开口要在车道外侧连续至少约 0.08 m，
    十字还要求横路过去之后正前方仍有路。
    """
    road = _as_road(bev_mask, binary_thresh)
    if road.shape != (bev.height_px, bev.width_px):
        raise ValueError(
            f"mask shape {road.shape} != bev ({bev.height_px}, {bev.width_px})"
        )

    lane_x, lane_w, near_ok = _lane_from_near(road, bev, lane_width_m)
    if not near_ok:
        return JunctionRead(KIND_UNKNOWN, False, False, False, lane_x, lane_w)

    half = lane_w * 0.5
    margin = 0.08
    left_ys = _side_ys(road, bev, lane_x, half, margin, left=True)
    right_ys = _side_ys(road, bev, lane_x, half, margin, left=False)
    left = _covers(left_ys, 0.06)
    right = _covers(right_ys, 0.06)
    corridor_ys = _corridor_ys(road, bev, lane_x, half)
    last_corridor = _continuous_corridor_end(corridor_ys, bev)

    if not left and not right:
        forward = last_corridor >= bev.y_max - 0.18
        kind = KIND_STRAIGHT if forward else KIND_BLOCKED
        return JunctionRead(
            kind, forward, False, False, lane_x, lane_w, None, last_corridor
        )

    side_ys = left_ys + right_ys
    junction_y = 0.5 * (min(side_ys) + max(side_ys))
    past = (max(side_ys) + 0.04) if side_ys else bev.y_min
    forward = _covers([y for y in corridor_ys if y >= past], 0.06)
    if forward and left and right:
        kind = KIND_CROSS
    elif forward:
        kind = KIND_T
    elif left and right:
        kind = KIND_T
    else:
        kind = KIND_CORNER
    return JunctionRead(
        kind, forward, left, right, lane_x, lane_w, junction_y, last_corridor
    )


def _as_road(bev_mask: np.ndarray, binary_thresh: int) -> np.ndarray:
    gray = bev_mask[:, :, 0] if bev_mask.ndim == 3 else bev_mask
    if gray.dtype == np.bool_ or gray.size == 0 or gray.max() <= 1:
        return gray.astype(bool)
    return gray > binary_thresh


def _lane_from_near(
    road: np.ndarray, bev: BevConfig, default_width: float
) -> tuple[float, float, bool]:
    """近处一条路的中心和宽度。近处几乎全是路时视为看不清车道。"""
    y0 = bev.y_min
    y1 = min(bev.y_max, bev.y_min + 0.22)
    centers: list[float] = []
    widths: list[float] = []
    road_px = 0
    band_px = 0
    for v in _rows_between(bev, y0, y1):
        xs = np.flatnonzero(road[v])
        band_px += road.shape[1]
        road_px += int(xs.size)
        if xs.size < 3:
            continue
        u = float(np.median(xs))
        x_m, _ = bev.bev_px_to_ground(u, float(v))
        centers.append(x_m)
        widths.append(float(xs.size) * bev.m_per_px)
    if len(centers) < 5 or band_px == 0 or road_px / band_px > 0.92:
        return 0.0, default_width, False
    lane_w = float(np.median(widths))
    if lane_w < 0.12 or lane_w > 0.95:
        lane_w = default_width
    return float(np.median(centers)), lane_w, True


def _side_ys(
    road: np.ndarray,
    bev: BevConfig,
    lane_x: float,
    half: float,
    margin: float,
    *,
    left: bool,
) -> list[float]:
    y0 = min(bev.y_max, bev.y_min + 0.28)
    y1 = min(bev.y_max, y0 + 0.45)
    found: list[float] = []
    min_px = max(3, int(round(0.06 / bev.m_per_px)))
    for v in _rows_between(bev, y0, y1):
        xs = np.flatnonzero(road[v])
        if xs.size == 0:
            continue
        ground_x = bev.x_min + xs.astype(np.float64) * bev.m_per_px
        if left:
            outside = ground_x < lane_x - half - margin
        else:
            outside = ground_x > lane_x + half + margin
        if int(np.count_nonzero(outside)) >= min_px:
            _, y_m = bev.bev_px_to_ground(0.0, float(v))
            found.append(y_m)
    return found


def _corridor_ys(road: np.ndarray, bev: BevConfig, lane_x: float, half: float) -> list[float]:
    found: list[float] = []
    limit = half + 0.04
    for v in range(bev.height_px):
        xs = np.flatnonzero(road[v])
        if xs.size == 0:
            continue
        ground_x = bev.x_min + xs.astype(np.float64) * bev.m_per_px
        if np.any(np.abs(ground_x - lane_x) <= limit):
            _, y_m = bev.bev_px_to_ground(0.0, float(v))
            found.append(y_m)
    return found


def _continuous_corridor_end(ys: list[float], bev: BevConfig) -> float:
    """只接受从车前方连续连通的走廊，忽略远处孤立误分割。"""
    ordered = sorted(set(ys))
    max_gap = max(0.03, 2.5 * bev.m_per_px)
    end = bev.y_min
    started = False
    for y_m in ordered:
        if not started:
            if y_m > bev.y_min + max_gap:
                break
            end = y_m
            started = True
        elif y_m - end <= max_gap:
            end = y_m
        else:
            break
    return end


def _rows_between(bev: BevConfig, y0: float, y1: float) -> range:
    if y1 < y0:
        y0, y1 = y1, y0
    v_far = int(np.floor((bev.y_max - y1) / bev.m_per_px))
    v_near = int(np.ceil((bev.y_max - y0) / bev.m_per_px))
    v_far = max(0, min(bev.height_px - 1, v_far))
    v_near = max(0, min(bev.height_px - 1, v_near))
    if v_near < v_far:
        return range(0)
    return range(v_far, v_near + 1)


def _covers(ys: list[float], span_m: float) -> bool:
    """这些距离里有没有一段连续开口，长度达到 span_m。"""
    if len(ys) < 2:
        return False
    ordered = sorted(ys)
    best = 0.0
    start = ordered[0]
    prev = ordered[0]
    for cur in ordered[1:]:
        if cur - prev > 0.03:
            best = max(best, prev - start)
            start = cur
        prev = cur
    best = max(best, prev - start)
    return best >= span_m
