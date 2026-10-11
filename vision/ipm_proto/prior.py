"""BEV 路宽先验：抑制同色场上过宽 / 过碎的 road mask 行。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .ipm import BevConfig

Point2D = Tuple[float, float]


@dataclass(frozen=True)
class RoadWidthPrior:
    """赛题走廊宽度约束。"""

    expected_width_m: float = 0.8
    min_width_m: float = 0.35
    max_width_m: float = 1.15
    min_pixels: int = 3


def _segments(xs: np.ndarray, max_gap_px: int = 0) -> List[Tuple[int, int]]:
    """把列索引拆成 [start, end]；只允许桥接指定大小的分割裂缝。"""
    if xs.size == 0:
        return []
    segs: List[Tuple[int, int]] = []
    start = int(xs[0])
    prev = start
    for x in xs[1:]:
        xi = int(x)
        if xi <= prev + 1 + max_gap_px:
            prev = xi
            continue
        segs.append((start, prev))
        start = prev = xi
    segs.append((start, prev))
    return segs


def _pick_band(
    xs: np.ndarray,
    bev: BevConfig,
    prior: RoadWidthPrior,
    preferred_x_m: float,
    lane_width_m: float | None = None,
    edge_reference_x_m: float | None = None,
) -> Optional[Tuple[float, float]]:
    """在一行道路像素中选出一带，返回 (x_m, width_m)。"""
    # 主路偶有 1～2 px 的分割裂缝，不能把左右半条路当作独立支路。
    segs = _segments(xs, max_gap_px=2)
    if not segs:
        return None

    best_score = (float("inf"), float("inf"))
    best = None
    pref_u, _ = bev.ground_to_bev_px(preferred_x_m, bev.y_min)
    reference_width = prior.expected_width_m if lane_width_m is None else lane_width_m

    for left, right in segs:
        width_m = (right - left + 1) * bev.m_per_px
        u_c = 0.5 * (left + right)
        x_m, _ = bev.bev_px_to_ground(u_c, 0.0)
        if right - left + 1 < prior.min_pixels:
            continue
        # 优先延续包含预测中心的连续段，不能因为支路宽度更标准就换道。
        distance = max(left - pref_u, pref_u - right, 0.0) * bev.m_per_px
        score = (distance, abs(width_m - reference_width) + 0.5 * abs(x_m - preferred_x_m))
        if score < best_score:
            best_score = score
            best = (x_m, width_m, left, right)

    if best is None:
        return None
    x_m, width_m, left, right = best
    if width_m < prior.min_width_m:
        # 最近主路段太窄时缺测，不为满足最小宽度而跳到另一条支路。
        return None

    # 与近端实际路宽比较，支路合并后的总宽度即使低于 max 也不能取整段中点。
    widening = lane_width_m is not None and width_m > lane_width_m + max(
        bev.m_per_px, 0.05 * lane_width_m
    ) + 1e-9
    if width_m > prior.max_width_m or widening:
        window_width = min(reference_width, prior.max_width_m, width_m)
        half_px = 0.5 * (window_width / bev.m_per_px - 1.0)
        center_u = pref_u
        if lane_width_m is not None:
            # 单侧开口时沿仍连续的另一侧边界走，真实斜路/弯路仍可更新方向。
            # 用近端干净短段的方向延续上一行窗口，避免扩宽中点污染局部斜率。
            previous_u, _ = bev.ground_to_bev_px(
                preferred_x_m if edge_reference_x_m is None else edge_reference_x_m, bev.y_min
            )
            left_error = abs(left - (previous_u - half_px))
            right_error = abs(right - (previous_u + half_px))
            if left_error + 0.5 < right_error:
                center_u = left + half_px
            elif right_error + 0.5 < left_error:
                center_u = right - half_px
        # 窗口必须落在选中的连续段内，不能跨过空隙借用别的区域边界。
        center_u = float(np.clip(center_u, left + half_px, right - half_px))
        x_m, _ = bev.bev_px_to_ground(center_u, 0.0)
        return x_m, window_width

    return x_m, width_m


def extract_centerline_with_width_prior(
    bev_mask: np.ndarray,
    bev: BevConfig,
    prior: RoadWidthPrior | None = None,
    *,
    binary_thresh: int = 127,
    preferred_x_m: float = 0.0,
) -> List[Point2D]:
    """按行提取中心线，并用路宽先验抑制过分割。

    从近端连续主路估计路宽和局部方向，再向远处追踪；侧向扩宽时只取
    主路窗口，保留连续路边给出的方向。原始 mask 不变，仍可用于路口检测。
    """
    if prior is None:
        prior = RoadWidthPrior()

    if bev_mask.ndim == 3:
        gray = bev_mask[:, :, 0]
    else:
        gray = bev_mask

    if gray.shape[0] != bev.height_px or gray.shape[1] != bev.width_px:
        raise ValueError(
            f"mask shape {gray.shape} != bev ({bev.height_px}, {bev.width_px})"
        )

    if gray.dtype == np.bool_ or gray.max() <= 1:
        road = gray.astype(bool)
    else:
        road = gray > binary_thresh

    points: List[Point2D] = []
    pref = preferred_x_m
    seed_widths: List[float] = []
    seed_slope = 0.0
    for v in range(bev.height_px - 1, -1, -1):
        xs = np.flatnonzero(road[v])
        if xs.size < prior.min_pixels:
            continue
        _, y_m = bev.bev_px_to_ground(0.0, float(v))
        if len(points) == 5:
            near = np.asarray(points)
            near_ys = near[:, 1] - near[:, 1].mean()
            seed_slope = float(np.dot(near_ys, near[:, 0]) / np.dot(near_ys, near_ys))
        if len(points) >= 5:
            recent = np.asarray(points[-8:])
            ys = recent[:, 1] - recent[:, 1].mean()
            slope = float(np.dot(ys, recent[:, 0]) / np.dot(ys, ys))
            pref = float(recent[:, 0].mean() + slope * (y_m - recent[:, 1].mean()))
        lane_width = float(np.median(seed_widths)) if seed_widths else None
        if lane_width is not None and 0.8 * prior.expected_width_m <= lane_width:
            # 近端轻微欠分割不应把正常标称路宽也当成开口。
            lane_width = max(lane_width, prior.expected_width_m)
        edge_reference = None
        if points:
            last_x, last_y = points[-1]
            edge_reference = last_x + seed_slope * (y_m - last_y)
        picked = _pick_band(xs, bev, prior, pref, lane_width, edge_reference)
        if picked is None:
            continue
        x_m, width = picked
        # 只用最靠近车辆的短段初始化宽度；不能沿着逐渐变宽的支路学习路宽。
        if len(seed_widths) < 8:
            seed_widths.append(width)
        points.append((x_m, y_m))
        pref = x_m
    return points


def road_prior_from_mapping(cfg: dict) -> RoadWidthPrior:
    """从 ``nav_camera.yaml`` / ``config_example.yaml`` 的 ``road_prior`` 段构造。"""
    p = cfg.get("road_prior", {}) or {}
    return RoadWidthPrior(
        expected_width_m=float(p.get("expected_width_m", 0.8)),
        min_width_m=float(p.get("min_width_m", 0.35)),
        max_width_m=float(p.get("max_width_m", 1.15)),
        min_pixels=int(p.get("min_pixels", 3)),
    )
