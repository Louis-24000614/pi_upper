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


def _segments(xs: np.ndarray) -> List[Tuple[int, int]]:
    """把已排序的列索引拆成连续段 [start, end]（含端点）。"""
    if xs.size == 0:
        return []
    segs: List[Tuple[int, int]] = []
    start = int(xs[0])
    prev = start
    for x in xs[1:]:
        xi = int(x)
        if xi == prev + 1:
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
) -> Optional[Tuple[float, float]]:
    """在一行道路像素中选出一带，返回 (x_m, width_m)。"""
    segs = _segments(xs)
    if not segs:
        return None

    best_score = float("inf")
    best: Optional[Tuple[float, float, float, float]] = None  # score, x, w, u_center

    for left, right in segs:
        width_m = (right - left + 1) * bev.m_per_px
        u_c = 0.5 * (left + right)
        x_m, _ = bev.bev_px_to_ground(u_c, 0.0)
        # 宽度贴近期望、中心贴近上一行/车体中线
        score = abs(width_m - prior.expected_width_m) + 0.5 * abs(x_m - preferred_x_m)
        if width_m < prior.min_width_m:
            score += 1.0
        if score < best_score:
            best_score = score
            best = (score, x_m, width_m, u_c)

    assert best is not None
    _, x_m, width_m, u_c = best

    if width_m < prior.min_width_m:
        return None

    if width_m > prior.max_width_m:
        half_px = 0.5 * prior.expected_width_m / bev.m_per_px
        # 在过宽段内取靠近 preferred 的期望宽窗口
        pref_u, _ = bev.ground_to_bev_px(preferred_x_m, bev.y_min)
        left = int(xs.min())
        right = int(xs.max())
        center_u = float(np.clip(pref_u, left + half_px, right - half_px))
        if right - left + 1 < 2 * half_px:
            center_u = 0.5 * (left + right)
        x_m, _ = bev.bev_px_to_ground(center_u, 0.0)
        return x_m, prior.expected_width_m

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

    - 多段时选宽度接近 ``expected_width_m`` 且中心靠近 ``preferred_x_m`` 的段；
    - 单段过宽时裁成期望宽度窗口（中心偏向 preferred）；
    - 沿 Y 近→远推进时用上一行中心更新 preferred，形成空间平滑。
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
    for v in range(bev.height_px - 1, -1, -1):
        xs = np.flatnonzero(road[v])
        if xs.size < prior.min_pixels:
            continue
        picked = _pick_band(xs, bev, prior, pref)
        if picked is None:
            continue
        x_m, _width = picked
        _, y_m = bev.bev_px_to_ground(0.0, float(v))
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
