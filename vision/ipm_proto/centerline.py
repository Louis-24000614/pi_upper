"""在 BEV 道路 mask 上提取米制中心线。"""

from __future__ import annotations

from typing import List, Tuple

import numpy as np

from .ipm import BevConfig

Point2D = Tuple[float, float]


def extract_centerline(
    bev_mask: np.ndarray,
    bev: BevConfig,
    *,
    min_pixels: int = 3,
    binary_thresh: int = 127,
) -> List[Point2D]:
    """按行（固定 Y）取道路像素 X 中位数，得到中心线点列。

    BEV mask 非零（或 > binary_thresh）视为道路。跳过道路像素过少的行。
    点按 Y 从近到远排序（y_min → y_max）。

    Args:
        bev_mask: HxW 单通道或可被阈值化的图。
        bev: 与 mask 对齐的鸟瞰配置。
        min_pixels: 该行至少多少个道路像素才采纳。
        binary_thresh: 灰度阈值；布尔 mask 可忽略。

    Returns:
        [(x_m, y_m), ...]，单位米。
    """
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
    # 图像行 v：0=远端，h-1=近端；输出按近→远
    for v in range(bev.height_px - 1, -1, -1):
        xs = np.flatnonzero(road[v])
        if xs.size < min_pixels:
            continue
        u = float(np.median(xs))
        x_m, y_m = bev.bev_px_to_ground(u, float(v))
        points.append((x_m, y_m))
    return points


def centerline_lateral_error(
    points: List[Point2D],
    *,
    near_y_max: float = 0.6,
) -> float:
    """近端中心线点的平均 |X|，用于直道自检。"""
    near = [p[0] for p in points if p[1] <= near_y_max]
    if not near:
        near = [p[0] for p in points[: max(1, len(points) // 4)]]
    if not near:
        return float("inf")
    return float(np.mean(np.abs(near)))
