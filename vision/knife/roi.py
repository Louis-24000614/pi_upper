"""在相机整帧中自动寻找单把刀的轻量取景区域。"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .preprocessing import segment_foreground


@dataclass(frozen=True)
class KnifeRoi:
    """原始帧坐标中的取景框及其来源。"""

    xyxy: tuple[int, int, int, int]
    source: str


def _largest(mask: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int], int] | None:
    """返回最大连通域；面积用于拒绝纸上文字等小噪声。"""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, width, height, area = map(int, stats[index])
    return np.where(labels == index, 255, 0).astype(np.uint8), (x, y, width, height), area


def _directional_roi(mask: np.ndarray, reverse: bool = False) -> tuple[int, int, int, int] | None:
    """沿已选连通域的长轴延伸，先朝下，反向请求朝上。"""
    ys, xs = np.nonzero(mask)
    if len(xs) < 20:
        return None
    points = np.column_stack((xs, ys)).astype(np.float32)
    _, eigenvectors = cv2.PCACompute(points, mean=None, maxComponents=2)
    axis = eigenvectors[0]
    # 图像 y 轴向下；接近水平时用向左作为稳定的首选。
    if axis[1] < 0 or (abs(axis[1]) < 1e-6 and axis[0] > 0):
        axis = -axis
    if reverse:
        axis = -axis
    projection = points @ axis
    length = max(20.0, float(np.max(projection) - np.min(projection)))
    dx, dy = axis * (2.0 * length)
    pad = max(12.0, 0.25 * length)
    height, width = mask.shape
    return (
        max(0, int(np.floor(xs.min() + min(0.0, dx) - pad))),
        max(0, int(np.floor(ys.min() + min(0.0, dy) - pad))),
        min(width, int(np.ceil(xs.max() + 1 + max(0.0, dx) + pad))),
        min(height, int(np.ceil(ys.max() + 1 + max(0.0, dy) + pad))),
    )


def _light_surface_roi(image: np.ndarray, reverse: bool = False) -> tuple[int, int, int, int] | None:
    """先找浅色纸面，再用局部对比找到刀柄或深色刀身。

    局部亮度差能忽略纸张的大面积阴影；直接对整张纸做全局阈值容易把纸边
    当作刀具，白色刀刃又容易消失。这里只估计宽松的取景框，最终类别仍由模型给出。
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    saturation, value = hsv[:, :, 1], hsv[:, :, 2]
    paper_seed = np.where((saturation < 45) & (value > 95), 255, 0).astype(np.uint8)
    paper_seed = cv2.morphologyEx(paper_seed, cv2.MORPH_CLOSE, np.ones((19, 19), np.uint8))
    paper = _largest(paper_seed)
    if paper is None or paper[2] < 0.18 * paper_seed.size:
        return None
    paper_mask, _, _ = paper
    contours, _ = cv2.findContours(paper_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    paper_mask[:] = 0
    cv2.drawContours(paper_mask, [max(contours, key=cv2.contourArea)], -1, 255, cv2.FILLED)
    inner = cv2.erode(paper_mask, np.ones((15, 15), np.uint8))

    local_light = cv2.GaussianBlur(value, (0, 0), sigmaX=25).astype(np.float32)
    ink = np.where(
        (inner > 0) & ((local_light - value > 28) | (saturation > 60)), 255, 0
    ).astype(np.uint8)
    ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    object_region = _largest(ink)
    if object_region is None or object_region[2] < 0.005 * ink.size:
        return None
    return _directional_roi(object_region[0], reverse)


def _simple_foreground_roi(image: np.ndarray, reverse: bool = False) -> tuple[int, int, int, int] | None:
    """无浅色纸面时，尝试从相对均匀的背景上找最大前景。"""
    mask = segment_foreground(image)
    object_region = _largest(mask)
    if object_region is None:
        return None
    mask, _, area = object_region
    ratio = area / mask.size
    if not 0.005 <= ratio <= 0.50:
        return None
    return _directional_roi(mask, reverse)


def select_knife_roi(frame: np.ndarray, reverse: bool = False) -> KnifeRoi | None:
    """返回原图中的自动单刀框；证据不足时返回None并保留原始整帧。

    最长边缩至640像素，避免GUI每次送1280×720帧时在CPU上做全分辨率分割。
    小于400×300的已裁剪图片保持原样，防止二次取景截断刀具。
    """
    if frame.ndim != 3 or frame.shape[2] not in (3, 4) or frame.dtype != np.uint8:
        raise ValueError(f"自动取景需要uint8 BGR/BGRA图像，实际={frame.shape} {frame.dtype}")
    height, width = frame.shape[:2]
    if width < 400 or height < 300:
        return None
    scale = min(1.0, 640.0 / max(width, height))
    small_width, small_height = round(width * scale), round(height * scale)
    small = cv2.resize(frame[:, :, :3], (small_width, small_height), interpolation=cv2.INTER_AREA)
    box = _light_surface_roi(small, reverse)
    source = "light_surface"
    if box is None:
        box = _simple_foreground_roi(small, reverse)
        source = "foreground"
    if box is None:
        return None
    x0, y0, x1, y1 = box
    if x1 - x0 < 40 or y1 - y0 < 40:
        return None
    return KnifeRoi(
        (max(0, round(x0 / scale)), max(0, round(y0 / scale)),
         min(width, round(x1 / scale)), min(height, round(y1 / scale))),
        source,
    )
