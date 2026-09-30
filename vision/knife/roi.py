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


def _light_surface_roi(image: np.ndarray) -> tuple[int, int, int, int] | None:
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
    paper_mask, (px, py, pw, ph), _ = paper
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
    _, (ix, iy, iw, ih), _ = object_region

    ratio = ih / max(iw, 1)
    if ratio >= 1.35:
        # 细长或面积较大的连通域通常已包含刀身，只需小幅留边。
        target_width, target_height = 1.2 * iw, 1.12 * ih
        cx, cy = ix + iw / 2, iy + ih / 2
    elif iw >= 150:
        # 较宽的连通域可能仅是横向刀柄；使用纸面范围留出浅色刀刃的位置。
        target_width = min(max(2.4 * iw, 0.53 * pw), 0.82 * pw)
        target_height = min(max(2.4 * ih, 0.80 * ph), 0.95 * ph)
        # 纸上浅色刀刃通常位于刀柄右侧，轻微右移可避免把刀刃尖端截掉。
        cx = 0.55 * (ix + iw / 2) + 0.45 * (px + pw / 2) + 25
        cy = 0.55 * (iy + ih / 2) + 0.45 * (py + ph / 2)
    else:
        # 小而近方形的深色区域多为刀柄，朝纸面中心扩张以纳入刀刃。
        target_width, target_height = 3.0 * iw, 2.4 * ih
        cx = ix + iw / 2 + np.clip((px + pw / 2 - ix - iw / 2) * .4, -55, 55)
        # 刀柄常在刀刃右下侧，方形刀柄向上留出刀刃，比向纸面中心更可靠。
        cy = iy + ih / 2 - 50
    x0 = max(0, round(cx - target_width / 2))
    y0 = max(0, round(cy - target_height / 2))
    x1 = min(image.shape[1], round(cx + target_width / 2))
    y1 = min(image.shape[0], round(cy + target_height / 2))
    return x0, y0, x1, y1


def _simple_foreground_roi(image: np.ndarray) -> tuple[int, int, int, int] | None:
    """无浅色纸面时，尝试从相对均匀的背景上找最大前景。"""
    mask = segment_foreground(image)
    object_region = _largest(mask)
    if object_region is None:
        return None
    _, (x, y, width, height), area = object_region
    ratio = area / mask.size
    if not 0.005 <= ratio <= 0.50:
        return None
    pad = max(12, round(max(width, height) * 0.25))
    return (max(0, x - pad), max(0, y - pad),
            min(image.shape[1], x + width + pad),
            min(image.shape[0], y + height + pad))


def select_knife_roi(frame: np.ndarray) -> KnifeRoi | None:
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
    box = _light_surface_roi(small)
    source = "light_surface"
    if box is None:
        box = _simple_foreground_roi(small)
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
