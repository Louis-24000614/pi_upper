"""刀具图像的固定几何预处理。

该实现与已经完成板端实验的算法保持一致。注册图和查询图必须使用同一流程，
否则模板向量与查询向量不再可比。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class PreprocessMetadata:
    """记录可用于排查现场失败的几何与质量信息。"""

    pca_angle_deg: float
    flipped: bool
    bbox_xywh: tuple[int, int, int, int]
    foreground_pixels: int
    foreground_ratio: float

    def to_dict(self) -> dict:
        """转换为可序列化字典。"""
        return asdict(self)


def _largest_component(mask: np.ndarray) -> np.ndarray:
    """只保留最大连通域，抑制背景纹理和压缩噪声。"""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return mask
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return np.where(labels == index, 255, 0).astype(np.uint8)


def segment_foreground(image: np.ndarray) -> np.ndarray:
    """优先使用alpha，否则以边框Lab颜色估计简单背景下的刀具前景。"""
    if image.ndim != 3 or image.shape[2] not in (3, 4):
        raise ValueError(f"输入必须是BGR或BGRA图像，实际shape={image.shape}")
    if image.dtype != np.uint8:
        raise ValueError(f"输入必须为uint8，实际dtype={image.dtype}")
    if image.shape[2] == 4:
        alpha = image[:, :, 3]
        if np.count_nonzero(alpha > 16) > image.shape[0] * image.shape[1] * 0.01:
            return _largest_component(np.where(alpha > 16, 255, 0).astype(np.uint8))

    bgr = image[:, :, :3]
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    border = np.concatenate((lab[0], lab[-1], lab[:, 0], lab[:, -1]), axis=0)
    background = np.median(border, axis=0)
    distance = np.linalg.norm(lab - background, axis=2)
    scaled = np.clip(distance * 3.0, 0, 255).astype(np.uint8)
    _, mask = cv2.threshold(scaled, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return _largest_component(mask)


def _rotate_bound(image: np.ndarray, angle: float, interpolation: int) -> np.ndarray:
    """旋转并扩展画布，防止细长刀具两端被裁切。"""
    height, width = image.shape[:2]
    center = (width / 2.0, height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    cosine, sine = abs(matrix[0, 0]), abs(matrix[0, 1])
    new_width = int(height * sine + width * cosine)
    new_height = int(height * cosine + width * sine)
    matrix[0, 2] += new_width / 2.0 - center[0]
    matrix[1, 2] += new_height / 2.0 - center[1]
    border = (0, 0, 0, 0) if image.shape[2] == 4 else 0
    return cv2.warpAffine(
        image,
        matrix,
        (new_width, new_height),
        flags=interpolation,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border,
    )


def _end_width(mask: np.ndarray, left: bool) -> float:
    """估计刀具两端粗细，用于统一刀尖方向。"""
    columns = np.flatnonzero(np.any(mask > 0, axis=0))
    if len(columns) == 0:
        return float("inf")
    start, end = int(columns[0]), int(columns[-1])
    length = max(end - start + 1, 1)
    widths = np.count_nonzero(mask > 0, axis=0).astype(np.float32)
    if left:
        area = widths[start + int(0.03 * length) : start + max(int(0.18 * length), 1)]
    else:
        area = widths[end - max(int(0.18 * length), 1) : end - int(0.03 * length)]
    nonzero = area[area > 0]
    return float(np.mean(nonzero)) if len(nonzero) else float("inf")


def _align_long_axis(image: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict]:
    """用PCA对齐刀具长轴，并令较尖的一端朝右。"""
    ys, xs = np.nonzero(mask > 0)
    if len(xs) < 20:
        raise ValueError("前景像素过少，无法可靠对齐")
    points = np.column_stack((xs, ys)).astype(np.float32)
    _, eigenvectors = cv2.PCACompute(points, mean=None, maxComponents=2)
    axis = eigenvectors[0]
    angle = float(np.degrees(np.arctan2(axis[1], axis[0])))
    aligned_image = _rotate_bound(image, angle, cv2.INTER_LINEAR)
    aligned_mask = _rotate_bound(mask[:, :, None].repeat(3, axis=2), angle, cv2.INTER_NEAREST)[:, :, 0]
    flipped = _end_width(aligned_mask, left=True) < _end_width(aligned_mask, left=False)
    if flipped:
        aligned_image = cv2.flip(aligned_image, 1)
        aligned_mask = cv2.flip(aligned_mask, 1)
    x, y, width, height = cv2.boundingRect(aligned_mask)
    pad = max(4, int(max(width, height) * 0.03))
    x0, y0 = max(0, x - pad), max(0, y - pad)
    x1 = min(aligned_mask.shape[1], x + width + pad)
    y1 = min(aligned_mask.shape[0], y + height + pad)
    return (
        aligned_image[y0:y1, x0:x1],
        aligned_mask[y0:y1, x0:x1],
        {"angle": angle, "flipped": bool(flipped), "bbox": (x, y, width, height)},
    )


def _letterbox(image: np.ndarray, mask: np.ndarray, size: int) -> tuple[np.ndarray, np.ndarray]:
    """保持长宽比缩放到92%画布并以127灰色补边。"""
    height, width = image.shape[:2]
    scale = min((size * 0.92) / max(width, 1), (size * 0.92) / max(height, 1))
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
    resized_mask = cv2.resize(mask, (new_width, new_height), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((size, size, 3), 127, dtype=np.uint8)
    mask_canvas = np.zeros((size, size), dtype=np.uint8)
    x0, y0 = (size - new_width) // 2, (size - new_height) // 2
    canvas[y0 : y0 + new_height, x0 : x0 + new_width] = resized
    mask_canvas[y0 : y0 + new_height, x0 : x0 + new_width] = resized_mask
    return canvas, mask_canvas


def preprocess_bgr(
    image: np.ndarray,
    size: int = 448,
    min_foreground_pixels: int = 20,
    min_foreground_ratio: float = 0.005,
) -> tuple[np.ndarray, np.ndarray, PreprocessMetadata]:
    """把内存中的BGR/BGRA图像转换为已对齐RGB输入。

    Raises:
        ValueError: 输入格式不符或没有足够前景时抛出，调用方应要求重新拍摄。
    """
    mask = segment_foreground(image)
    foreground_pixels = int(np.count_nonzero(mask))
    foreground_ratio = foreground_pixels / float(mask.size)
    if foreground_pixels < min_foreground_pixels or foreground_ratio < min_foreground_ratio:
        raise ValueError(
            f"前景不足: pixels={foreground_pixels}, ratio={foreground_ratio:.6f}"
        )
    aligned, aligned_mask, geometry = _align_long_axis(image, mask)
    if aligned.shape[2] == 4:
        alpha = aligned[:, :, 3:4].astype(np.float32) / 255.0
        bgr = (aligned[:, :, :3] * alpha + 127.0 * (1.0 - alpha)).astype(np.uint8)
    else:
        bgr = aligned[:, :, :3]
    boxed, boxed_mask = _letterbox(bgr, aligned_mask, size)
    metadata = PreprocessMetadata(
        pca_angle_deg=geometry["angle"],
        flipped=geometry["flipped"],
        bbox_xywh=geometry["bbox"],
        foreground_pixels=foreground_pixels,
        foreground_ratio=foreground_ratio,
    )
    return cv2.cvtColor(boxed, cv2.COLOR_BGR2RGB), boxed_mask, metadata


def normalize_rgb(rgb: np.ndarray) -> np.ndarray:
    """按已验收模型约定执行一次ImageNet归一化，输出连续NHWC张量。"""
    if rgb.shape != (448, 448, 3) or rgb.dtype != np.uint8:
        raise ValueError(f"归一化输入应为(448,448,3) uint8，实际={rgb.shape} {rgb.dtype}")
    mean = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)[None, None, None, :]
    std = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)[None, None, None, :]
    tensor = rgb[None].astype(np.float32) / np.float32(255.0)
    return np.ascontiguousarray((tensor - mean) / std)
