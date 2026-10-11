"""单个推理 worker 的临时内存；返回的最终 mask 仍归调用者独立持有。"""
from __future__ import annotations

import cv2
import numpy as np


class SegmentBuffers:
    """固定 640 输入的画布/RGB/缩放缓存，不跨 worker 共享可写数组。"""

    def __init__(self, size: int = 640) -> None:
        self.size = size
        self.canvas = np.empty((size, size, 3), np.uint8)
        self.rgb = np.empty_like(self.canvas)
        self.merged = np.empty((size, size), bool)
        self.up = np.empty((size, size), np.float32)
        self.resized: np.ndarray | None = None

    def letterbox(self, bgr: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        """与原实现的缩放/补边/颜色转换完全相同，只改变 dst 的内存归属。"""
        height, width = bgr.shape[:2]
        ratio = min(self.size / height, self.size / width)
        rw, rh = int(round(width * ratio)), int(round(height * ratio))
        if self.resized is None or self.resized.shape != (rh, rw, 3):
            self.resized = np.empty((rh, rw, 3), np.uint8)
        cv2.resize(bgr, (rw, rh), dst=self.resized, interpolation=cv2.INTER_LINEAR)
        # 每次填满画布，避免输入宽高比改变后留下上一帧像素。
        self.canvas.fill(114)
        left, top = (self.size - rw) // 2, (self.size - rh) // 2
        self.canvas[top:top + rh, left:left + rw] = self.resized
        cv2.cvtColor(self.canvas, cv2.COLOR_BGR2RGB, dst=self.rgb)
        return self.rgb, ratio, left, top
