"""按真实经过时间录制相机帧，供离线复盘道路分割。"""

from __future__ import annotations

import math
import threading
from pathlib import Path

import cv2
import numpy as np


class VideoRecorder:
    """将不定间隔的相机帧写成固定帧率的 MJPEG AVI。"""

    def __init__(self, path: Path, fps: int = 10) -> None:
        if fps <= 0:
            raise ValueError("录制帧率必须大于 0")
        if path.suffix.lower() != ".avi":
            raise ValueError("录像文件必须以 .avi 结尾")
        if path.exists():
            raise FileExistsError(f"录像文件已存在，不覆盖: {path}")
        self.path = path
        self.fps = fps
        self.frames_written = 0
        self._writer: cv2.VideoWriter | None = None
        self._first_s: float | None = None
        self._last_frame: np.ndarray | None = None
        self._shape: tuple[int, int, int] | None = None
        self._closing = threading.Event()
        self._close_padding_remaining = 1

    def write(self, frame: np.ndarray, captured_s: float) -> None:
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("录像只接受 uint8 BGR 三通道相机帧")
        if self._writer is None:
            if self.path.exists():
                raise FileExistsError(f"录像文件已存在，不覆盖: {self.path}")
            self.path.parent.mkdir(parents=True, exist_ok=True)
            height, width = frame.shape[:2]
            writer = cv2.VideoWriter(
                str(self.path), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (width, height)
            )
            if not writer.isOpened():
                writer.release()
                raise RuntimeError(f"无法创建录像: {self.path}")
            self._writer = writer
            self._first_s = captured_s
            self._shape = frame.shape
        elif frame.shape != self._shape:
            raise ValueError(f"相机分辨率在录制中改变: {self._shape} -> {frame.shape}")

        assert self._first_s is not None
        target_index = int(max(0.0, captured_s - self._first_s) * self.fps + 1e-6)
        self._fill_until(target_index)
        if self.frames_written <= target_index:
            self._writer.write(frame)
            self.frames_written += 1
        self._last_frame = frame.copy()

    def _fill_until(self, target_index: int) -> None:
        if self._writer is None or self._last_frame is None:
            return
        while self.frames_written < target_index:
            if self._closing.is_set():
                # 关闭后整个队列最多保留一个间隔补帧，及时结束已经在途的长补帧。
                if self._close_padding_remaining == 0:
                    break
                self._close_padding_remaining -= 1
            self._writer.write(self._last_frame)
            self.frames_written += 1

    def request_close(self) -> None:
        """仅发送补帧停止信号；编码器仍由原线程完成剩余真实帧和封尾。"""
        self._closing.set()

    def close(self, stopped_s: float | None = None) -> None:
        self.request_close()
        if self._writer is None:
            return
        try:
            if stopped_s is not None and self._first_s is not None:
                elapsed_frames = max(0.0, stopped_s - self._first_s) * self.fps
                # 封尾最多再补一帧，不把长时间无画面补到退出时刻。
                if self.frames_written < math.ceil(elapsed_frames - 1e-6):
                    self._writer.write(self._last_frame)
                    self.frames_written += 1
        finally:
            try:
                self._writer.release()
            finally:
                self._writer = None
