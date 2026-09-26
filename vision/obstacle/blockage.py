"""把逐帧障碍检测框转换成可供上层消费的硬堵塞事件。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from .detect import Detection


@dataclass(frozen=True)
class HardBlockConfig:
    """当前道路障碍的空间过滤和时序确认参数。"""

    confirm_frames: int = 3
    min_score: float = 0.45
    min_box_area_ratio: float = 0.002
    min_bottom_ratio: float = 0.35
    corridor_left_ratio: float = 0.20
    corridor_right_ratio: float = 0.80
    min_corridor_overlap: float = 0.30
    min_track_iou: float = 0.20


@dataclass(frozen=True)
class ObstacleObservation:
    """一帧障碍判定；`hard_blocked` 一旦确认会保持到 `reset()`。"""

    detected: bool
    candidate: bool
    hard_blocked: bool
    just_confirmed: bool
    stable_frames: int
    reason: str
    score: float = 0.0
    bbox: tuple[float, float, float, float] | None = None


def hard_block_config_from_mapping(config: dict) -> HardBlockConfig:
    """从 `config/obstacle.yaml` 根映射读取硬堵塞判定参数。"""
    raw = config.get("hard_block", {}) or {}
    detect = config.get("detect", {}) or {}
    left = float(raw.get("corridor_left_ratio", 0.20))
    right = float(raw.get("corridor_right_ratio", 0.80))
    if not 0.0 <= left < right <= 1.0:
        raise ValueError("hard_block corridor ratios must satisfy 0 <= left < right <= 1")
    return HardBlockConfig(
        confirm_frames=max(1, int(raw.get("confirm_frames", 3))),
        min_score=float(raw.get("min_score", detect.get("conf_thres", 0.45))),
        min_box_area_ratio=max(0.0, float(raw.get("min_box_area_ratio", 0.002))),
        min_bottom_ratio=float(raw.get("min_bottom_ratio", 0.35)),
        corridor_left_ratio=left,
        corridor_right_ratio=right,
        min_corridor_overlap=float(raw.get("min_corridor_overlap", 0.30)),
        min_track_iou=float(raw.get("min_track_iou", 0.20)),
    )


def _bbox(item: Detection) -> tuple[float, float, float, float]:
    return item.x1, item.y1, item.x2, item.y2


def _iou(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> float:
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    right_area = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0.0 else 0.0


class HardBlockageJudge:
    """过滤并跟踪当前道路上的障碍，连续确认后锁存整边堵塞。"""

    def __init__(self, config: HardBlockConfig | None = None) -> None:
        self.config = config or HardBlockConfig()
        self.reset()

    def reset(self) -> None:
        """进入新道路时清除上一条道路的确认结果。"""
        self._hard_blocked = False
        self._stable_frames = 0
        self._last_bbox: tuple[float, float, float, float] | None = None

    def update(
        self,
        detections: Sequence[Detection],
        image_shape: tuple[int, ...],
    ) -> ObstacleObservation:
        """消费一帧检测框并返回稳定的硬堵塞观察结果。"""
        if len(image_shape) < 2 or image_shape[0] <= 0 or image_shape[1] <= 0:
            raise ValueError("image_shape must contain positive height and width")
        if self._hard_blocked:
            return ObstacleObservation(
                detected=bool(detections),
                candidate=True,
                hard_blocked=True,
                just_confirmed=False,
                stable_frames=self._stable_frames,
                reason="latched",
                bbox=self._last_bbox,
            )

        height, width = float(image_shape[0]), float(image_shape[1])
        candidate, rejected_reason = self._select_candidate(detections, width, height)
        if candidate is None:
            self._stable_frames = 0
            self._last_bbox = None
            return ObstacleObservation(
                detected=bool(detections),
                candidate=False,
                hard_blocked=False,
                just_confirmed=False,
                stable_frames=0,
                reason=rejected_reason,
            )

        bbox = _bbox(candidate)
        if self._last_bbox is None or _iou(self._last_bbox, bbox) >= self.config.min_track_iou:
            self._stable_frames += 1
        else:
            self._stable_frames = 1
        self._last_bbox = bbox
        just_confirmed = self._stable_frames >= self.config.confirm_frames
        if just_confirmed:
            self._hard_blocked = True
        return ObstacleObservation(
            detected=True,
            candidate=True,
            hard_blocked=self._hard_blocked,
            just_confirmed=just_confirmed,
            stable_frames=self._stable_frames,
            reason="confirmed" if just_confirmed else "tracking",
            score=float(candidate.score),
            bbox=bbox,
        )

    def _select_candidate(
        self,
        detections: Sequence[Detection],
        width: float,
        height: float,
    ) -> tuple[Detection | None, str]:
        cfg = self.config
        corridor_left = cfg.corridor_left_ratio * width
        corridor_right = cfg.corridor_right_ratio * width
        accepted: list[Detection] = []
        rejection = "no_detection"
        for item in detections:
            x1 = max(0.0, min(width, float(item.x1)))
            y1 = max(0.0, min(height, float(item.y1)))
            x2 = max(0.0, min(width, float(item.x2)))
            y2 = max(0.0, min(height, float(item.y2)))
            box_width = max(0.0, x2 - x1)
            box_height = max(0.0, y2 - y1)
            if float(item.score) < cfg.min_score:
                rejection = "low_score"
                continue
            if box_width * box_height / (width * height) < cfg.min_box_area_ratio:
                rejection = "too_small"
                continue
            if y2 / height < cfg.min_bottom_ratio:
                rejection = "too_far"
                continue
            overlap = max(0.0, min(x2, corridor_right) - max(x1, corridor_left))
            if box_width <= 0.0 or overlap / box_width < cfg.min_corridor_overlap:
                rejection = "outside_corridor"
                continue
            accepted.append(item)
        if not accepted:
            return None, rejection
        return max(accepted, key=lambda item: (float(item.score), (item.x2 - item.x1) * (item.y2 - item.y1))), "candidate"

