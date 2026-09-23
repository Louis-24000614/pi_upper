"""中心线帧间平滑，减轻白底白沿下单帧抖边。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point2D = Tuple[float, float]


@dataclass
class CenterlineSmoother:
    """按 Y 邻近匹配后对 X 做 EMA。"""

    alpha: float = 0.45
    y_match_tol_m: float = 0.04
    _prev: Optional[List[Point2D]] = None

    def reset(self) -> None:
        self._prev = None

    def update(self, points: Sequence[Point2D]) -> List[Point2D]:
        """融合当前帧中心线；点数过少时仍返回当前（或空），并更新状态。"""
        cur = [(float(x), float(y)) for x, y in points]
        if not cur:
            self._prev = None
            return []

        if self._prev is None:
            self._prev = list(cur)
            return list(cur)

        alpha = float(self.alpha)
        alpha = min(1.0, max(0.0, alpha))
        out: List[Point2D] = []
        for x, y in cur:
            px = self._nearest_x(y)
            if px is None:
                sx = x
            else:
                sx = alpha * x + (1.0 - alpha) * px
            out.append((sx, y))
        self._prev = list(out)
        return out

    def _nearest_x(self, y: float) -> Optional[float]:
        assert self._prev is not None
        best_x: Optional[float] = None
        best_dy = float("inf")
        for px, py in self._prev:
            dy = abs(py - y)
            if dy < best_dy and dy <= self.y_match_tol_m:
                best_dy = dy
                best_x = px
        return best_x


def temporal_from_mapping(cfg: dict) -> CenterlineSmoother:
    t = cfg.get("temporal", {}) or {}
    return CenterlineSmoother(
        alpha=float(t.get("smooth_alpha", 0.45)),
        y_match_tol_m=float(t.get("y_match_tol_m", 0.04)),
    )
