"""前视道路 mask 的 IPM（鸟瞰）与中心线提取原型。

无 YOLO / 串口依赖；用合成数据验证几何，算法形状对齐 Stage-1 导航规格。
"""

from .centerline import extract_centerline
from .ipm import BevConfig, Ipm
from .prior import RoadWidthPrior, extract_centerline_with_width_prior
from .temporal import CenterlineSmoother

__all__ = [
    "BevConfig",
    "Ipm",
    "extract_centerline",
    "RoadWidthPrior",
    "extract_centerline_with_width_prior",
    "CenterlineSmoother",
]
