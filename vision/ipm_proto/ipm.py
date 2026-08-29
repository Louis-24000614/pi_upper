"""逆透视（IPM）：前视像素 ↔ 地面鸟瞰栅格。

地面坐标系：X 向右、Y 向前（米），Z 向上。BEV 图中默认下方为近端（小 Y）、
上方为远端（大 Y），列从左到右对应 X 增大。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class BevConfig:
    """鸟瞰窗与分辨率。

    Attributes:
        y_min: 车前最近距离（米）。
        y_max: 车前最远距离（米）。
        x_min: 左侧边界（米，通常为负）。
        x_max: 右侧边界（米）。
        m_per_px: 每像素对应的米数（各向同性）。
    """

    y_min: float = 0.3
    y_max: float = 1.5
    x_min: float = -0.5
    x_max: float = 0.5
    m_per_px: float = 0.01

    def __post_init__(self) -> None:
        if self.y_max <= self.y_min:
            raise ValueError("y_max must be > y_min")
        if self.x_max <= self.x_min:
            raise ValueError("x_max must be > x_min")
        if self.m_per_px <= 0:
            raise ValueError("m_per_px must be > 0")

    @property
    def width_px(self) -> int:
        return int(round((self.x_max - self.x_min) / self.m_per_px))

    @property
    def height_px(self) -> int:
        return int(round((self.y_max - self.y_min) / self.m_per_px))

    def size(self) -> tuple[int, int]:
        """返回 (width, height) 像素。"""
        return self.width_px, self.height_px

    def ground_to_bev_px(self, x_m: float, y_m: float) -> tuple[float, float]:
        """地面 (X,Y) 米 → BEV 像素 (u,v)，v=0 为远端。"""
        u = (x_m - self.x_min) / self.m_per_px
        v = (self.y_max - y_m) / self.m_per_px
        return u, v

    def bev_px_to_ground(self, u: float, v: float) -> tuple[float, float]:
        """BEV 像素 (u,v) → 地面 (X,Y) 米。"""
        x_m = self.x_min + u * self.m_per_px
        y_m = self.y_max - v * self.m_per_px
        return x_m, y_m

    def corner_grounds(self) -> np.ndarray:
        """BEV 四角地面坐标，顺序：近左、近右、远右、远左。shape (4,2)。"""
        return np.array(
            [
                [self.x_min, self.y_min],
                [self.x_max, self.y_min],
                [self.x_max, self.y_max],
                [self.x_min, self.y_max],
            ],
            dtype=np.float64,
        )

    def corner_bev_px(self) -> np.ndarray:
        """与 corner_grounds 对应的 BEV 像素角点。"""
        w, h = self.width_px, self.height_px
        return np.array(
            [
                [0, h - 1],
                [w - 1, h - 1],
                [w - 1, 0],
                [0, 0],
            ],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class CameraExtrinsics:
    """针孔相机相对地面的简化外参。

    相机光心在车辆原点正上方 height_m 处；光轴在水平面内指向 +Y，再绕相机 X
    轴低头 pitch_rad（弧度，正值=向下看）。偏航默认 0。
    """

    height_m: float
    pitch_rad: float
    fx: float
    fy: float
    cx: float
    cy: float

    def project_ground(self, points_xy: np.ndarray) -> np.ndarray:
        """将地面点 (N,2) 的 (X,Y) 投影到图像像素 (N,2)。

        Raises:
            ValueError: 若有点落在相机后方（Z_cam<=0）。
        """
        pts = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        # 车辆坐标：相机在 (0,0,h)，点在地面 z=0
        x = pts[:, 0]
        y = pts[:, 1]
        z = np.zeros_like(x)
        # 平移到相机中心
        x_c = x
        y_c = y
        z_c = z - self.height_m
        # 车辆(x右,y前,z上) → OpenCV 相机(x右,y下,z前)，再低头 pitch
        # 先转到「光轴沿 +Y」的中间系：X=x, Y=-z, Z=y
        x1 = x_c
        y1 = -z_c
        z1 = y_c
        c = np.cos(self.pitch_rad)
        s = np.sin(self.pitch_rad)
        # 绕相机 X 轴旋转 pitch（向下为正）：Y' = c*Y - s*Z? 低头看地面：
        # 正 pitch 把原来的前向抬到光轴下方 → 地面进入视野
        # R_x(+pitch): Y' = cos*Y - sin*Z, Z' = sin*Y + cos*Z  （Y下 Z前）
        y2 = c * y1 - s * z1
        z2 = s * y1 + c * z1
        if np.any(z2 <= 1e-6):
            raise ValueError("ground point projects behind or on the camera")
        u = self.fx * (x1 / z2) + self.cx
        v = self.fy * (y2 / z2) + self.cy
        return np.stack([u, v], axis=1)


class Ipm:
    """前视 ↔ BEV 的单应封装。"""

    def __init__(self, H_img_to_bev: np.ndarray, bev: BevConfig) -> None:
        self.bev = bev
        self.H_img_to_bev = np.asarray(H_img_to_bev, dtype=np.float64).reshape(3, 3)
        inv = cv2.invert(self.H_img_to_bev)[1]
        self.H_bev_to_img = inv

    @classmethod
    def from_image_ground_points(
        cls,
        image_pts: Sequence[Sequence[float]],
        ground_pts: Sequence[Sequence[float]],
        bev: BevConfig,
    ) -> "Ipm":
        """由 4 组「图像点 ↔ 地面(X,Y)米」估计单应。"""
        img = np.asarray(image_pts, dtype=np.float64).reshape(4, 2)
        gnd = np.asarray(ground_pts, dtype=np.float64).reshape(4, 2)
        bev_pts = np.array(
            [bev.ground_to_bev_px(float(x), float(y)) for x, y in gnd],
            dtype=np.float64,
        )
        H = cv2.getPerspectiveTransform(img.astype(np.float32), bev_pts.astype(np.float32))
        return cls(H, bev)

    @classmethod
    def from_extrinsics(cls, cam: CameraExtrinsics, bev: BevConfig) -> "Ipm":
        """用俯仰/高度把 BEV 四角投到图像，再求单应。"""
        gnd = bev.corner_grounds()
        img = cam.project_ground(gnd)
        return cls.from_image_ground_points(img, gnd, bev)

    def warp_to_bev(self, image: np.ndarray, flags: int = cv2.INTER_LINEAR) -> np.ndarray:
        """将前视图或 mask warp 到 BEV。mask 建议用 INTER_NEAREST。"""
        w, h = self.bev.size()
        return cv2.warpPerspective(
            image,
            self.H_img_to_bev,
            (w, h),
            flags=flags,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def warp_to_image(
        self,
        bev_image: np.ndarray,
        out_size: tuple[int, int],
        flags: int = cv2.INTER_LINEAR,
    ) -> np.ndarray:
        """BEV → 前视（用于合成数据）。out_size 为 (width, height)。"""
        return cv2.warpPerspective(
            bev_image,
            self.H_bev_to_img,
            out_size,
            flags=flags,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def image_points_to_ground(self, pts_uv: Iterable[Sequence[float]]) -> np.ndarray:
        """图像点 → 地面 (X,Y) 米。返回 (N,2)。"""
        pts = np.asarray(list(pts_uv), dtype=np.float64).reshape(-1, 1, 2)
        bev_px = cv2.perspectiveTransform(pts, self.H_img_to_bev).reshape(-1, 2)
        out = np.array(
            [self.bev.bev_px_to_ground(float(u), float(v)) for u, v in bev_px],
            dtype=np.float64,
        )
        return out
