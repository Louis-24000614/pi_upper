"""合成透视道路图 / mask，供无实拍时验证 IPM 与中心线。"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .ipm import BevConfig, CameraExtrinsics, Ipm


@dataclass(frozen=True)
class SynthScene:
    """一次合成的结果。"""

    image_bgr: np.ndarray
    mask: np.ndarray
    ipm: Ipm
    road_width_m: float


def make_straight_road_bev(
    bev: BevConfig,
    road_width_m: float = 0.8,
    *,
    road_value: int = 255,
) -> np.ndarray:
    """在 BEV 上画居中直道 mask。"""
    h, w = bev.height_px, bev.width_px
    mask = np.zeros((h, w), dtype=np.uint8)
    half = road_width_m / 2.0
    for v in range(h):
        # 整列按 X 范围填充
        u0, _ = bev.ground_to_bev_px(-half, bev.y_min)
        u1, _ = bev.ground_to_bev_px(half, bev.y_min)
        left = max(0, int(np.floor(min(u0, u1))))
        right = min(w, int(np.ceil(max(u0, u1))))
        mask[v, left:right] = road_value
    return mask


def default_camera(image_size: tuple[int, int] = (1280, 720)) -> CameraExtrinsics:
    """与示例配置一致的简化针孔参数。"""
    w, h = image_size
    # 近似 63° 水平 FOV 的焦距
    fx = fy = 0.5 * w / np.tan(np.deg2rad(63.3) / 2.0)
    return CameraExtrinsics(
        height_m=0.25,
        pitch_rad=np.deg2rad(28.0),
        fx=float(fx),
        fy=float(fy),
        cx=w / 2.0,
        cy=h / 2.0,
    )


def _road_polygon_image(
    cam: CameraExtrinsics,
    *,
    road_width_m: float,
    y_min: float,
    y_max: float,
    image_size: tuple[int, int],
    samples: int = 40,
) -> np.ndarray:
    """将地面矩形道路的左右边投影到图像，填充得到 mask。"""
    half = road_width_m / 2.0
    ys = np.linspace(y_min, y_max, samples)
    left = np.stack([-half * np.ones_like(ys), ys], axis=1)
    right = np.stack([half * np.ones_like(ys), ys], axis=1)
    # 多边形：近左→远左→远右→近右（沿边界走一圈）
    ring = np.concatenate([left, right[::-1]], axis=0)
    pix = cam.project_ground(ring)
    w, h = image_size
    mask = np.zeros((h, w), dtype=np.uint8)
    poly = np.round(pix).astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(mask, [poly], 255)
    return mask


def synthesize_straight_road(
    *,
    image_size: tuple[int, int] = (1280, 720),
    bev: BevConfig | None = None,
    road_width_m: float = 0.8,
    cam: CameraExtrinsics | None = None,
) -> SynthScene:
    """地面直道投影到前视，再配上同外参的 Ipm（避免 BEV↔图双重 warp 吃边）。"""
    if bev is None:
        bev = BevConfig()
    if cam is None:
        cam = default_camera(image_size)

    ipm = Ipm.from_extrinsics(cam, bev)
    # 略扩展 Y，避免窗边界裁切
    mask = _road_polygon_image(
        cam,
        road_width_m=road_width_m,
        y_min=max(0.05, bev.y_min - 0.1),
        y_max=bev.y_max + 0.2,
        image_size=image_size,
    )
    w, h = image_size
    image_bgr = np.zeros((h, w, 3), dtype=np.uint8)
    image_bgr[:] = (40, 40, 40)
    image_bgr[mask > 0] = (60, 180, 60)
    return SynthScene(
        image_bgr=image_bgr,
        mask=mask,
        ipm=ipm,
        road_width_m=road_width_m,
    )


def measure_bev_road_width_m(
    bev_mask: np.ndarray,
    bev: BevConfig,
    *,
    at_y_m: float | None = None,
    row_from_bottom: int | None = None,
) -> float:
    """在指定地面 Y（或距底行列）测量道路像素宽度（米）。

    近端道路常超出前视水平视场，warp 后会被裁窄；默认在窗口中部 ``(y_min+y_max)/2`` 测量。
    """
    if at_y_m is None and row_from_bottom is None:
        at_y_m = 0.5 * (bev.y_min + bev.y_max)
    if at_y_m is not None:
        _, v = bev.ground_to_bev_px(0.0, at_y_m)
        v = int(np.clip(round(v), 0, bev.height_px - 1))
    else:
        assert row_from_bottom is not None
        v = bev.height_px - 1 - row_from_bottom
        v = int(np.clip(v, 0, bev.height_px - 1))
    xs = np.flatnonzero(bev_mask[v] > 0)
    if xs.size < 2:
        return 0.0
    return float((xs.max() - xs.min() + 1) * bev.m_per_px)
