"""前视道路 mask → 鸟瞰中心线 → 速度指令。"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ipm_proto.centerline import extract_centerline
from ipm_proto.ipm import BevConfig, CameraExtrinsics, Ipm
from ipm_proto.prior import extract_centerline_with_width_prior, road_prior_from_mapping
from ipm_proto.temporal import CenterlineSmoother

from road_follow.backup import near_lane_heading, near_lane_x
from road_follow.control import FollowConfig, VelocityCommand, command_from_centerline, follow_config_from_mapping


@dataclass(frozen=True)
class FollowDiagnostics:
    """一帧从原始 mask 到控制指令的关键中间量。"""

    mask_road_pixels: int
    bev_road_pixels: int
    prior_points: int
    raw_points: int
    output_points: int
    used_fallback: bool
    y_min_m: float | None
    y_max_m: float | None
    lookahead_covered: bool
    near_x_m: float | None
    lane_heading_rad: float | None = None
    centerline_points: tuple = ()


def make_ipm(cfg: dict, image_shape: tuple[int, ...]) -> Ipm:
    """按 ``nav_camera.yaml`` 的外参做鸟瞰。内参缺省时用约 63.3° 水平视场。"""
    b = cfg.get("bev", {}) or {}
    bev = BevConfig(
        y_min=float(b.get("y_min", 0.20)),
        y_max=float(b.get("y_max", 1.00)),
        x_min=float(b.get("x_min", -0.5)),
        x_max=float(b.get("x_max", 0.5)),
        m_per_px=float(b.get("m_per_px", 0.01)),
    )
    cam_cfg = cfg.get("camera", {}) or {}
    height, width = image_shape[:2]
    fx = float(cam_cfg.get("fx", 0.5 * width / np.tan(np.deg2rad(63.3) / 2.0)))
    fy = float(cam_cfg.get("fy", fx))
    cam = CameraExtrinsics(
        height_m=float(cam_cfg.get("height_m", 0.16)),
        pitch_rad=float(np.deg2rad(cam_cfg.get("pitch_deg", 28.0))),
        fx=fx,
        fy=fy,
        cx=float(cam_cfg.get("cx", width / 2.0)),
        cy=float(cam_cfg.get("cy", height / 2.0)),
    )
    return Ipm.from_extrinsics(cam, bev)


def command_from_mask(
    mask: np.ndarray,
    cfg: dict,
    smoother: CenterlineSmoother,
    follow: FollowConfig | None = None,
) -> VelocityCommand:
    """单帧道路 mask 变成速度指令。``smoother`` 要跨帧复用。"""
    command, _ = command_from_mask_with_diagnostics(mask, cfg, smoother, follow)
    return command


def command_from_mask_with_diagnostics(
    mask: np.ndarray,
    cfg: dict,
    smoother: CenterlineSmoother,
    follow: FollowConfig | None = None,
    *,
    projection: tuple[Ipm, np.ndarray] | None = None,
    lazy_raw: bool = False,
) -> tuple[VelocityCommand, FollowDiagnostics]:
    """生成速度指令，同时返回可解释停车原因的几何诊断。"""
    if projection is None:
        ipm = make_ipm(cfg, mask.shape)
        bev_mask = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
    else:
        # 同一帧的中心线和路口共享投影；保持最近邻插值和诊断点数语义。
        ipm, bev_mask = projection
    mask_gray = mask[:, :, 0] if mask.ndim == 3 else mask
    gray = bev_mask[:, :, 0] if bev_mask.ndim == 3 else bev_mask
    mask_road_pixels = _road_pixels(mask_gray)
    road_pixels = _road_pixels(gray)
    gains = follow if follow is not None else follow_config_from_mapping(cfg)
    prior_points = extract_centerline_with_width_prior(
        bev_mask, ipm.bev, road_prior_from_mapping(cfg)
    )
    used_fallback = len(prior_points) < gains.min_points
    if lazy_raw and not used_fallback:
        # 普通中心线没有参与控制时，诊断只需要“合法行数”，无须每行计算中位数。
        # 保持 raw_points 的原有点数语义，而不是把未计算错误标成 0。
        road = gray.astype(bool) if gray.dtype == np.bool_ or gray.max() <= 1 else gray > 127
        raw_count = int(np.count_nonzero(np.count_nonzero(road, axis=1) >= 3))
        raw_points = []
    else:
        raw_points = extract_centerline(bev_mask, ipm.bev)
        raw_count = len(raw_points)
    points = raw_points if used_fallback else prior_points
    points = smoother.update(points)
    ys = [float(y) for _, y in points]
    lookahead_covered = _covers_lookahead(ys, gains.lookahead_m)
    command = command_from_centerline(points, road_pixels, gains)
    diagnostics = FollowDiagnostics(
        mask_road_pixels=mask_road_pixels,
        bev_road_pixels=road_pixels,
        prior_points=len(prior_points),
        raw_points=raw_count,
        output_points=len(points),
        used_fallback=used_fallback,
        y_min_m=min(ys) if ys else None,
        y_max_m=max(ys) if ys else None,
        lookahead_covered=lookahead_covered,
        near_x_m=near_lane_x(points),
        lane_heading_rad=near_lane_heading(points),
        centerline_points=tuple(points),
    )
    return command, diagnostics


class BevProjector:
    """每个有序消费者持有一份 IPM；相机/BEV 参数或分辨率改变就失效。"""

    def __init__(self) -> None:
        self._key = None
        self._ipm: Ipm | None = None

    def project(self, mask: np.ndarray, cfg: dict) -> tuple[Ipm, np.ndarray]:
        # 只比较影响投影的参数，速度/状态机变化不会重建矩阵。
        key = (mask.shape[:2], tuple(sorted((cfg.get("camera") or {}).items())),
               tuple(sorted((cfg.get("bev") or {}).items())))
        if key != self._key:
            self._ipm = make_ipm(cfg, mask.shape)
            self._key = key
        assert self._ipm is not None
        return self._ipm, self._ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)


def _road_pixels(gray: np.ndarray) -> int:
    if gray.dtype == np.bool_:
        return int(np.count_nonzero(gray))
    return int(np.count_nonzero(gray > 127))


def _covers_lookahead(ys: list[float], lookahead_m: float) -> bool:
    ordered = sorted(ys)
    return any(y0 <= lookahead_m <= y1 for y0, y1 in zip(ordered, ordered[1:]))
