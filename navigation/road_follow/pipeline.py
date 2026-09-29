"""前视道路 mask → 鸟瞰中心线 → 速度指令。"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ipm_proto.centerline import extract_centerline
from ipm_proto.ipm import BevConfig, CameraExtrinsics, Ipm
from ipm_proto.prior import extract_centerline_with_width_prior, road_prior_from_mapping
from ipm_proto.temporal import CenterlineSmoother

from road_follow.backup import near_lane_x
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
) -> tuple[VelocityCommand, FollowDiagnostics]:
    """生成速度指令，同时返回可解释停车原因的几何诊断。"""
    ipm = make_ipm(cfg, mask.shape)
    bev_mask = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
    mask_gray = mask[:, :, 0] if mask.ndim == 3 else mask
    gray = bev_mask[:, :, 0] if bev_mask.ndim == 3 else bev_mask
    mask_road_pixels = _road_pixels(mask_gray)
    road_pixels = _road_pixels(gray)
    gains = follow if follow is not None else follow_config_from_mapping(cfg)
    prior_points = extract_centerline_with_width_prior(
        bev_mask, ipm.bev, road_prior_from_mapping(cfg)
    )
    raw_points = extract_centerline(bev_mask, ipm.bev)
    used_fallback = len(prior_points) < gains.min_points
    points = raw_points if used_fallback else prior_points
    points = smoother.update(points)
    ys = [float(y) for _, y in points]
    lookahead_covered = _covers_lookahead(ys, gains.lookahead_m)
    command = command_from_centerline(points, road_pixels, gains)
    diagnostics = FollowDiagnostics(
        mask_road_pixels=mask_road_pixels,
        bev_road_pixels=road_pixels,
        prior_points=len(prior_points),
        raw_points=len(raw_points),
        output_points=len(points),
        used_fallback=used_fallback,
        y_min_m=min(ys) if ys else None,
        y_max_m=max(ys) if ys else None,
        lookahead_covered=lookahead_covered,
        near_x_m=near_lane_x(points),
    )
    return command, diagnostics


def _road_pixels(gray: np.ndarray) -> int:
    if gray.dtype == np.bool_:
        return int(np.count_nonzero(gray))
    return int(np.count_nonzero(gray > 127))


def _covers_lookahead(ys: list[float], lookahead_m: float) -> bool:
    ordered = sorted(ys)
    return any(y0 <= lookahead_m <= y1 for y0, y1 in zip(ordered, ordered[1:]))
