"""前视道路 mask → 鸟瞰中心线 → 速度指令。"""

from __future__ import annotations

import cv2
import numpy as np

from ipm_proto.centerline import extract_centerline
from ipm_proto.ipm import BevConfig, CameraExtrinsics, Ipm
from ipm_proto.prior import extract_centerline_with_width_prior, road_prior_from_mapping
from ipm_proto.temporal import CenterlineSmoother

from road_follow.control import FollowConfig, VelocityCommand, command_from_centerline, follow_config_from_mapping


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
    ipm = make_ipm(cfg, mask.shape)
    bev_mask = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
    gray = bev_mask[:, :, 0] if bev_mask.ndim == 3 else bev_mask
    if gray.dtype == np.bool_:
        road_pixels = int(np.count_nonzero(gray))
    else:
        road_pixels = int(np.count_nonzero(gray > 127))
    gains = follow if follow is not None else follow_config_from_mapping(cfg)
    points = extract_centerline_with_width_prior(
        bev_mask, ipm.bev, road_prior_from_mapping(cfg)
    )
    # 投影后的路比 0.35 m 窄时，路宽先验会把整行丢掉。这时改用原始中心线，避免有路却停车。
    if len(points) < gains.min_points:
        points = extract_centerline(bev_mask, ipm.bev)
    points = smoother.update(points)
    return command_from_centerline(points, road_pixels, gains)
