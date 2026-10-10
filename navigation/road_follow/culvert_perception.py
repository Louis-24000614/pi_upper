"""只投影涵洞入口地面点；使用独立标定识别当前道路上的候选。"""
from dataclasses import dataclass
from collections import deque
import math

import cv2
import numpy as np

from ipm_proto.ipm import BevConfig, CameraExtrinsics, Ipm
from ipm_proto.prior import extract_centerline_with_width_prior, road_prior_from_mapping
from road_follow.backup import near_lane_heading
from road_follow.culvert import CulvertTarget


class CulvertCalibration:
    def __init__(self, mapping):
        self.mapping = mapping
        self.ipm = None
        self.valid = False
        self.size = None
        self.mode = "four_point"
        self.metadata = {"mode": self.mode, "verified": False}
        if not mapping.get("verified", False):
            return
        if mapping.get("origin") != "navigation_camera_ground_projection":
            raise ValueError("涵洞标定原点必须是导航相机光心的地面投影")
        if not mapping.get("ground_contact_verified", False):
            raise ValueError("涵洞框底边地面接触位置尚未核实")
        size = mapping.get("image_size", [])
        if len(size) != 2 or any(not isinstance(v, int) or v <= 0 for v in size):
            raise ValueError("涵洞标定必须记录原始图像宽高")
        image = np.asarray(mapping.get("image_points", []), np.float64)
        ground = np.asarray(mapping.get("ground_points_m", []), np.float64)
        if image.shape != (4, 2) or ground.shape != (4, 2) or not np.isfinite(image).all() or not np.isfinite(ground).all():
            raise ValueError("涵洞标定需要四组有限的像素/地面坐标")
        for points in (image, ground):
            if abs(cv2.contourArea(cv2.convexHull(points.astype(np.float32)))) < 1e-5:
                raise ValueError("涵洞标定点退化")
        if (image < 0).any() or (image[:, 0] >= size[0]).any() or (image[:, 1] >= size[1]).any():
            raise ValueError("标定点超出原始图像")
        self.size = tuple(size)
        self.ipm = Ipm.from_image_ground_points(image, ground,
                                               BevConfig(y_min=.12, y_max=1, x_min=-.5, x_max=.5, m_per_px=.01))
        self.valid = True
        self.metadata = {"mode": self.mode, "verified": True,
                         "origin": mapping["origin"], "image_size": list(self.size),
                         "ground_contact_verified": True}

    @classmethod
    def from_manual_activation(cls, active, nav_config, image_size):
        """复用循迹的已选车辆坐标；保留涵洞框底边地面接触确认。"""
        if active.get("ground_contact_verified") is not True:
            raise ValueError("已选手动标定尚未确认涵洞框底边对应地面入口，不能启用涵洞停车")
        from road_follow.pipeline import make_ipm
        result = cls({})
        result.ipm = make_ipm(nav_config, (image_size[1], image_size[0]))
        result.size, result.valid, result.mode = tuple(image_size), True, "manual_vehicle_ground"
        result.metadata = {"mode": result.mode, "verified": True, "verification": "user_reviewed_check_points",
                           "origin": "navigation_camera_ground_projection", "image_size": list(image_size),
                           "ground_contact_verified": True, "calibration_file": active["calibration_file"],
                           "applied_at": active["applied_at"], "checks": active["checks"]}
        return result

    @classmethod
    def from_camera_parameters(cls, nav_config, image_size):
        """显式试运行：复用导航针孔参数，不声称已完成地面标定。"""
        size = tuple(image_size)
        if len(size) != 2 or any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in size):
            raise ValueError("估算涵洞投影需要有效的原始图像宽高")
        camera = nav_config.get("camera", {}) or {}
        parameters = {}
        for key in ("height_m", "pitch_deg", "fx", "fy", "cx", "cy"):
            value = camera.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"估算涵洞投影缺少有限的相机参数: {key}")
            parameters[key] = float(value)
        if parameters["height_m"] <= 0 or parameters["fx"] <= 0 or parameters["fy"] <= 0:
            raise ValueError("相机高度和焦距必须大于零")
        if not 0 < parameters["pitch_deg"] < 90:
            raise ValueError("估算涵洞投影要求相机倾角在 0–90° 之间")
        if not (0 <= parameters["cx"] < size[0] and 0 <= parameters["cy"] < size[1]):
            raise ValueError("相机主点超出原始图像")
        result = cls({})
        result.size = size
        result.mode = "estimated_camera"
        result.ipm = Ipm.from_extrinsics(CameraExtrinsics(
            height_m=parameters["height_m"], pitch_rad=math.radians(parameters["pitch_deg"]),
            fx=parameters["fx"], fy=parameters["fy"], cx=parameters["cx"], cy=parameters["cy"]),
            BevConfig(y_min=.12, y_max=1, x_min=-.5, x_max=.5, m_per_px=.01))
        if not np.isfinite(result.ipm.H_img_to_bev).all() or abs(np.linalg.det(result.ipm.H_img_to_bev)) < 1e-12:
            raise ValueError("估算涵洞投影矩阵无效")
        result.valid = True
        result.metadata = {"mode": result.mode, "verified": False,
                           "origin": "navigation_camera_ground_projection",
                           "image_size": list(size), "camera": parameters,
                           "ground_contact_verified": False}
        return result

    def check_shape(self, image_shape):
        if not self.valid:
            return False
        return (image_shape[1], image_shape[0]) == self.size

    def road_points(self, mask, nav_config):
        bev = self.ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
        return extract_centerline_with_width_prior(bev, self.ipm.bev, road_prior_from_mapping(nav_config))


@dataclass(frozen=True)
class CulvertObservation:
    reason: str
    target: CulvertTarget | None = None
    stable_frames: int = 0
    bbox: tuple | None = None
    distance_m: float | None = None
    lateral_m: float | None = None
    corrected_distance_m: float | None = None


def lane_x_at(points, y):
    ordered = sorted(points, key=lambda point: point[1])
    for a, b in zip(ordered, ordered[1:]):
        if a[1] <= y <= b[1] and b[1] > a[1] and b[1] - a[1] <= .10:
            ratio = (y - a[1]) / (b[1] - a[1])
            return a[0] + ratio * (b[0] - a[0])
    return None


class CulvertPerception:
    def __init__(self, config, calibration, culvert_class_id, *, min_score=.45, min_area=.002):
        self.config, self.calibration = config, calibration
        self.class_id = culvert_class_id
        self.min_score, self.min_area = min_score, min_area
        self.key = None
        self.last_frame_s = None
        self.last_signature = None
        self.positions = deque(maxlen=config.confirm_frames)

    def reset(self):
        self.positions.clear()
        self.last_frame_s = self.last_signature = None

    def observe(self, detections, *, image_shape, captured_s, now, frame_signature,
                edge, from_node, to_node, points, history, already_done=False):
        cfg = self.config
        key = (edge.id, from_node, to_node)
        if key != self.key:
            self.reset()
            self.key = key
        if already_done:
            return CulvertObservation("already_done")
        if not math.isfinite(now-captured_s) or now < captured_s:
            self.positions.clear()
            return CulvertObservation("stale_frame")
        if self.last_frame_s is not None and captured_s <= self.last_frame_s:
            return CulvertObservation("duplicate_or_reordered_frame")
        if self.last_frame_s is not None and captured_s - self.last_frame_s > cfg.odom_timeout_s:
            self.positions.clear()
        self.last_frame_s = captured_s
        if frame_signature is not None and frame_signature == self.last_signature:
            return CulvertObservation("duplicate_image")
        self.last_signature = frame_signature
        if not self.calibration.check_shape(image_shape):
            self.positions.clear()
            return CulvertObservation("calibration_missing_or_shape_mismatch")
        s0 = history.progress_at(captured_s)
        if s0 is None or not history.fresh(now):
            self.positions.clear()
            return CulvertObservation("unaligned_odom")
        heading = near_lane_heading(points)
        if heading is None or abs(heading) > cfg.max_heading_rad:
            self.positions.clear()
            return CulvertObservation("lane_not_straight")
        height, width = image_shape[:2]
        accepted = []
        reason = "no_culvert"
        for item in detections:
            if item.class_id != self.class_id:
                continue
            box = (item.x1, item.y1, item.x2, item.y2)
            if not all(math.isfinite(v) for v in (*box, item.score)):
                reason = "invalid_box"
                continue
            if item.score < self.min_score or (item.x2-item.x1)*(item.y2-item.y1)/(width*height) < self.min_area:
                reason = "low_score_or_small_box"
                continue
            if not 0 < item.x1 < item.x2 < width-1 or not 0 < item.y1 < item.y2 < height-1:
                reason = "clipped_entry"
                continue
            feet = self.calibration.ipm.image_points_to_ground([(item.x1, item.y2), (item.x2, item.y2)])
            if not np.isfinite(feet).all():
                reason = "invalid_ground_point"
                continue
            x, distance = np.mean(feet, axis=0)
            if not cfg.min_distance_m <= distance <= cfg.max_distance_m:
                reason = "outside_distance_window"
                continue
            road_x = lane_x_at(points, distance)
            if road_x is None:
                reason = "no_road_at_entry"
                continue
            lateral = float(x - road_x)
            if abs(lateral) > cfg.max_lateral_m or not min(feet[:,0]) < road_x < max(feet[:,0]):
                reason = "outside_current_lane"
                continue
            # 原始投影仍用于既有距离窗和道路归属判断，补偿统一作用于沿边位置。
            corrected_distance = float(distance) + cfg.entry_distance_bias_m
            if not math.isfinite(corrected_distance) or corrected_distance <= 0:
                reason = "corrected_entry_not_ahead"
                continue
            entrance = s0 + corrected_distance
            exit_s = entrance + cfg.length_m
            if not 0 < entrance < exit_s < edge.length_m:
                reason = "outside_current_edge"
                continue
            accepted.append((entrance, item.score, box, float(distance), lateral, corrected_distance))
        if not accepted:
            self.positions.clear()
            return CulvertObservation(reason)
        # 最近的合格入口，避免远洞抢先锁定。
        entrance, _, box, distance, lateral, corrected_distance = min(accepted, key=lambda candidate: candidate[0])
        if self.positions and (max(*self.positions, entrance) - min(*self.positions, entrance) > cfg.position_spread_m):
            self.positions.clear()
        self.positions.append(entrance)
        if len(self.positions) < cfg.confirm_frames:
            return CulvertObservation("tracking", stable_frames=len(self.positions), bbox=box,
                                      distance_m=distance, lateral_m=lateral,
                                      corrected_distance_m=corrected_distance)
        entrance = float(np.median(self.positions))
        target_s = entrance + cfg.length_m/2
        offset = target_s if from_node == edge.u else edge.length_m - target_s
        target = CulvertTarget(edge.id, from_node, to_node, entrance, target_s,
                               entrance+cfg.length_m, offset, captured_s)
        return CulvertObservation("confirmed", target, len(self.positions), box, distance, lateral,
                                  corrected_distance)
