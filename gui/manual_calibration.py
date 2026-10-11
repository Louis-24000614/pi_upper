"""手选四个外角的地面标定；区域坐标和车辆坐标候选独立于导航配置。"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CORNER_ORDER = ["region_top_left", "region_top_right", "region_bottom_right", "region_bottom_left"]
UNITS = {"mm": .001, "cm": .01}


def number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(label + "必须是有限数值")
    if positive and value <= 0:
        raise ValueError(label + "必须大于 0")
    return float(value)


def dimensions(columns, rows, values):
    for value, name in ((columns, "横向格数 N"), (rows, "纵向格数 M")):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(name + "必须是正整数；格数不是内角点数量")
    if not isinstance(values, dict) or values.get("unit") not in UNITS:
        raise ValueError("尺寸单位请选择 mm 或 cm")
    mode, scale = values.get("mode"), UNITS[values["unit"]]
    if mode == "cell":
        if any(values.get(k) is not None for k in ("width", "height")):
            raise ValueError("单格边长和区域总宽高不能同时提供")
        cell = number(values.get("cell_size"), "单格边长", positive=True)
        width, height = columns * cell * scale, rows * cell * scale
        used = {"mode": mode, "unit": values["unit"], "cell_size": cell}
    elif mode == "total":
        if values.get("cell_size") is not None:
            raise ValueError("区域总宽高和单格边长不能同时提供")
        width_input = number(values.get("width"), "区域总宽", positive=True)
        height_input = number(values.get("height"), "区域总高", positive=True)
        width, height = width_input * scale, height_input * scale
        used = {"mode": mode, "unit": values["unit"], "width": width_input, "height": height_input}
    else:
        raise ValueError("请选择单格边长或实测区域总宽高")
    if not all(math.isfinite(v) and v >= 1e-6 for v in (width, height)):
        raise ValueError("实际宽高无效或过小，请检查尺寸与单位")
    return width, height, used


def corners(image_points, image_size):
    try:
        points = np.asarray(image_points, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("角点必须是四组原图像素坐标") from exc
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError("请完整选择四个有限的角点")
    if any(isinstance(value, (bool, np.bool_, str)) for row in image_points for value in row):
        raise ValueError("角点坐标必须是数值，不能使用文本或布尔值")
    if len(image_size) != 2 or any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in image_size):
        raise ValueError("原图尺寸无效")
    width, height = image_size
    if np.any(points < 0) or np.any(points[:, 0] >= width) or np.any(points[:, 1] >= height):
        raise ValueError("角点超出原始图像范围，请重新选择")
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    if lengths.min() < 5:
        raise ValueError("角点过近或区域太小，请选取更大的棋盘格区域")
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    # 原图 Y 向下，左上→右上→右下→左下的凸四边形为正绕向；绝不自动排序。
    if np.any(cross <= 0):
        raise ValueError("四点须按区域自身左上、右上、右下、左下组成凸四边形；请检查顺序与交叉连线")
    area = .5 * abs(np.sum(points[:, 0] * np.roll(points[:, 1], -1) - points[:, 1] * np.roll(points[:, 0], -1)))
    if area < 100 or np.min(cross / (lengths * np.roll(lengths, -1))) < .01:
        raise ValueError("区域太小或四点接近共线，无法可靠标定")
    return points


def homography(image, ground):
    matrix, _ = cv2.findHomography(np.asarray(image, np.float64), np.asarray(ground, np.float64), 0)
    if matrix is None or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("投影矩阵无效，请检查角点与实际尺寸")
    denominator = np.column_stack((image, np.ones(4))) @ matrix[2]
    if np.any(np.abs(denominator) < 1e-12) or not (np.all(denominator > 0) or np.all(denominator < 0)):
        raise ValueError("投影在所选区域内退化，请重新选择四角")
    projected = cv2.perspectiveTransform(np.asarray(image, np.float64).reshape(-1, 1, 2), matrix).reshape(-1, 2)
    if not np.allclose(projected, ground, rtol=1e-5, atol=1e-7):
        raise ValueError("四点拟合误差过大，无法生成有效矩阵")
    return matrix


def navigation_settings(config_path):
    """只读当前导航参数；标定前矩阵明确保留车辆米坐标与 BEV 像素两种单位。"""
    config_path = Path(config_path)
    data = config_path.read_bytes()
    config = yaml.safe_load(data)
    capture = config.get("capture", {})
    size = [capture.get("width"), capture.get("height")]
    if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in size):
        raise ValueError("导航配置中的采集分辨率无效")
    device = capture.get("device")
    if not isinstance(device, str) or not device:
        raise ValueError("导航配置缺少相机设备")
    baseline = {"config_path": str(config_path.resolve()), "config_sha256": hashlib.sha256(data).hexdigest(),
                "loaded_at": datetime.now(timezone.utc).isoformat(),
                "image_size": size, "image_space": "raw_distorted", "method": "navigation_camera_parameters",
                "coordinate_reference": "navigation_camera_ground_projection", "verified": False}
    try:
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from vision.ipm_proto.ipm import BevConfig, CameraExtrinsics, Ipm
        camera = config["camera"]
        cam_values = {k: number(camera.get(k), k, positive=k in ("height_m", "fx", "fy"))
                      for k in ("height_m", "pitch_deg", "fx", "fy", "cx", "cy")}
        bev_values = {k: number(config["bev"].get(k), k) for k in ("x_min", "x_max", "y_min", "y_max", "m_per_px")}
        bev = BevConfig(**bev_values)
        ipm = Ipm.from_extrinsics(CameraExtrinsics(height_m=cam_values["height_m"],
            pitch_rad=math.radians(cam_values["pitch_deg"]), **{k: cam_values[k] for k in ("fx", "fy", "cx", "cy")}), bev)
        to_ground = np.array([[bev.m_per_px, 0, bev.x_min], [0, -bev.m_per_px, bev.y_max], [0, 0, 1.]])
        ground_matrix = to_ground @ ipm.H_img_to_bev
        if not np.isfinite(ground_matrix).all() or np.linalg.matrix_rank(ground_matrix) != 3:
            raise ValueError("当前导航投影矩阵无效")
        baseline.update(camera=cam_values, bev=bev_values, H_img_to_vehicle_ground_m=ground_matrix.tolist(),
                        H_img_to_navigation_bev_px=ipm.H_img_to_bev.tolist(), error=None)
    except (ValueError, KeyError, TypeError, cv2.error) as exc:
        baseline.update(error="无法计算标定前导航 H：" + str(exc))
    return {"device": device, "image_size": size, "baseline": baseline}


def vehicle_candidate(record, values):
    if not isinstance(values, dict) or not isinstance(values.get("axes_aligned", False), bool):
        raise ValueError("车辆方向确认值无效")
    if not values.get("axes_aligned", False):
        return None
    unit = values.get("unit")
    if unit not in UNITS:
        raise ValueError("相机垂足偏移距离单位请选择 mm 或 cm")
    offset = np.array([number(values.get("x"), "中心横向距离"), number(values.get("y"), "中心前向距离")]) * UNITS[unit]
    transform = np.array([[1., 0, offset[0]], [0, 1., offset[1]], [0, 0, 1.]])
    return {"version": 1, "coordinate_reference": "navigation_camera_ground_projection",
            "axes": {"x": "vehicle_right", "y": "vehicle_forward", "unit": "m"},
            "region_center_m": offset.tolist(), "axes_aligned_by_user": True,
            "image_space": "raw_distorted", "image_size": record["image_size"],
            "image_points": record["image_points"],
            "ground_points_m": (np.asarray(record["ground_points_m"]) + offset).tolist(),
            "H_img_to_vehicle_ground_m": (transform @ np.asarray(record["H_img_to_region_ground_m"])).tolist(),
            "created_at": record["created_at"], "camera_source": copy.deepcopy(record["camera_source"]),
            "verified": False, "applied": False}


def solve_calibration(parameters, image_size, source, baseline):
    if not isinstance(parameters, dict):
        raise ValueError("标定参数必须是对象")
    columns, rows = parameters.get("columns"), parameters.get("rows")
    width, height, used = dimensions(columns, rows, parameters.get("dimensions"))
    points = corners(parameters.get("image_points"), image_size)
    ground = np.array([[-width/2, height/2], [width/2, height/2], [width/2, -height/2], [-width/2, -height/2]])
    matrix = homography(points, ground)
    scale = max(width, height) * 1.2 / 720
    preview_size = [max(100, round(width * 1.2 / scale)), max(100, round(height * 1.2 / scale))]
    if min(width, height) / scale < 5:
        raise ValueError("区域长宽比过大，无法生成清晰的等比例俯视预览")
    to_preview = np.array([[1/scale, 0, (preview_size[0]-1)/2], [0, -1/scale, (preview_size[1]-1)/2], [0, 0, 1.]])
    record = {"version": 1, "method": "manual_chessboard_four_outer_corners",
              "created_at": datetime.now(timezone.utc).isoformat(), "image_space": "raw_distorted",
              "camera_source": copy.deepcopy(source), "image_size": list(image_size),
              "grid": {"columns": columns, "rows": rows, "meaning": "selected_rectangle_cell_counts_not_inner_corners"},
              "dimension_input": used, "actual_width_m": width, "actual_height_m": height,
              "corner_order": CORNER_ORDER.copy(), "image_points": points.tolist(), "ground_points_m": ground.tolist(),
              "coordinate_reference": "selected_region_center", "origin": "selected_rectangle_center",
              "axes": {"x": "region_right", "y": "region_top", "unit": "m"},
              "H_img_to_region_ground_m": matrix.tolist(), "H_img_to_preview_px": (to_preview @ matrix).tolist(),
              "preview": {"size": preview_size, "m_per_px": scale}, "baseline": copy.deepcopy(baseline),
              "verified": False, "applied": False,
              "validation_note": "四点拟合成功不代表整个测距范围精度已验证，请用独立距离点检查尺度与畸变。"}
    record["vehicle_candidate"] = vehicle_candidate(record, parameters.get("vehicle", {}))
    return record


def validate_record(record):
    try:
        expected = solve_calibration({"columns": record["grid"]["columns"], "rows": record["grid"]["rows"],
            "dimensions": record["dimension_input"], "image_points": record["image_points"]},
            record["image_size"], record["camera_source"], record["baseline"])
        for key in ("version", "method", "image_space", "coordinate_reference", "origin", "axes", "corner_order", "preview"):
            if record[key] != expected[key]:
                raise ValueError("标定记录的坐标、单位或预览信息已失效")
        for key in ("actual_width_m", "actual_height_m", "ground_points_m", "H_img_to_region_ground_m", "H_img_to_preview_px"):
            if not np.allclose(record[key], expected[key], rtol=1e-8, atol=1e-10):
                raise ValueError("矩阵与当前四点或实际尺寸不匹配，请重新计算")
        if record.get("applied") is not False or record.get("verified") is not False:
            raise ValueError("手动四点标定必须保留未应用、未验收状态")
        candidate = record.get("vehicle_candidate")
        if candidate is not None:
            offset = candidate["region_center_m"]
            expected_candidate = vehicle_candidate(record, {"axes_aligned": True, "unit": "cm", "x": offset[0]*100, "y": offset[1]*100})
            for key in ("coordinate_reference", "axes", "axes_aligned_by_user", "image_space", "verified", "applied"):
                if candidate[key] != expected_candidate[key]:
                    raise ValueError("车辆坐标候选信息不一致")
            for key in ("image_points", "ground_points_m", "H_img_to_vehicle_ground_m"):
                if not np.allclose(candidate[key], expected_candidate[key], rtol=1e-8, atol=1e-10):
                    raise ValueError("车辆坐标候选矩阵已失效")
    except (KeyError, TypeError) as exc:
        raise ValueError("标定记录不完整，请重新计算") from exc


def render_images(frame, record):
    validate_record(record)
    overlay = frame.copy()
    points = np.round(record["image_points"]).astype(np.int32)
    cv2.polylines(overlay, [points], True, (80, 220, 80), 2)
    for index, (x, y) in enumerate(points, 1):
        cv2.circle(overlay, (x, y), 6, (40, 100, 255), -1)
        cv2.putText(overlay, str(index), (x+8, y+18), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 220, 255), 2)
    preview = cv2.warpPerspective(frame, np.asarray(record["H_img_to_preview_px"]), tuple(record["preview"]["size"]))
    center = tuple((np.asarray(record["preview"]["size"]) - 1).astype(int) // 2)
    cv2.arrowedLine(preview, center, (center[0]+40, center[1]), (50, 90, 255), 2)
    cv2.arrowedLine(preview, center, (center[0], center[1]-40), (50, 220, 80), 2)
    cv2.putText(preview, "X", (center[0]+42, center[1]+5), cv2.FONT_HERSHEY_SIMPLEX, .5, (50, 90, 255), 1)
    cv2.putText(preview, "Y", (center[0]+4, center[1]-40), cv2.FONT_HERSHEY_SIMPLEX, .5, (50, 220, 80), 1)
    return overlay, preview
