"""手动地面标定的显式应用记录；导航进程固定使用启动时的同一份快照。"""
from __future__ import annotations

from datetime import datetime, timezone
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import threading
import uuid

import cv2
import numpy as np

from .ipm import Ipm

DIRECTORY = Path(__file__).resolve().parents[2] / "data/calibration/manual"
_lock = threading.RLock()


def _array(value, shape, label):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(label + "无效")
    return array


def validate_candidate(candidate):
    if (candidate.get("coordinate_reference") != "navigation_camera_ground_projection"
            or candidate.get("axes") != {"x": "vehicle_right", "y": "vehicle_forward", "unit": "m"}
            or candidate.get("axes_aligned_by_user") is not True
            or candidate.get("image_space") != "raw_distorted"):
        raise ValueError("只能应用已确认方向的相机地面垂足坐标，不能使用区域坐标或去畸变图像")
    size = candidate.get("image_size")
    if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
        raise ValueError("标定原图尺寸无效")
    image = _array(candidate.get("image_points"), (4, 2), "像素角点")
    ground = _array(candidate.get("ground_points_m"), (4, 2), "车辆地面角点")
    center = _array(candidate.get("region_center_m"), (2,), "区域中心偏移")
    matrix = _array(candidate.get("H_img_to_vehicle_ground_m"), (3, 3), "车辆投影矩阵")
    if np.any(image < 0) or np.any(image[:, 0] >= size[0]) or np.any(image[:, 1] >= size[1]):
        raise ValueError("标定角点超出原图")
    edges = np.roll(image, -1, axis=0) - image
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0]*following[:, 1] - edges[:, 1]*following[:, 0]
    if np.any(cross <= 0) or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("标定角点顺序或矩阵无效")
    width, height = ground[1, 0]-ground[0, 0], ground[0, 1]-ground[3, 1]
    expected = np.array([[-width/2, height/2], [width/2, height/2],
                         [width/2, -height/2], [-width/2, -height/2]]) + center
    if width <= 0 or height <= 0 or not np.allclose(expected, ground, atol=1e-8, rtol=1e-8):
        raise ValueError("车辆地面角点与区域偏移不一致")
    denominators = np.column_stack((image, np.ones(4))) @ matrix[2]
    if np.any(np.abs(denominators) < 1e-12) or not (np.all(denominators > 0) or np.all(denominators < 0)):
        raise ValueError("标定矩阵在所选区域内退化")
    if not np.allclose(project(matrix, image), ground, atol=1e-7, rtol=1e-5):
        raise ValueError("车辆矩阵与四点不一致")
    return matrix


def project(matrix, points):
    points = np.asarray(points, np.float64).reshape(-1, 2)
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ np.asarray(matrix).T
    if not np.isfinite(homogeneous).all() or np.any(np.abs(homogeneous[:, 2]) < 1e-10):
        raise ValueError("检查点接近投影地平线，无法测距")
    return homogeneous[:, :2] / homogeneous[:, 2, None]


def validate_checks(candidate, checks):
    matrix = validate_candidate(candidate)
    if not isinstance(checks, list) or not checks:
        raise ValueError("请至少记录一个未参与四角拟合的实测检查点，并核对误差")
    for check in checks:
        point = _array(check.get("image_point"), (2,), "检查像素点")
        size = candidate["image_size"]
        if np.any(point < 0) or point[0] >= size[0] or point[1] >= size[1]:
            raise ValueError("检查点超出原图")
        if np.min(np.linalg.norm(np.asarray(candidate["image_points"])-point, axis=1)) < 5:
            raise ValueError("检查点不能重复使用参与拟合的四个角点")
        measured = _array(check.get("measured_ground_m"), (2,), "检查点实测坐标")
        predicted = project(matrix, [point])[0]
        if (check.get("coordinate_reference") != "navigation_camera_ground_projection"
                or not np.allclose(check.get("predicted_ground_m"), predicted, atol=1e-8)
                or not np.allclose(check.get("error_xy_m"), predicted-measured, atol=1e-8)
                or not np.isclose(check.get("error_m"), np.linalg.norm(predicted-measured), atol=1e-8)):
            raise ValueError("检查点记录与当前矩阵不一致")


def _child(directory, name):
    path = (directory / name).resolve()
    if not path.is_relative_to(directory.resolve()) or path == directory.resolve():
        raise ValueError("标定文件路径超出手动标定目录")
    return path


def validate_activation(record, directory):
    try:
        if (record["version"] != 1 or record["measurement_reviewed"] is not True
                or record["image_source_confirmed"] is not True
                or type(record["ground_contact_verified"]) is not bool):
            raise ValueError("应用确认记录不完整")
        candidate = record["candidate"]
        validate_checks(candidate, record["checks"])
        source = candidate["camera_source"]
        device = source.get("device") if source.get("kind") == "navigation_camera" else source.get("navigation_device")
        if not device or device != record["device"]:
            raise ValueError("标定图像的导航相机来源不一致")
        for key in ("calibration", "candidate"):
            data = _child(directory, record[key+"_file"]).read_bytes()
            if hashlib.sha256(data).hexdigest() != record[key+"_sha256"]:
                raise ValueError("已选标定文件发生变化，拒绝使用")
            if key == "candidate" and json.loads(data) != candidate:
                raise ValueError("已选车辆矩阵与保存文件不一致")
        previous = record.get("previous")
        if previous is not None:
            _child(directory, previous)
    except (KeyError, TypeError, OSError, json.JSONDecodeError) as exc:
        raise ValueError("已选手动标定损坏或文件缺失，请在网页恢复旧标定："+str(exc)) from exc
    return record


def read_activation(directory=None):
    directory = Path(directory) if directory is not None else DIRECTORY
    path = directory / "active.json"
    if not path.exists():
        return None
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
        return validate_activation(record, directory)
    except (OSError, json.JSONDecodeError, TypeError) as exc:
        raise ValueError("无法读取已选手动标定："+str(exc)) from exc


@lru_cache(maxsize=1)
def runtime_activation():
    # 包括“未选标定”也固定为启动快照；网页应用/恢复只影响下一次启动。
    return read_activation()


def activation_ipm(record, bev, image_size, device):
    candidate = record["candidate"]
    if list(image_size) != candidate["image_size"] or device != record["device"]:
        raise ValueError("已选手动标定与导航相机设备或原图分辨率不同，拒绝使用")
    matrix = validate_candidate(candidate)
    to_bev = np.array([[1/bev.m_per_px, 0, -bev.x_min/bev.m_per_px],
                       [0, -1/bev.m_per_px, bev.y_max/bev.m_per_px], [0, 0, 1.]])
    return Ipm(to_bev @ matrix, bev)


def _atomic_write(path, data):
    pending = path.with_name(path.name+"."+uuid.uuid4().hex+".pending")
    try:
        with pending.open("xb") as output:
            output.write(data); output.flush(); os.fsync(output.fileno())
        os.replace(pending, path)
    finally:
        if pending.exists():
            pending.unlink()


def _backup(directory, data):
    folder = directory / "activation_history"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")+"_"+uuid.uuid4().hex[:8]+".json")
    with path.open("xb") as output:
        output.write(data)
    return path.relative_to(directory).as_posix()


def apply_activation(directory, calibration_path, candidate, checks, *, device,
                     measurement_reviewed, image_source_confirmed, ground_contact_verified):
    directory = Path(directory).resolve()
    with _lock:
        if measurement_reviewed is not True or image_source_confirmed is not True or type(ground_contact_verified) is not bool:
            raise ValueError("请明确确认检查误差及原图来源")
        path = Path(calibration_path).resolve()
        candidate_path = path.with_name("vehicle_candidate.json")
        if not path.is_relative_to(directory):
            raise ValueError("不能应用手动标定目录以外的结果")
        record = {"version": 1, "applied_at": datetime.now(timezone.utc).isoformat(), "device": device,
                  "candidate": candidate, "checks": checks, "measurement_reviewed": True,
                  "image_source_confirmed": True, "ground_contact_verified": ground_contact_verified,
                  "previous": None}
        for key, file in (("calibration", path), ("candidate", candidate_path)):
            record[key+"_file"] = file.relative_to(directory).as_posix()
            record[key+"_sha256"] = hashlib.sha256(file.read_bytes()).hexdigest()
        validate_activation(record, directory)
        active = directory / "active.json"
        if active.exists():
            read_activation(directory)
            record["previous"] = _backup(directory, active.read_bytes())
        _atomic_write(active, (json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False)+"\n").encode())
        return record


def restore_activation(directory, expected_applied_at, expected_sha256=None):
    directory = Path(directory).resolve()
    with _lock:
        active = directory / "active.json"
        if not active.exists():
            raise ValueError("已选标定发生变化，请刷新后再恢复")
        raw = active.read_bytes()
        if expected_sha256 is not None:
            if hashlib.sha256(raw).hexdigest() != expected_sha256:
                raise ValueError("已选标定发生变化，请刷新后再恢复")
            # 允许从文件缺失/损坏的当前选择恢复；备份原始记录，不伪造新标定。
            try:
                current = json.loads(raw)
                if not isinstance(current, dict): current = {}
            except (json.JSONDecodeError, UnicodeDecodeError):
                current = {}
        else:
            current = read_activation(directory)
            if not current or current["applied_at"] != expected_applied_at:
                raise ValueError("已选标定发生变化，请刷新后再恢复")
        previous = current.get("previous")
        if previous:
            data = _child(directory, previous).read_bytes()
            restored = validate_activation(json.loads(data), directory)
        else:
            data, restored = None, None
        _backup(directory, raw)
        if data is None:
            active.unlink()
        else:
            _atomic_write(active, data)
        return restored
