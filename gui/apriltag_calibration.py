"""测试2：单个地面 AprilTag 四点单应标定，不接管导航投影。"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH = ROOT / "config" / "bev_calibration.json"
TAG_SIZE_M = 0.134
CORNER_ORDER = ["tag_top_left", "tag_top_right", "tag_bottom_right", "tag_bottom_left"]


# 独立标定预览：采用已核对的当前导航窗口大小与尺度，原点移到标签中心。
# 不读取或修改导航配置；这些值只决定标定预览的栅格。
PREVIEW_BEV = {"x_min": -0.5, "x_max": 0.5, "y_min": -0.4, "y_max": 0.4, "m_per_px": 0.01}


def bev_size(bev: dict) -> tuple[int, int]:
    values = np.array([bev[k] for k in ("x_min", "x_max", "y_min", "y_max", "m_per_px")], dtype=float)
    if not np.isfinite(values).all() or values[4] <= 0:
        raise ValueError("BEV 参数无效")
    width, height = values[1] - values[0], values[3] - values[2]
    if width <= 0 or height <= 0:
        raise ValueError("BEV 范围无效")
    size = (round(width / values[4]), round(height / values[4]))
    if min(size) < 1 or max(size) > 4096:
        raise ValueError("BEV 输出尺寸无效")
    return size


def detect_tag(frame: np.ndarray) -> np.ndarray:
    """只接受 tag36h11 ID 0，角点按标签解码方向排列。"""
    if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
        raise ValueError("需要原始 uint8 BGR 三通道图像")
    aruco = getattr(cv2, "aruco", None)
    if aruco is None or not hasattr(aruco, "DICT_APRILTAG_36h11"):
        raise ValueError("当前 OpenCV 不支持 AprilTag 检测")
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    parameters = aruco.DetectorParameters()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = aruco.ArucoDetector(dictionary, parameters).detectMarkers(
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    )
    matches = [] if ids is None else [i for i, tag_id in enumerate(ids.reshape(-1)) if int(tag_id) == 0]
    if len(matches) != 1:
        raise ValueError("需完整看到唯一一个 tag36h11 ID 0；请调整标签位置、清晰度和光照")
    return np.asarray(corners[matches[0]], dtype=np.float64).reshape(4, 2)


def solve_calibration(image_points, image_size, bev: dict, source: str) -> dict:
    """四角对应标签中心坐标；原始图像未去畸变。"""
    points = np.asarray(image_points, dtype=np.float64)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError("需要四个有限的角点")
    width, height = image_size
    if width <= 0 or height <= 0 or np.any(points < 0) or np.any(points[:, 0] >= width) or np.any(points[:, 1] >= height):
        raise ValueError("角点超出原始图像范围")
    contour = points.astype(np.float32).reshape(-1, 1, 2)
    if not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour)) < 100:
        raise ValueError("四角退化或标签太小，无法可靠标定")
    output_size = bev_size(bev)
    half = TAG_SIZE_M / 2
    ground = np.array([[-half, half], [half, half], [half, -half], [-half, -half]], dtype=np.float64)
    target = np.column_stack(((ground[:, 0] - bev["x_min"]) / bev["m_per_px"],
                              (bev["y_max"] - ground[:, 1]) / bev["m_per_px"]))
    matrix = cv2.getPerspectiveTransform(points.astype(np.float32), target.astype(np.float32))
    if not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("单应矩阵无效")
    return {
        "version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "method": "apriltag_four_corners",
        "coordinate_reference": "tag_center",
        "axes": {"x": "tag_right", "y": "tag_top", "unit": "m"},
        "image_space": "raw_distorted",
        "camera_source": source,
        "image_size": [int(width), int(height)],
        "tag": {"family": "tag36h11", "id": 0, "outer_black_size_m": TAG_SIZE_M},
        "corner_order": CORNER_ORDER.copy(),
        "image_points": points.tolist(),
        "ground_points": ground.tolist(),
        "bev": dict(bev),
        "bev_size": list(output_size),
        "H_img_to_bev": matrix.tolist(),
    }


def validate_calibration(record: dict) -> np.ndarray:
    """读取时校验矩阵与四点、尺度、图像尺寸是否一致。"""
    if not isinstance(record, dict) or record.get("version") != 1:
        raise ValueError("不支持的标定记录格式")
    if record.get("coordinate_reference") != "tag_center" or record.get("image_space") != "raw_distorted":
        raise ValueError("标定坐标基准或图像空间不匹配")
    if record.get("axes") != {"x": "tag_right", "y": "tag_top", "unit": "m"}:
        raise ValueError("标定坐标轴或单位不匹配")
    if record.get("tag") != {"family": "tag36h11", "id": 0, "outer_black_size_m": TAG_SIZE_M}:
        raise ValueError("标签参数不匹配")
    if record.get("corner_order") != CORNER_ORDER:
        raise ValueError("角点顺序不匹配")
    expected = solve_calibration(record["image_points"], record["image_size"], record["bev"], record["camera_source"])
    if record["bev_size"] != expected["bev_size"] or not np.allclose(record["ground_points"], expected["ground_points"], atol=1e-10, rtol=0):
        raise ValueError("BEV 尺寸或地面角点不匹配")
    matrix = np.asarray(record["H_img_to_bev"], dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
        raise ValueError("标定矩阵无效")
    # 单应矩阵允许整体倍乘，归一化后比较。
    index = np.unravel_index(np.argmax(np.abs(matrix)), matrix.shape)
    reference = np.asarray(expected["H_img_to_bev"])
    if not np.allclose(matrix / matrix[index], reference / reference[index], atol=1e-8, rtol=1e-6):
        raise ValueError("矩阵与记录的四点对应关系不匹配")
    return matrix


def save_calibration(record: dict, path: Path = DEFAULT_PATH) -> None:
    """完整精度 JSON 原子写入；文件已存在时保留一份时间戳历史记录。"""
    validate_calibration(record)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        previous = path.read_bytes()
        archive = path.with_name(f"{path.stem}_{datetime.now(timezone.utc):%Y%m%dT%H%M%S_%fZ}{path.suffix}")
        with archive.open("xb") as handle:
            handle.write(previous)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            json.dump(record, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def load_calibration(path: Path = DEFAULT_PATH) -> dict:
    with Path(path).open("r", encoding="utf-8") as handle:
        record = json.load(handle)
    validate_calibration(record)
    return record
