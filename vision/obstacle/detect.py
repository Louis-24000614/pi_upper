"""按 config/obstacle.yaml 加载障碍物 RKNN，并把检测头解成原图上的框。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


def _import_rknn_lite():
    """Lite2 的平台配置解析依赖 ruamel，板上用 PyYAML 接上。"""
    import yaml as pyyaml
    import rknnlite.utils.yaml_parser as yaml_parser

    def load(stream_or_path, *args, **kwargs):
        del args, kwargs
        if hasattr(stream_or_path, "read"):
            return pyyaml.safe_load(stream_or_path)
        with open(stream_or_path, "r", encoding="utf-8") as handle:
            return pyyaml.safe_load(handle)

    yaml_parser.load = load
    from rknnlite.api import RKNNLite

    return RKNNLite


def letterbox(bgr: np.ndarray, size: int = 640, color: int = 114) -> tuple[np.ndarray, float, int, int]:
    """把 BGR 图放进正方形画布。返回 RGB 画布、缩放、左灰边、上灰边。"""
    height, width = bgr.shape[:2]
    ratio = min(size / height, size / width)
    resized_w = int(round(width * ratio))
    resized_h = int(round(height * ratio))
    resized = cv2.resize(bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), color, dtype=np.uint8)
    top = (size - resized_h) // 2
    left = (size - resized_w) // 2
    canvas[top : top + resized_h, left : left + resized_w] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return rgb, ratio, left, top


@dataclass
class Detection:
    """原图像素坐标上的一个框。类别名缺失时 label 为 cls0 这种序号。"""

    class_id: int
    label: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float


class ObstacleDetector:
    """加载障碍物检测模型。``.rknn`` 按配置里的 NPU 核初始化。"""

    def __init__(self, config_path: Path, root: Path | None = None) -> None:
        self.root = root or config_path.resolve().parents[1]
        self.config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        model = self.config["model"]
        self.model_path = self.root / model["path"]
        self.input_size = int(model["input"]["size"])
        self.class_count = int(model["classes"]["count"])
        names = model["classes"].get("names") or []
        self.names = [str(name) for name in names if str(name)]
        self.conf_thres = float(self.config["detect"]["conf_thres"])
        self.iou_thres = float(self.config["detect"]["iou_thres"])
        self._core_mask = str(model.get("core_mask", "1"))
        self._rknn = None

    def close(self) -> None:
        if self._rknn is not None:
            self._rknn.release()
            self._rknn = None

    def __enter__(self) -> "ObstacleDetector":
        self._load()
        return self

    def __exit__(self, *exc) -> None:
        del exc
        self.close()

    def _load(self):
        if self._rknn is not None:
            return self._rknn
        if not self.model_path.is_file():
            raise FileNotFoundError(f"找不到障碍物权重: {self.model_path}")
        RKNNLite = _import_rknn_lite()
        masks = {
            "0": RKNNLite.NPU_CORE_0,
            "1": RKNNLite.NPU_CORE_1,
            "2": RKNNLite.NPU_CORE_2,
            "0_1_2": RKNNLite.NPU_CORE_0_1_2,
        }
        if self._core_mask not in masks:
            raise ValueError(f"未知 core_mask: {self._core_mask}")
        runtime = RKNNLite(verbose=False)
        if runtime.load_rknn(str(self.model_path)) != 0:
            raise RuntimeError(f"load_rknn 失败: {self.model_path}")
        if runtime.init_runtime(core_mask=masks[self._core_mask]) != 0:
            runtime.release()
            raise RuntimeError(f"init_runtime 失败，NPU {self._core_mask} 号核没有起来")
        self._rknn = runtime
        return runtime

    def detect(self, bgr: np.ndarray) -> tuple[list[Detection], float]:
        """返回原图上的框，以及这一帧推理耗时（毫秒）。"""
        import time

        session = self._load()
        canvas, ratio, left, top = letterbox(bgr, size=self.input_size)
        tensor = np.ascontiguousarray(canvas)[None, ...]
        started = time.perf_counter()
        outputs = session.inference(inputs=[tensor], data_format=["nhwc"])
        elapsed_ms = (time.perf_counter() - started) * 1000
        if not outputs or len(outputs) < 6:
            raise RuntimeError("NPU 没有返回检测头")
        boxes, scores, classes = self._decode(outputs)
        height, width = bgr.shape[:2]
        detections = []
        if len(scores) == 0:
            return detections, elapsed_ms
        keep = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), self.conf_thres, self.iou_thres)
        if keep is None or len(keep) == 0:
            return detections, elapsed_ms
        for index in np.array(keep).reshape(-1):
            class_id = int(classes[index])
            x1, y1, x2, y2 = _unletter(boxes[index], ratio, left, top, width, height)
            detections.append(
                Detection(
                    class_id=class_id,
                    label=self.names[class_id] if class_id < len(self.names) else f"cls{class_id}",
                    score=float(scores[index]),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )
        return detections, elapsed_ms

    def _decode(self, outputs):
        boxes, scores, classes = [], [], []
        for scale in range(3):
            raw_box = np.squeeze(outputs[scale * 3]).astype(np.float32)
            raw_cls = np.squeeze(outputs[scale * 3 + 1]).astype(np.float32)
            if raw_box.shape[0] != 64 and raw_box.shape[-1] == 64:
                raw_box = np.transpose(raw_box, (2, 0, 1))
            if raw_cls.shape[0] != self.class_count and raw_cls.shape[-1] == self.class_count:
                raw_cls = np.transpose(raw_cls, (2, 0, 1))
            class_id = raw_cls.argmax(axis=0)
            class_score = raw_cls.max(axis=0)
            xyxy = _dfl_boxes(raw_box, self.input_size)
            keep = class_score >= self.conf_thres
            if not np.any(keep):
                continue
            boxes.append(xyxy[keep])
            scores.append(class_score[keep])
            classes.append(class_id[keep])
        if not boxes:
            return np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,), np.int32)
        return (
            np.concatenate(boxes, axis=0),
            np.concatenate(scores, axis=0),
            np.concatenate(classes, axis=0).astype(np.int32),
        )


def draw(bgr: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """在副本上画框，不改原图。"""
    vis = bgr.copy()
    for item in detections:
        p1 = (int(item.x1), int(item.y1))
        p2 = (int(item.x2), int(item.y2))
        cv2.rectangle(vis, p1, p2, (0, 140, 255), 2)
        cv2.putText(
            vis,
            f"{item.label} {item.score:.2f}",
            (p1[0], max(22, p1[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 140, 255),
            2,
        )
    return vis


def _dfl_boxes(raw: np.ndarray, input_size: int) -> np.ndarray:
    channels, height, width = raw.shape
    bins = channels // 4
    dist = raw.reshape(4, bins, height, width).astype(np.float32)
    dist = dist - dist.max(axis=1, keepdims=True)
    exp = np.exp(dist)
    dist = exp / exp.sum(axis=1, keepdims=True)
    acc = np.arange(bins, dtype=np.float32).reshape(1, bins, 1, 1)
    ltrb = (dist * acc).sum(axis=1)
    gy, gx = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    stride = input_size / height
    x1 = (gx + 0.5 - ltrb[0]) * stride
    y1 = (gy + 0.5 - ltrb[1]) * stride
    x2 = (gx + 0.5 + ltrb[2]) * stride
    y2 = (gy + 0.5 + ltrb[3]) * stride
    return np.stack([x1, y1, x2, y2], axis=-1)


def _unletter(box, ratio: float, left: int, top: int, width: int, height: int):
    x1 = float(np.clip((box[0] - left) / ratio, 0, width - 1))
    y1 = float(np.clip((box[1] - top) / ratio, 0, height - 1))
    x2 = float(np.clip((box[2] - left) / ratio, 0, width - 1))
    y2 = float(np.clip((box[3] - top) / ratio, 0, height - 1))
    return x1, y1, x2, y2
