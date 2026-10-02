"""YOLO11n-seg 道路 mask。输入 640，单类 road，框和原型分开输出。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np

from road_follow.segment_buffers import SegmentBuffers


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


def _orient_heads(pred: np.ndarray, proto: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把 NPU 的 NHWC 原型转回 CHW。检测头仍交给解码器处理两种排布。"""
    pred = np.squeeze(pred)
    proto = np.squeeze(proto)
    if proto.ndim == 3 and proto.shape[-1] in (32, 37) and proto.shape[0] != 32:
        proto = np.transpose(proto, (2, 0, 1))
    return pred, proto


def _sigmoid(values: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(values, -50.0, 50.0)))


def _as_nms_indices(raw) -> np.ndarray:
    if raw is None or len(raw) == 0:
        return np.zeros((0,), dtype=np.int32)
    return np.array(raw).reshape(-1).astype(np.int32)


def decode_road_mask(
    pred: np.ndarray,
    proto: np.ndarray,
    ratio: float,
    left: int,
    top: int,
    orig_hw: tuple[int, int],
    *,
    conf_thres: float = 0.45,
    iou_thres: float = 0.5,
    input_size: int = 640,
    correct_nms: bool = False,
    buffers: SegmentBuffers | None = None,
) -> np.ndarray:
    """把分割头变成原图大小的 0/255 道路 mask。没有够置信度的框时返回全 0。"""
    pred = np.squeeze(pred)
    proto = np.squeeze(proto)
    if pred.ndim != 2 or proto.ndim != 3:
        raise ValueError(f"unexpected heads pred={pred.shape} proto={proto.shape}")
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T

    boxes_xywh = pred[:, 0:4].astype(np.float32)
    conf = pred[:, 4].astype(np.float32)
    coeffs = pred[:, 5:].astype(np.float32)
    if float(np.max(conf)) > 1.0 or float(np.min(conf)) < 0.0:
        conf = _sigmoid(conf)
    if float(np.max(boxes_xywh)) <= 1.5:
        boxes_xywh = boxes_xywh * float(input_size)

    keep = conf >= conf_thres
    height, width = orig_hw
    blank = np.zeros((height, width), dtype=np.uint8)
    if not np.any(keep):
        return blank

    boxes_xywh = boxes_xywh[keep]
    conf = conf[keep]
    coeffs = coeffs[keep]
    xyxy = np.empty_like(boxes_xywh)
    xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] * 0.5
    xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] * 0.5
    xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] * 0.5
    xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] * 0.5

    # OpenCV Rect 需要左上角+宽高；原实现误传右下角，单独作为 A0→A1
    # 正确性实验启用，不能把由此造成的输出变化算作等价性能优化。
    nms_boxes = xyxy.copy() if correct_nms else xyxy
    if correct_nms:
        nms_boxes[:, 2:4] -= nms_boxes[:, 0:2]
    indices = _as_nms_indices(
        cv2.dnn.NMSBoxes(nms_boxes.tolist(), conf.tolist(), conf_thres, iou_thres)
    )
    if indices.size == 0:
        return blank

    xyxy = xyxy[indices]
    coeffs = coeffs[indices]
    channels, mask_h, mask_w = proto.shape
    flat = proto.reshape(channels, -1).astype(np.float32)
    masks = _sigmoid(coeffs @ flat).reshape(-1, mask_h, mask_w)

    merged = np.zeros((input_size, input_size), dtype=bool) if buffers is None else buffers.merged
    if buffers is not None:
        merged.fill(False)
    for mask, box in zip(masks, xyxy):
        up = cv2.resize(mask, (input_size, input_size),
                        dst=None if buffers is None else buffers.up,
                        interpolation=cv2.INTER_LINEAR)
        x1 = max(0, int(np.floor(box[0])))
        y1 = max(0, int(np.floor(box[1])))
        x2 = min(input_size, int(np.ceil(box[2])))
        y2 = min(input_size, int(np.ceil(box[3])))
        if x2 <= x1 or y2 <= y1:
            continue
        patch = up[y1:y2, x1:x2] > 0.5
        merged[y1:y2, x1:x2] |= patch

    resized_h = int(round(height * ratio))
    resized_w = int(round(width * ratio))
    cropped = merged[top : top + resized_h, left : left + resized_w]
    if cropped.size == 0:
        return blank
    restored = cv2.resize(
        cropped.astype(np.uint8) * 255,
        (width, height),
        interpolation=cv2.INTER_NEAREST,
    )
    return restored


@dataclass
class RoadSegmenter:
    """加载道路分割模型。``.rknn`` 走 NPU 0 号核，``.onnx`` 走 CPU。"""

    model_path: Path
    conf_thres: float = 0.45
    _session: object = None
    correct_nms: bool = False
    reuse_buffers: bool = False
    core_mask: int | None = None
    measure: bool = False
    _buffers: SegmentBuffers | None = None
    last_timings: dict | None = None
    backend: str = "lite"

    def _load(self):
        if self._session is not None:
            return self._session
        if self.model_path.suffix == ".rknn":
            if self.backend != "lite":
                from road_follow.native_backend import NativeRknnSession
                if self.backend not in ("c-standard", "c-input-zero"):
                    raise ValueError(f"未知推理后端: {self.backend}")
                self._session = NativeRknnSession(self.model_path, self.core_mask or 1,
                                                 self.backend == "c-input-zero")
                return self._session
            RKNNLite = _import_rknn_lite()
            runtime = RKNNLite(verbose=False)
            if runtime.load_rknn(str(self.model_path)) != 0:
                runtime.release()
                raise RuntimeError(f"load_rknn 失败: {self.model_path}")
            selected_core = RKNNLite.NPU_CORE_0 if self.core_mask is None else self.core_mask
            if runtime.init_runtime(core_mask=selected_core) != 0:
                runtime.release()
                raise RuntimeError(f"init_runtime 失败，core_mask={selected_core}")
            self._session = runtime
            return runtime
        import onnxruntime as ort

        self._session = ort.InferenceSession(
            str(self.model_path),
            providers=["CPUExecutionProvider"],
        )
        return self._session

    def mask(self, bgr: np.ndarray) -> np.ndarray:
        session = self._load()
        t0 = time.perf_counter() if self.measure else 0.0
        if self.reuse_buffers and self._buffers is None:
            self._buffers = SegmentBuffers()
        canvas, ratio, left, top = (
            self._buffers.letterbox(bgr) if self._buffers is not None
            else letterbox(bgr, size=640)
        )
        t1 = time.perf_counter() if self.measure else 0.0
        if self.model_path.suffix == ".rknn":
            tensor = np.ascontiguousarray(canvas)[None, ...]
            outputs = session.inference(inputs=[tensor], data_format=["nhwc"])
            if not outputs or len(outputs) < 2:
                raise RuntimeError("NPU 没有返回检测头和原型")
            pred, proto = _orient_heads(outputs[0], outputs[1])
        else:
            blob = np.transpose(canvas.astype(np.float32) / 255.0, (2, 0, 1))[None]
            input_name = session.get_inputs()[0].name
            pred, proto = session.run(None, {input_name: blob})
            pred, proto = _orient_heads(pred, proto)
        t2 = time.perf_counter() if self.measure else 0.0
        result = decode_road_mask(
            pred,
            proto,
            ratio,
            left,
            top,
            bgr.shape[:2],
            conf_thres=self.conf_thres,
            correct_nms=self.correct_nms,
            buffers=self._buffers,
        )
        if self.measure:
            t3 = time.perf_counter()
            self.last_timings = {"pre_ms": (t1-t0)*1000, "rknn_ms": (t2-t1)*1000,
                                 "post_ms": (t3-t2)*1000}
        return result

    def close(self) -> None:
        """实验轮次结束及时释放 context，避免跨轮资源残留影响 A/B。"""
        if self._session is not None and self.model_path.suffix == ".rknn":
            self._session.release()
        self._session = None
