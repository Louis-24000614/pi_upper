"""DINOv3 RKNN常驻推理引擎。

引擎只负责一张BGR图像到候选类别的映射；相机、Qt、任务状态和播报属于上层。
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import threading
import time

import numpy as np

from .preprocessing import normalize_rgb, preprocess_bgr
from .templates import TemplateStore


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _resolve(config_path: Path, value: str) -> Path:
    """相对路径统一按配置文件所在项目根解析，避免启动目录改变行为。"""
    path = Path(value)
    if path.is_absolute():
        return path
    return (config_path.parent.parent / path).resolve()


class KnifeRecognizer:
    """线程安全的单RKNN实例刀具识别器。

    同一实例的推理由互斥锁串行化。服务启动一次加载，退出时调用close释放。
    """

    def __init__(self, config_path: str | Path) -> None:
        self.config_path = Path(config_path).resolve()
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))
        self.model_path = _resolve(self.config_path, self.config["model_path"])
        self.templates_path = _resolve(self.config_path, self.config["templates_path"])
        expected_sha = str(self.config["model_sha256"])
        actual_sha = _sha256(self.model_path)
        if actual_sha != expected_sha:
            raise ValueError(f"RKNN模型SHA-256不匹配: {actual_sha}")
        self.templates = TemplateStore(self.templates_path, expected_sha)
        self._lock = threading.Lock()
        self._closed = False

        # 板端才提供rknnlite；延迟导入使Windows可执行配置和模板单元测试。
        from rknnlite.api import RKNNLite

        masks = {
            "0": RKNNLite.NPU_CORE_0,
            "1": RKNNLite.NPU_CORE_1,
            "2": RKNNLite.NPU_CORE_2,
            "0_1_2": RKNNLite.NPU_CORE_0_1_2,
        }
        mask_name = str(self.config.get("core_mask", "0_1_2"))
        if mask_name not in masks:
            raise ValueError(f"不支持的core_mask: {mask_name}")
        self._rknn = RKNNLite(verbose=False)
        if self._rknn.load_rknn(str(self.model_path)) != 0:
            raise RuntimeError("load_rknn失败")
        if self._rknn.init_runtime(core_mask=masks[mask_name]) != 0:
            self._rknn.release()
            raise RuntimeError("init_runtime失败")

    @staticmethod
    def _unit(raw: np.ndarray) -> np.ndarray:
        """把RKNN原始输出转换为FP32单位向量。"""
        value = np.asarray(raw, dtype=np.float32).reshape(-1)
        if value.shape != (384,) or not np.isfinite(value).all():
            raise ValueError(f"NPU输出形状或数值异常: {value.shape}")
        norm = float(np.linalg.norm(value))
        if norm < 1e-12:
            raise ValueError("NPU输出为零向量")
        return value / np.float32(norm)

    def embed(self, bgr: np.ndarray) -> tuple[np.ndarray, dict, dict]:
        """对一张BGR/BGRA图提取descriptor，供识别与板端模板导出共用。"""
        if self._closed:
            raise RuntimeError("识别器已关闭")
        started = time.perf_counter_ns()
        rgb, _, metadata = preprocess_bgr(
            bgr,
            size=448,
            min_foreground_pixels=int(self.config.get("min_foreground_pixels", 20)),
            min_foreground_ratio=float(self.config.get("min_foreground_ratio", 0.005)),
        )
        after_preprocess = time.perf_counter_ns()
        tensor = normalize_rgb(rgb)
        after_normalize = time.perf_counter_ns()
        with self._lock:
            outputs = self._rknn.inference(inputs=[tensor], data_format=["nhwc"])
        after_npu = time.perf_counter_ns()
        if outputs is None or len(outputs) != 1:
            raise RuntimeError("RKNN推理没有返回唯一输出")
        descriptor = self._unit(outputs[0])
        finished = time.perf_counter_ns()
        timing = {
            "preprocess": (after_preprocess - started) / 1e6,
            "normalize": (after_normalize - after_preprocess) / 1e6,
            "npu_api": (after_npu - after_normalize) / 1e6,
            "postprocess": (finished - after_npu) / 1e6,
            "total": (finished - started) / 1e6,
        }
        return descriptor, metadata.to_dict(), timing

    def recognize(self, bgr: np.ndarray, request_id: str = "") -> dict:
        """识别一张BGR/BGRA图像，返回Top-2候选和分阶段耗时。"""
        descriptor, metadata, timing = self.embed(bgr)
        match_started = time.perf_counter_ns()
        scores, ranking = self.templates.match(descriptor)
        match_finished = time.perf_counter_ns()
        # 模板匹配虽然耗时很小，也应计入端到端总耗时，便于上层准确观测。
        matching_ms = (match_finished - match_started) / 1e6
        timing["matching"] = matching_ms
        timing["total"] += matching_ms
        top1, top2 = int(ranking[0]), int(ranking[1])
        return {
            "request_id": request_id,
            "model_id": self.config.get("model_id", "dinov3_448_fp16_v1"),
            "template_version": self.templates.version,
            # 当前没有未知类阈值验证，因此只能返回候选，不冒充已安全接受。
            "decision": "candidate",
            "top1_class": self.templates.classes[top1],
            "top1_score": float(scores[top1]),
            "top2_class": self.templates.classes[top2],
            "top2_score": float(scores[top2]),
            "margin": float(scores[top1] - scores[top2]),
            "quality_ok": True,
            "preprocess": metadata,
            "timing_ms": timing,
        }

    def warmup(self, bgr: np.ndarray, count: int = 2) -> None:
        """服务ready前执行少量真实预处理和推理，避免首请求冷启动。"""
        for index in range(max(0, count)):
            self.recognize(bgr, request_id=f"warmup-{index}")

    def close(self) -> None:
        """幂等释放RKNN上下文。"""
        if not self._closed:
            self._rknn.release()
            self._closed = True

    def __enter__(self) -> "KnifeRecognizer":
        return self

    def __exit__(self, *_unused) -> None:
        self.close()
