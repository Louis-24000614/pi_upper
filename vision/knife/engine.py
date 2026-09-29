"""DINOv3 RKNN常驻推理引擎。

引擎只负责一张BGR图像到候选类别的映射；相机、Qt、任务状态和播报属于上层。
"""

from __future__ import annotations

import hashlib
import json
import logging
from pathlib import Path
import threading
import time

import numpy as np

from .preprocessing import normalize_rgb, prepare_raw_roi, preprocess_bgr
from .templates import TemplateStore


_LOG = logging.getLogger(__name__)


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


def _import_rknn_lite():
    """导入RKNNLite，并用系统PyYAML承接Lite2的平台配置解析。

    Lite2 2.3.2的yaml_parser依赖完整ruamel.yaml API；本板只有PyYAML 5.4.1。
    在init_runtime之前替换load，即可读取cpu_npu_mapper.yml。
    """
    import yaml as pyyaml
    import rknnlite.utils.yaml_parser as yaml_parser

    def load(stream_or_path, *args, **kwargs):
        if hasattr(stream_or_path, "read"):
            return pyyaml.safe_load(stream_or_path)
        with open(stream_or_path, "r", encoding="utf-8") as handle:
            return pyyaml.safe_load(handle)

    yaml_parser.load = load
    from rknnlite.api import RKNNLite

    return RKNNLite


class KnifeRecognizer:
    """线程安全的单RKNN实例刀具识别器。

    同一实例的推理由互斥锁串行化。服务启动一次加载，退出时调用close释放。
    """

    # 这些数值只决定是否尝试单刀 ROI 补救，不是“有刀具”的验收阈值。
    # 现场打印照片中，浅色刀刃漏分割时前景比例不足20%，且常规候选分数低于0.8。
    ROI_MAX_FOREGROUND_RATIO = 0.20
    ROI_MAX_BASE_SCORE = 0.80
    ROI_MIN_SCORE_GAIN = 0.07
    ROI_MIN_MARGIN = 0.03

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
        RKNNLite = _import_rknn_lite()

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

    def _raw_roi_scores(self, bgr: np.ndarray) -> np.ndarray:
        """对完整 ROI 的四个直角方向取逐类最高分，补救刀刃漏分割。

        仅在用户明确框选单刀且常规结果质量较低时调用；四个方向覆盖现场照片
        与登记图的摆放差异。每次仍使用同一RKNN实例和互斥锁，避免并发访问。
        """
        best = np.full(len(self.templates.classes), -1.0, dtype=np.float32)
        for turns in range(4):
            rotated = np.ascontiguousarray(np.rot90(bgr, turns))
            tensor = normalize_rgb(prepare_raw_roi(rotated))
            with self._lock:
                outputs = self._rknn.inference(inputs=[tensor], data_format=["nhwc"])
            if outputs is None or len(outputs) != 1:
                raise RuntimeError("ROI补救推理没有返回唯一输出")
            scores, _ = self.templates.match(self._unit(outputs[0]))
            best = np.maximum(best, scores)
        return best

    def recognize(self, bgr: np.ndarray, request_id: str = "", roi_selected: bool = False) -> dict:
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
        recognition_mode = "segmented"
        # 只在明确框选单刀、低前景且低分时尝试；原图容易带入桌面或其他刀具。
        if (roi_selected
                and metadata["foreground_ratio"] < self.ROI_MAX_FOREGROUND_RATIO
                and float(scores[top1]) < self.ROI_MAX_BASE_SCORE):
            fallback_started = time.perf_counter_ns()
            try:
                raw_scores = self._raw_roi_scores(bgr)
                raw_ranking = np.argsort(-raw_scores)
                raw_top1, raw_top2 = int(raw_ranking[0]), int(raw_ranking[1])
                raw_margin = float(raw_scores[raw_top1] - raw_scores[raw_top2])
                # 原图分数需明显更高且类别间仍有间隔，减少背景导致的随意翻转。
                if (float(raw_scores[raw_top1]) > float(scores[top1]) + self.ROI_MIN_SCORE_GAIN
                        and raw_margin >= self.ROI_MIN_MARGIN):
                    scores = raw_scores
                    top1, top2 = raw_top1, raw_top2
                    recognition_mode = "raw_roi_rotation"
            except (RuntimeError, ValueError) as error:
                # 补救路径是可选的；异常时保留已完成的常规识别，避免整次请求失败。
                _LOG.warning("ROI补救推理失败，使用常规候选: %s", error)
            finally:
                timing["roi_fallback"] = (time.perf_counter_ns() - fallback_started) / 1e6
                timing["total"] += timing["roi_fallback"]
        return {
            "request_id": request_id,
            "model_id": self.config.get("model_id", "dinov3_448_fp16_v1"),
            "template_version": self.templates.version,
            # 当前没有未知类阈值验证，因此只能返回候选，不冒充已安全接受。
            "decision": "candidate",
            "recognition_mode": recognition_mode,
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
