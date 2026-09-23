"""刀具模板库加载、校验与余弦匹配。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class TemplateStore:
    """只读的类别顺序与L2归一化模板矩阵。"""

    def __init__(self, path: str | Path, expected_model_sha256: str) -> None:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema_version") != 1:
            raise ValueError("不支持的模板schema_version")
        if payload.get("model_sha256") != expected_model_sha256:
            raise ValueError("模板与RKNN模型SHA-256不匹配")
        self.classes = tuple(str(item) for item in payload["classes"])
        if self.classes != tuple(f"knife_{i:02d}" for i in range(1, 11)):
            raise ValueError(f"模板类别顺序异常: {self.classes}")
        self.embeddings = np.asarray(payload["embeddings"], dtype=np.float32)
        if self.embeddings.shape != (10, 384) or not np.isfinite(self.embeddings).all():
            raise ValueError(f"模板形状或数值异常: {self.embeddings.shape}")
        norms = np.linalg.norm(self.embeddings, axis=1)
        if float(np.max(np.abs(norms - 1.0))) >= 1e-5:
            raise ValueError("模板未正确L2归一化")
        self.version = str(payload.get("template_version", "unknown"))

    def match(self, descriptor: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """返回余弦分数和稳定降序索引；同分时保持类别固定顺序。"""
        descriptor = np.asarray(descriptor, dtype=np.float32).reshape(-1)
        if descriptor.shape != (384,) or not np.isfinite(descriptor).all():
            raise ValueError("查询descriptor必须是finite的384维向量")
        scores = self.embeddings @ descriptor
        ranking = np.argsort(-scores, kind="stable")
        return scores, ranking
