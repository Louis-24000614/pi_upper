#!/usr/bin/env python3
"""从已核验板端结果中只导出前10个注册模板，不复制测试向量。"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--embedding-npz", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    labels = json.loads(args.labels.read_text(encoding="utf-8"))
    classes = labels["reference_classes"]
    expected_classes = [f"knife_{i:02d}" for i in range(1, 11)]
    if classes != expected_classes:
        raise ValueError(f"注册类别顺序异常: {classes}")
    with np.load(args.embedding_npz, allow_pickle=False) as package:
        embeddings = np.asarray(package["embeddings"][:10], dtype=np.float32).copy()
    if embeddings.shape != (10, 384) or not np.isfinite(embeddings).all():
        raise ValueError(f"模板矩阵异常: {embeddings.shape}")
    if float(np.max(np.abs(np.linalg.norm(embeddings, axis=1) - 1.0))) >= 1e-5:
        raise ValueError("模板没有正确L2归一化")
    payload = {
        "schema_version": 1,
        "template_version": "single_reference_v1",
        "model_sha256": sha256(args.model),
        "source_run": args.embedding_npz.parent.parent.name,
        "classes": classes,
        "embedding_dim": 384,
        "embeddings": embeddings.tolist(),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(args.output)


if __name__ == "__main__":
    main()
