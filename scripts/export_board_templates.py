#!/usr/bin/env python3
"""在目标RK3588上用生产预处理和实际RKNN重新生成10个注册模板。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision.knife.engine import KnifeRecognizer


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "config/knife.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    classes = [f"knife_{index:02d}" for index in range(1, 11)]
    references = ROOT / "models/knife/dinov3_448_fp16_v1/references"
    descriptors = []
    with KnifeRecognizer(args.config) as recognizer:
        for name in classes:
            path = references / name / "reference.png"
            frame = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            if frame is None:
                raise ValueError(f"无法读取注册图: {path}")
            descriptor, _, _ = recognizer.embed(frame)
            descriptors.append(descriptor)
        model_sha256 = recognizer.config["model_sha256"]
    embeddings = np.stack(descriptors).astype(np.float32)
    if embeddings.shape != (10, 384) or not np.isfinite(embeddings).all():
        raise ValueError(f"模板矩阵异常: {embeddings.shape}")
    if float(np.max(np.abs(np.linalg.norm(embeddings, axis=1) - 1.0))) >= 1e-5:
        raise ValueError("模板没有正确L2归一化")
    payload = {
        "schema_version": 1,
        "template_version": "single_reference_board_preprocess_v1",
        "model_sha256": model_sha256,
        "source": "target board production preprocessing + target board RKNN inference",
        "classes": classes,
        "embedding_dim": 384,
        "embeddings": embeddings.tolist(),
    }
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(args.output)


if __name__ == "__main__":
    main()
