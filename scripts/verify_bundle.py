#!/usr/bin/env python3
"""在Windows或板端只读核验部署包的模型、模板和manifest。"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "models/knife/dinov3_448_fp16_v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    manifest = json.loads((RELEASE / "manifest.json").read_text(encoding="utf-8"))
    for relative, expected in manifest["files_sha256"].items():
        actual = sha256(RELEASE / relative)
        if actual != expected:
            raise ValueError(f"哈希不匹配: {relative}: {actual}")
    templates = json.loads((RELEASE / "templates.json").read_text(encoding="utf-8"))
    vectors = np.asarray(templates["embeddings"], dtype=np.float32)
    assert vectors.shape == (10, 384)
    assert np.isfinite(vectors).all()
    assert np.max(np.abs(np.linalg.norm(vectors, axis=1) - 1)) < 1e-5
    assert templates["classes"] == [f"knife_{i:02d}" for i in range(1, 11)]
    print("PASS: bundle hashes, class order, template shape and L2 norms")


if __name__ == "__main__":
    main()
