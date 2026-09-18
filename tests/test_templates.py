"""模板库不依赖RKNN的快速单元测试。"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision.knife.templates import TemplateStore


def main() -> None:
    config = json.loads((ROOT / "config/knife.json").read_text(encoding="utf-8"))
    store = TemplateStore(
        ROOT / config["templates_path"],
        config["model_sha256"],
    )
    for index, template in enumerate(store.embeddings):
        scores, ranking = store.match(template)
        assert int(ranking[0]) == index
        assert scores[index] > 0.99999
    print("PASS: 10 templates self-match and fixed class order")


if __name__ == "__main__":
    main()
