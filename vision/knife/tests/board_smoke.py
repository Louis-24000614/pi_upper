"""板端短验收：加载一次模型并确认10张注册图都自匹配正确。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2

# 工具收归模块后，向上三级定位仓库根，使配置和模型路径不受启动目录影响。
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from vision.knife.engine import KnifeRecognizer


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "config/knife.json")
    args = parser.parse_args()
    references = ROOT / "models/knife/dinov3_448_fp16_v1/references"
    results = []
    with KnifeRecognizer(args.config) as recognizer:
        for index in range(1, 11):
            expected = f"knife_{index:02d}"
            path = references / expected / "reference.png"
            frame = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            if frame is None:
                raise ValueError(f"无法读取注册图: {path}")
            result = recognizer.recognize(frame, request_id=f"smoke-{index}")
            results.append((expected, result["top1_class"], result["top1_score"]))
            if result["top1_class"] != expected:
                raise AssertionError(f"注册图自匹配失败: {results[-1]}")
    print("PASS: 10/10 reference images matched their board templates")
    for expected, actual, score in results:
        print(f"{expected}: {actual}, score={score:.6f}")


if __name__ == "__main__":
    main()
