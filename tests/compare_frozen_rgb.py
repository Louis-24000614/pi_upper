"""板端核对迁移后的内存预处理与冻结实验RGB是否逐像素一致。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision.knife.preprocessing import preprocess_bgr


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--deployment-inputs", type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.deployment_inputs, allow_pickle=False) as package:
        expected = np.asarray(package["rgb448"][:10], dtype=np.uint8)
    actual = []
    references = ROOT / "models/knife/dinov3_448_fp16_v1/references"
    for index in range(1, 11):
        path = references / f"knife_{index:02d}" / "reference.png"
        frame = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        rgb, _, _ = preprocess_bgr(frame)
        actual.append(rgb)
    actual = np.stack(actual)
    difference = np.abs(actual.astype(np.int16) - expected.astype(np.int16))
    if not np.array_equal(actual, expected):
        raise AssertionError(
            f"预处理不一致: differing_values={np.count_nonzero(difference)}, "
            f"max_abs={difference.max()}"
        )
    print("PASS: 10/10 migrated preprocessing outputs exactly match frozen RGB448")


if __name__ == "__main__":
    main()
