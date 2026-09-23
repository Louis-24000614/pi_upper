"""比较迁移实现、原实验实现及冻结RGB，定位跨环境的像素差异来源。"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np

# 工具收归模块后，向上三级定位仓库根，使配置和模型路径不受启动目录影响。
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from vision.knife.preprocessing import preprocess_bgr


def _load_reference(reference_root: Path):
    """从指定实验源码根加载原始preprocess_image，不依赖当前工作目录。"""
    sys.path.insert(0, str(reference_root))
    module_path = reference_root / "src/preprocessing/normalization.py"
    spec = importlib.util.spec_from_file_location("frozen_reference_normalization", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.preprocess_image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-root", type=Path, required=True)
    parser.add_argument("--deployment-inputs", type=Path, required=True)
    args = parser.parse_args()
    reference_preprocess = _load_reference(args.reference_root.resolve())
    with np.load(args.deployment_inputs, allow_pickle=False) as package:
        frozen = np.asarray(package["rgb448"][:10], dtype=np.uint8)
    references = ROOT / "models/knife/dinov3_448_fp16_v1/references"
    migrated, original = [], []
    for index in range(1, 11):
        path = references / f"knife_{index:02d}" / "reference.png"
        frame = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        migrated.append(preprocess_bgr(frame)[0])
        original.append(reference_preprocess(path, 448)[0])
    migrated, original = np.stack(migrated), np.stack(original)
    if not np.array_equal(migrated, original):
        diff = np.abs(migrated.astype(np.int16) - original.astype(np.int16))
        raise AssertionError(
            f"迁移代码与原代码不一致: differing_values={np.count_nonzero(diff)}, max_abs={diff.max()}"
        )
    environment_diff = np.abs(original.astype(np.int16) - frozen.astype(np.int16))
    print("PASS: migrated preprocessing exactly matches reference implementation on this board")
    print(
        "Reference implementation vs frozen cache: "
        f"differing_values={np.count_nonzero(environment_diff)}, max_abs={environment_diff.max()}"
    )


if __name__ == "__main__":
    main()
