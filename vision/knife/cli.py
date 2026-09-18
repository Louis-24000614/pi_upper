"""板端单图识别CLI，用于接GUI前的最小闭环测试。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from .engine import KnifeRecognizer


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--config", type=Path, default=Path("config/knife.json"))
    args = parser.parse_args()
    frame = cv2.imread(str(args.image), cv2.IMREAD_UNCHANGED)
    if frame is None:
        raise ValueError(f"无法读取图片: {args.image}")
    with KnifeRecognizer(args.config) as recognizer:
        result = recognizer.recognize(frame, request_id="cli")
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
