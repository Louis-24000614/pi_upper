"""把采集线程中的原始 BGR 帧落盘，供道路分割训练使用。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# 仓库根下的道路实采目录；PNG 不入库，见根目录 .gitignore。
DEFAULT_DIR = Path(__file__).resolve().parents[1] / "data" / "road"

_ROLE_PREFIX = {
    "recognition_camera": "rec",
    "navigation_camera": "nav",
}


def snapshot_path(
    role: str,
    directory: Path | None = None,
    when: datetime | None = None,
) -> Path:
    """按逻辑摄像头角色和时间戳生成 PNG 路径。

    `role` 为 `recognition_camera` 或 `navigation_camera`。文件名不含路径分隔符，
    便于直接作为训练样本清单条目。
    """
    stamp = (when or datetime.now()).strftime("%Y%m%d_%H%M%S_%f")[:-3]
    prefix = _ROLE_PREFIX.get(role, "cam")
    root = directory if directory is not None else DEFAULT_DIR
    return root / f"{prefix}_{stamp}.png"


def save_frame(frame: object, dest: Path) -> tuple[bool, str]:
    """将一份 BGR 帧拷贝后写入 `dest`。

    必须先拷贝再写盘：采集线程会持续覆盖 `latest_frame`。失败时第二项为原因。
    """
    if frame is None:
        return False, "当前没有可用画面"
    image = np.asarray(frame)
    if image.ndim != 3 or image.shape[2] != 3:
        return False, "画面不是 BGR 三通道图像"
    dest.parent.mkdir(parents=True, exist_ok=True)
    copied = image.copy()
    if not cv2.imwrite(str(dest), copied):
        return False, f"写入失败：{dest}"
    return True, str(dest)
