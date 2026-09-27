"""道路采集落盘：路径规则与写盘结果（无硬件）。"""

from datetime import datetime
from pathlib import Path

import numpy as np

from snap import save_frame, snapshot_path


def test_snapshot_path_uses_role_prefix_and_timestamp(tmp_path: Path) -> None:
    when = datetime(2026, 9, 1, 16, 22, 3, 123000)
    dest = snapshot_path("navigation_camera", directory=tmp_path, when=when)
    assert dest == tmp_path / "nav_20260901_162203_123.png"


def test_snapshot_path_unknown_role_uses_cam_prefix(tmp_path: Path) -> None:
    when = datetime(2026, 9, 1, 16, 22, 3, 456000)
    dest = snapshot_path("unknown", directory=tmp_path, when=when)
    assert dest.name.startswith("cam_")


def test_save_frame_rejects_missing_image(tmp_path: Path) -> None:
    ok, message = save_frame(None, tmp_path / "empty.png")
    assert ok is False
    assert "没有可用画面" in message


def test_save_frame_writes_png_copy(tmp_path: Path) -> None:
    frame = np.zeros((8, 12, 3), dtype=np.uint8)
    frame[0, 0] = (0, 0, 255)
    dest = tmp_path / "sample.png"
    ok, message = save_frame(frame, dest)
    assert ok is True
    assert Path(message) == dest
    assert dest.is_file()
    assert dest.stat().st_size > 0
