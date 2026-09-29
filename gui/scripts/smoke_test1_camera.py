"""板端真实 USB 摄像头到 Qt 与两套识别服务的短时冒烟测试。"""

from __future__ import annotations

import os
from pathlib import Path
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication
from main_window import MainWindow


def wait_for(app: QApplication, predicate, seconds: float) -> bool:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(0.03)
    return False


def main() -> int:
    app = QApplication([])
    window = MainWindow()
    try:
        if not wait_for(app, lambda: window.recognition_frame is not None, 8):
            raise RuntimeError(f"未收到识别摄像头画面：{window.camera_errors}")
        frame = window.recognition_frame
        source = window.role_sources["recognition_camera"]
        print(f"摄像头：{source}，原始帧 {frame.shape[1]}x{frame.shape[0]}")
        print(f"格式：{window.physical_formats.get(source, '未知')}")
        window._start_test1()
        if not wait_for(app, lambda: bool(window._last_knife_message and window._last_face_message), 12):
            raise RuntimeError("测试1未能从两套服务获得响应")
        print(f"识别画面帧数：{window.recognition_frame_id}")
        print(f"实际采集 FPS：{window.physical_fps.get(source, 0):.1f}")
        print(f"刀具：{window.run_knife.text()}")
        print(f"人脸：{window.run_face.text()}")
        if window.recognition_frame_id < 2 or window.physical_fps.get(source, 0) <= 0:
            raise RuntimeError("摄像头未稳定采集")
        return 0
    finally:
        window.close()


if __name__ == "__main__":
    raise SystemExit(main())
