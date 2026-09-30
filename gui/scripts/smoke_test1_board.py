"""在板端用登记照和刀具参考图验证 Qt 到两套常驻服务的实际链路。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
from PySide6.QtWidgets import QApplication
import main_window


def run_case(window: main_window.MainWindow, app: QApplication,
             image_path: Path, expected: str, label: str) -> str:
    frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"无法读取 {image_path}")
    window._start_test1()
    window._on_frame_ready("test-image", frame, 1.0)
    window._recognition_tick()
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        app.processEvents()
        actual = window.run_face.text() if label == "face" else window.run_knife.text()
        if expected in actual:
            print(f"{label} Qt 接线通过：{actual}")
            window._stop_test1()
            return actual
        time.sleep(0.03)
    raise RuntimeError(f"{label} Qt 接线失败：人脸={window.run_face.text()}；刀具={window.run_knife.text()}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--field-photo", type=Path)
    parser.add_argument("--roi", nargs=4, type=int, metavar=("X0", "Y0", "X1", "Y1"))
    parser.add_argument("--min-score", type=float, default=0.0)
    args = parser.parse_args()
    app = QApplication([])
    main_window.list_cameras = lambda: []
    window = main_window.MainWindow()
    window.role_sources["recognition_camera"] = "test-image"
    try:
        run_case(window, app,
                 Path("/home/orangepi/vision_compare_data_v2/face_enroll/suspect_01/reference.jpg"),
                 "suspect_01", "face")
        run_case(window, app,
                 Path("/home/orangepi/pi_upper/models/knife/dinov3_448_fp16_v1/references/knife_01/reference.png"),
                 "knife_01", "knife")
        if args.field_photo:
            if args.roi is None:
                parser.error("--field-photo 需要同时提供 --roi")
            window.knife_roi = tuple(args.roi)
            field_result = run_case(window, app, args.field_photo, "knife_01", "knife")
            score = re.search(r"相似度 ([0-9.]+)", field_result)
            if score is None or float(score.group(1)) < args.min_score:
                raise RuntimeError(f"单刀裁剪相似度未达现场样例预期：{field_result}")
        return 0
    finally:
        window.close()


if __name__ == "__main__":
    raise SystemExit(main())
