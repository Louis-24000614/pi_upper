"""全屏查看一路摄像头并保存原始帧。

上位机左右分屏会把预览缩小并裁边。本程序把同一路采集按比例铺满屏幕，
整幅画面都可见。空格或回车把原始 BGR 帧写到 data/road/，不写入屏幕上的提示字。
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

from camera_controls import list_cameras
from snap import save_frame, snapshot_path


CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
CAPTURE_FPS = 60

ROLE_LABEL = {
    "recognition_camera": "识别",
    "navigation_camera": "导航",
}


def fitted_size(src_w: int, src_h: int, dst_w: int, dst_h: int) -> tuple[int, int]:
    """整幅放进目标矩形，保持比例，不裁切。"""
    if src_w <= 0 or src_h <= 0 or dst_w < 2 or dst_h < 2:
        return src_w, src_h
    scale = min(dst_w / src_w, dst_h / src_h)
    width = min(dst_w, max(1, int(round(src_w * scale))))
    height = min(dst_h, max(1, int(round(src_h * scale))))
    return width, height


def resolve_device(requested: str | None, paths: list[str]) -> str | None:
    """指定节点优先；否则用枚举到的第一路。"""
    if requested:
        return requested
    return paths[0] if paths else None


def ordered_paths(requested: str | None, discovered: list[str]) -> list[str]:
    """打开顺序：指定设备在前，其余留下供 1/2 切换。"""
    if requested and requested not in discovered:
        return [requested, *discovered]
    if requested:
        return [requested, *[path for path in discovered if path != requested]]
    return list(discovered)


def role_for_path(path: str, discovered: list[str], override: str | None) -> str:
    """文件名前缀与上位机一致：第一路 rec_，第二路 nav_。"""
    if override:
        return override
    try:
        index = discovered.index(path)
    except ValueError:
        return "recognition_camera"
    if index == 1:
        return "navigation_camera"
    return "recognition_camera"


def bgr_to_pixmap(frame: np.ndarray, target_w: int, target_h: int) -> QPixmap:
    """把完整 BGR 帧缩进目标尺寸。缩的是显示副本，不改原始帧。"""
    src_h, src_w = frame.shape[:2]
    width, height = fitted_size(src_w, src_h, target_w, target_h)
    if (width, height) != (src_w, src_h):
        interpolation = cv2.INTER_AREA if width < src_w or height < src_h else cv2.INTER_LINEAR
        shown = cv2.resize(frame, (width, height), interpolation=interpolation)
    else:
        shown = frame
    rgb = np.ascontiguousarray(cv2.cvtColor(shown, cv2.COLOR_BGR2RGB))
    image = QImage(rgb.data, width, height, int(rgb.strides[0]), QImage.Format.Format_RGB888).copy()
    return QPixmap.fromImage(image)


class SnapCapture(QThread):
    """一路 V4L2 采集。请求参数与上位机相同。"""

    ready = Signal()
    opened = Signal(int, int)
    failed = Signal(str)

    def __init__(self, device_path: str) -> None:
        super().__init__()
        self.device_path = device_path
        self._running = True
        self._lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None

    def copy_latest(self) -> np.ndarray | None:
        with self._lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def stop(self) -> None:
        self._running = False

    def run(self) -> None:
        capture = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            self.failed.emit("摄像头打开失败。请先关掉上位机，它可能正占用这路画面。")
            return
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.opened.emit(width, height)
        last_emitted = 0.0
        try:
            while self._running:
                ok, frame = capture.read()
                if not ok:
                    self.failed.emit("读取画面失败")
                    self.msleep(30)
                    continue
                with self._lock:
                    self.latest_frame = frame
                now = time.monotonic()
                if now - last_emitted >= 1 / 30:
                    self.ready.emit()
                    last_emitted = now
        finally:
            capture.release()


class SnapWindow(QWidget):
    def __init__(self, paths: list[str], discovered: list[str], role_override: str | None) -> None:
        super().__init__()
        self.paths = paths
        self.discovered = discovered
        self.role_override = role_override
        self.index = 0
        self.capture: SnapCapture | None = None
        self.frame_size = ""
        self.notice = ""
        self.setWindowTitle("全屏拍照")
        self.setStyleSheet("background: #000;")
        self.view = QLabel("正在打开摄像头…")
        self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setStyleSheet("background: #000; color: #e8e8e8; font-size: 22px;")
        self.banner = QLabel()
        self.banner.setWordWrap(True)
        self.banner.setMinimumHeight(56)
        self.banner.setStyleSheet("background: #111; color: white; font-size: 18px; padding: 8px 14px;")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.view, 1)
        layout.addWidget(self.banner)
        self._refresh_banner()
        self._open_index(0)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._paint()

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self.setFocus()

    def _open_index(self, index: int) -> None:
        if index < 0 or index >= len(self.paths):
            return
        self._stop_capture()
        self.index = index
        self.frame_size = ""
        self.notice = ""
        self._display_frame = None
        self.view.setText("正在打开摄像头…")
        self.view.setPixmap(QPixmap())
        thread = SnapCapture(self.paths[index])
        thread.ready.connect(self._on_frame)
        thread.opened.connect(self._on_opened)
        thread.failed.connect(self._on_failed)
        self.capture = thread
        thread.start()
        self._refresh_banner()

    def _stop_capture(self) -> None:
        thread = self.capture
        self.capture = None
        if thread is None:
            return
        thread.stop()
        thread.wait(1500)

    def _current_role(self) -> str:
        return role_for_path(self.paths[self.index], self.discovered, self.role_override)

    def _refresh_banner(self) -> None:
        path = self.paths[self.index] if self.paths else "无设备"
        role = ROLE_LABEL.get(self._current_role(), self._current_role())
        size = self.frame_size or "分辨率读取中"
        if len(self.paths) > 1 and self.role_override is None:
            labels = [
                f"{number} {ROLE_LABEL[role_for_path(path, self.discovered, None)]}"
                for number, path in enumerate(self.paths[:2], start=1)
            ]
            switch = "    " + "    ".join(labels)
        elif len(self.paths) > 1:
            switch = "    1/2 切换"
        else:
            switch = ""
        text = f"{role}  {path}  {size}    空格拍照    Esc 退出{switch}"
        if self.notice:
            text = f"{text}\n{self.notice}"
        self.banner.setText(text)

    def _on_opened(self, width: int, height: int) -> None:
        if self.sender() is not self.capture:
            return
        self.frame_size = f"{width}×{height}"
        self._refresh_banner()

    def _on_failed(self, message: str) -> None:
        if self.sender() is not self.capture:
            return
        self.view.setPixmap(QPixmap())
        self.view.setText(message)
        self.notice = message
        self._refresh_banner()

    def _on_frame(self) -> None:
        thread = self.capture
        if self.sender() is not thread or thread is None:
            return
        frame = thread.copy_latest()
        if frame is None:
            return
        self._display_frame = frame
        self._paint()

    def _paint(self) -> None:
        frame = getattr(self, "_display_frame", None)
        if frame is None or self.view.width() < 2 or self.view.height() < 2:
            return
        pixmap = bgr_to_pixmap(frame, self.view.width(), self.view.height())
        self.view.setPixmap(pixmap)
        self.view.setText("")

    def _save(self) -> None:
        thread = self.capture
        frame = thread.copy_latest() if thread is not None else None
        dest = snapshot_path(self._current_role())
        ok, message = save_frame(frame, dest)
        self.notice = f"已保存 {message}" if ok else f"拍照失败：{message}"
        self._refresh_banner()

    def keyPressEvent(self, event) -> None:
        key = event.key()
        if key in (Qt.Key.Key_Escape, Qt.Key.Key_Q):
            self.close()
            return
        if key in (Qt.Key.Key_Space, Qt.Key.Key_Return, Qt.Key.Key_Enter):
            self._save()
            return
        if key == Qt.Key.Key_1 and self.index != 0 and len(self.paths) >= 1:
            self._open_index(0)
            return
        if key == Qt.Key.Key_2 and self.index != 1 and len(self.paths) >= 2:
            self._open_index(1)
            return
        super().keyPressEvent(event)

    def closeEvent(self, event) -> None:
        self._stop_capture()
        event.accept()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="全屏查看摄像头并保存原始帧到 data/road/")
    parser.add_argument("--device", help="V4L2 节点，例如 /dev/video0。默认第一路 USB 摄像头。")
    parser.add_argument(
        "--role",
        choices=("recognition_camera", "navigation_camera"),
        help="文件名前缀。默认第一路 rec_、第二路 nav_，与上位机相同。",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    discovered = [camera.path for camera in list_cameras()]
    paths = ordered_paths(args.device, discovered)
    device = resolve_device(args.device, discovered)
    if device is None or not paths:
        print("没有找到 USB 摄像头。", file=sys.stderr)
        return 1
    app = QApplication(sys.argv if argv is None else [sys.argv[0], *argv])
    app.setApplicationName("全屏拍照")
    window = SnapWindow(paths, discovered, args.role)
    window.showFullScreen()
    window.raise_()
    window.activateWindow()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
