"""测试2标定窗口：冻结原始帧、检测四角、预览并保存 H。"""

from __future__ import annotations

import json
import cv2
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QDialog, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QVBoxLayout

from apriltag_calibration import (DEFAULT_PATH, PREVIEW_BEV, bev_size, detect_tag, load_calibration,
                                 save_calibration, solve_calibration)


class ApriltagCalibrationDialog(QDialog):
    def __init__(self, frame_provider, parent=None, path=DEFAULT_PATH) -> None:
        super().__init__(parent)
        self.setWindowTitle("测试2 · AprilTag 四点标定")
        self.frame_provider = frame_provider
        self.bev = dict(PREVIEW_BEV)
        self.path = path
        self.record = None
        self.frame = None
        self.saved_path = None
        self._preview_frames = []
        layout = QVBoxLayout(self)
        hint = QLabel("tag36h11 ID 0 · 黑色外沿 134 mm · 平整贴地、完整可见。\n"
                      "以标签中心为原点，X 向标签右、Y 向标签上；原图未去畸变。保存结果不会替换导航 H。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        previews = QHBoxLayout()
        self.source_view = QLabel("待检测导航相机原始帧")
        self.bev_view = QLabel("BEV 预览")
        for view in (self.source_view, self.bev_view):
            view.setAlignment(Qt.AlignmentFlag.AlignCenter)
            view.setMinimumSize(200, 180)
            previews.addWidget(view, 1)
        layout.addLayout(previews, 1)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumHeight(170)
        layout.addWidget(self.details)
        buttons = QHBoxLayout()
        for text, action in (("重新取帧并检测", self.capture), ("读取已保存 H", self.read_saved)):
            button = QPushButton(text)
            button.clicked.connect(action)
            buttons.addWidget(button)
        self.save_button = QPushButton("保存 H")
        self.save_button.setObjectName("primaryButton")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save)
        buttons.addWidget(self.save_button)
        close = QPushButton("关闭")
        close.clicked.connect(self.accept)
        buttons.addWidget(close)
        layout.addLayout(buttons)
        screen_size = self.screen().availableGeometry().size()
        self.resize(min(1000, screen_size.width()), min(680, screen_size.height()))
        self.capture()

    def _show_image(self, view, frame, smooth=True) -> None:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        image = QImage(rgb.data, width, height, channels * width, QImage.Format.Format_RGB888).copy()
        mode = Qt.TransformationMode.SmoothTransformation if smooth else Qt.TransformationMode.FastTransformation
        view.setPixmap(QPixmap.fromImage(image).scaled(view.size(), Qt.AspectRatioMode.KeepAspectRatio, mode))

    def capture(self) -> None:
        self.record = None
        self.frame = None
        self._preview_frames = []
        self.save_button.setEnabled(False)
        self.details.clear()
        self.source_view.clear()
        self.bev_view.clear()
        try:
            frame, source = self.frame_provider()
            self.frame = frame.copy()
            corners = detect_tag(self.frame)
            height, width = self.frame.shape[:2]
            self.record = solve_calibration(corners, (width, height), self.bev, source)
            annotated = self.frame.copy()
            cv2.polylines(annotated, [np.round(corners).astype(np.int32)], True, (0, 255, 0), 2)
            for i, point in enumerate(corners):
                x, y = np.round(point).astype(int)
                cv2.circle(annotated, (x, y), 5, (0, 0, 255), -1)
                cv2.putText(annotated, str(i), (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            bev_frame = cv2.warpPerspective(self.frame, np.asarray(self.record["H_img_to_bev"]), bev_size(self.bev))
            self._preview_frames = [(self.source_view, annotated, True), (self.bev_view, bev_frame, False)]
            self._show_image(self.source_view, annotated)
            self._show_image(self.bev_view, bev_frame, smooth=False)
            self._show_record(self.record)
            self.status.setText(f"已识别四角：原图 {width}×{height}，BEV {bev_size(self.bev)}，"
                                "标签坐标；四点拟合不代表全画面精度验收。")
            self.save_button.setEnabled(True)
        except (ValueError, KeyError, TypeError, cv2.error) as exc:
            self.record = None
            self.status.setText(f"检测失败：{exc}")
            if self.frame is not None:
                self._preview_frames = [(self.source_view, self.frame, True)]
                self._show_image(self.source_view, self.frame)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self._render_previews()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._render_previews()

    def _render_previews(self) -> None:
        for view, frame, smooth in self._preview_frames:
            self._show_image(view, frame, smooth)

    def _show_record(self, record) -> None:
        matrix = np.array2string(np.asarray(record["H_img_to_bev"]), precision=8)
        self.details.setPlainText("H_img_to_bev（原图 → 标签坐标 BEV）\n" + matrix +
                                  "\n\n完整持久化记录：\n" + json.dumps(record, ensure_ascii=False, indent=2))

    def save(self) -> None:
        if self.record is None:
            return
        try:
            frame, source = self.frame_provider()
            if source != self.record["camera_source"] or list(frame.shape[1::-1]) != self.record["image_size"]:
                raise ValueError("导航摄像头来源或分辨率已改变，请重新取帧")
            save_calibration(self.record, self.path)
            self.saved_path = self.path
            self.status.setText(f"已保存：{self.path}。导航仍使用原参数生成的 H。")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status.setText(f"保存失败：{exc}")

    def read_saved(self) -> None:
        try:
            record = load_calibration(self.path)
            self.record = None
            self.save_button.setEnabled(False)
            self._preview_frames = []
            self.source_view.clear()
            self.bev_view.clear()
            self.source_view.setText("已保存记录；重新取帧可查看新的检测结果")
            self._show_record(record)
            self.status.setText(f"已读取：{self.path}；坐标基准为标签中心。")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status.setText(f"读取失败：{exc}")
