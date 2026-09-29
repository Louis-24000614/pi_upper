"""在完整相机帧上框选单把刀具的取景区域。"""

from __future__ import annotations

import cv2
import numpy as np
from PySide6.QtCore import QPoint, QRect, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


def source_roi(selection: QRect, image_rect: QRect, width: int, height: int) -> tuple[int, int, int, int] | None:
    """把完整画面显示区内的拖选框映射回原始帧坐标。"""
    clipped = selection.normalized().intersected(image_rect)
    if clipped.width() < 5 or clipped.height() < 5 or image_rect.isEmpty():
        return None
    x0 = round((clipped.x() - image_rect.x()) * width / image_rect.width())
    y0 = round((clipped.y() - image_rect.y()) * height / image_rect.height())
    x1 = round((clipped.x() + clipped.width() - image_rect.x()) * width / image_rect.width())
    y1 = round((clipped.y() + clipped.height() - image_rect.y()) * height / image_rect.height())
    return (max(0, x0), max(0, y0), min(width, x1), min(height, y1))


class RoiCanvas(QWidget):
    selection_changed = Signal(object)

    def __init__(self, frame: np.ndarray, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        image = QImage(rgb.data, width, height, channels * width, QImage.Format.Format_RGB888).copy()
        self.pixmap = QPixmap.fromImage(image)
        self.source_width = width
        self.source_height = height
        self.image_rect = QRect()
        self.selection = QRect()
        self._start: QPoint | None = None
        self.setMouseTracking(True)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#070b0f"))
        size = self.pixmap.size().scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio)
        self.image_rect = QRect((self.width() - size.width()) // 2,
                                (self.height() - size.height()) // 2,
                                size.width(), size.height())
        painter.drawPixmap(self.image_rect, self.pixmap)
        selected = self.selection.normalized().intersected(self.image_rect)
        if not selected.isEmpty():
            painter.fillRect(selected, QColor(46, 210, 174, 55))
            painter.setPen(QPen(QColor("#62d7ae"), 3))
            painter.drawRect(selected)
        painter.end()

    def _clamp(self, point: QPoint) -> QPoint:
        return QPoint(max(self.image_rect.left(), min(self.image_rect.right(), point.x())),
                      max(self.image_rect.top(), min(self.image_rect.bottom(), point.y())))

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self.image_rect.contains(event.position().toPoint()):
            self._start = self._clamp(event.position().toPoint())
            self.selection = QRect(self._start, self._start)
            self.update()

    def mouseMoveEvent(self, event) -> None:
        if self._start is not None:
            self.selection = QRect(self._start, self._clamp(event.position().toPoint())).normalized()
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if self._start is None:
            return
        self.selection = QRect(self._start, self._clamp(event.position().toPoint())).normalized()
        self._start = None
        self.selection_changed.emit(self.get_roi())
        self.update()

    def get_roi(self) -> tuple[int, int, int, int] | None:
        return source_roi(self.selection, self.image_rect, self.source_width, self.source_height)


class KnifeRoiDialog(QDialog):
    def __init__(self, frame: np.ndarray, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("框选单把刀具")
        self.setWindowState(Qt.WindowState.WindowFullScreen)
        layout = QVBoxLayout(self)
        hint = QLabel("拖动框住一把完整刀具，周围留少量背景；不要包含另一把刀或手。Esc 取消。")
        hint.setObjectName("notice")
        layout.addWidget(hint)
        self.canvas = RoiCanvas(frame)
        layout.addWidget(self.canvas, 1)
        self.status = QLabel("尚未框选")
        layout.addWidget(self.status)
        buttons = QHBoxLayout()
        cancel = QPushButton("取消")
        cancel.clicked.connect(self.reject)
        self.confirm = QPushButton("使用此区域")
        self.confirm.setObjectName("primaryButton")
        self.confirm.setEnabled(False)
        self.confirm.clicked.connect(self.accept)
        buttons.addStretch(1)
        buttons.addWidget(cancel)
        buttons.addWidget(self.confirm)
        layout.addLayout(buttons)
        self.canvas.selection_changed.connect(self._on_selection)

    def _on_selection(self, roi: tuple[int, int, int, int] | None) -> None:
        valid = roi is not None and roi[2] - roi[0] >= 80 and roi[3] - roi[1] >= 80
        self.confirm.setEnabled(valid)
        self.status.setText(f"取景：{roi[2]-roi[0]} × {roi[3]-roi[1]} 像素" if valid else "区域太小，请重选")

    def get_roi(self) -> tuple[int, int, int, int] | None:
        return self.canvas.get_roi()
