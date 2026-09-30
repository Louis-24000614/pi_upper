"""完整预览与屏幕画面切换回归。"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication
from main_window import CameraPreview, MainWindow


class TestCameraDisplay(unittest.TestCase):
    def test_preview_fits_full_frame_after_resize(self) -> None:
        app = QApplication.instance() or QApplication([])
        view = CameraPreview()
        view.resize(400, 300)
        view.show()
        app.processEvents()
        image = QImage(1600, 900, QImage.Format.Format_RGB888)
        image.fill(Qt.GlobalColor.red)
        view.set_frame_image(image)
        self.assertEqual((view.pixmap().width(), view.pixmap().height()), (400, 225))
        view.resize(320, 320)
        app.processEvents()
        self.assertEqual((view.pixmap().width(), view.pixmap().height()), (320, 180))
        view.close()

    def test_display_toggle_keeps_camera_mapping_and_frames(self) -> None:
        app = QApplication.instance() or QApplication([])
        with patch("main_window.list_cameras", return_value=[]):
            window = MainWindow()
        try:
            window.role_sources.update(recognition_camera="rec", navigation_camera="nav")
            rec_frame = np.zeros((90, 160, 3), dtype=np.uint8)
            nav_frame = np.full((90, 160, 3), 255, dtype=np.uint8)
            window._on_frame_ready("rec", rec_frame, 30.0)
            window._on_frame_ready("nav", nav_frame, 30.0)
            sources = window.role_sources.copy()
            self.assertEqual(window.display_role, "recognition_camera")
            self.assertIs(window.camera_stack.currentWidget(), window.recognition_view.parentWidget())
            window.display_switch_button.click()
            self.assertEqual(window.display_role, "navigation_camera")
            self.assertIs(window.camera_stack.currentWidget(), window.navigation_view.parentWidget())
            self.assertEqual(window.display_notice.text(), "当前显示：导航摄像头")
            self.assertIsNotNone(window.navigation_view.pixmap())
            window.display_switch_button.click()
            self.assertEqual(window.display_role, "recognition_camera")
            self.assertEqual(window.role_sources, sources)
            self.assertIs(window.recognition_frame, rec_frame)
            self.assertIs(window.navigation_frame, nav_frame)
            app.processEvents()
        finally:
            window.close()
