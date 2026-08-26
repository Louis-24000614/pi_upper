"""RoboCup 侦查机器人上位机主界面。"""

from __future__ import annotations

import time

import cv2
from PySide6.QtCore import QThread, Qt, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QSplitter,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from camera_controls import CameraDevice, V4L2Control, list_cameras, set_control, v4l2_available


CONTROL_NAMES = {
    "brightness": "亮度", "contrast": "对比度", "saturation": "饱和度",
    "hue": "色调", "gain": "增益", "sharpness": "锐度",
    "exposure_auto": "自动曝光模式", "exposure_absolute": "曝光值",
    "white_balance_temperature_auto": "自动白平衡",
    "white_balance_temperature": "白平衡色温", "focus_auto": "自动对焦",
    "focus_absolute": "焦距", "zoom_absolute": "变焦",
    "white_balance_automatic": "自动白平衡", "gamma": "Gamma",
    "power_line_frequency": "工频抑制", "auto_exposure": "自动曝光模式",
    "exposure_time_absolute": "曝光时间", "exposure_dynamic_framerate": "动态帧率",
    "pan_absolute": "水平转动", "tilt_absolute": "垂直转动",
    "backlight_compensation": "背光补偿",
}


class CameraCaptureThread(QThread):
    """一个物理 V4L2 摄像头对应一个采集线程。"""

    frame_ready = Signal(str, object, float)
    camera_error = Signal(str, str)

    def __init__(self, device_path: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.device_path = device_path
        self._running = True

    def run(self) -> None:
        capture = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            self.camera_error.emit(self.device_path, "摄像头打开失败")
            return

        frame_count = 0
        fps_started = time.monotonic()
        last_emitted = 0.0
        last_fps = 0.0
        try:
            while self._running:
                ok, frame = capture.read()
                if not ok:
                    self.camera_error.emit(self.device_path, "读取画面失败")
                    self.msleep(30)
                    continue
                frame_count += 1
                elapsed = time.monotonic() - fps_started
                if elapsed >= 1.0:
                    last_fps = frame_count / elapsed
                    frame_count = 0
                    fps_started = time.monotonic()
                now = time.monotonic()
                if now - last_emitted >= 1 / 30:
                    self.frame_ready.emit(self.device_path, frame, last_fps)
                    last_emitted = now
        finally:
            capture.release()

    def stop(self) -> None:
        self._running = False


class MainWindow(QMainWindow):
    """固定双逻辑摄像头视图，右侧仅切换任务与输入源。"""

    def __init__(self) -> None:
        super().__init__()
        self.camera_devices: list[CameraDevice] = []
        self.capture_threads: dict[str, CameraCaptureThread] = {}
        self.physical_frames: dict[str, object] = {}
        self.physical_fps: dict[str, float] = {}
        self.camera_errors: dict[str, str] = {}
        self.control_widgets: dict[str, QWidget] = {}
        self.role_sources: dict[str, str | None] = {
            "recognition_camera": None,
            "navigation_camera": None,
        }
        self.current_task = "测试1"

        self.setWindowTitle("RoboCup 侦查机器人上位机")
        self.setMinimumSize(1024, 600)
        self.resize(1280, 720)
        self._build_ui()
        self._discover_and_start_cameras()

    @property
    def recognition_frame(self):
        """供后续识别算法使用的最新原始 OpenCV BGR Frame。"""
        source = self.role_sources["recognition_camera"]
        return self.physical_frames.get(source) if source else None

    @property
    def navigation_frame(self):
        """供后续导航算法使用的最新原始 OpenCV BGR Frame。"""
        source = self.role_sources["navigation_camera"]
        return self.physical_frames.get(source) if source else None

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(8)
        root.addLayout(self._build_header())
        root.addWidget(self._build_content(), 1)
        root.addWidget(self._build_footer())

    def _build_header(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        layout.setSpacing(8)
        title = QLabel("RoboCup 侦查机器人上位机")
        title.setObjectName("title")
        layout.addWidget(title)
        layout.addStretch(1)
        self.sidebar_buttons = QButtonGroup(self)
        self.sidebar_buttons.setExclusive(True)
        for index, text in enumerate((
            "切换", "调参", "摄像机参数", "运行", "任务", "视觉", "导航", "状态",
        )):
            button = QPushButton(text)
            button.setObjectName("navButton")
            button.setCheckable(True)
            button.setMinimumHeight(40)
            button.clicked.connect(lambda checked=False, page=index: self.pages.setCurrentIndex(page))
            self.sidebar_buttons.addButton(button, index)
            layout.addWidget(button)
        self.sidebar_buttons.button(0).setChecked(True)
        return layout

    def _build_content(self) -> QSplitter:
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        camera_area = self._build_camera_area()
        sidebar = self._build_sidebar()
        camera_area.setMinimumWidth(650)
        sidebar.setMinimumWidth(260)
        splitter.addWidget(camera_area)
        splitter.addWidget(sidebar)
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([700, 300])
        return splitter

    def _build_camera_area(self) -> QWidget:
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 6, 0)
        layout.setSpacing(8)
        recognition_panel, self.recognition_view, self.recognition_device, self.recognition_fps = self._camera_panel(
            "识别摄像头"
        )
        navigation_panel, self.navigation_view, self.navigation_device, self.navigation_fps = self._camera_panel(
            "导航摄像头"
        )
        layout.addWidget(recognition_panel, 1)
        layout.addWidget(navigation_panel, 1)
        return container

    def _camera_panel(self, title: str) -> tuple[QFrame, QLabel, QLabel, QLabel]:
        panel = QFrame()
        panel.setObjectName("cameraPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(4)
        header = QHBoxLayout()
        name = QLabel(title)
        name.setObjectName("cameraTitle")
        device = QLabel("设备：未连接")
        device.setObjectName("cameraDevice")
        fps = QLabel("-- FPS")
        fps.setObjectName("cameraFps")
        header.addWidget(name)
        header.addSpacing(10)
        header.addWidget(device)
        header.addStretch(1)
        header.addWidget(fps)
        view = QLabel("未连接")
        view.setObjectName("cameraView")
        view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        view.setMinimumHeight(170)
        view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout.addLayout(header)
        layout.addWidget(view, 1)
        return panel, view, device, fps

    def _build_sidebar(self) -> QFrame:
        panel = QFrame()
        panel.setObjectName("sidePanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        self.pages = QStackedWidget()
        self.pages.addWidget(self._build_switch_page())
        self.pages.addWidget(self._scrollable(self._build_input_page()))
        for builder in (
            self._build_camera_page, self._build_run_page, self._build_task_page,
            self._build_vision_page, self._build_navigation_page, self._build_status_page,
        ):
            self.pages.addWidget(self._scrollable(builder()))
        self.pages.currentChanged.connect(self._sync_sidebar_button)
        layout.addWidget(self.pages)
        return panel

    @staticmethod
    def _scrollable(widget: QWidget) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(widget)
        return scroll

    def _build_switch_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        self.task_group = QButtonGroup(self)
        self.task_group.setExclusive(True)
        for index, name in enumerate(("测试1", "测试2", "测试3", "正式任务")):
            button = QPushButton(name)
            button.setObjectName("taskButton")
            button.setCheckable(True)
            button.setMinimumHeight(72)
            button.clicked.connect(lambda checked=False, task=name: self._select_task(task))
            self.task_group.addButton(button, index)
            layout.addWidget(button, 1)
        self.task_group.button(0).setChecked(True)
        return page

    @staticmethod
    def _page() -> tuple[QWidget, QVBoxLayout]:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(10)
        return page, layout

    @staticmethod
    def _title(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("pageTitle")
        return label

    @staticmethod
    def _value(text: str = "--") -> QLabel:
        label = QLabel(text)
        label.setObjectName("value")
        label.setWordWrap(True)
        return label

    def _info_grid(self, rows: tuple[tuple[str, QLabel], ...]) -> QGridLayout:
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(7)
        for row, (name, value) in enumerate(rows):
            grid.addWidget(QLabel(name), row, 0)
            grid.addWidget(value, row, 1)
        grid.setColumnStretch(1, 1)
        return grid

    def _build_run_page(self) -> QWidget:
        page, layout = self._page()
        layout.addWidget(self._title("运行控制"))
        layout.addLayout(self._info_grid((
            ("系统状态", self._value("待机")), ("当前任务", self._value("--")),
            ("当前目标", self._value("--")), ("当前位置", self._value("--")),
            ("比赛计时", self._value("00:00")),
        )))
        for text, object_name in (("开始任务", "primaryButton"), ("停止任务", "stopButton"), ("软件急停", "emergencyButton")):
            button = QPushButton(text)
            button.setObjectName(object_name)
            button.setMinimumHeight(48)
            button.clicked.connect(lambda checked=False, name=text: self._log(f"收到{name}请求：控制接口未接入"))
            layout.addWidget(button)
        notice = QLabel("任务控制接口尚未接入")
        notice.setObjectName("notice")
        layout.addWidget(notice)
        layout.addStretch(1)
        return page

    def _build_task_page(self) -> QWidget:
        page, layout = self._page()
        layout.addWidget(self._title("比赛任务"))
        layout.addLayout(self._info_grid((
            ("巡逻进度", self._value("0 / 12")), ("侦查进度", self._value("0 / 8")),
            ("隧道通过", self._value("0 / 4")), ("障碍物", self._value("0 / 3")),
        )))
        layout.addWidget(QLabel("巡逻任务点（UID）"))
        points = QGridLayout()
        points.setSpacing(5)
        for index in range(12):
            point = QLabel(f"{index + 1:02d}\n待巡逻")
            point.setObjectName("patrolPoint")
            point.setAlignment(Qt.AlignmentFlag.AlignCenter)
            point.setMinimumHeight(42)
            points.addWidget(point, index // 4, index % 4)
        layout.addLayout(points)
        hint = QLabel("完成 12 个带 UID 的巡逻点、8 个侦查点、4 段隧道和 3 个障碍物后返回出发区。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addStretch(1)
        return page

    def _build_vision_page(self) -> QWidget:
        page, layout = self._page()
        layout.addWidget(self._title("视觉状态"))
        for heading, rows in (
            ("识别摄像头", (("目标检测", "未加载"), ("人脸识别", "未加载"), ("FPS", "--"))),
            ("导航摄像头", (("道路分割", "未加载"), ("路径辅助", "未加载"), ("FPS", "--"))),
        ):
            section = QLabel(heading)
            section.setObjectName("sectionTitle")
            layout.addWidget(section)
            layout.addLayout(self._info_grid(tuple((name, self._value(value)) for name, value in rows)))
        layout.addStretch(1)
        return page

    def _build_camera_page(self) -> QWidget:
        """保留独立的 V4L2 摄像机参数页；与“调参”页没有复用关系。"""
        page, layout = self._page()
        layout.addWidget(self._title("摄像机参数"))
        selector_row = QHBoxLayout()
        selector_row.addWidget(QLabel("正在调节"))
        self.camera_selector = QComboBox()
        self.camera_selector.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.camera_selector.currentIndexChanged.connect(self._show_selected_camera)
        selector_row.addWidget(self.camera_selector, 1)
        refresh = QPushButton("重新读取")
        refresh.clicked.connect(self._refresh_camera_controls)
        selector_row.addWidget(refresh)
        layout.addLayout(selector_row)
        self.device_path = self._value("--")
        self.device_name = self._value("--")
        self.device_driver = self._value("--")
        self.device_format = self._value("--")
        self.device_resolution = self._value("--")
        self.device_fps = self._value("--")
        layout.addLayout(self._info_grid((
            ("设备", self.device_path), ("名称", self.device_name),
            ("驱动", self.device_driver), ("当前格式", self.device_format),
            ("当前分辨率", self.device_resolution), ("当前 FPS", self.device_fps),
        )))
        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setObjectName("divider")
        layout.addWidget(divider)
        self.controls_title = QLabel("可调参数")
        self.controls_title.setObjectName("sectionTitle")
        layout.addWidget(self.controls_title)
        self.controls_container = QWidget()
        self.controls_layout = QVBoxLayout(self.controls_container)
        self.controls_layout.setContentsMargins(0, 0, 0, 0)
        self.controls_layout.setSpacing(8)
        layout.addWidget(self.controls_container)
        apply = QPushButton("应用到当前摄像头")
        apply.setObjectName("primaryButton")
        apply.setMinimumHeight(44)
        apply.clicked.connect(self._apply_controls)
        layout.addWidget(apply)
        self.camera_notice = QLabel("正在读取 V4L2 摄像头能力…")
        self.camera_notice.setObjectName("notice")
        self.camera_notice.setWordWrap(True)
        layout.addWidget(self.camera_notice)
        layout.addStretch(1)
        return page

    def _build_navigation_page(self) -> QWidget:
        page, layout = self._page()
        layout.addWidget(self._title("导航状态"))
        layout.addLayout(self._info_grid(tuple((name, self._value(value)) for name, value in (
            ("当前位置", "--"), ("目标点", "--"), ("方向", "--"),
            ("线速度", "--"), ("角速度", "--"), ("规划状态", "未启动"),
        ))))
        layout.addStretch(1)
        return page

    def _build_status_page(self) -> QWidget:
        page, layout = self._page()
        layout.addWidget(self._title("系统状态"))
        layout.addLayout(self._info_grid(tuple((name, self._value(value)) for name, value in (
            ("识别摄像头", "检测中"), ("导航摄像头", "检测中"),
            ("STM32", "未连接"), ("RKNN", "未加载"),
        ))))
        layout.addWidget(QLabel("运行日志"))
        self.log_output = QPlainTextEdit()
        self.log_output.setObjectName("logOutput")
        self.log_output.setReadOnly(True)
        self.log_output.setMinimumHeight(150)
        layout.addWidget(self.log_output, 1)
        return page

    def _build_input_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(12)
        title = QLabel("摄像头输入源")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        hint = QLabel("交换识别摄像头和导航摄像头当前绑定的物理 USB 摄像头。")
        hint.setObjectName("hint")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        button = QPushButton("切换识别 / 导航摄像头输入源")
        button.setObjectName("primaryButton")
        button.setMinimumHeight(58)
        button.clicked.connect(self._swap_camera_sources)
        layout.addWidget(button)
        self.mapping_notice = QLabel("正在检测物理摄像头…")
        self.mapping_notice.setObjectName("notice")
        self.mapping_notice.setWordWrap(True)
        layout.addWidget(self.mapping_notice)
        layout.addStretch(1)
        return page

    def _refresh_camera_controls(self) -> None:
        """重新读取 V4L2 设备与 controls；不会改变逻辑摄像头映射或采集线程。"""
        self.camera_devices = list_cameras()
        self.camera_selector.blockSignals(True)
        self.camera_selector.clear()
        for device in self.camera_devices:
            self.camera_selector.addItem(f"{device.path} · {device.name}", device.path)
        self.camera_selector.blockSignals(False)
        self._show_selected_camera()

    def _show_selected_camera(self) -> None:
        index = self.camera_selector.currentIndex()
        if index < 0 or index >= len(self.camera_devices):
            self.device_path.setText("--")
            self.device_name.setText("--")
            self.device_driver.setText("--")
            self.device_format.setText("--")
            self.device_resolution.setText("--")
            self.device_fps.setText("--")
            reason = "尚未发现可调摄像头" if v4l2_available() else "系统未安装 v4l2-ctl"
            self._clear_controls(reason)
            self.camera_notice.setText("请连接摄像头后点击“重新读取”。")
            return
        device = self.camera_devices[index]
        self.device_path.setText(device.path)
        self.device_name.setText(device.name)
        self.device_driver.setText(device.driver)
        self.device_format.setText(device.format_text)
        self.device_resolution.setText(device.resolution)
        self.device_fps.setText(device.fps)
        self._populate_controls(device)
        self.camera_notice.setText(f"已从 {device.path} 读取 {len(device.controls)} 个实际 V4L2 control。")

    def _clear_controls(self, text: str) -> None:
        while self.controls_layout.count():
            item = self.controls_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.control_widgets.clear()
        self.controls_title.setText("可调参数")
        empty = QLabel(text)
        empty.setObjectName("hint")
        empty.setWordWrap(True)
        self.controls_layout.addWidget(empty)

    def _populate_controls(self, device: CameraDevice) -> None:
        self._clear_controls("")
        if not device.controls:
            self.controls_layout.itemAt(0).widget().setText("该设备未返回可调 V4L2 control")
            return
        empty = self.controls_layout.takeAt(0).widget()
        if empty:
            empty.deleteLater()
        self.controls_title.setText(f"可调参数（{len(device.controls)}）")
        for control in device.controls:
            self._add_control(control)

    def _add_control(self, control: V4L2Control) -> None:
        label = QLabel(f"{CONTROL_NAMES.get(control.name, control.name.replace('_', ' ').title())}  ({control.name})")
        label.setObjectName("controlLabel")
        label.setWordWrap(True)
        self.controls_layout.addWidget(label)
        if control.kind in {"bool", "boolean"}:
            widget = QCheckBox("启用")
            widget.setChecked(bool(control.value))
            self.controls_layout.addWidget(widget)
        elif control.kind in {"menu", "intmenu"}:
            widget = QComboBox()
            for value, text in control.menu_items.items():
                widget.addItem(text, value)
            selected = widget.findData(control.value)
            if selected >= 0:
                widget.setCurrentIndex(selected)
            self.controls_layout.addWidget(widget)
        elif control.minimum is not None and control.maximum is not None:
            row = QHBoxLayout()
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(control.minimum, control.maximum)
            slider.setSingleStep(control.step or 1)
            slider.setValue(control.value if control.value is not None else control.minimum)
            spin = QSpinBox()
            spin.setRange(control.minimum, control.maximum)
            spin.setSingleStep(control.step or 1)
            spin.setValue(slider.value())
            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)
            row.addWidget(slider, 1)
            row.addWidget(spin)
            self.controls_layout.addLayout(row)
            widget = spin
        else:
            widget = QLabel("设备返回的 control 类型暂不支持编辑")
            widget.setObjectName("hint")
            widget.setWordWrap(True)
            self.controls_layout.addWidget(widget)
        if "inactive" in control.flags:
            widget.setEnabled(False)
            inactive = QLabel("当前模式下不可调（设备标记为 inactive）")
            inactive.setObjectName("hint")
            inactive.setWordWrap(True)
            self.controls_layout.addWidget(inactive)
            return
        self.control_widgets[control.name] = widget

    def _apply_controls(self) -> None:
        index = self.camera_selector.currentIndex()
        if index < 0 or index >= len(self.camera_devices):
            self.camera_notice.setText("没有可应用参数的摄像头。")
            return
        errors: list[str] = []
        for name, widget in self.control_widgets.items():
            if isinstance(widget, QCheckBox):
                value = int(widget.isChecked())
            elif isinstance(widget, QComboBox):
                value = int(widget.currentData())
            elif isinstance(widget, QSpinBox):
                value = widget.value()
            else:
                continue
            ok, message = set_control(self.camera_devices[index].path, name, value)
            if not ok:
                errors.append(f"{name}: {message}")
        if errors:
            self.camera_notice.setText("部分参数未应用：" + "；".join(errors))
            self._log("摄像机参数应用失败：" + "；".join(errors))
        else:
            self.camera_notice.setText("已将界面中的实际 V4L2 control 写入当前摄像头。")
            self._log("摄像机参数已应用")

    def _build_footer(self) -> QFrame:
        footer = QFrame()
        footer.setObjectName("footer")
        layout = QHBoxLayout(footer)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(0)
        self.footer_task = self._footer_label("任务：测试1")
        self.footer_recognition = self._footer_label("识别：未连接 · -- FPS")
        self.footer_navigation = self._footer_label("导航：未连接 · -- FPS")
        self.footer_stm32 = self._footer_label("STM32：未连接")
        self.footer_rknn = self._footer_label("RKNN：未加载")
        for label in (
            self.footer_task, self.footer_recognition, self.footer_navigation,
            self.footer_stm32, self.footer_rknn,
        ):
            layout.addWidget(label)
            layout.addWidget(QLabel("  |  "))
        layout.addStretch(1)
        return footer

    @staticmethod
    def _footer_label(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("footerLabel")
        return label

    def _discover_and_start_cameras(self) -> None:
        self.camera_devices = list_cameras()
        paths = [camera.path for camera in self.camera_devices]
        self.role_sources["recognition_camera"] = paths[0] if paths else None
        self.role_sources["navigation_camera"] = paths[1] if len(paths) > 1 else None
        for path in paths:
            thread = CameraCaptureThread(path, self)
            thread.frame_ready.connect(self._on_frame_ready)
            thread.camera_error.connect(self._on_camera_error)
            self.capture_threads[path] = thread
            thread.start()
        self._update_camera_ui()
        self._refresh_camera_controls()

    def _on_frame_ready(self, path: str, frame: object, fps: float) -> None:
        self.physical_frames[path] = frame
        self.physical_fps[path] = fps
        self.camera_errors.pop(path, None)
        if path == self.role_sources["recognition_camera"]:
            self._show_frame(self.recognition_view, frame)
        if path == self.role_sources["navigation_camera"]:
            self._show_frame(self.navigation_view, frame)
        self._update_camera_ui()

    def _on_camera_error(self, path: str, message: str) -> None:
        self.camera_errors[path] = message
        self._update_camera_ui()

    def _show_frame(self, view: QLabel, frame: object) -> None:
        if frame is None:
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        image = QImage(rgb.data, width, height, channels * width, QImage.Format.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(image).scaled(
            view.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
        )
        view.setPixmap(pixmap)
        view.setText("")

    def _update_camera_ui(self) -> None:
        self._update_role_ui("recognition_camera", self.recognition_view, self.recognition_device, self.recognition_fps)
        self._update_role_ui("navigation_camera", self.navigation_view, self.navigation_device, self.navigation_fps)
        self.footer_recognition.setText(f"识别：{self._role_status('recognition_camera')}")
        self.footer_navigation.setText(f"导航：{self._role_status('navigation_camera')}")
        recognition_path = self.role_sources["recognition_camera"] or "未连接"
        navigation_path = self.role_sources["navigation_camera"] or "未连接"
        self.mapping_notice.setText(f"识别摄像头 → {recognition_path}\n导航摄像头 → {navigation_path}")

    def _update_role_ui(self, role: str, view: QLabel, device_label: QLabel, fps_label: QLabel) -> None:
        path = self.role_sources[role]
        if not path:
            device_label.setText("设备：未连接")
            fps_label.setText("-- FPS")
            view.setPixmap(QPixmap())
            view.setText("未连接")
            return
        device_label.setText(f"设备：{path}")
        fps = self.physical_fps.get(path, 0.0)
        fps_label.setText(f"{fps:.1f} FPS" if fps else "-- FPS")
        if path in self.camera_errors:
            view.setPixmap(QPixmap())
            view.setText(self.camera_errors[path])
        elif path not in self.physical_frames:
            view.setPixmap(QPixmap())
            view.setText("正在打开摄像头…")

    def _role_status(self, role: str) -> str:
        path = self.role_sources[role]
        if not path:
            return "未连接 · -- FPS"
        if path in self.camera_errors:
            return f"{path} · {self.camera_errors[path]}"
        fps = self.physical_fps.get(path, 0.0)
        return f"{path} · {fps:.1f} FPS" if fps else f"{path} · -- FPS"

    def _select_task(self, task: str) -> None:
        self.current_task = task
        self.footer_task.setText(f"任务：{task}")

    def _swap_camera_sources(self) -> None:
        recognition = self.role_sources["recognition_camera"]
        navigation = self.role_sources["navigation_camera"]
        if not recognition or not navigation:
            self.mapping_notice.setText("需要检测到两路物理 USB 摄像头后，才能交换输入源。")
            return
        self.role_sources["recognition_camera"], self.role_sources["navigation_camera"] = navigation, recognition
        self._render_mapped_frames()
        self._update_camera_ui()

    def _render_mapped_frames(self) -> None:
        if self.recognition_frame is not None:
            self._show_frame(self.recognition_view, self.recognition_frame)
        if self.navigation_frame is not None:
            self._show_frame(self.navigation_view, self.navigation_frame)

    def _sync_sidebar_button(self, index: int) -> None:
        button = self.sidebar_buttons.button(index)
        if button:
            button.setChecked(True)

    def closeEvent(self, event) -> None:
        for thread in self.capture_threads.values():
            thread.stop()
        for thread in self.capture_threads.values():
            thread.wait(1500)
        event.accept()

    def _log(self, message: str) -> None:
        if hasattr(self, "log_output"):
            self.log_output.appendPlainText(message)
