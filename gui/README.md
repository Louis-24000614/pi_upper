# RoboCup 公共安全赛项侦查机器人上位机

这是运行在 Linux / Raspberry Pi 上的 RoboCup 侦查机器人上位机。当前项目使用 Python、PySide6、OpenCV 和 V4L2，主要完成比赛界面、真实 USB 摄像头预览、任务选择、摄像头逻辑角色切换以及摄像机参数调节。

测试1已接入刀具 DINOv3 与 ArcFace 人脸并行识别，部署与身份库说明见 [TEST1.md](TEST1.md)。YOLO 道路分割、UART、UID、路径规划和正式比赛状态机尚未接入，这些区域显示真实的“未加载”或“未连接”状态。

当前显示的预览面板标题栏有「拍照」。按下后保存该逻辑摄像头的**原始 BGR 帧**（请求 1280×720 采集分辨率，以设备实际返回为准）到仓库 `data/road/`；板端路径为 `/home/orangepi/pi_upper/data/road/`，识别摄像头照片以 `rec_` 开头。单摄像头时画面挂在「识别摄像头」上，用那一侧的按钮即可。

## 界面结构

界面由四个始终存在的区域组成：

```text
┌─────────────────────────────────────────────────────────────┐
│ 标题      [切换] [调参] [摄像机参数] [运行] [任务] [状态] │
├─────────────────────────────────────────┬───────────────────┤
│ 识别摄像头 / 导航摄像头（单路切换）     │                   │
│                                         │    右侧功能页     │
│          实时画面区域                   │                   │
│                                         │                   │
├─────────────────────────────────────────┴───────────────────┤
│ 任务 | 识别设备/FPS | 导航设备/FPS | STM32 | RKNN          │
└─────────────────────────────────────────────────────────────┘
```

- 顶部标题栏负责切换右侧功能页。
- 左侧摄像头区域固定存在，不会因为右侧换页而消失。
- 左侧一次显示一个逻辑摄像头，画面占满原先两个预览面板的区域。默认显示识别摄像头；在“调参”页可切换到导航摄像头，再按一次切回。当前面板标题栏右侧有「拍照」。
- 右侧侧栏约占主区域的 30%。
- 底部状态栏持续显示比赛中最需要快速确认的信息。

摄像头预览等比例缩进画面框，完整保留帧的四边，不拉伸或裁剪。画面比例与窗口不同时会留黑边；窗口大小变化后会重新适配。

## 顶部页面说明

### 切换

用于选择当前比赛任务：

- 测试1
- 测试2
- 测试3
- 正式任务

四个按钮纵向等比例填满右侧区域。被选中的任务会高亮，并同步写入底部状态栏。当前只记录任务名称，不执行具体业务流程。

### 调参

这是比赛算法和运行逻辑的调试入口，不等同于“摄像机参数”。

“切换识别 / 导航摄像头输入源”交换逻辑角色与物理摄像头之间的映射。

例如：

```text
切换前：识别 -> Camera A，导航 -> Camera B
切换后：识别 -> Camera B，导航 -> Camera A
```

交换后会同步改变：

- 左侧当前预览对应的物理画面；
- `recognition_frame` 和 `navigation_frame` 的原始数据来源；
- 底部摄像头设备状态；
- 后续识别和导航算法应使用的数据源。

只有检测到两个独立的物理视频采集节点时才能交换。单摄像头情况下会给出提示，不会把同一摄像头的 metadata 节点误认为第二路画面。

“屏幕画面切换”只决定左侧显示哪一个逻辑摄像头。按一次显示导航摄像头，再按一次显示识别摄像头；两路采集持续进行，逻辑映射、识别和导航数据源不变。单摄像头时也可以切到导航面板查看“未连接”状态。

### 摄像机参数

专门调节 USB 摄像头自身的 V4L2 参数，与“调参”页面完全独立。

页面会根据摄像头实际返回的能力动态显示：

- 设备路径、名称和驱动；
- 当前像素格式、分辨率和目标 FPS；
- 亮度、对比度、饱和度、增益等整数参数；
- 自动曝光、自动白平衡等开关；
- 曝光模式、工频抑制等菜单参数；
- 设备实际支持的其他 V4L2 controls。

被设备标记为 `inactive` 的参数会保留显示但禁止编辑。例如启用自动曝光时，手动曝光时间通常不可修改。

点击“应用到当前摄像头”才会通过 `v4l2-ctl` 写入设备。侧栏只允许上下滚动，不会出现横向滚动条。

### 运行

保留开始任务、停止任务和软件急停入口，同时显示系统状态、当前任务、目标、位置和比赛计时。

测试1的开始、停止按钮控制双识别；「框选单把刀」从全屏原始画面选出刀具取景区，人脸仍处理整帧。STM32 / UART 控制接口尚未接入，软件急停只停止识别，不能制动底盘。

### 任务

显示比赛任务总体信息：

- 12 个 UID 巡逻点；
- 8 个侦查点；
- 4 段隧道；
- 3 个障碍物；
- 完成后返回出发区。

目前显示的是待执行状态和任务规模，还没有接入 UID 读取、目标识别或正式任务流程。

### 状态

集中展示原来分散在“视觉”和“导航”页面的信息，包含：

- 识别摄像头、导航摄像头、STM32 和 RKNN 设备状态；
- 目标检测、人脸识别、道路分割和路径辅助状态；
- 当前位置、目标点、方向、速度和规划状态；
- 运行日志。

后续视觉算法和导航模块接入后，应直接更新这里已有的状态控件，不再新增独立的顶部“视觉”或“导航”页面。

## 摄像头角色与采集架构

程序区分“物理摄像头”和“逻辑摄像头”。

```text
物理摄像头：/dev/video0、/dev/video2 等真实 V4L2 采集节点
逻辑摄像头：识别摄像头、导航摄像头
```

程序启动时通过 `camera_controls.list_cameras()` 枚举设备，只保留 `uvcvideo` 驱动的真实 `Video Capture` 节点，排除 metadata、ISP 和编解码节点。

每个物理摄像头对应一个 `CameraCaptureThread`：

```text
USB Camera
    ↓
cv2.VideoCapture + CAP_V4L2
    ↓
独立 QThread 持续读取
    ├── latest_frame：最新原始 BGR Frame，供后续算法使用
    └── frame_ready：发送给 Qt 主线程用于预览
```

采集线程优先请求：

```text
MJPG
1280 × 720
60 FPS
```

实际生效的格式、分辨率和目标帧率会从 OpenCV / V4L2 重新读取，不会只显示请求值。设备不支持时允许驱动回退，摄像头打不开也不会导致整个程序退出。

物理采集目标为 60 FPS，但 UI 预览信号限制在约 30 FPS，避免高分辨率 QImage/QPixmap 缩放堵塞 Qt 主线程。`latest_frame` 仍在每次成功采集后更新，因此算法读取的是最新原始帧，不是缩放后的预览图。

OpenCV 画面显示链路为：

```text
BGR numpy.ndarray
    ↓ cv2.cvtColor
RGB
    ↓
QImage
    ↓
QPixmap
    ↓ 等比例缩进、完整显示
CameraPreview QLabel
```

`recognition_frame` 和 `navigation_frame` 是根据当前逻辑映射动态返回的属性。后续算法不应在代码里写死 `/dev/video0`。

拍照走 `snap.save_frame`：从上述属性取最新帧、拷贝后写 PNG。文件名 `nav_YYYYMMDD_HHMMSS_mmm.png` 或 `rec_…png`。成功或失败写入状态页运行日志。PNG 被根目录 `.gitignore` 排除，不提交仓库。

## PySide6 UI 架构

主窗口是 `MainWindow(QMainWindow)`，使用布局管理器组织界面，没有使用固定坐标。

```text
MainWindow
└── centralWidget / QVBoxLayout
    ├── 顶部 QHBoxLayout
    │   ├── 标题
    │   └── 独占 QButtonGroup
    ├── 水平 QSplitter
    │   ├── 摄像头区域 / QStackedWidget
    │   │   ├── 识别摄像头 QFrame（默认）
    │   │   └── 导航摄像头 QFrame
    │   └── 右侧 QStackedWidget
    │       ├── 切换页
    │       ├── 调参页
    │       ├── 摄像机参数页
    │       ├── 运行页
    │       ├── 任务页
    │       └── 状态页
    └── 底部状态栏 QFrame
```

顶部按钮通过索引切换右侧 `QStackedWidget`。摄像头区域和底部状态栏位于 `QStackedWidget` 外部，所以换页不会销毁或隐藏视频画面。

除“切换”页需要四个任务按钮完整填满侧栏外，其他页面使用 `QScrollArea` 包装，并强制关闭水平滚动条。窗口按当前显示器的可用区域最大化，最低内容尺寸为 800×480，以便在 1024×600 的 HDMI 屏上完整显示。

## 文件说明

```text
main.py               程序入口、QApplication 创建、加载 QSS
main_window.py        主窗口、页面、采集线程、逻辑映射、画面显示和拍照入口
camera_controls.py    V4L2 设备发现、能力解析和 control 写入
snap.py               原始 BGR 帧落盘（道路分割训练样本）
snap_view.py          全屏完整画面拍照。先关掉上位机再运行
snap_view.sh          全屏拍照启动脚本（HDMI 模式与 run.sh 相同）
style.qss             深色工业控制台样式
tests/test_snap.py    落盘路径与写盘结果的无硬件单测
tests/test_snap_view.py  全屏拍照的设备选择与不裁切缩放
```

## 运行

在项目目录中：

推荐一键启动（会先把 HDMI 设成可用模式）：

```bash
cd /home/orangepi/Desktop/pi_upper/gui
./run.sh
```

或手动：

```bash
export DISPLAY=:0
export XAUTHORITY=/home/orangepi/.Xauthority
export QT_QPA_PLATFORM=xcb
xrandr --output HDMI-1 --mode 1280x720 --rate 60
xrandr --output HDMI-1 --set color_format rgb
cd /home/orangepi/Desktop/pi_upper/gui
/home/orangepi/pyside6-venv/bin/python main.py
```

`python main.py` 成功后终端不会再打印、也不会回到提示符，这是 Qt 事件循环在前台运行，不是卡死。关掉窗口或 `Ctrl+C` 才会退出。请用 `pyside6-venv` 解释器，系统 `python` 没有 PySide6。

本机是 X11/XFCE，用 `DISPLAY=:0`，不要设 `WAYLAND_DISPLAY`。

**HDMI 小屏注意：** 外接屏 EDID 常把 `1024x600` 标成首选，但 Orange Pi 5 Plus 的 Rockchip HDMI PHY 在该模式下会出现 `hdptx phy lane can't ready` / `phy poweron failed`，软件截图正常、实体屏全黑。请用 `1280x720` 或 `1920x1080`（`dmesg` 里应出现 `hdptx phy lane locked`）。并建议 `color_format=rgb`。

**开机默认分辨率：** 当前开机默认会落到无信号的 `1024x600`。安装一次后即可每次开机自动用 `1280x720`：

```bash
sudo bash /home/orangepi/Desktop/pi_upper/gui/scripts/install-hdmi-lcd.sh
sudo reboot
```

该脚本会改 `/boot/orangepiEnv.txt`（内核 `video=`）、LightDM `display-setup-script`、Xorg PreferredMode，以及 XFCE 自启动。

## 道路数据采集

赛题考核纯视觉道路分割。场地可能是 **白底白沿**（路面/侧沿/岛区同色，只靠高度差），采集约定：

1. 构图可用上位机“调参”页切换到导航摄像头，左侧预览会完整显示画面；也可用下述全屏拍照程序。相机姿态参考 `config/nav_camera.yaml`。
2. 在同色矮沿练车场上覆盖直道、弯道、偏航、远近、阴影与过曝，按空格逐张保存。黑胶带边线图不要当主集。
3. 在 `data/road/` 得到 PNG 后，用多边形标 `road`：**只标可行驶底面**；侧沿顶面/立面、岛区、场外不要标成路。可选第二类 `curb`。
4. 交叉口把可见可行驶底面都标为 `road`。
5. 细节见 `data/road/README.md` 与 `docs/superpowers/specs/2026-09-03-white-on-white-road-perception-design.md`。

全屏程序独占摄像头，先关掉上位机：

```bash
cd /home/orangepi/pi_upper/gui
./snap_view.sh
```

画面按比例缩进屏幕，四周留黑边，不裁切。空格或回车保存原始帧到 `data/road/`，Esc 退出。两路摄像头时 `1` 是识别（`rec_`）、`2` 是导航（`nav_`）。指定设备：

```bash
./snap_view.sh --device /dev/video0 --role navigation_camera
```

```bash
cd gui
PYTHONPATH=. python -m pytest tests/test_snap.py
```

## 当前未实现

- YOLO / YOLO-seg 道路与障碍推理；
- STM32 / UART 通信；
- UID / PN532 读取；
- BEV、道路识别和路径规划；
- 测试2、测试3的具体业务，以及测试1的底盘控制与任务流程；
- 正式任务状态机和语音播报。

这些功能接入时应继续沿用现有逻辑摄像头映射、状态页和底部状态栏，不要直接绑定固定的 `/dev/video*` 节点。
