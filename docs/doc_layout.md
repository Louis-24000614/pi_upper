# Doc Layout

本仓库手写文档的地图，每个文档一行介绍。

- `vision/knife/README.md` — 刀具识别安装、启动和板端验收。
- `docs/reference/perception/knife.md` — 刀具模块架构、设计和测试。
- `docs/reference/perception/knife-reality.md` — 5张实拍图手写测试记录。
- `docs/api/knife.md` — 刀具识别HTTP接口契约。

- `docs/conventions.md` — 全仓库编码与文档规范（提交前缀、模块文档结构、API 契约格式）。
- `docs/architecture/overview.md` — 上位机总体架构规划：模块划分、通信边界、技术选型与开发顺序。
- `docs/architecture/layout.md` — 仓库目录结构与顶层目录职责。
- `docs/nav.md` — 导航与障碍重规划：拓扑全局 + BEV 局部 + 寻线执行；`CMD_VEL` / `MOTION_ACTION` 分工与视觉校准 ICM42688 相对航向。
- `config/nav_topology.yaml` — 赛题图 3 先验拓扑运行配置（`行_列`：左右巡逻格 + 中间双十字、边与隧道初标）。
- `docs/reference/navigation/upper-planner.md` — 上层路径规划：12 巡逻点、四段隧道、封边重搜、到点转向和记分记忆。
- `docs/reference/navigation/chinese-postman.md` — 中国邮递员道路覆盖：欧拉条件、奇点最小权匹配、Hierholzer、在线封边重规划及赛题语义。
- `docs/reference/navigation/postman-integration-plan.md` — 中国邮递员算法实车接入：边级状态、遇障整边封闭、原路倒车、UART 确认与分阶段验收。
- `agent/README.md` — Agent 与 Navigation 的边界、道路覆盖规划入口及后续扩展位置。
- `vision/obstacle/README.md` — 障碍检测框到整边硬堵塞事件：空间过滤、跨帧确认、锁存与重置接口。
- `docs/reference/navigation/topo_proto.md` — 拓扑 Dijkstra 原型：加载 YAML、封边重搜、CLI。
- `docs/reference/navigation/road-follow-actions.md` — 道路跟随有限动作：路口/RFID 转向、遇障倒车、UART 文本接口、运行与测试。
- `docs/reference/navigation/topology-rfid-navigation.md` — 固定物理节点与现场卡号分离；侧边角锁存、正前方 mask 检测带、唯一一次 20 cm 接近及拓扑状态推进。
- `docs/reference/navigation/vision-odom-junction-turn.md` — 道路分割发现路口、BEV 正前方检测带、编码器/IMU 固定 20 cm 补盲与 90° 转弯交接。
- `navigation/README.md` — navigation 目录说明（含 `topo_proto/`）。
- `docs/api/face.md` — arcface-lite 服务的 HTTP REST 与 WebSocket 线协议契约。
- `docs/api/uart.md` — 下位机串口通信在上位机侧的实现约定（线协议以下位机仓库的 `UART_PROTOCOL.md` 为权威）。
- `docs/reference/comm/uart.md` — uart 模块文档：分层架构、设计取舍、API 与测试方法。
- `docs/reference/perception/arcface-lite.md` — arcface-lite 模块文档：架构、设计取舍、API 与测试方法。
- `docs/reference/perception/ipm_proto.md` — IPM/中心线 Python 原型：运行方式、模块与配置。
- `gui/README.md` — PySide6 调试界面：双路预览、逻辑摄像头映射、V4L2 调参、道路分割拍照采集。
- `docs/superpowers/specs/2026-08-29-visual-nav-road-follow-design.md` — Stage-1 视觉寻线设计规格。
- `docs/superpowers/specs/2026-08-29-ipm-centerline-proto-design.md` — IPM/中心线原型设计规格。
- `docs/superpowers/specs/2026-09-02-mission-topology-and-gui-design.md` — 任务拓扑策略与 GUI 展示设计（点序、UID、选岔与界面字段）。
- `docs/superpowers/specs/2026-09-03-white-on-white-road-perception-design.md` — 白底白沿道路感知：同色矮沿数据、相机姿态、路宽先验与双目后备。
- `docs/reference/perception/road_seg_deploy.md` — 道路分割 RKNN 部署与亮场验收清单。
- `docs/reference/perception/stereo_curb_fallback.md` — 单目不稳时的双目测沿评估清单。
- `config/nav_camera.yaml` — 导航前视与 BEV/路宽先验运行配置。
- `config/road_seg_train.yaml` — 道路分割训练增强提示。
- `config/servo.yaml` — 舵机 PWM 芯片路径、通道与脉宽端点。
- `docs/reference/hw/servo.md` — Pin 7 / PWM14_M2 接线、overlay、C++ CLI 与角度校准。
- `servo/README.md` — 舵机模块运行命令。

各模块上手步骤（安装、注册人脸、运行）写在模块目录自己的 `README.md` 里，例如 `vision/arcface-lite/README.md`。
