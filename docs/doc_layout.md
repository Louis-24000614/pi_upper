# Doc Layout

本仓库手写文档的地图，每个文档一行介绍。

- `docs/conventions.md` — 全仓库编码与文档规范（提交前缀、模块文档结构、API 契约格式）。
- `docs/architecture/overview.md` — 上位机总体架构规划：模块划分、通信边界、技术选型与开发顺序。
- `docs/architecture/layout.md` — 仓库目录结构与顶层目录职责。
- `docs/nav.md` — 导航与障碍重规划：拓扑全局 + BEV 局部 + 寻线执行的分层设计。
- `config/nav_topology.yaml` — 赛题图 3 先验拓扑运行配置（`行_列`：左右巡逻格 + 中间双十字、边与隧道初标）。
- `docs/reference/navigation/topo_proto.md` — 拓扑 Dijkstra 原型：加载 YAML、封边重搜、CLI。
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

各模块上手步骤（安装、注册人脸、运行）写在模块目录自己的 `README.md` 里，例如 `vision/arcface-lite/README.md`。
