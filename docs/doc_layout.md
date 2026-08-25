# Doc Layout

本仓库手写文档的地图，每个文档一行介绍。

- `docs/conventions.md` — 全仓库编码与文档规范（提交前缀、模块文档结构、API 契约格式）。
- `docs/architecture/overview.md` — 上位机总体架构规划：模块划分、通信边界、技术选型与开发顺序。
- `docs/architecture/layout.md` — 仓库目录结构与顶层目录职责。
- `docs/api/face.md` — arcface-lite 服务的 HTTP REST 与 WebSocket 线协议契约。
- `docs/reference/perception/arcface-lite.md` — arcface-lite 模块文档：架构、设计取舍、API 与测试方法。

各模块上手步骤（安装、注册人脸、运行）写在模块目录自己的 `README.md` 里，例如 `arcface-lite/README.md`。
