# Repository Layout

`pi_upper` 是香橙派上位机仓库，按"一个文件夹一个关注点"组织：每个可独立运行的模块一个顶层文件夹，文档集中在 `docs/`。

```mermaid
flowchart TD
  root["pi_upper/"]
  root --> uart["uart/<br/>下位机串口通信模块（C++）"]
  root --> vision["vision/<br/>视觉相关模块"]
  vision --> af["arcface-lite/<br/>轻量人脸识别服务（FastAPI + InsightFace）"]
  root --> docs["docs/<br/>手写文档（规范、API 契约、模块文档）"]
  root --> models["models/<br/>模型文件（不入 git）"]
```

当前顶层目录：

`uart/` — 与 STM32 下位机的串口通信模块，内部分 `proto/`（纯协议编解码）与 `link/`（串口、时钟、会话状态机）两层。模块文档见 `docs/reference/comm/uart.md`，线协议见下位机仓库的 `UART_PROTOCOL.md`，上位机侧实现约定见 `docs/api/uart.md`。

`vision/arcface-lite/` — 人脸识别服务与测试客户端，模块文档见 `docs/reference/perception/arcface-lite.md`，上手步骤见其 `README.md`。

`docs/` — 手写文档，文档地图见 `docs/doc_layout.md`。

`models/` — 模型文件（RKNN/ONNX），体积大不入库，各机器自行准备。

后续新增的代码模块归属上位机 C++/Qt 单体工程，物理目录结构随骨架搭建时确定并更新本文；视觉类独立服务归入 `vision/`（如 arcface-lite），并按 `docs/conventions.md` 的要求在 `docs/reference/<layer>/` 下补一篇模块文档。
