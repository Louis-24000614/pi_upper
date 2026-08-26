# 侦察机器人上位机（pi_upper）

本仓库为侦察机器人上位机程序，运行在 **香橙派 Orange Pi 5 Plus**（RK3588）上，主要负责视觉识别、导航规划、状态管理、Qt 调试界面以及与 STM32 下位机通信。

## Contents

- [运行环境](#运行环境)
- [硬件说明](#硬件说明)
- [上位机主要功能](#上位机主要功能)
- [软件结构](#软件结构)
- [构建与运行](#构建与运行)
- [与下位机通信](#与下位机通信)
- [文档](#文档)

## 运行环境

- 主控板：Orange Pi 5 Plus（RK3588，NPU 6 TOPS）
- 系统：Ubuntu 22.04
- 开发语言：C++ / Qt
- 主要依赖：OpenCV、Qt、RKNN Runtime（librknnrt）

## 硬件说明

本项目使用两个 USB 摄像头，分工明确。

### 前视导航摄像头

用于道路识别、障碍物识别和视觉导航。

- 130 万像素，USB 2.0，**全局快门**，最高 180 FPS
- 支持 1280×1024 / 1280×960 / 1280×720，视场角约 63.3°，UVC 兼容

### 侧向识别摄像头

用于侧边目标识别、嫌疑人识别和物体识别。

- 最高分辨率 3852×2172，USB 2.0，滚动快门
- 支持 MJPG / YUY2，常用模式 1280×720 @ 60 FPS
- 支持自动曝光、自动白平衡、自动增益

### 其他

- 下位机：STM32H743VIT6（IMU 为 ICM42688，由下位机侧负责解算）
- 上下位机链路：杜邦线直连 UART，香橙派 40 针 TX/RX/GND 对接 STM32 的 USART3（PD8/PD9），3.3 V TTL，921600 8N1
- 扬声器：语音播报输出（USB 声卡或 3.5mm）

## 上位机主要功能

- Qt 调试界面
- 双摄像头图像采集
- YOLO-seg 道路分割
- YOLO-seg 障碍物识别
- 物体识别
- ArcFace 嫌疑人人脸识别
- 视觉导航
- BEV 鸟瞰图转换
- 局部栅格地图生成
- 局部路径规划
- 机器人任务状态机
- 语音播报
- 与 STM32 下位机串口通信

## 软件结构

上位机采用 **"C++ 后端 + Qt 调试界面"** 的单体应用结构，不使用 ROS。Qt 只负责显示和调试，不直接承担核心控制逻辑——采集、推理、规划、通信各自跑在独立线程中，脱离界面也能运行。

推理加速走 RK3588 的 NPU：YOLO 系列模型在训练主机上训练，导出 ONNX 后经 RKNN-Toolkit2 量化转换为 `.rknn`，板端用 RKNN Runtime 加载推理。模型文件统一放 `models/`，不提交到仓库。

## 构建与运行

CMake + Qt 标准工作流（项目骨架建立后补充具体说明）：

```bash
cmake -B build
cmake --build build -j
```

## 与下位机通信

杜邦线直连 UART，921600 8N1，二进制帧 `COBS(帧头 || payload || CRC32C) || 0x00`，帧头魔数 `0xA55A`。上位机建链后取 `boot_id`，经操作者按键确认拿到 `arm_token`，再以 50 Hz 下发 `CMD_VEL`（零速也必须持续发，250 ms 断流下位机自动安全停车）；下位机回传里程计、IMU、系统状态与故障事件。

线协议以下位机仓库的 `UART_PROTOCOL.md` 为唯一权威，上位机侧的实现约定见 `docs/api/uart.md`。

## 文档

手写文档统一放 `docs/`，文档地图见 `docs/doc_layout.md`：

- `docs/conventions.md` — 编码与文档规范
- `docs/architecture/overview.md` — 总体架构规划
- `docs/api/face.md` — 人脸识别接口契约
- `docs/api/uart.md` — 下位机串口通信的上位机侧实现约定

### 当前仓库状态

`vision/arcface-lite/` 为嫌疑人脸识别的 Python 参考实现（InsightFace buffalo_sc，CPU 可跑，含 HTTP/WebSocket 接口与文档），用于先行验证识别效果与建库流程。主体工程按纯 C++/Qt 推进后，人脸识别可继续以独立服务方式被调用（HTTP），或后续用 RKNN 重写并入视觉模块。
