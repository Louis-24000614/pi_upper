# arcface-lite

轻量级人脸识别服务，针对 ARM CPU（香橙派 RK3588）优化。从 Radish 项目的 `service/arcface` 提取而来，去掉了 ROS 依赖，可独立运行。

模型包使用 InsightFace `buffalo_sc`（SCRFD-500M 检测 + MobileFaceNet 识别），纯 CPU 推理，比默认的 `buffalo_l` 在 ARM 上快约 5–10 倍。**注意：buffalo_sc 的 embedding 与 buffalo_l 不兼容，换模型后必须重新注册人脸。**

## Contents

- [环境要求](#环境要求)
- [安装](#安装)
- [注册人脸](#注册人脸)
- [运行服务](#运行服务)
- [运行客户端](#运行客户端)
- [性能测试](#性能测试)
- [常见问题](#常见问题)

## 环境要求

硬件：香橙派（RK3588）或任意 Linux 主机；识别服务本身不需要摄像头，摄像头接在客户端一侧（USB 摄像头或 RealSense D435i）。

系统：Ubuntu 22.04，Python 3.10（已验证组合）。其他版本未测试，`onnxruntime<1.20` 与 `numpy<2.0` 的约束来自 `requirements.txt`，请勿随意升级。

## 安装

```bash
cd arcface-lite
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

首次运行时 InsightFace 会自动联网下载 `buffalo_sc` 模型包到 `~/.insightface/models/`，只需下载一次。离线机器可先从能联网的机器拷贝该目录。

RealSense 客户端额外需要 `pyrealsense2`，它不在 `requirements.txt` 中：

```bash
.venv/bin/pip install pyrealsense2
```

x86 主机上直接装即可；aarch64 上 PyPI 通常没有预编译 wheel，需要从源码编译 librealsense（参考 <https://github.com/IntelRealSense/librealsense>）。只用 USB 摄像头的话不需要装它。

## 注册人脸

人脸库存储在本地 `face_db.npz`，**不入库**（见 `.gitignore`），每台机器需要自行注册。两种方式：

方式一，离线批量注册（适合已有照片）。目录结构为 `root_dir/姓名/*.jpg`：

```bash
.venv/bin/python engine.py register photos/ face_db.npz
# photos/
# ├── zhangsan/1.jpg, 2.jpg ...
# └── lisi/1.jpg ...
```

方式二，在线注册（服务运行中，拍一张录一张）：

```bash
curl -X POST http://127.0.0.1:20004/api/v1/register \
  -F "image=@photo.jpg" -F "name=zhangsan"
```

同名重复注册会把新旧 embedding 平均融合，适合多角度补录。

## 运行服务

```bash
.venv/bin/python server.py --host 0.0.0.0 --port 20004
```

可选环境变量：`DB_PATH`（人脸库路径，默认 `./face_db.npz`）、`FACE_THRESHOLD`（余弦相似度阈值，默认 0.45）、`DET_W`/`DET_H`（检测输入尺寸，默认 320）、`MODEL_NAME`（默认 `buffalo_sc`）。

健康检查：`curl http://127.0.0.1:20004/health`。完整接口见 [../docs/api/face.md](../docs/api/face.md)。

## 运行客户端

USB 摄像头：

```bash
.venv/bin/python usb_camera_client.py --uri ws://<服务器IP>:20004/ws/recognize --show
```

RealSense D435i：

```bash
.venv/bin/python realsense_client.py --uri ws://<服务器IP>:20004/ws/recognize --show
```

`--every N` 控制每 N 帧识别一次（默认 3），其余帧直接显示上一轮的识别结果。无显示器的机器去掉 `--show`。

## 性能测试

```bash
.venv/bin/python bench.py photo.jpg face_db.npz 10
```

输出模型加载耗时与每帧识别耗时（avg/min/max）。

## 常见问题

识别结果全是 `Unknown`：先确认 `face_db.npz` 存在且注册时用的是同一个模型包（`MODEL_NAME`）。 buffalo_sc 与 buffalo_l 的库混用必然全部 Unknown。

注册时报 `No face detected` 或 `Face too small`：照片里人脸太小或太模糊，换近距离、正脸、清晰的照片。注册流程自带多尺度金字塔检测，稍远的脸也能找到，但对模糊照片无能为力。

`Cannot open /dev/video0`：摄像头没插好或设备号不对，试 `--device 1`。`ls /dev/video*` 可查看当前设备。
