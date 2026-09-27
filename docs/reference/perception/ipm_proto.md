# ipm_proto — IPM / 中心线 Python 原型

`vision/ipm_proto/`：前视道路 mask 的逆透视与中心线提取，供 Stage-1 导航几何验证。不依赖 YOLO / RKNN / 串口。

## Contents

- [运行](#运行)
- [模块](#模块)
- [配置](#配置)
- [相关文档](#相关文档)

## 运行

在仓库根目录：

```bash
PYTHONPATH=vision python3 -m ipm_proto.tests.test_pipeline
PYTHONPATH=vision python3 -m ipm_proto.tests.test_prior_temporal
PYTHONPATH=vision python3 -m ipm_proto synth --out /tmp/ipm_demo
PYTHONPATH=vision python3 -m ipm_proto run --image front.jpg --mask road.png \
  --config config/nav_camera.yaml --out /tmp/ipm_run
```

依赖：OpenCV、NumPy、PyYAML（板端一般已具备）。

## 模块

| 文件 | 职责 |
| --- | --- |
| `ipm.py` | `BevConfig`、`CameraExtrinsics`、`Ipm` |
| `centerline.py` | BEV mask → 米制中心线 |
| `prior.py` | 路宽先验（抑制同色过分割） |
| `temporal.py` | 中心线帧间 EMA |
| `synth.py` | 合成自洽直道前视图/mask |
| `cli.py` | `synth` / `run` 子命令 |

坐标系：地面 X 右、Y 前（米）；BEV 图下方为近端。

## 配置

权威运行配置：`config/nav_camera.yaml`。包内示例：`vision/ipm_proto/config_example.yaml`。可用相机高+俯仰，或 `image_points` + `ground_points` 四点单应。配置含 `road_prior` 时 `run` 自动启用路宽先验。

## 相关文档

- `docs/superpowers/specs/2026-08-29-visual-nav-road-follow-design.md`
- `docs/superpowers/specs/2026-08-29-ipm-centerline-proto-design.md`
- `docs/superpowers/specs/2026-09-03-white-on-white-road-perception-design.md`
- `docs/reference/perception/road_seg_deploy.md`
