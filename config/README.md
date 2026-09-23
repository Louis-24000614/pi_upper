# config/

运行时加载的配置（与 `docs/` 手写说明分离）。

`knife.json` — DINOv3刀具识别运行配置（模型、模板、NPU核掩码及默认关闭开关）；见[模块说明](../docs/reference/perception/knife.md)。

| 文件 | 用途 |
| --- | --- |
| `nav_topology.yaml` | 赛题图 3 先验路网（`行_列`：`_1` 左点、`_2`/`_3` 中间双十字、`_4` 右点）；设计见 `docs/nav.md` |
| `knife.json` | DINOv3 刀具识别：模型路径、模板、NPU 核掩码、`enabled` 默认关闭 |
| `nav_camera.yaml` | 导航前视安装、BEV 窗、路宽先验与时间平滑；白底白沿见 `docs/superpowers/specs/2026-09-03-white-on-white-road-perception-design.md` |
| `road_seg_train.yaml` | 主机 YOLO-seg 训练增强/数据比例提示（板端不加载） |
| `servo.yaml` | 180° 舵机 sysfs PWM 芯片、通道、周期与脉宽端点（C++ `LoadConfig`）；见 `docs/reference/hw/servo.md` |
