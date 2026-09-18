# config/

运行时加载的配置（与 `docs/` 手写说明分离）。

`knife.json` — DINOv3刀具识别运行配置（模型、模板、NPU核掩码及默认关闭开关）；见[模块说明](../docs/reference/perception/knife.md)。

| 文件 | 用途 |
| --- | --- |
| `nav_topology.yaml` | 赛题图 3 先验路网（`行_列`：`_1` 左点、`_2`/`_3` 中间双十字、`_4` 右点）；设计见 `docs/nav.md` |
