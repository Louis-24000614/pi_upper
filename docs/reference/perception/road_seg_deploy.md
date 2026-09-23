# 道路分割 RKNN 部署与亮场验收

配合 [`2026-09-03-white-on-white-road-perception-design.md`](../../superpowers/specs/2026-09-03-white-on-white-road-perception-design.md) 与 Stage-1 寻线规格。

## Contents

- [导出路径](#导出路径)
- [板端几何](#板端几何)
- [亮场验收](#亮场验收)

## 导出路径

1. 主机：同色主集训练 YOLOv11n-seg（增强见 `config/road_seg_train.yaml`）。
2. 导出 ONNX（输入边长优先 640）。
3. RKNN-Toolkit2 量化 → `models/*.rknn`（**不提交仓库**）。
4. 板端加载后输出二值 `road` mask（可选 `curb` 并入非可行驶）。

## 板端几何

```bash
# 单元：路宽先验 + 时间平滑
PYTHONPATH=vision python3 -m ipm_proto.tests.test_pipeline
PYTHONPATH=vision python3 -m ipm_proto.tests.test_prior_temporal

# 实拍 mask 跑通（配置用权威相机文件）
PYTHONPATH=vision python3 -m ipm_proto run \
  --image front.jpg --mask road.png \
  --config config/nav_camera.yaml --out /tmp/ipm_run
```

`config/nav_camera.yaml` 含 `road_prior` 时 CLI 自动启用路宽先验。多帧环内持有 `CenterlineSmoother`，不要每帧新建。

## 亮场验收

| # | 项 | 通过标准 |
| --- | --- | --- |
| 1 | 直道 | 大致居中，不持续蹭侧沿 |
| 2 | 缓弯 | 跟线，不冲出底面 |
| 3 | 失效 | 遮挡/大偏航 → 减速停车 |
| 4 | 几何单测 | `test_pipeline` + `test_prior_temporal` 通过 |
| 5 | 延迟 | 曝光到 `(v,ω)` 目标 &lt; 100 ms（可按实车调） |

不通过且已排除控制问题时，再开 [`stereo_curb_fallback.md`](stereo_curb_fallback.md)。
