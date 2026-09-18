# pi_upper 刀具识别部署包

仓库集成说明：代码已合入对应目录，原项目根README补充入口，vision/__init__.py保持原有内容。models/knife/dinov3_448_fp16_v1/中的模型、模板和参考图随本分支提交，克隆后可直接执行下文核验。bundle_manifest.json和build_bundle_manifest.py属于独立bushu交付包，未合入本仓库。实拍测试结果见[实拍报告](reality_20260918/REPORT.md)。

本目录是可合入 `pi_upper` 的增量部署包，默认模型为 **DINOv3 448 FP16**。它在RK3588S冻结测试集上实测正确1637/1640（99.817%），困难集297/300（99.000%）。选择它是因为比赛侦查错误播报会扣分，准确性优先；ArcFace继续使用CPU，DINO主要占用NPU且CPU后处理较轻。

当前包已包含模型、10类板端生产模板、固定预处理、单图CLI、单worker本机HTTP服务、配置、核验脚本和systemd示例。生产模板已在目标RK3588上以本包预处理和实际RKNN重算；对保存的1640条板端测试descriptor复核为1637条正确，且相对正式Benchmark模板没有任何Top-1变化。**没有自动修改你的 `pi_upper` 仓库或GUI**；正式接入仍需将GUI的 `recognition_frame` 异步提交到本服务，并处理请求ID、相机代次和过期结果。

## 目录

```text
config/knife.json                         运行配置，默认enabled=false
models/knife/dinov3_448_fp16_v1/         模型、模板、注册原图与发布清单
vision/knife/                             预处理、RKNN引擎、CLI和HTTP服务
gui/knife_client.py                       PySide6异步客户端
scripts/export_templates.py               模板导出工具
scripts/verify_bundle.py                  只读完整性核验
bundle_manifest.json                      整个交付目录的文件大小与SHA-256
tests/test_templates.py                   不调用NPU的模板测试
tests/board_smoke.py                      板端10张注册图短验收
tests/compare_frozen_rgb.py               迁移预处理逐像素回归
tests/compare_reference_implementation.py 迁移代码与实验源码对照
systemd/knife-recognition.service.example 常驻服务示例
```

## 板端环境

已验证环境：Python 3.9.23、NumPy 1.26.4、OpenCV 4.11.0、RKNN Toolkit Lite2 2.3.2、RKNPU driver 0.9.6。当前测试板是RK3588S Orange Pi 5 Pro；换到Orange Pi 5 Plus后至少重新做单图和整机混合负载验收。

在板端检出本分支后进入 `pi_upper` 根目录。固定版本RKNN文件约50.9MiB，已随仓库提供。

```bash
cd /实际的/pi_upper
KNIFE_PY=/home/orangepi/knife_embedding_ab/.conda-env/bin/python3.9
"$KNIFE_PY" scripts/verify_bundle.py
"$KNIFE_PY" tests/test_templates.py
"$KNIFE_PY" tests/board_smoke.py
```

若使用独立新环境，先安装 `requirements-knife.txt`，再安装与系统匹配且已验证的RKNN Lite2 2.3.2 wheel。不要在比赛板上盲目升级RKNN runtime或驱动。

## 单图测试

用包内注册图验证模型加载、预处理和NPU推理；这不是成功率Benchmark：

```bash
cd /实际的/pi_upper
KNIFE_PY=/home/orangepi/knife_embedding_ab/.conda-env/bin/python3.9
"$KNIFE_PY" -m vision.knife.cli \
  models/knife/dinov3_448_fp16_v1/references/knife_01/reference.png \
  --config config/knife.json
```

预期Top-1为 `knife_01`。模型返回的是候选类别和余弦分数；当前没有未知刀/无刀阈值，不把余弦分数写成“成功概率”。

`templates.json` 是目标板生产预处理重新生成的默认模板；
`templates_frozen_benchmark.json` 仅保留作来源审计，不由配置加载。板端OpenCV重新执行原算法时，
与冻结RGB缓存有最多2灰度级的插值舍入差异，但迁移代码和原实验源码在目标板上逐像素完全一致。

## 启动本机服务

```bash
cd /实际的/pi_upper
KNIFE_PY=/home/orangepi/knife_embedding_ab/.conda-env/bin/python3.9
"$KNIFE_PY" -m vision.knife.service \
  --config config/knife.json --host 127.0.0.1 --port 20005
```

另开终端：

```bash
curl http://127.0.0.1:20005/health
curl -X POST http://127.0.0.1:20005/api/v1/knife/recognize \
  -F 'image=@models/knife/dinov3_448_fp16_v1/references/knife_01/reference.png' \
  -F 'request_id=manual-001' -F 'frame_id=1' -F 'camera_epoch=0'
```

服务固定单worker、一次只处理一个请求；忙时返回409，不积压60FPS摄像头旧帧。默认只监听127.0.0.1。systemd示例中的占位符必须替换后才能安装。

## 接入比赛程序

从现有 `MainWindow.recognition_frame` 获取原始BGR帧，不使用QPixmap截图，也不写死 `/dev/video0`。PNG编码和HTTP等待均放到Qt worker线程，主线程只接收结果信号。切换摄像头或任务时递增 `camera_epoch`；结果回来后同时核对 `request_id`、`frame_id`、`camera_epoch`，过期结果直接丢弃。

`gui/knife_client.py` 已提供 `KnifeClient`。在 `MainWindow` 中创建一次实例、连接
`result_ready/request_failed/busy_changed` 信号，并在调试按钮或停车识别状态调用
`submit(...)`。当前主窗口还没有统一frame_id和采集时间戳，需要在摄像头线程发布帧时一并记录；
不要用收到按钮点击的时刻伪造采集时间。关闭窗口时调用 `shutdown()`，其返回False时只记录超时，
不要在Qt主线程无限等待。

建议只在机器人到侦查点、停车稳定后触发刀具识别。行驶阶段导航优先，不持续运行DINO。ArcFace的CPU服务和刀具NPU服务可以并行，但上线前必须做DINO+ArcFace+导航的整机混合负载测试。不要默认启动三个RKNN实例；它们不等于把单张图自动拆到三个核。

配置中 `enabled=false` 是安全默认值。GUI适配完成、现场验证通过后由上层读取该开关启用；本服务本身不会擅自操作摄像头、串口、播报或机器人运动。

## 预处理约束

查询图和注册图统一执行：alpha优先/简单背景分割、最大连通域、PCA长轴对齐、尖端朝右、占448画布92%的等比例letterbox、127灰背景、BGR转RGB、ImageNet归一化一次。模型输入通过Lite2的NHWC接口，输出384维向量做FP32 L2和10模板余弦匹配。

整幅复杂场景直接resize到448会破坏实验条件。比赛现场应先保证侧向相机画面中只有一把目标刀具或提供稳定ROI。当前最大连通域不是通用目标检测器。

## 验收重点

- 现场固定清单按真实标签统计成功率，超时、质量拒绝和无响应保留在任务成功率分母。
- 错误播报会扣分；没有独立validation阈值前，结果只作为候选，不对无刀/未知刀强制播报。
- 混合负载下检查ArcFace延迟、导航周期、刀具总时延、内存、温度和三个NPU核心。
- 三核查看命令：`watch -n 0.2 'cat /proc/rknpu/load'`。
- 关闭刀具功能后，原双摄像头、ArcFace、导航和UART行为必须保持不变。

详细设计依据在实验项目：`D:\RoboCom\knife_embedding_ab\reports\final\pi_upper_knife_deployment.md`；完整板端测试：`D:\RoboCom\knife_embedding_ab\results\rknn\board\retest_20260917_223622\REPORT.md`。
