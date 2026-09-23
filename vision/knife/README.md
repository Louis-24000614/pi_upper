# Knife Recognition

DINOv3 448 FP16刀具识别模块。代码、依赖、维护脚本、测试和systemd示例集中在本目录；运行配置在config/knife.json，发布资产在models/knife/。

设计见[模块说明](../../docs/reference/perception/knife.md)，接口见[HTTP契约](../../docs/api/knife.md)，实拍结果见[测试记录](../../docs/reference/perception/knife-reality.md)。

## Install

所有命令从pi_upper仓库根执行。已验证Python 3.9.23、NumPy 1.26.4、OpenCV 4.11.0、RKNN Lite2/runtime 2.3.2、驱动0.9.6。先安装匹配系统的RKNN Lite2 wheel，再安装模块依赖。

```bash
python3 -m pip install -r vision/knife/requirements.txt
python3 vision/knife/scripts/verify_bundle.py
python3 vision/knife/tests/test_templates.py
```

模型、生产模板、审计模板和10张参考图随仓库保存在models/knife/dinov3_448_fp16_v1/。

## Run

```bash
python3 -m vision.knife.cli models/knife/dinov3_448_fp16_v1/references/knife_01/reference.png --config config/knife.json
python3 -m vision.knife.service --config config/knife.json --host 127.0.0.1 --port 20005
```

健康检查：`curl http://127.0.0.1:20005/health`。上传测试：

```bash
curl -X POST http://127.0.0.1:20005/api/v1/knife/recognize -F 'image=@models/knife/dinov3_448_fp16_v1/references/knife_01/reference.png'
```

常驻服务示例位于vision/knife/systemd/knife-recognition.service.example，替换用户、工作目录和Python路径后安装。

## GUI

gui/knife_client.py提供异步客户端，主窗口尚未接线。调用方传入recognition_frame原始BGR帧及真实采集时间，核对request_id、frame_id和camera_epoch，丢弃过期结果，退出时调用shutdown()。

config/knife.json中的enabled=false由上层读取，手动启动服务不受该开关控制。建议停车后触发，ArcFace保留CPU执行，上线前做导航与识别混合负载验收。

## Testing

```bash
python3 vision/knife/tests/board_smoke.py
watch -n 0.2 'cat /proc/rknpu/load'
```

注册图自匹配不代表实拍成功率。tests/内还保留预处理回归和实拍测时脚本。reality.py的--root指定测试工作目录（含images/、config/、models/），运行时PYTHONPATH应包含仓库根；生成的result.json留在本地，不提交Git。

scripts/export_board_templates.py用于板端重新生成生产模板，scripts/export_templates.py用于导出历史审计模板；常规部署无需运行，更换模板后需更新manifest校验信息。
