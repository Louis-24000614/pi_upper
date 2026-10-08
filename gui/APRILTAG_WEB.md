# 局域网 AprilTag 四点标定

使用已确认的 tag36h11 ID 0，实测黑色外沿边长 13.4 cm，不包括白边。
把标签平整贴在道路地面，相机与小车保持固定，四个黑色外沿角点完整可见。

在 .33 上运行（不需要 sudo，不调频，也不启动 UART 或导航）：

```sh
cd /home/orangepi/pi_upper
/home/orangepi/pyside6-venv/bin/python -u -B gui/apriltag_web.py \
  --device /dev/video0 --width 1280 --height 720 --port 8081
```

同一局域网浏览器打开 `http://192.168.92.33:8081/`。
实时画面标出识别到的唯一 ID 0 四角；点击“冻结当前帧并检测四角”，查看原图、角点和俯视预览。
确认标签贴地、角点正确且相机未移动后，点击“保存本次标定”。没有标签、重复标签、过期画面、读帧失败或分辨率改变时，不允许保存无效草稿。
新一次检测失败会清除上一草稿，防止保存旧结果。

每次独立保存到 `data/calibration/apriltag/<UTC时间_草稿ID>/`，包含：

- `calibration.json`：四个原始图像角点、标签地面角点、完整精度单应矩阵、相机来源与尺寸。
- `raw.png`：冻结的原始 BGR 图像，不含绘制的角点。
- `corners.png`：四角编号预览。
- `bev.png`：标签坐标俯视预览。

复用现有 `apriltag_calibration.py`，不修改其实现。本次四点坐标以标签中心为原点，X 指向标签右、Y 指向标签上；图像未去畸变。
标定不写入 `config/bev_calibration.json`，也不改 `config/nav_camera.yaml` 或 `config/culvert.yaml`。
页面没有应用配置的接口，结果的 `applied` 固定为 false。

后续要换成涵洞测距，必须对齐导航相机光心垂足的地面原点。可选区域允许填写标签中心的实测横向和前向距离（厘米）。
仅当标签 0→1 边位于远端、标签坐标 X/Y 分别与车右/车前对齐时，勾选方向确认。
填写后只保存独立的 `vehicle_reference` 候选四点；`verified` 仍为 false，不替换任何控制参数。
留空时只保存标签坐标；不能把标签中心直接当作相机原点。
单个 13.4 cm 标签的四点拟合不代表整个 0.20～0.60 m 测距范围已通过验收，后续需使用独立距离点验证尺度与畸变影响。

点击“停止推流并释放相机”，或者在启动终端按 Ctrl+C，结束服务。标定期间相机由本服务占用，停止后才能重新运行导航。

离线验证只使用合成标签、本机 HTTP 和系统临时目录，不打开任何硬件：

```sh
PYTHONPATH=gui /home/orangepi/pyside6-venv/bin/python -B -m unittest discover \
  -s gui/tests -p test_apriltag_web.py
```
