# 测试2：AprilTag 地面四点标定

在“切换”页选择“测试2”，会进入“运行”页。点击“AprilTag 四点标定 / 查看 H”：

1. 从当前逻辑**导航摄像头**取得最新原始 BGR 帧并冻结，不额外打开摄像头。
2. 检测唯一一个 `tag36h11`、ID `0` 标签的四个黑色外沿角点。
3. 按实测外沿边长 **0.134 m** 求 `H_img_to_bev`，显示编号四角、BEV 和矩阵记录。
4. 确认结果后点击“保存 H”。“重新取帧并检测”用于重新拍取一帧；检测失败不会保存旧矩阵。

标签必须平整贴地、保留白边、完整可见。四点对应关系使用标签解码方向，不按画面位置重新排序，因此标签旋转仍保持同一坐标约定。未去畸变；四点可拟合不代表标签之外的尺度精度通过验收。

## 坐标和预览

标定结果以**标签中心**为原点，X 指向标签解码方向的右侧，Y 指向标签上侧，单位米。
四角顺序为标签的左上、右上、右下、左下（在画面中不一定处于这些位置）。

独立预览采用目前已核对的导航 BEV 窗口大小及像素尺度，将范围平移到标签中心：
当前 X 为 −0.50～+0.50 m，Y 为 −0.40～+0.40 m，1 cm/像素，输出宽 100、高 80 像素。
这让标签显示在预览中央，不假设标签到车辆的距离。该 H 不是现有导航车辆坐标系中的 H，不能未经坐标对齐直接用于控制。
预览参数位于 `apriltag_calibration.PREVIEW_BEV`，不会随导航配置修改而自动变化。

## 持久化

点击保存后写入仓库 `config/bev_calibration.json`，板端为 `/home/orangepi/pi_upper/config/bev_calibration.json`。
记录版本、UTC 标定时间、实际相机来源及原图尺寸、标签参数、角点顺序、图像/地面四点、BEV 参数、输出尺寸和完整精度 `H_img_to_bev`。
图像空间标记为 `raw_distorted`，坐标基准标记为 `tag_center`。

使用标准库 JSON，不要求 GUI 环境安装 PyYAML。写入采用同目录临时文件及原子替换；已有文件会先归档为 `bev_calibration_UTC时间.json`。
保存前重新校验相机来源及分辨率。没有真实相机采集时不创建虚假的标定结果文件。
“读取已保存 H”可以在程序重启后查看记录，即使暂时没有相机帧也可读取。

模块提供 `load_calibration(path)`，加载时校验矩阵与四点对应关系、尺度和尺寸的一致性。
本次仅标定、预览和持久化，**导航仍由原来的相机参数计算 H**，不修改 `nav_camera.yaml` 或导航算法。

## 离线验证

不连接相机、不加载模型、不使用 UART。测试只向系统临时目录写入合成标定记录：

```bash
cd /home/orangepi/pi_upper
QT_QPA_PLATFORM=offscreen PYTHONPATH=gui PYTHONDONTWRITEBYTECODE=1 \
  /home/orangepi/miniconda3/envs/pi_upper/bin/python -B -m unittest discover \
  -s gui/tests -p test_apriltag_calibration.py
```
