# Knife

## Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [API](#api)
- [Design](#design)
- [Testing](#testing)

## Overview

DINOv3 448 FP16刀具识别独立Python服务，使用RK3588 NPU。安装与运行见[README](../../../vision/knife/README.md)。输出10类候选，不直接控制相机、运动、串口或播报。

## Architecture

vision/knife/保存代码、scripts/、tests/、systemd/和requirements.txt。GUI适配器归gui/knife_client.py，配置归config/knife.json，发布资产归models/knife/dinov3_448_fp16_v1/。

单worker、单RKNN实例，推理加锁，HTTP忙时返回409。默认监听127.0.0.1:20005。主窗口尚未接线，enabled默认false，上层负责触发识别和过滤过期结果。

## API

KnifeRecognizer提供embed、recognize、warmup和close。HTTP接口返回Top-1/Top-2类别、余弦分数、分差和分阶段毫秒耗时，详见[契约](../../api/knife.md)。

## Design

默认先将整帧最长边缩到640像素，在浅色纸面上用局部对比和最大连通域估计单刀框；无浅色纸面时尝试简单前景框。找不到可靠框时退回整帧，并在响应中标记。该轻量规则只定位取景区域，不提供通用目标检测或无刀判断；应继续用实体刀具和不同背景验证。

框内预处理执行背景分割、最大连通域、PCA长轴对齐、尖端朝右、448画布letterbox和ImageNet归一化；输出384维FP32单位向量，与10个模板做余弦匹配。

自动或调用方框选单刀ROI后，若常规分割的前景比例低于20%、首选分数低于0.8，额外保留完整ROI并尝试四个直角方向。只有原图路线的最高分比常规路线高0.07以上、前两类分差至少0.03，才采用补救候选。这样可以缓解浅色刀刃印在白纸上时被分割丢弃的问题；这些门槛不是有刀/未知刀的拒识阈值，结果仍是候选。

templates.json是目标板生产模板，templates_frozen_benchmark.json只作审计。模型和模板必须配套，用发布清单核验SHA-256。固定模型按用户要求作为models/默认忽略规则的例外随仓库提交。

准确率优先选DINO：历史冻结集1637/1640，困难集297/300。core_mask=0_1_2不保证负载均匀。ArcFace继续CPU推理，仍需整机混合负载验收。

## Testing

本地核验和板端短测见模块README。[5张实拍记录](knife-reality.md)中Top-1正确率5/5，Top-2命中率5/5，平均465.85ms；不能外推为现场100%成功率，未测试无刀/未知刀误报率。

原始JSON和运行日志保留在本地实验目录。仓库只保留可复用脚本与手写结果摘要。
