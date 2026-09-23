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

预处理执行背景分割、最大连通域、PCA长轴对齐、尖端朝右、448画布letterbox和ImageNet归一化；输出384维FP32单位向量，与10个模板做余弦匹配。该分割不是通用目标检测，现场需要稳定ROI或单刀背景。

templates.json是目标板生产模板，templates_frozen_benchmark.json只作审计。模型和模板必须配套，用发布清单核验SHA-256。固定模型按用户要求作为models/默认忽略规则的例外随仓库提交。

准确率优先选DINO：历史冻结集1637/1640，困难集297/300。core_mask=0_1_2不保证负载均匀。ArcFace继续CPU推理，仍需整机混合负载验收。

## Testing

本地核验和板端短测见模块README。[5张实拍记录](knife-reality.md)中Top-1正确率5/5，Top-2命中率5/5，平均465.85ms；不能外推为现场100%成功率，未测试无刀/未知刀误报率。

原始JSON和运行日志保留在本地实验目录。仓库只保留可复用脚本与手写结果摘要。
