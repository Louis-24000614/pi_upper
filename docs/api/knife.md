# Knife API

## Contents

- [Overview](#overview)
- [Authentication](#authentication)
- [Interfaces](#interfaces)
- [GET /health](#get-health)
- [POST /api/v1/knife/recognize](#post-apiv1kniferecognize)

## Overview

本机识别服务默认127.0.0.1:20005。模型常驻，不保存上传图；部署见[README](../../vision/knife/README.md)。

## Authentication

无身份认证，默认仅本机访问。

## Interfaces

上传为multipart/form-data，响应为JSON。余弦分数不是概率；decision固定candidate。错误响应为FastAPI的detail字段。

## GET /health

无参数，200响应包含status（ok/not_ready）、ready、busy、model_id。客户端须检查ready。

## POST /api/v1/knife/recognize

必填文件字段image；可选request_id字符串、frame_id整数（默认-1）、camera_epoch整数（默认0）、captured_monotonic_ns整数（默认0）。这些元数据原样回传；单调时间戳仅在同一时钟域内比较。

编码上限12MiB，解码上限1600万像素，须为3或4通道。成功200，外层status=success、message=ok、data为结果。data包括元数据、model_id、template_version、decision、top1_class、top1_score、top2_class、top2_score、margin、quality_ok、preprocess和timing_ms。类别为knife_01至knife_10。timing_ms包含preprocess、normalize、npu_api、postprocess、matching、total，单位ms。

错误：400 invalid_image；409 model_busy；413 image_too_large或decoded_image_too_large；422请求字段或预处理校验失败；503 model_not_ready；500推理异常。忙时等待下一张有效帧，不积压旧请求。
