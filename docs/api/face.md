# face — 人脸识别服务接口契约

`server.py` 对外暴露的 HTTP REST 与 WebSocket 线协议。供客户端作者使用，不需要打开服务端代码。

## Contents

- [Overview](#overview)
- [Authentication](#authentication)
- [Interfaces](#interfaces)
- [Routes](#routes)
  - [GET /health](#get-health)
  - [POST /api/v1/register](#post-apiv1register)
  - [POST /api/v1/recognize](#post-apiv1recognize)
  - [GET /api/v1/faces](#get-apiv1faces)
  - [DELETE /api/v1/faces](#delete-apiv1faces)
  - [WebSocket /ws/recognize](#websocket-wsrecognize)

## Overview

服务拥有人脸库（本地 `face_db.npz`），提供注册、识别、名单管理三类能力。默认监听 `0.0.0.0:20004`。识别请求在服务端串行执行（全局推理锁），并发客户端会排队而不是并行推理。

服务不做的事：不提供图像存储（识别完即弃）、不做活体检测、不维护注册人之外的任何身份信息。

## Authentication

无认证。服务假设部署在受信局域网内；如需暴露到更大范围，应在外层加反向代理与访问控制。

## Interfaces

REST 请求体：`multipart/form-data`（带图片的端点）或 `application/json`（DELETE）。

统一响应信封：

```json
{ "status": "success", "message": "ok", "data": { } }
```

成功时 `status` 为 `"success"`，业务数据在 `data` 里。失败时 `status` 为 `"error"`，`message` 为人可读原因，`data` 为 `null`。参数缺失/格式错误由 FastAPI 直接返回 422（信封外，FastAPI 默认结构）。

识别结果项（REST 与 WebSocket 共用字段名）：

```json
{
  "name": "zhangsan",
  "score": 0.62,
  "bbox": [120, 80, 260, 250],
  "direction_px": 87.5
}
```

`name` 为注册名或 `"Unknown"`；`score` 为余弦相似度（约 0–1，越大越像，阈值默认 0.45）；`bbox` 为 `[x1, y1, x2, y2]` 像素坐标；`direction_px` 为图像中心 x 减人脸中心 x（正 = 人脸偏左），单位像素。

## Routes

### GET /health

探活。无参数。

```json
{ "status": "ok", "db_exists": true }
```

注意此端点不走统一信封，直接返回上述扁平结构。

### POST /api/v1/register

注册人脸。`multipart/form-data`：`image` 为图片文件（JPEG/PNG 等 OpenCV 可解码格式），`name` 为身份名（字符串，首尾空白会被去掉）。取画面中最大的人脸入库；同名再注册时 embedding 与旧值平均融合，可用于多角度补录。

成功 200：

```json
{ "status": "success", "message": "Registered zhangsan successfully", "data": { "name": "zhangsan" } }
```

失败：400（未检测到人脸 / 人脸太小 / 检测分过低，见 `message`）；422（缺 `image` 或 `name`）。

### POST /api/v1/recognize

单帧识别（调试/一次性场景；实时流请用 WebSocket）。`multipart/form-data`：`image` 为图片文件。

成功 200：

```json
{
  "status": "success",
  "message": "ok",
  "data": {
    "face_count": 1,
    "results": [
      { "name": "zhangsan", "score": 0.62, "bbox": [120, 80, 260, 250], "direction_px": 87.5 }
    ]
  }
}
```

失败：404（人脸库不存在，尚未注册任何人）；422（图片无法解码）。

### GET /api/v1/faces

列出已注册身份名。无参数。

```json
{ "status": "success", "message": "ok", "data": { "names": ["zhangsan", "lisi"] } }
```

### DELETE /api/v1/faces

删除一个身份。`application/json` 请求体：

```json
{ "name": "zhangsan" }
```

成功 200 的 `data` 为 `{ "deleted": "zhangsan" }`。删除最后一个身份时人脸库文件会被移除（后续识别返回 404）。失败：404（名字不存在或库不存在）；422（缺 `name`）。

### WebSocket /ws/recognize

实时识别流。连接后客户端持续发送**二进制帧**（一帧 = 一张完整 JPEG 图片），每收到一帧，服务端回一条 JSON 文本消息。逐帧一问一答，不支持乱序并发。

成功响应与 `POST /api/v1/recognize` 的 `data` 结构相同：

```json
{
  "status": "success",
  "message": "ok",
  "data": { "face_count": 1, "results": [ { "name": "zhangsan", "score": 0.62, "bbox": [120, 80, 260, 250], "direction_px": 87.5 } ] }
}
```

错误时 `status` 为 `"error"`，`message` 取值：`invalid image`（帧无法解码为 BGR 图）、`database not found`（尚未注册人脸）、`service not initialized`（模型未加载完）。错误消息不中断连接，客户端可继续发下一帧。

节流由客户端负责：参考实现（`usb_camera_client.py` / `realsense_client.py`）按 `--every N` 每 N 帧发一帧，其余帧沿用上次结果显示。

**Related:** 模块设计见 [../reference/perception/arcface-lite.md](../reference/perception/arcface-lite.md)。
