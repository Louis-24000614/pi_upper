"""本机刀具识别HTTP服务；默认只监听127.0.0.1。"""

from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager
import os
from pathlib import Path
from typing import Optional

import cv2
from fastapi import FastAPI, File, Form, HTTPException, UploadFile
import numpy as np

from .engine import KnifeRecognizer

_engine: Optional[KnifeRecognizer] = None
_inference_lock = asyncio.Lock()
_config_path = Path(os.environ.get("KNIFE_CONFIG", "config/knife.json"))
MAX_UPLOAD_BYTES = 12 * 1024 * 1024
MAX_DECODED_PIXELS = 16_000_000


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """模型生命周期与服务进程一致，避免每个请求重复加载。"""
    global _engine
    _engine = KnifeRecognizer(_config_path)
    yield
    _engine.close()
    _engine = None


app = FastAPI(title="Knife recognition", version="1", lifespan=lifespan)


@app.get("/health")
async def health() -> dict:
    """返回模型是否就绪和当前忙状态。"""
    return {
        "status": "ok" if _engine is not None else "not_ready",
        "ready": _engine is not None,
        "busy": _inference_lock.locked(),
        "model_id": _engine.config.get("model_id") if _engine else None,
    }


@app.post("/api/v1/knife/recognize")
async def recognize(
    image: UploadFile = File(...),
    request_id: str = Form(""),
    frame_id: int = Form(-1),
    camera_epoch: int = Form(0),
    captured_monotonic_ns: int = Form(0),
) -> dict:
    """串行处理一张PNG/JPEG；忙时立即返回409，避免积压过期相机帧。"""
    if _engine is None:
        raise HTTPException(status_code=503, detail="model_not_ready")
    if _inference_lock.locked():
        raise HTTPException(status_code=409, detail="model_busy")
    content = await image.read(MAX_UPLOAD_BYTES + 1)
    if len(content) > MAX_UPLOAD_BYTES:
        raise HTTPException(status_code=413, detail="image_too_large")
    encoded = np.frombuffer(content, dtype=np.uint8)
    frame = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if frame is None or frame.ndim != 3 or frame.shape[2] not in (3, 4):
        raise HTTPException(status_code=400, detail="invalid_image")
    if frame.shape[0] * frame.shape[1] > MAX_DECODED_PIXELS:
        raise HTTPException(status_code=413, detail="decoded_image_too_large")
    try:
        async with _inference_lock:
            data = await asyncio.to_thread(_engine.recognize, frame, request_id)
    except ValueError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    except RuntimeError as error:
        raise HTTPException(status_code=500, detail=str(error)) from error
    data.update(
        frame_id=frame_id,
        camera_epoch=camera_epoch,
        captured_monotonic_ns=captured_monotonic_ns,
    )
    return {"status": "success", "message": "ok", "data": data}


def main() -> None:
    """以单worker启动服务；多worker会重复加载RKNN实例。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("config/knife.json"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=20005)
    args = parser.parse_args()
    global _config_path
    _config_path = args.config.resolve()
    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port, workers=1)


if __name__ == "__main__":
    main()
