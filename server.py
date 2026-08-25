"""Standalone face recognition HTTP + WebSocket server (ARM/CPU, no ROS dependency).

Usage:
  python server.py [--host 0.0.0.0] [--port 20004]

Env config (all optional):
  DB_PATH         face database file          (default: ./face_db.npz)
  FACE_THRESHOLD  cosine similarity threshold (default: 0.45)
  DET_W / DET_H   detection input size        (default: 320)
  MODEL_NAME      InsightFace model pack      (default: buffalo_sc)

Endpoints:
  WebSocket /ws/recognize   — binary JPEG frames in, JSON recognition results out
  POST      /api/v1/register  — multipart image + name
  POST      /api/v1/recognize — multipart image (one-shot / debug)
  GET       /api/v1/faces     — list registered names
  DELETE    /api/v1/faces     — JSON body {"name": "..."}
  GET       /health
"""

from __future__ import annotations

import argparse
import asyncio
import os
from contextlib import asynccontextmanager
from typing import Optional

import cv2
import numpy as np
from fastapi import FastAPI, File, Form, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from engine import FacialRecognition

_inference_lock = asyncio.Lock()
_face: Optional[FacialRecognition] = None


def _get_env_float(key: str, default: float) -> float:
    raw = os.environ.get(key)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _get_env_int(key: str, default: int) -> int:
    raw = os.environ.get(key)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        return default


DB_PATH = os.environ.get(
    "DB_PATH", os.path.join(os.path.dirname(os.path.abspath(__file__)), "face_db.npz")
)
THRESHOLD = _get_env_float("FACE_THRESHOLD", 0.45)
DET_SIZE = (_get_env_int("DET_W", 320), _get_env_int("DET_H", 320))
MODEL_NAME = os.environ.get("MODEL_NAME", "buffalo_sc")


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _face
    print(f"[server] Loading model pack {MODEL_NAME!r}, det_size={DET_SIZE} ...")
    _face = FacialRecognition(model_name=MODEL_NAME)
    _face.init_app(det_size=DET_SIZE)
    print("[server] Ready.")
    yield
    _face = None


app = FastAPI(title="Face recognition (lite)", lifespan=lifespan)


def decode_upload_to_bgr(content: bytes) -> Optional[np.ndarray]:
    """Decode raw image bytes to BGR uint8 (H, W, 3)."""
    if not content:
        return None
    arr = np.frombuffer(content, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return None
    if img.ndim != 3 or img.shape[2] != 3:
        return None
    return img


@app.get("/health")
def health():
    return {"status": "ok", "db_exists": os.path.exists(DB_PATH)}


# ---------------------------------------------------------------------------
# WebSocket recognition endpoint
# ---------------------------------------------------------------------------


@app.websocket("/ws/recognize")
async def ws_recognize(ws: WebSocket) -> None:
    await ws.accept()
    try:
        while True:
            data = await ws.receive_bytes()
            if _face is None:
                await ws.send_json({"status": "error", "message": "service not initialized"})
                continue

            bgr = decode_upload_to_bgr(data)
            if bgr is None:
                await ws.send_json({"status": "error", "message": "invalid image"})
                continue

            if not os.path.exists(DB_PATH):
                await ws.send_json({"status": "error", "message": "database not found"})
                continue

            async with _inference_lock:

                def _run():
                    _, results = _face.recognize_from_frame(bgr, DB_PATH, threshold=THRESHOLD)
                    return results

                results = await asyncio.to_thread(_run)

            await ws.send_json({
                "status": "success",
                "message": "ok",
                "data": {
                    "face_count": len(results),
                    "results": [
                        {
                            "name": r.get("name", "Unknown"),
                            "score": r.get("score", 0.0),
                            "bbox": list(r.get("bbox", [])),
                            "direction_px": r.get("direction_px", 0.0),
                        }
                        for r in results
                    ],
                },
            })
    except WebSocketDisconnect:
        pass


# ---------------------------------------------------------------------------
# REST endpoints
# ---------------------------------------------------------------------------


@app.post("/api/v1/register")
async def register(image: UploadFile = File(...), name: str = Form(...)):
    raw = await image.read()
    bgr = decode_upload_to_bgr(raw)
    if bgr is None:
        raise HTTPException(status_code=422, detail="Invalid or empty image")
    nm = (name or "").strip()
    if not nm:
        raise HTTPException(status_code=422, detail="Field 'name' is required")

    async with _inference_lock:
        ok, msg = await asyncio.to_thread(
            _face.register_face_from_frame, bgr, nm, DB_PATH
        )

    if not ok:
        return JSONResponse(status_code=400, content={"status": "error", "message": msg, "data": None})
    return {"status": "success", "message": msg, "data": {"name": nm}}


@app.post("/api/v1/recognize")
async def recognize(image: UploadFile = File(...)):
    raw = await image.read()
    bgr = decode_upload_to_bgr(raw)
    if bgr is None:
        raise HTTPException(status_code=422, detail="Invalid or empty image")

    if not os.path.exists(DB_PATH):
        return JSONResponse(
            status_code=404,
            content={"status": "error", "message": f"Database not found: {DB_PATH}", "data": None},
        )

    async with _inference_lock:

        def _run():
            _, results = _face.recognize_from_frame(bgr, DB_PATH, threshold=THRESHOLD)
            return results

        results = await asyncio.to_thread(_run)

    return {
        "status": "success",
        "message": "ok",
        "data": {
            "face_count": len(results),
            "results": [
                {
                    "name": r.get("name", "Unknown"),
                    "score": r.get("score", 0.0),
                    "bbox": [int(v) for v in r.get("bbox", [])],
                    "direction_px": float(r.get("direction_px", 0.0)),
                }
                for r in results
            ],
        },
    }


@app.get("/api/v1/faces")
def list_faces():
    return {"status": "success", "message": "ok", "data": {"names": _face.list_faces(DB_PATH)}}


@app.delete("/api/v1/faces")
async def delete_faces(request: Request):
    body = await request.json()
    name = body.get("name", "").strip()
    if not name:
        raise HTTPException(status_code=422, detail='Field "name" is required')

    async with _inference_lock:
        ok, msg = await asyncio.to_thread(_face.delete_face, DB_PATH, name)

    if not ok:
        return JSONResponse(status_code=404, content={"status": "error", "message": msg, "data": None})
    return {"status": "success", "message": msg, "data": {"deleted": name}}


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------


def main() -> None:
    p = argparse.ArgumentParser(description="Lite face server: HTTP API + WebSocket recognition")
    p.add_argument("--host", default=os.environ.get("HOST", "0.0.0.0"))
    p.add_argument("--port", type=int, default=int(os.environ.get("PORT", "20004")))
    args = p.parse_args()

    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
