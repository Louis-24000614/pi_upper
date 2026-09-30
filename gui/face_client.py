"""人脸识别服务的单请求异步客户端；结果只在 Qt 主线程消费。"""

from __future__ import annotations

import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
from PySide6.QtCore import QObject, QThread, Signal

from knife_client import _multipart


class _FaceRequest(QThread):
    succeeded = Signal(dict)
    failed = Signal(dict)

    def __init__(self, endpoint: str, frame: np.ndarray, request_id: str,
                 timeout_s: float, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.endpoint = endpoint
        self.frame = np.ascontiguousarray(frame.copy())
        self.request_id = request_id
        self.timeout_s = timeout_s

    def run(self) -> None:
        try:
            ok, encoded = cv2.imencode(".jpg", self.frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise ValueError("JPEG 编码失败")
            body, content_type = _multipart({}, encoded.tobytes(), "frame.jpg", "image/jpeg")
            request = Request(self.endpoint, data=body,
                              headers={"Content-Type": content_type}, method="POST")
            with urlopen(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
            if payload.get("status") != "success" or not isinstance(payload.get("data"), dict):
                raise ValueError(payload.get("message", "人脸服务响应异常"))
            self.succeeded.emit({"request_id": self.request_id, "data": payload["data"]})
        except HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            self.failed.emit({"request_id": self.request_id, "error": f"HTTP {error.code}: {detail}"})
        except (URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError) as error:
            self.failed.emit({"request_id": self.request_id, "error": str(error)})


class FaceClient(QObject):
    result_ready = Signal(dict)
    request_failed = Signal(dict)

    def __init__(self, endpoint: str = "http://127.0.0.1:20004/api/v1/recognize",
                 timeout_ms: int = 3000, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.endpoint = endpoint
        self.timeout_s = max(0.1, timeout_ms / 1000.0)
        self._worker: _FaceRequest | None = None

    @property
    def busy(self) -> bool:
        return self._worker is not None

    def submit(self, frame: np.ndarray, request_id: str) -> bool:
        if self.busy:
            return False
        if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
            self.request_failed.emit({"request_id": request_id, "error": "识别帧格式无效"})
            return False
        worker = _FaceRequest(self.endpoint, frame, request_id, self.timeout_s, self)
        worker.succeeded.connect(self.result_ready)
        worker.failed.connect(self.request_failed)
        worker.finished.connect(self._finish_request)
        self._worker = worker
        worker.start()
        return True

    def _finish_request(self) -> None:
        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.deleteLater()

    def shutdown(self, timeout_ms: int = 4500) -> bool:
        worker = self._worker
        if worker is None:
            return True
        finished = worker.wait(timeout_ms)
        if finished:
            self._finish_request()
        return finished
