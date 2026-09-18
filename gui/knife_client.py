"""刀具识别服务的非阻塞Qt客户端。

调用方从MainWindow.recognition_frame取得原始BGR快照后提交；网络和PNG编码在工作线程
执行，结果通过信号回到Qt主线程。该模块不直接操作任何界面控件。
"""

from __future__ import annotations

import json
import secrets
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
from PySide6.QtCore import QObject, QThread, Signal


def _multipart(fields: dict[str, object], image_png: bytes) -> tuple[bytes, str]:
    """构造一个只含简单字段和单张PNG的multipart请求。"""
    boundary = "----KnifeBoundary" + secrets.token_hex(12)
    chunks: list[bytes] = []
    for name, value in fields.items():
        chunks.extend(
            [
                f"--{boundary}\r\n".encode(),
                f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode(),
                str(value).encode("utf-8"),
                b"\r\n",
            ]
        )
    chunks.extend(
        [
            f"--{boundary}\r\n".encode(),
            b'Content-Disposition: form-data; name="image"; filename="frame.png"\r\n',
            b"Content-Type: image/png\r\n\r\n",
            image_png,
            b"\r\n",
            f"--{boundary}--\r\n".encode(),
        ]
    )
    return b"".join(chunks), f"multipart/form-data; boundary={boundary}"


class _RequestThread(QThread):
    """执行一次PNG编码和HTTP请求；每个KnifeClient最多有一个活动线程。"""

    succeeded = Signal(dict)
    failed = Signal(str)

    def __init__(
        self,
        endpoint: str,
        frame: np.ndarray,
        metadata: dict[str, object],
        timeout_s: float,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        # 提交时复制，防止摄像头线程在编码期间改写同一块图像内存。
        self.endpoint = endpoint
        self.frame = np.ascontiguousarray(frame.copy())
        self.metadata = metadata
        self.timeout_s = timeout_s

    def run(self) -> None:
        try:
            ok, encoded = cv2.imencode(".png", self.frame)
            if not ok:
                raise ValueError("PNG编码失败")
            body, content_type = _multipart(self.metadata, encoded.tobytes())
            request = Request(
                self.endpoint,
                data=body,
                headers={"Content-Type": content_type, "Content-Length": str(len(body))},
                method="POST",
            )
            with urlopen(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
            if payload.get("status") != "success" or not isinstance(payload.get("data"), dict):
                raise ValueError(f"服务响应异常: {payload}")
            self.succeeded.emit(payload["data"])
        except HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            self.failed.emit(f"HTTP {error.code}: {detail}")
        except (URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError) as error:
            self.failed.emit(str(error))


class KnifeClient(QObject):
    """GUI持有的单请求刀具客户端。

    `submit`返回False表示已有请求在途；结果信号仍需由MainWindow检查request_id、
    frame_id和camera_epoch，防止切换相机或任务后显示过期结果。
    """

    result_ready = Signal(dict)
    request_failed = Signal(str)
    busy_changed = Signal(bool)

    def __init__(
        self,
        endpoint: str = "http://127.0.0.1:20005/api/v1/knife/recognize",
        timeout_ms: int = 3000,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self.endpoint = endpoint
        self.timeout_s = max(0.1, timeout_ms / 1000.0)
        self._worker: _RequestThread | None = None

    @property
    def busy(self) -> bool:
        """是否已有请求正在编码、传输或推理。"""
        return self._worker is not None and self._worker.isRunning()

    def submit(
        self,
        frame: np.ndarray,
        request_id: str,
        frame_id: int,
        camera_epoch: int,
        captured_monotonic_ns: int,
    ) -> bool:
        """提交一个原始BGR/BGRA帧；忙时拒绝，不排队。"""
        if self.busy:
            return False
        if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] not in (3, 4):
            self.request_failed.emit("识别帧格式无效")
            return False
        metadata = {
            "request_id": request_id,
            "frame_id": frame_id,
            "camera_epoch": camera_epoch,
            "captured_monotonic_ns": captured_monotonic_ns,
        }
        worker = _RequestThread(self.endpoint, frame, metadata, self.timeout_s, self)
        worker.succeeded.connect(self.result_ready)
        worker.failed.connect(self.request_failed)
        worker.finished.connect(self._finish_request)
        self._worker = worker
        self.busy_changed.emit(True)
        worker.start()
        return True

    def _finish_request(self) -> None:
        worker = self._worker
        self._worker = None
        self.busy_changed.emit(False)
        if worker is not None:
            worker.deleteLater()

    def shutdown(self, timeout_ms: int = 3500) -> bool:
        """停止接收新请求并等待当前请求结束；不会强杀服务进程。"""
        worker = self._worker
        if worker is None:
            return True
        finished = worker.wait(timeout_ms)
        if finished:
            self._finish_request()
        return finished
