"""前视相机障碍物检测。

在仓库根目录执行::

    python3 -m vision.obstacle
    python3 -m vision.obstacle --once --save /tmp/obstacle_live.jpg
"""

from __future__ import annotations

import argparse
import signal
import time
from pathlib import Path

import cv2
import yaml

from vision.obstacle.blockage import HardBlockageJudge, hard_block_config_from_mapping
from vision.obstacle.detect import ObstacleDetector, draw

ROOT = Path(__file__).resolve().parents[2]


def _open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"打不开相机 {device}。若提示 busy，先确认没有别的程序占着 /dev/video*")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return capture


def _format(detections, elapsed_ms: float) -> str:
    if not detections:
        return f"none {elapsed_ms:.0f}ms"
    parts = [
        f"{item.label} {item.score:.3f} ({item.x1:.0f},{item.y1:.0f})-({item.x2:.0f},{item.y2:.0f})"
        for item in detections
    ]
    return f"{elapsed_ms:.0f}ms " + " | ".join(parts)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="前视相机障碍物检测")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "obstacle.yaml")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config" / "nav_camera.yaml")
    parser.add_argument("--device", default=None, help="覆盖 nav_camera.yaml 里的采集设备")
    parser.add_argument("--once", action="store_true", help="抓一帧后退出")
    parser.add_argument("--save", type=Path, default=None, help="把画了框的画面写到这个 jpg")
    args = parser.parse_args(argv)

    camera_cfg = yaml.safe_load(args.camera_config.read_text(encoding="utf-8"))
    capture_cfg = camera_cfg.get("capture", {}) or {}
    device = args.device or str(capture_cfg.get("device", "/dev/video0"))
    width = int(capture_cfg.get("width", 1280))
    height = int(capture_cfg.get("height", 720))

    stopping = False

    def _request_stop(signum, _frame) -> None:
        del signum, _frame
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    capture = _open_camera(device, width, height)
    try:
        with ObstacleDetector(args.config, root=ROOT) as detector:
            judge = HardBlockageJudge(hard_block_config_from_mapping(detector.config))
            warmed = 0
            while not stopping:
                ok, frame = capture.read()
                if not ok or frame is None:
                    print("读相机失败", flush=True)
                    time.sleep(0.05)
                    continue
                warmed += 1
                if warmed < 8:
                    continue
                detections, elapsed_ms = detector.detect(frame)
                observation = judge.update(detections, frame.shape)
                print(
                    f"{_format(detections, elapsed_ms)} "
                    f"candidate={int(observation.candidate)} "
                    f"stable={observation.stable_frames} "
                    f"hard_blocked={int(observation.hard_blocked)} "
                    f"reason={observation.reason}",
                    flush=True,
                )
                if args.save is not None:
                    args.save.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(args.save), draw(frame, detections))
                if args.once:
                    break
    finally:
        capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
