"""实车寻线：相机 → 道路分割 → (v, ω)。加 --drive 才打开串口。"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from ipm_proto.temporal import temporal_from_mapping
from road_follow.control import VelocityCommand
from road_follow.pipeline import command_from_mask
from road_follow.segment import RoadSegmenter

ROOT = Path(__file__).resolve().parents[2]
SLOW_S = 0.20


def _load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"打不开相机 {device}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return capture


def write_velocity(proc: subprocess.Popen, line: str) -> bool:
    """把一帧速度写给串口进程。对端已退出时返回 False。"""
    if proc.stdin is None or proc.poll() is not None:
        return False
    try:
        proc.stdin.write(line + "\n")
        proc.stdin.flush()
    except (BrokenPipeError, ValueError):
        return False
    return proc.poll() is None


def _stop_bridge(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.stdin is None:
        return
    try:
        proc.stdin.write("0 0\n")
        proc.stdin.flush()
        proc.stdin.close()
    except BrokenPipeError:
        pass
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="道路分割寻线，经串口速度环驱动")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "nav_camera.yaml")
    parser.add_argument("--model", type=Path, default=ROOT / "models" / "road_yolo11n_seg.rknn")
    parser.add_argument("--drive", action="store_true", help="打开串口并使能，按寻线速度行驶")
    parser.add_argument("--uart-bin", type=Path, default=None)
    parser.add_argument("--frames", type=int, default=0, help="跑满 N 帧后退出；0 表示一直跑")
    parser.add_argument("--preview", type=Path, default=None, help="把第一帧 mask 叠加图写到这里")
    args = parser.parse_args(argv)

    cfg = _load_config(args.config)
    capture_cfg = cfg.get("capture", {}) or {}
    uart_cfg = cfg.get("uart", {}) or {}
    device = str(capture_cfg.get("device", "/dev/video0"))
    width = int(capture_cfg.get("width", 1280))
    height = int(capture_cfg.get("height", 720))

    segmenter = RoadSegmenter(args.model)
    segmenter.mask(np.zeros((height, width, 3), dtype=np.uint8))
    smoother = temporal_from_mapping(cfg)
    bridge: subprocess.Popen | None = None
    uart_bin = args.uart_bin or (ROOT / "build" / "uart" / "uart_vel")
    if args.drive:
        if not uart_bin.is_file():
            print(f"找不到 {uart_bin}，先编译 uart_vel", file=sys.stderr)
            return 1
        serial = str(uart_cfg.get("device", "/dev/ttyS6"))
        baud = str(int(uart_cfg.get("baud", 921600)))
        bridge = subprocess.Popen(
            [str(uart_bin), "--device", serial, "--baud", baud],
            stdin=subprocess.PIPE,
            text=True,
        )

    stopping = False

    def _request_stop(signum, _frame) -> None:
        del signum
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    capture = _open_camera(device, width, height)
    frames = 0
    try:
        while not stopping:
            ok, frame = capture.read()
            if not ok:
                print("0.000 0.000 stop_camera", flush=True)
                if bridge is not None and not write_velocity(bridge, "0.000 0.000"):
                    break
                time.sleep(0.05)
                continue

            t0 = time.monotonic()
            mask = segmenter.mask(frame)
            infer_s = time.monotonic() - t0
            if infer_s > SLOW_S:
                command = VelocityCommand(0.0, 0.0, "stop_slow")
            else:
                command = command_from_mask(mask, cfg, smoother)

            line = f"{command.v_mps:.3f} {command.omega_radps:.3f}"
            print(
                f"{line} {command.reason} infer_ms={infer_s * 1000:.0f} road={int(np.count_nonzero(mask))}",
                flush=True,
            )
            if bridge is not None and not write_velocity(bridge, line):
                if not stopping:
                    print("串口进程已退出，速度没有发出去。", file=sys.stderr)
                    return 1
                break

            if args.preview is not None and frames == 0:
                overlay = frame.copy()
                color = np.zeros_like(frame)
                color[:, :, 1] = mask
                overlay = cv2.addWeighted(overlay, 0.7, color, 0.3, 0)
                args.preview.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(args.preview), overlay)

            frames += 1
            if args.frames and frames >= args.frames:
                break
    finally:
        capture.release()
        _stop_bridge(bridge)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
