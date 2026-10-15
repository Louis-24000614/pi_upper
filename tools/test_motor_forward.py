#!/usr/bin/env python3
"""独立电机干扰测试：持续给定任务巡航速度，角速度固定为零。

先架空驱动轮并退出导航及其他串口程序，再运行本脚本。Ctrl+C 结束。
复用 uart_vel 的握手、使能、UID 播报和退出停车，不启动相机或导航。
"""

import argparse
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def check_serial_idle(device):
    """只检查占用，不终止其他进程，也不打开串口。"""
    if not device.is_char_device():
        raise ValueError(f"串口不存在或不是字符设备：{device}")
    fuser = shutil.which("fuser")
    if fuser is None:
        raise ValueError("找不到 fuser，无法检查串口占用；未启动测试")
    result = subprocess.run(
        [fuser, "-s", str(device.resolve())],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
        timeout=5,
    )
    if result.returncode == 0:
        raise ValueError(f"串口 {device} 已被占用，请先退出导航或其他串口程序")
    if result.returncode != 1 or result.stderr.strip():
        raise ValueError(f"无法确认串口 {device} 是否空闲；未启动测试")


def stop_bridge(bridge):
    """零速后关闭 stdin，让现有桥执行 Shutdown 和 DISARM。"""
    try:
        os.write(bridge.stdin.fileno(), b"0 0\n")
    except (OSError, ValueError):
        pass
    bridge.stdin.close()
    try:
        return bridge.wait(timeout=2)
    except subprocess.TimeoutExpired:
        bridge.terminate()
        try:
            return bridge.wait(timeout=2)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait()
            print("串口进程未能正常退出，请立即人工确认停车。", file=sys.stderr)
            return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "nav_camera.yaml",
                        help="读取正常任务的 UART 和 follow.cruise_mps 配置")
    parser.add_argument("--uart-bin", type=Path, default=ROOT / "build-turn" / "uart" / "uart_vel")
    parser.add_argument("--device", type=Path, help="仅本次测试覆盖串口路径")
    parser.add_argument("--baud", type=int, help="仅本次测试覆盖波特率")
    parser.add_argument("--dry-run", action="store_true", help="只显示参数，不打开串口或使能电机")
    args = parser.parse_args()

    try:
        import yaml
        config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
        speed = float(config["follow"]["cruise_mps"])
        device = args.device if args.device is not None else Path(config["uart"]["device"])
        baud = args.baud if args.baud is not None else int(config["uart"]["baud"])
        uart_bin = args.uart_bin.resolve()
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("follow.cruise_mps 必须是有限的正向速度")
        if baud <= 0:
            raise ValueError("波特率必须为正整数")
        if not uart_bin.is_file() or not os.access(uart_bin, os.X_OK):
            raise ValueError(f"缺少可执行程序：{uart_bin}")
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        parser.error(str(exc))

    print(f"配置：{args.config}", flush=True)
    print(f"测试目标：持续前进 {speed:g} m/s（{speed * 1000:g} mm/s），角速度 0 rad/s",
          flush=True)
    print(f"串口：{device}，波特率：{baud}；复用：{uart_bin}", flush=True)
    if args.dry_run:
        print("仅检查参数，未启动串口进程。")
        return 0
    try:
        check_serial_idle(device)
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))

    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)

    print("电机将持续转动，请保持驱动轮架空；Ctrl+C 请求停车。", flush=True)
    bridge = None
    result = 0
    try:
        if stopping:
            return 0
        bridge = subprocess.Popen(
            [str(uart_bin), "--device", str(device), "--baud", str(baud)],
            stdin=subprocess.PIPE, bufsize=0, start_new_session=True,
        )
        os.set_blocking(bridge.stdin.fileno(), False)
        command = f"{speed:.9g} 0\n".encode("ascii")
        while not stopping:
            if bridge.poll() is not None:
                raise RuntimeError(f"串口进程提前退出，退出码 {bridge.returncode}")
            if os.write(bridge.stdin.fileno(), command) != len(command):
                raise RuntimeError("速度指令未完整写入，结束测试")
            time.sleep(0.05)
    except (OSError, RuntimeError) as exc:
        print(f"测试结束：{exc}", file=sys.stderr)
        result = 1
    finally:
        if bridge is not None:
            result = stop_bridge(bridge) or result
            print("已请求停车并结束测试。", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
