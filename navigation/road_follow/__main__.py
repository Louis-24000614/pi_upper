"""实车寻线：相机 → 道路分割 → (v, ω)。加 --drive 才打开串口。"""

from __future__ import annotations

import argparse
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from ipm_proto.junction import (
    KIND_BLOCKED,
    KIND_CORNER,
    KIND_CROSS,
    KIND_T,
    KIND_UNKNOWN,
    JunctionRead,
    JunctionTracker,
    classify_junction,
)
from ipm_proto.temporal import temporal_from_mapping
from road_follow.backup import Backup, BackupConfig, EdgeProgress, step_backup
from road_follow.control import VelocityCommand
from road_follow.junction_turn import (
    JunctionCue,
    JunctionTurn,
    junction_turn_config_from_mapping,
    road_end_turn_cue,
    step_junction_turn,
)
from road_follow.pipeline import command_from_mask_with_diagnostics, make_ipm
from road_follow.rfid_turn import (
    RfidTurn,
    rfid_turn_config_from_mapping,
    step_rfid_turn,
)
from road_follow.segment import RoadSegmenter

ROOT = Path(__file__).resolve().parents[2]
SLOW_S = 0.20


def _junction_read(
    mask: np.ndarray, cfg: dict, tracker: JunctionTracker
) -> tuple[str, JunctionRead]:
    """返回稳定类型和本帧几何；失败时不参与转弯。"""
    try:
        ipm = make_ipm(cfg, mask.shape)
        bev = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
        reading = classify_junction(bev, ipm.bev)
        return tracker.update(reading), reading
    except (ValueError, cv2.error):
        return KIND_UNKNOWN, JunctionRead(KIND_UNKNOWN, False, False, False, 0.0, 0.20)


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


def _watch_uart_notes(
    proc: subprocess.Popen,
    action_notes: queue.Queue[str],
    rfid_events: queue.Queue[tuple[int, int]],
    odom_samples: queue.Queue[tuple[float, float, float]] | None = None,
) -> None:
    """分发串口桥输出的有限动作终态、新读卡事件和里程计。"""
    stdout = proc.stdout
    if stdout is None:
        return
    for line in stdout:
        text = line.strip()
        if text.startswith(("FORWARD_", "BACKWARD_", "TURN_", "STOP_")):
            print(f"EVENT uart={text}", file=sys.stderr, flush=True)
            action_notes.put(text)
        elif text.startswith("ODOM ") and odom_samples is not None:
            fields = text.split()
            if len(fields) != 5 or fields[4] == "0":
                continue
            try:
                odom_samples.put((float(fields[1]), float(fields[2]), float(fields[3])))
            except ValueError:
                continue
        elif text.startswith("RFID_EVENT "):
            fields = text.split()
            if len(fields) != 3:
                continue
            try:
                card_number = int(fields[1])
                generation = int(fields[2])
            except ValueError:
                continue
            print(
                f"EVENT rfid=detected card={card_number} generation={generation}",
                file=sys.stderr,
                flush=True,
            )
            rfid_events.put((card_number, generation))
        elif text.startswith(("RFID_REMOVED", "RFID_INVALID")):
            print(f"EVENT uart={text}", file=sys.stderr, flush=True)


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
    parser.add_argument(
        "--left-at-junction",
        action="store_true",
        help="兼容选项：等价于 --turn-at-junction left",
    )
    parser.add_argument(
        "--turn-at-junction",
        choices=("left", "right"),
        default=None,
        help="分割确认支路或道路末端后，定距前进到路口中心并转 90°",
    )
    parser.add_argument(
        "--turn-at-rfid",
        choices=("left", "right"),
        default=None,
        help="首次读到有效 RFID 后停车，按 IMU 原地转 90°，再恢复视觉循迹",
    )
    parser.add_argument(
        "--backup-on-obstacle",
        action="store_true",
        help="障碍物连续出现后，用近处路面摆正，再按里程计距离倒回上一个路口",
    )
    args = parser.parse_args(argv)

    if args.left_at_junction and args.turn_at_junction not in (None, "left"):
        parser.error("--left-at-junction 不能与 --turn-at-junction right 同时使用")
    turn_side = "left" if args.left_at_junction else args.turn_at_junction
    if turn_side is not None and args.turn_at_rfid is not None:
        parser.error("局部测试不能同时启用 --turn-at-junction 和 --turn-at-rfid")
    if args.backup_on_obstacle and (turn_side is not None or args.turn_at_rfid is not None):
        parser.error("遇障倒车不能和路口转向或 RFID 转向同时使用")
    if args.backup_on_obstacle and not args.drive:
        parser.error("遇障倒车需要 --drive，才会把 backward 发给下位机")

    cfg = _load_config(args.config)
    capture_cfg = cfg.get("capture", {}) or {}
    uart_cfg = cfg.get("uart", {}) or {}
    device = str(capture_cfg.get("device", "/dev/video0"))
    width = int(capture_cfg.get("width", 1280))
    height = int(capture_cfg.get("height", 720))

    segmenter = RoadSegmenter(args.model)
    segmenter.mask(np.zeros((height, width, 3), dtype=np.uint8))
    smoother = temporal_from_mapping(cfg)
    junctions = JunctionTracker()
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
            stdout=subprocess.PIPE
            if turn_side is not None or args.turn_at_rfid is not None or args.backup_on_obstacle
            else None,
            text=True,
        )
    action_notes: queue.Queue[str] = queue.Queue()
    rfid_events: queue.Queue[tuple[int, int]] = queue.Queue()
    odom_samples: queue.Queue[tuple[float, float, float]] = queue.Queue()
    if bridge is not None and bridge.stdout is not None:
        threading.Thread(
            target=_watch_uart_notes,
            args=(bridge, action_notes, rfid_events, odom_samples),
            daemon=True,
        ).start()
    junction_turn = JunctionTurn()
    junction_turn_cfg = junction_turn_config_from_mapping(cfg)
    backup = Backup()
    backup_cfg = BackupConfig()
    progress = EdgeProgress()
    obstacle_run = 0
    detector = None
    if args.backup_on_obstacle:
        from vision.obstacle.detect import ObstacleDetector

        detector = ObstacleDetector(ROOT / "config" / "obstacle.yaml", root=ROOT)
    rfid_turn = RfidTurn(side=args.turn_at_rfid or "none")
    rfid_turn_cfg = rfid_turn_config_from_mapping(cfg)

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
            command, follow_diag = command_from_mask_with_diagnostics(
                mask, cfg, smoother
            )
            if infer_s > SLOW_S:
                command = VelocityCommand(0.0, 0.0, "stop_slow")
            opening, junction = _junction_read(mask, cfg, junctions)
            cue = JunctionCue(False)
            if turn_side is not None:
                side_open = junction.left if turn_side == "left" else junction.right
                cue = JunctionCue(
                    detected=(
                        opening in (KIND_T, KIND_CROSS, KIND_CORNER) and side_open
                    ),
                    side=turn_side,
                    distance_m=junction.junction_y_m,
                    source="side_branch",
                )
                if not cue.detected:
                    cue = road_end_turn_cue(
                        side=turn_side,
                        stable_blocked=opening == KIND_BLOCKED,
                        raw_blocked=junction.kind == KIND_BLOCKED,
                        command=command,
                        road_end_y_m=junction.corridor_end_y_m,
                        lane_x_m=junction.lane_x_m,
                        lane_width_m=junction.lane_width_m,
                        cfg=junction_turn_cfg,
                        approach_latched=junction_turn.branch_latched,
                    )
            if turn_side is not None and bridge is not None:
                previous_phase = junction_turn.phase
                junction_turn, command = step_junction_turn(
                    junction_turn,
                    cue,
                    command,
                    action_notes,
                    lambda line: write_velocity(bridge, line),
                    junction_turn_cfg,
                )
                if junction_turn.phase != previous_phase:
                    print(
                        f"EVENT phase={previous_phase}->{junction_turn.phase} "
                        f"side={junction_turn.side} forward_mm={junction_turn.forward_mm}",
                        file=sys.stderr,
                        flush=True,
                    )

            if args.turn_at_rfid is not None and bridge is not None:
                detection = None
                try:
                    detection = rfid_events.get_nowait()
                except queue.Empty:
                    pass
                previous_phase = rfid_turn.phase
                rfid_turn, command = step_rfid_turn(
                    rfid_turn,
                    detection,
                    command,
                    action_notes,
                    lambda line: write_velocity(bridge, line),
                    time.monotonic(),
                    rfid_turn_cfg,
                )
                if rfid_turn.phase != previous_phase:
                    print(
                        f"EVENT rfid_phase={previous_phase}->{rfid_turn.phase} "
                        f"side={rfid_turn.side} card={rfid_turn.card_number} "
                        f"generation={rfid_turn.generation}",
                        file=sys.stderr,
                        flush=True,
                    )

            if args.backup_on_obstacle:
                if backup.phase == "idle":
                    try:
                        rfid_events.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        progress.reset()
                while True:
                    try:
                        sample = odom_samples.get_nowait()
                    except queue.Empty:
                        break
                    if backup.phase == "idle":
                        progress.update(*sample)
                triggered = False
                if backup.phase == "idle" and detector is not None:
                    detections, _elapsed_ms = detector.detect(frame)
                    obstacle_run = obstacle_run + 1 if detections else 0
                    triggered = obstacle_run >= 3
                previous_phase = backup.phase
                backup, command = step_backup(
                    backup,
                    triggered,
                    follow_diag.near_x_m,
                    progress.s_m,
                    command,
                    action_notes,
                    lambda line: write_velocity(bridge, line) if bridge is not None else False,
                    time.monotonic(),
                    backup_cfg,
                )
                if backup.phase != previous_phase:
                    print(
                        f"EVENT backup={previous_phase}->{backup.phase} "
                        f"s={progress.s_m:.2f} distance_mm={backup.distance_mm}",
                        file=sys.stderr,
                        flush=True,
                    )

            line = f"{command.v_mps:.3f} {command.omega_radps:.3f}"
            junction_distance = (
                "--" if junction.junction_y_m is None else f"{junction.junction_y_m:.2f}"
            )
            cue_distance = (
                "--" if cue.distance_m is None else f"{cue.distance_m:.2f}"
            )
            corridor_end = (
                "--"
                if junction.corridor_end_y_m is None
                else f"{junction.corridor_end_y_m:.2f}"
            )
            y_range = (
                "--"
                if follow_diag.y_min_m is None or follow_diag.y_max_m is None
                else f"{follow_diag.y_min_m:.2f}..{follow_diag.y_max_m:.2f}"
            )
            directions = "".join(
                name
                for enabled, name in (
                    (junction.forward, "F"),
                    (junction.left, "L"),
                    (junction.right, "R"),
                )
                if enabled
            ) or "-"
            print(
                f"{line} {command.reason} infer_ms={infer_s * 1000:.0f} "
                f"mask_px={follow_diag.mask_road_pixels} "
                f"bev_px={follow_diag.bev_road_pixels} "
                f"pts={follow_diag.prior_points}/{follow_diag.raw_points}/"
                f"{follow_diag.output_points} fallback={int(follow_diag.used_fallback)} "
                f"y={y_range} look={int(follow_diag.lookahead_covered)} "
                f"jraw={junction.kind} junc={opening} dirs={directions} "
                f"lane_x={junction.lane_x_m:.2f} lane_w={junction.lane_width_m:.2f} "
                f"jdist={junction_distance} cend={corridor_end} "
                f"cue={int(cue.detected)} "
                f"cue_src={cue.source} cdist={cue_distance} "
                f"latched={int(junction_turn.branch_latched)} "
                f"arm={junction_turn.arm} phase={junction_turn.phase} "
                f"rfid_phase={rfid_turn.phase} rfid_card={rfid_turn.card_number} "
                f"s={progress.s_m:.2f} near={follow_diag.near_x_m} "
                f"backup={backup.phase}",
                flush=True,
            )
            if junction_turn.phase in ("forward", "turning") or rfid_turn.phase in (
                "searching",
                "stopping_wait",
                "turning",
            ) or backup.phase == "backing":
                frames += 1
                if args.frames and frames >= args.frames:
                    break
                continue
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
        if detector is not None:
            detector.close()
        capture.release()
        _stop_bridge(bridge)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
