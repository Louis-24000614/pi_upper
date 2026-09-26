"""实车寻线：相机 → 道路分割 → (v, ω)。加 --drive 才打开串口。"""

from __future__ import annotations

import argparse
import queue
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
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
from road_follow.departure import handoff_arrival, handoff_entrance
from road_follow.entrance import (
    EntranceDeparture,
    entrance_config_from_mapping,
    step_entrance_departure,
)
from road_follow.junction_turn import (
    JunctionCue,
    JunctionTurn,
    junction_turn_config_from_mapping,
    odom_handoff_turn_cue,
    road_end_turn_cue,
    should_stop_at_expected_junction,
    step_junction_turn,
)
from road_follow.pipeline import command_from_mask_with_diagnostics, make_ipm
from road_follow.recording import VideoRecorder
from road_follow.rfid_arrival import (
    RfidArrival,
    rfid_arrival_config_from_mapping,
    step_rfid_arrival,
)
from road_follow.rfid_turn import (
    RfidTurn,
    rfid_turn_config_from_mapping,
    step_rfid_turn,
)
from road_follow.segment import RoadSegmenter

ROOT = Path(__file__).resolve().parents[2]
SLOW_S = 0.20
STATUS_LOG_INTERVAL_S = 2.0

COMMAND_NAMES = {
    "follow": "视觉循迹",
    "follow_near": "近距离低速循迹",
    "entrance_forward": "出发区定距前进",
    "entrance_stop": "出发区停车",
    "entrance_stop_settle": "出发区等待停稳",
    "entrance_turn": "出发区原地转弯",
    "entrance_reacquire": "转弯后重新识别道路",
    "entrance_recovery": "转弯后低速恢复",
    "blind_forward": "路口定距前进",
    "forward": "定距前进",
    "forward_wait": "等待定距前进",
    "stopping": "正在停车",
    "stopped": "等待停稳",
    "junction_stop_settle": "路口等待停稳",
    "reacquire": "等待重新识别道路",
    "turning": "原地转弯",
    "turn_wait": "等待转弯",
    "rfid_searching": "RFID 区域定距搜索",
    "rfid_arrived": "已到达 RFID 点",
    "rfid_turning": "RFID 点原地转弯",
    "rfid_reacquire": "RFID 转弯后重新识别道路",
    "stop_road": "停车：未识别到道路",
    "stop_centerline": "停车：中心线点不足",
    "stop_lookahead": "停车：预瞄距离不足",
    "stop_slow": "停车：视觉推理过慢",
    "stop_camera": "停车：相机读取失败",
    "stop_entrance_fail": "停车：出发动作失败",
    "stop_action_fail": "停车：路口动作失败",
    "stop_bad_turn_distance": "停车：转弯前进距离不安全",
    "stop_odom_junction_wait": "停车：里程计到达预计路口但视觉未确认",
    "stop_rfid": "停车：检测到 RFID",
    "stop_rfid_action": "RFID 点停车",
    "stop_rfid_action_fail": "停车：RFID 停车动作失败",
    "stop_rfid_not_found": "停车：定距搜索后仍未读到 RFID",
    "stop_rfid_search_fail": "停车：RFID 定距搜索失败",
    "stop_rfid_settle": "RFID 点等待停稳",
    "stop_rfid_turn_fail": "停车：RFID 转弯失败",
    "stop_rfid_unsafe": "停车：RFID 搜索时视觉条件不安全",
    "backup": "视觉纠偏倒车",
    "visual_backup": "视觉纠偏倒车",
    "stop_backup": "停车：准备倒车",
    "stop_backup_fault": "停车：倒车动作失败",
    "stop_backup_distance_limit": "停车：达到最大倒车距离",
    "stop_backup_timeout": "停车：倒车超时",
    "stop_backup_road_lost": "停车：倒车时道路丢失",
    "stop_backup_no_vision": "停车：倒车时暂时看不到道路",
    "backup_done": "倒车完成",
    "backup_done_at_entry": "停车：已经位于路段入口",
    "creep": "低速前探",
    "cooldown": "转弯后等待",
    "arrived": "已到达节点",
    "done": "任务完成",
    "fault": "故障停车",
}

JUNCTION_NAMES = {
    "straight": "直路",
    "t_junction": "丁字路口",
    "cross": "十字路口",
    "corner": "转角",
    "blocked": "前方道路结束",
    "unknown": "未确认",
}

DIRECTION_NAMES = {
    "left": "左转",
    "right": "右转",
    "straight": "直行",
    "backup": "倒车",
    "none": "未确定",
}

ACTION_NAMES = {
    "FORWARD_DONE": "定距前进完成",
    "FORWARD_FAIL": "定距前进失败",
    "BACKWARD_DONE": "定距后退完成",
    "BACKWARD_FAIL": "定距后退失败",
    "TURN_DONE": "原地转弯完成",
    "TURN_FAIL": "原地转弯失败",
    "STOP_DONE": "停车完成",
    "STOP_FAIL": "停车失败",
}

PHASE_NAMES = {
    "pending": "准备",
    "follow": "视觉循迹",
    "approach": "接近路口",
    "forward": "定距前进",
    "blind_forward": "定距搜索",
    "searching": "定距搜索",
    "stopping_wait": "等待停车确认",
    "stopping": "正在停车",
    "stopped": "等待停稳",
    "turning": "原地转弯",
    "reacquire": "重新识别道路",
    "recovery": "低速恢复",
    "arrived": "已到达",
    "backup": "倒车",
    "backing": "正在倒车",
    "cooldown": "倒车结束缓冲",
    "odom_wait": "等待视觉确认路口",
    "done": "完成",
    "complete": "完成",
    "fault": "故障停车",
}


def _phase_name(phase: str) -> str:
    return PHASE_NAMES.get(phase, phase)


def _rfid_edge_name(state: RfidArrival) -> str:
    if state.edge_left_seen and state.edge_right_seen:
        return "两侧"
    if state.edge_left_seen:
        return "左侧"
    if state.edge_right_seen:
        return "右侧"
    return "方向未确认"


def _event(category: str, message: str) -> None:
    """输出面向实车调试的中文关键事件。"""
    print(f"[{category}] {message}", file=sys.stderr, flush=True)


def _status_signature(
    command: VelocityCommand,
    opening: str,
    entrance: EntranceDeparture,
    junction_turn: JunctionTurn,
    rfid_turn: RfidTurn,
    rfid_arrival: RfidArrival,
    backup: Backup,
) -> tuple[object, ...]:
    """只有这些关键状态改变时才立即打印，连续数值变化不触发刷屏。"""
    return (
        command.reason,
        opening,
        entrance.phase,
        junction_turn.phase,
        junction_turn.branch_latched,
        rfid_turn.phase,
        rfid_arrival.phase,
        rfid_arrival.edge_latched,
        rfid_arrival.edge_left_seen,
        rfid_arrival.edge_right_seen,
        backup.phase,
    )


def _junction_read(
    mask: np.ndarray, cfg: dict, tracker: JunctionTracker
) -> tuple[str, JunctionRead]:
    """返回稳定类型和本帧几何；失败时不参与转弯。"""
    try:
        ipm = make_ipm(cfg, mask.shape)
        bev = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
        raw = cfg.get("junction_turn", {}) or {}
        reading = classify_junction(
            bev,
            ipm.bev,
            forward_band_min_y_m=float(raw.get("road_end_band_min_y_m", 0.34)),
            forward_band_max_y_m=float(raw.get("road_end_band_max_y_m", 0.48)),
        )
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
    rfid_enabled: bool = False,
) -> None:
    """分发有限动作、里程计；仅 RFID 独立测试接收读卡事件。"""
    stdout = proc.stdout
    if stdout is None:
        return
    for line in stdout:
        text = line.strip()
        if text.startswith(("FORWARD_", "BACKWARD_", "TURN_", "STOP_")):
            _event("动作", ACTION_NAMES.get(text, text))
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
            if not rfid_enabled:
                continue
            fields = text.split()
            if len(fields) != 3:
                continue
            try:
                card_number = int(fields[1])
                generation = int(fields[2])
            except ValueError:
                continue
            _event("RFID", f"读到 {card_number} 号标签（第 {generation} 次）")
            rfid_events.put((card_number, generation))
        elif text.startswith("RFID_REMOVED"):
            if rfid_enabled:
                _event("RFID", "标签已移开")
        elif text.startswith("RFID_INVALID"):
            if rfid_enabled:
                _event("RFID", "读到无效标签")


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


def _start_route():
    """加载拓扑并取出第一条 FollowEdge。转向不使用启动参数里的 left/right。"""
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from agent.route_agent import RouteAgent

    try:
        from topo_proto.graph import load_topology
    except ImportError:
        from navigation.topo_proto.graph import load_topology

    graph = load_topology()
    agent = RouteAgent(graph)
    first = agent.start()
    node_xy = {node_id: (node.x, node.y) for node_id, node in graph.nodes.items()}
    return agent, node_xy, first


def _velocity_for_departure(state: JunctionTurn, visual: VelocityCommand) -> VelocityCommand:
    if state.phase == "follow":
        return visual
    if state.phase == "fault":
        return VelocityCommand(0.0, 0.0, "stop_action_fail")
    return VelocityCommand(0.0, 0.0, state.phase)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="道路分割寻线，经串口速度环驱动")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "nav_camera.yaml")
    parser.add_argument("--model", type=Path, default=ROOT / "models" / "road_yolo11n_seg.rknn")
    parser.add_argument("--drive", action="store_true", help="打开串口并使能，按寻线速度行驶")
    parser.add_argument("--uart-bin", type=Path, default=None)
    parser.add_argument("--frames", type=int, default=0, help="跑满 N 帧后退出；0 表示一直跑")
    parser.add_argument("--preview", type=Path, default=None, help="把第一帧 mask 叠加图写到这里")
    parser.add_argument(
        "--record-video",
        nargs="?",
        const="",
        default=None,
        metavar="AVI",
        help="录制原始相机画面；不指定路径时自动保存到 data/road/drive_时间.avi",
    )
    parser.add_argument(
        "--left-at-junction",
        action="store_true",
        help="兼容选项：等价于 --turn-at-junction left",
    )
    parser.add_argument(
        "--turn-at-junction",
        choices=("left", "right"),
        default=None,
        help="观察这一侧开口并定距走到路口；FORWARD_DONE 之后由 Agent 的下一条边决定转向",
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
        help="障碍物连续出现后，用负速度和视觉纠偏倒回上一个路口",
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
        parser.error("遇障倒车需要 --drive，才会向下位机持续发送负速度 CMD_VEL")

    recorder = None
    if args.record_video is not None:
        record_path = (
            ROOT / "data" / "road" / f"drive_{datetime.now():%Y%m%d_%H%M%S_%f}.avi"
            if args.record_video == ""
            else Path(args.record_video)
        )
        try:
            recorder = VideoRecorder(record_path)
        except (ValueError, FileExistsError) as exc:
            parser.error(str(exc))

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
            # 始终由监视线程消费 ODOM 等高频内部消息，避免直接刷满终端。
            stdout=subprocess.PIPE,
            text=True,
        )
    action_notes: queue.Queue[str] = queue.Queue()
    rfid_events: queue.Queue[tuple[int, int]] = queue.Queue()
    odom_samples: queue.Queue[tuple[float, float, float]] = queue.Queue()
    if bridge is not None:
        threading.Thread(
            target=_watch_uart_notes,
            args=(
                bridge,
                action_notes,
                rfid_events,
                odom_samples,
                turn_side is None,
            ),
            daemon=True,
        ).start()
    junction_turn = JunctionTurn()
    entrance = EntranceDeparture()
    entrance_cfg = entrance_config_from_mapping(cfg)
    junction_turn_cfg = junction_turn_config_from_mapping(cfg)
    backup = Backup()
    backup_cfg = BackupConfig()
    progress = EdgeProgress()
    route_agent = None
    node_xy: dict[str, tuple[float, float]] = {}
    if turn_side is not None and bridge is not None:
        route_agent, node_xy, first = _start_route()
        if type(first).__name__ != "FollowEdge":
            print(f"Agent 没有给出第一条路: {first}", file=sys.stderr)
            _stop_bridge(bridge)
            return 1
        _event("路线", f"开始路段 {first.edge_id}：{first.from_node} → {first.to_node}")
    detector = None
    obstacle_judge = None
    if args.backup_on_obstacle:
        from vision.obstacle.blockage import HardBlockageJudge, hard_block_config_from_mapping
        from vision.obstacle.detect import ObstacleDetector

        detector = ObstacleDetector(ROOT / "config" / "obstacle.yaml", root=ROOT)
        obstacle_judge = HardBlockageJudge(
            hard_block_config_from_mapping(detector.config)
        )
    rfid_turn = RfidTurn(side=args.turn_at_rfid or "none")
    rfid_turn_cfg = rfid_turn_config_from_mapping(cfg)
    rfid_arrival = RfidArrival()
    rfid_arrival_cfg = rfid_arrival_config_from_mapping(cfg)

    stopping = False
    last_status_signature: tuple[object, ...] | None = None
    last_status_s = 0.0
    last_camera_warning_s = 0.0

    def _request_stop(signum, _frame) -> None:
        del signum
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    capture: cv2.VideoCapture | None = None
    frames = 0
    try:
        capture = _open_camera(device, width, height)
        while not stopping:
            ok, frame = capture.read()
            if not ok:
                now_s = time.monotonic()
                if now_s - last_camera_warning_s >= STATUS_LOG_INTERVAL_S:
                    _event("警告", "相机读取失败，已发送停车指令")
                    last_camera_warning_s = now_s
                if bridge is not None and not write_velocity(bridge, "0.000 0.000"):
                    break
                time.sleep(0.05)
                continue

            if recorder is not None:
                first_recorded_frame = recorder.frames_written == 0
                recorder.write(frame, time.monotonic())
                if first_recorded_frame:
                    _event("录像", f"开始录制：{recorder.path}")

            t0 = time.monotonic()
            mask = segmenter.mask(frame)
            infer_s = time.monotonic() - t0
            command, follow_diag = command_from_mask_with_diagnostics(
                mask, cfg, smoother
            )
            if infer_s > SLOW_S:
                command = VelocityCommand(0.0, 0.0, "stop_slow")
            opening, junction = _junction_read(mask, cfg, junctions)
            if turn_side is not None:
                while True:
                    try:
                        sample = odom_samples.get_nowait()
                    except queue.Empty:
                        break
                    progress.update(*sample)
            cue = JunctionCue(False)
            if turn_side is not None:
                side_open = junction.left if turn_side == "left" else junction.right
                side_cue = JunctionCue(
                    detected=(
                        opening in (KIND_T, KIND_CROSS, KIND_CORNER) and side_open
                    ),
                    side=turn_side,
                    distance_m=junction.junction_y_m,
                    source="side_branch",
                )
                end_cue = road_end_turn_cue(
                    side=turn_side,
                    stable_blocked=opening == KIND_BLOCKED,
                    raw_blocked=junction.kind == KIND_BLOCKED,
                    command=command,
                    road_end_y_m=junction.corridor_end_y_m,
                    lane_x_m=junction.lane_x_m,
                    lane_width_m=junction.lane_width_m,
                    cfg=junction_turn_cfg,
                    approach_latched=junction_turn.branch_latched,
                    forward_band_ratio=junction.forward_band_ratio,
                )
                # 侧边角仍可见时也要检查正前方检测带；端头到位信号优先。
                cue = end_cue if end_cue.detected else side_cue
            if turn_side is not None and bridge is not None and entrance.phase != "done":
                entrance_before = entrance.phase
                entrance, command = step_entrance_departure(
                    entrance,
                    action_notes,
                    lambda line: write_velocity(bridge, line),
                    command,
                    entrance_cfg,
                )
                if (
                    entrance_before == "forward"
                    and entrance.phase == "stopping_wait"
                    and route_agent is not None
                ):
                    try:
                        entrance.turn_side, next_edge = handoff_entrance(
                            route_agent, node_xy
                        )
                    except (KeyError, ValueError) as exc:
                        entrance.phase = "fault"
                        command = VelocityCommand(0.0, 0.0, "stop_entrance_fail")
                        _event("错误", f"出发路线交接失败：{exc}")
                    else:
                        _event(
                            "路线",
                            f"到达 {next_edge.from_node}；下一路段 "
                            f"{next_edge.edge_id}：{next_edge.from_node} → "
                            f"{next_edge.to_node}；原地"
                            f"{DIRECTION_NAMES[entrance.turn_side]}",
                        )
                if entrance_before == "turning" and entrance.phase == "reacquire":
                    # 原地转弯期间的画面不属于新道路，禁止污染新方向的 EMA。
                    smoother.reset()
                    entrance.clear = 0
                if entrance.phase != entrance_before:
                    _event(
                        "出发",
                        f"{_phase_name(entrance_before)} → {_phase_name(entrance.phase)}；"
                        f"转向={DIRECTION_NAMES.get(entrance.turn_side, '未确定')}",
                    )
                    if entrance.phase == "done":
                        progress.reset()
            elif turn_side is not None and bridge is not None:
                previous_phase = junction_turn.phase
                visual_command = command
                target = None
                if route_agent is not None and route_agent.state.current_edge:
                    edge = route_agent.graph.edges[route_agent.state.current_edge]
                    target = route_agent.graph.nodes.get(route_agent.state.to_node or "")
                patrol_mode = (
                    target is not None
                    and target.role == "patrol_slot"
                    and junction_turn.phase == "follow"
                )
                if patrol_mode:
                    previous_rfid_phase = rfid_arrival.phase
                    follow_cfg = cfg.get("follow", {}) or {}
                    visual_safe = (
                        follow_diag.bev_road_pixels
                        >= int(follow_cfg.get("min_road_pixels", 400))
                        and follow_diag.output_points
                        >= int(follow_cfg.get("min_points", 8))
                    )
                    # 保留原巡检点视觉状态机和全部阈值，只取消 UID 输入。
                    rfid_arrival, command = step_rfid_arrival(
                        rfid_arrival,
                        None,
                        command,
                        action_notes,
                        lambda line: write_velocity(bridge, line),
                        rfid_arrival_cfg,
                        edge_left_visible=junction.left,
                        edge_right_visible=junction.right,
                        forward_band_ratio=junction.forward_band_ratio,
                        visual_safe=visual_safe,
                    )
                    if rfid_arrival.phase != previous_rfid_phase:
                        _event(
                            "巡检点",
                            f"{_phase_name(previous_rfid_phase)} → "
                            f"{_phase_name(rfid_arrival.phase)}；"
                            f"位置={target.id}，"
                            f"侧边={_rfid_edge_name(rfid_arrival)}，"
                            f"已前进={rfid_arrival.searched_mm} mm",
                        )
                    if rfid_arrival.phase == "arrived" and route_agent is not None:
                        junction_turn.phase = "arrived"
                        junction_turn = handoff_arrival(
                            junction_turn,
                            route_agent,
                            node_xy,
                            lambda line: write_velocity(bridge, line),
                            junction_turn_cfg.stop_settle_s,
                        )
                        command = _velocity_for_departure(
                            junction_turn, visual_command
                        )
                        rfid_arrival = RfidArrival()
                else:
                    if target is not None:
                        odom_cue = odom_handoff_turn_cue(
                            side=junction_turn.side,
                            progress_m=progress.s_m,
                            edge_length_m=edge.length_m,
                            target_role=target.role,
                            state=junction_turn,
                            command=command,
                            cfg=junction_turn_cfg,
                        )
                        if odom_cue.detected:
                            cue = odom_cue
                    junction_turn, command = step_junction_turn(
                        junction_turn,
                        cue,
                        command,
                        action_notes,
                        lambda line: write_velocity(bridge, line),
                        junction_turn_cfg,
                    )
                    if target is not None and should_stop_at_expected_junction(
                        progress.s_m,
                        edge.length_m,
                        target.role,
                        junction_turn,
                        junction_turn_cfg,
                    ):
                        junction_turn.phase = "odom_wait"
                        junction_turn.stop_started_s = time.monotonic()
                        command = VelocityCommand(
                            0.0, 0.0, "stop_odom_junction_wait"
                        )
                        _event(
                            "安全停车",
                            f"路段 {edge.id} 已行驶 {progress.s_m:.2f} m，"
                            f"预计节点在 {edge.length_m:.2f} m，路口交接尚未完成",
                        )
                    if junction_turn.phase == "arrived" and route_agent is not None:
                        junction_turn = handoff_arrival(
                            junction_turn,
                            route_agent,
                            node_xy,
                            lambda line: write_velocity(bridge, line),
                            junction_turn_cfg.stop_settle_s,
                        )
                        command = _velocity_for_departure(
                            junction_turn, visual_command
                        )
                if junction_turn.phase == "backup":
                    backup_before = backup.phase
                    backup, command = step_backup(
                        backup,
                        backup.phase == "idle",
                        follow_diag.near_x_m,
                        progress.s_m,
                        command,
                        time.monotonic(),
                        backup_cfg,
                    )
                    if backup.phase == "done" and backup_before != "done":
                        if command.reason == "backup_done_at_entry":
                            junction_turn.phase = "fault"
                            command = VelocityCommand(0.0, 0.0, "stop_action_fail")
                            _event("倒车", "失败：车辆已经位于路段入口")
                        else:
                            finished_at = route_agent.state.to_node if route_agent is not None else None
                            finished = (
                                route_agent.node_reached(finished_at)
                                if route_agent is not None and finished_at
                                else None
                            )
                            progress.reset()
                            backup = Backup()
                            if type(finished).__name__ == "Stop":
                                junction_turn.phase = "fault"
                                junction_turn.departure = "none"
                                command = VelocityCommand(0.0, 0.0, "stop_action_fail")
                            else:
                                junction_turn.phase = "follow"
                                junction_turn.suppress_cue = True
                                junction_turn.departure = "none"
                                command = visual_command
                            _event(
                                "倒车",
                                f"完成；下一动作={type(finished).__name__ if finished else '无'}",
                            )
                if previous_phase != "follow" and junction_turn.phase == "follow":
                    progress.reset()
                if junction_turn.phase != previous_phase:
                    _event(
                        "路口",
                        f"{_phase_name(previous_phase)} → {_phase_name(junction_turn.phase)}；"
                        f"规划={DIRECTION_NAMES.get(junction_turn.departure, junction_turn.departure)}，"
                        f"定距={junction_turn.forward_mm} mm",
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
                    _event(
                        "RFID 转弯",
                        f"{_phase_name(previous_phase)} → {_phase_name(rfid_turn.phase)}；"
                        f"方向={DIRECTION_NAMES.get(rfid_turn.side, rfid_turn.side)}，"
                        f"标签={rfid_turn.card_number or '未读到'}",
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
                    if backup.phase in ("idle", "backing"):
                        progress.update(*sample)
                triggered = False
                if backup.phase == "idle" and detector is not None and obstacle_judge is not None:
                    detections, _elapsed_ms = detector.detect(frame)
                    obstacle = obstacle_judge.update(detections, frame.shape)
                    triggered = obstacle.hard_blocked
                    if obstacle.just_confirmed:
                        _event(
                            "障碍物",
                            f"确认道路被阻挡；置信度={obstacle.score:.2f}，"
                            f"连续={obstacle.stable_frames} 帧",
                        )
                previous_phase = backup.phase
                backup, command = step_backup(
                    backup,
                    triggered,
                    follow_diag.near_x_m,
                    progress.s_m,
                    command,
                    time.monotonic(),
                    backup_cfg,
                )
                if backup.phase != previous_phase:
                    _event(
                        "倒车",
                        f"{_phase_name(previous_phase)} → {_phase_name(backup.phase)}；"
                        f"路段进度={progress.s_m:.2f} m，"
                        f"已倒车={backup.reverse_distance_m:.2f} m",
                    )

            line = f"{command.v_mps:.3f} {command.omega_radps:.3f}"
            y_range = (
                "不可见"
                if follow_diag.y_min_m is None or follow_diag.y_max_m is None
                else f"{follow_diag.y_min_m:.2f}～{follow_diag.y_max_m:.2f} m"
            )
            directions = "、".join(
                name
                for enabled, name in (
                    (junction.forward, "前"),
                    (junction.left, "左"),
                    (junction.right, "右"),
                )
                if enabled
            ) or "无"
            status_signature = _status_signature(
                command,
                opening,
                entrance,
                junction_turn,
                rfid_turn,
                rfid_arrival,
                backup,
            )
            now_s = time.monotonic()
            if (
                status_signature != last_status_signature
                or now_s - last_status_s >= STATUS_LOG_INTERVAL_S
            ):
                details = [
                    COMMAND_NAMES.get(command.reason, command.reason),
                    f"速度={command.v_mps:.3f} m/s",
                    f"角速度={command.omega_radps:.3f} rad/s",
                    f"道路={follow_diag.bev_road_pixels} px",
                    f"中心线={follow_diag.output_points} 点",
                    f"可见距离={y_range}",
                    f"路口={JUNCTION_NAMES.get(opening, opening)}",
                    f"开口={directions}",
                ]
                if turn_side is not None:
                    details.extend(
                        (
                            f"检测带={junction.forward_band_ratio:.2f}",
                            f"支路={'已锁存' if junction_turn.branch_latched else '未锁存'}",
                            f"路段进度={progress.s_m:.2f} m",
                        )
                    )
                if rfid_arrival.edge_latched or rfid_arrival.phase != "follow":
                    details.extend(
                        (
                            f"巡检点={_phase_name(rfid_arrival.phase)}",
                            f"巡检侧边={_rfid_edge_name(rfid_arrival)}",
                        )
                    )
                details.append(f"推理={infer_s * 1000:.0f} ms")
                _event("状态", "；".join(details))
                last_status_signature = status_signature
                last_status_s = now_s
            if entrance.phase in (
                "forward",
                "stopping_wait",
                "stopping",
                "turning",
            ) or junction_turn.phase in (
                "forward",
                "stopping",
                "stopped",
                "turning",
            ) or rfid_turn.phase in (
                "searching",
                "stopping_wait",
                "turning",
            ) or rfid_arrival.phase in (
                "blind_forward",
                "stopping_wait",
            ) or backup.phase == "backing":
                frames += 1
                if args.frames and frames >= args.frames:
                    break
                continue
            if bridge is not None and not write_velocity(bridge, line):
                if not stopping:
                    _event("错误", "串口进程已退出，速度指令没有发出去")
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
        recording_stopped_s = time.monotonic()
        _stop_bridge(bridge)
        if recorder is not None:
            recorder.close(recording_stopped_s)
            if recorder.frames_written:
                _event(
                    "录像",
                    f"已保存：{recorder.path}（{recorder.frames_written} 帧）",
                )
        if detector is not None:
            detector.close()
        if capture is not None:
            capture.release()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
