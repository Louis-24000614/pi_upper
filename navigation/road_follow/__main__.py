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
    KIND_UNKNOWN,
    JunctionRead,
    JunctionTracker,
    classify_junction,
)
from ipm_proto.temporal import temporal_from_mapping
from road_follow.arrival_policy import resolve_arrival_policy, step_route_arrival
from road_follow.backup import Backup, BackupConfig, EdgeProgress, step_backup
from road_follow.control import VelocityCommand, follow_config_from_mapping, heading_anchor_enabled_from_mapping
from road_follow.departure import handoff_arrival, handoff_entrance
from road_follow.entrance import (
    EntranceDeparture,
    entrance_config_from_mapping,
    step_entrance_departure,
)
from road_follow.junction_turn import (
    JunctionTurn,
    junction_turn_config_from_mapping,
)
from road_follow.obstacle_route import (
    ObstacleRecovery,
    detection_armed,
    step_obstacle_route,
)
from road_follow.pipeline import BevProjector, command_from_mask_with_diagnostics, make_ipm
from road_follow.recording import VideoRecorder
from road_follow.async_recording import AsyncVideoRecorder
from road_follow.rfid_arrival import (
    RfidArrival,
    rfid_arrival_config_from_mapping,
)
from road_follow.rfid_turn import (
    RfidTurn,
    rfid_turn_config_from_mapping,
    step_rfid_turn,
)
from road_follow.segment import RoadSegmenter
from road_follow.parallel_segment import OrderedSegmentStream, CameraReadError
from road_follow.latest_segment import LatestSegmentStream
from road_follow.cpu_threads import cpu_thread_budget
from road_follow.heap_reclaim import HeapReclaimer
from road_follow.frequency_runtime import needs_frequency_guard, run_guarded

ROOT = Path(__file__).resolve().parents[2]
STATUS_LOG_INTERVAL_S = 2.0


COMMAND_NAMES = {
    "follow": "视觉循迹",
    "follow_near": "近距离低速循迹",
    "align": "交接前摆正",
    "entrance_forward": "出发区定距前进",
    "entrance_stop": "出发区停车",
    "entrance_stop_settle": "出发区等待停稳",
    "entrance_turn": "出发区原地转弯",
    "entrance_reacquire": "转弯后重新识别道路",
    "entrance_recovery": "转弯后低速恢复",
    "blind_forward": "路口定距前进",
    "heading_hold": "锁视觉航向",
    "anchor_wait": "等待停稳与航向参考同步",
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
    "stop_forward_strip": "停车：正前方窄带没有路",
    "stop_camera": "停车：相机读取失败",
    "stop_entrance_fail": "停车：出发动作失败",
    "stop_action_fail": "停车：路口动作失败",
    "stop_bad_turn_distance": "停车：转弯前进距离不安全",
    "stop_odom_junction_wait": "停车：里程计到达预计路口但视觉未确认",
    "stop_arrival_guard": "停车：超过到点保护位置，交接未完成",
    "stop_odom_stale": "停车：缺少有效的新里程计数据",
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
    "backup_hold": "停车：遇障后等待停稳",
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
    "anchor_wait": "等待航向参考同步",
    "pending": "准备",
    "follow": "视觉循迹",
    "align": "交接前摆正",
    "approach": "接近路口",
    "forward": "定距前进",
    "heading_hold": "锁视觉航向",
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
    "holding": "遇障后等待停稳",
    "backing": "正在倒车",
    "cooldown": "倒车结束缓冲",
    "odom_wait": "到点保护停车，等待人工处理",
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
    entrance: EntranceDeparture,
    junction_turn: JunctionTurn,
    rfid_turn: RfidTurn,
    rfid_arrival: RfidArrival,
    backup: Backup,
    recovery_phase: str = "idle",
) -> tuple[object, ...]:
    """只有这些关键状态改变时才立即打印。路口分类抖动不算变化。"""
    return (
        command.reason,
        entrance.phase,
        junction_turn.phase,
        junction_turn.branch_latched,
        rfid_turn.phase,
        rfid_arrival.phase,
        rfid_arrival.edge_latched,
        rfid_arrival.edge_left_seen,
        rfid_arrival.edge_right_seen,
        backup.phase,
        recovery_phase,
    )


def _junction_read(
    mask: np.ndarray, cfg: dict, tracker: JunctionTracker, *, projection=None
) -> tuple[str, JunctionRead]:
    """返回稳定类型和本帧几何；失败时不参与转弯。"""
    try:
        if projection is None:
            ipm = make_ipm(cfg, mask.shape)
            bev = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
        else:
            ipm, bev = projection
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
    odom_samples: queue.Queue[tuple[float, float, float, float]] | None = None,
    rfid_enabled: bool = False,
    map_rfid_events=None,
    anchor_notes: queue.Queue[tuple[str, float]] | None = None,
    speech_notes: queue.Queue[str] | None = None,
) -> None:
    """分发有限动作、里程计。读卡始终记日志；只有 RFID 独立测试才把卡号送进转弯状态机。"""
    stdout = proc.stdout
    if stdout is None:
        return
    for line in stdout:
        text = line.strip()
        if text == "ANCHOR_DONE" or text.startswith("ANCHOR_FAIL "):
            if anchor_notes is not None:
                anchor_notes.put((text, time.monotonic()))
            _event("航向校正", "下位机动作参考已同步" if text == "ANCHOR_DONE" else text)
        elif text.startswith(("SPEECH_QUEUED ", "SPEECH_REJECT ", "SPEECH_SENT ", "SPEECH_RESULT ")):
            if speech_notes is not None:
                speech_notes.put(text)
        elif text.startswith(("FORWARD_", "BACKWARD_", "TURN_", "STOP_")):
            _event("动作", ACTION_NAMES.get(text, text))
            action_notes.put(text)
        elif text.startswith("ODOM ") and odom_samples is not None:
            fields = text.split()
            if len(fields) != 5 or fields[4] == "0":
                continue
            try:
                odom_samples.put((float(fields[1]), float(fields[2]), float(fields[3]), time.monotonic()))
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
            _event("RFID", f"{card_number} 号")
            if map_rfid_events is not None and 1 <= card_number <= 12:
                map_rfid_events.put((card_number, generation, time.monotonic()))
            if rfid_enabled:
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


def _finite_action_active(
    entrance, junction_turn, rfid_turn, rfid_arrival, backup, recovery=None
):
    """有限动作期间不能插入 CMD_VEL。视觉倒车本身就是负速度，必须继续下发。"""
    recovery_phase = "idle" if recovery is None else recovery.phase
    return (entrance.phase in ("forward", "stopping_wait", "stopping", "turning")
            or junction_turn.phase in ("forward", "stopping", "stopped", "turning")
            or rfid_turn.phase in ("searching", "stopping_wait", "turning")
            or rfid_arrival.phase in ("blind_forward", "stopping_wait")
            or recovery_phase in ("holding", "stopping", "stopped", "turning"))


def _invalidate_turn_frames(before, states, stream, smoother):
    """TURN_DONE 后当前/在途画面可能拍于转弯前，重新识路必须使用新一代帧。"""
    changed = [state for phase, state in zip(before, states)
               if phase == "turning" and state.phase not in ("turning", "fault")]
    if not changed:
        return False
    for state in changed:
        # 状态函数可能已把当前旧画面计入 clear；撤回这些证据，帧数阈值仍用原配置。
        state.phase, state.clear = "reacquire", 0
    smoother.reset()
    stream.invalidate()
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="道路分割寻线，经串口速度环驱动")
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "nav_camera.yaml")
    parser.add_argument("--model", type=Path, default=ROOT / "models" / "road_yolo11n_seg.rknn")
    # 已通过等价性和性能验证的组合成为默认值；旧正向参数保留兼容历史脚本，
    # 无须为了使用优化追加启动参数。负向选项仅用于定位问题或对照回放。
    parser.add_argument("--fps-opt", action=argparse.BooleanOptionalAction, default=True,
                        help="默认复用 IPM/BEV 和分割临时缓冲")
    parser.add_argument("--lazy-raw-centerline", action="store_true", help="实验：普通中心线仅回退时计算，诊断点数不变")
    parser.add_argument("--correct-nms", action=argparse.BooleanOptionalAction, default=True,
                        help="默认正确 NMS：框使用左上角和宽高")
    parser.add_argument("--npu-core-mask", type=int, choices=(1,2,4,3,7), default=None,
                        help="实验：单 context 的 NPU 核心位掩码；默认保持 0 号核")
    parser.add_argument("--npu-contexts", type=int, choices=(1,2,3), default=None,
                        help="RKNN 默认两个独立 context，绑定核 1、2；核 0 留给目标检测；ONNX 或指定单核时自动单 context")
    parser.add_argument("--latest-frame", action="store_true",
                        help="实验：独立读取线程只保留一帧，空闲 context 取最新画面")
    parser.add_argument("--latest-result-order", choices=("completion", "capture"),
                        default="completion", help="实验：最新帧结果按完成顺序或输入顺序消费")
    parser.add_argument("--rknn-backend", choices=("lite", "c-standard", "c-input-zero"),
                        default="lite", help="实验：RKNN IO 后端；C 路径需要先编译原生组件")
    parser.add_argument("--async-record-video", action=argparse.BooleanOptionalAction, default=None,
                        help="请求录像时默认使用有界异步编码队列")
    parser.add_argument("--opencv-threads", type=int, default=8, help="OpenCV 默认 8 线程")
    parser.add_argument("--blas-threads", type=int, default=1, help="BLAS 默认单线程，避免争抢多路推理资源")
    parser.add_argument("--heap-trim-interval", type=float, default=60 if sys.platform == 'linux' else 0,
                        help="Linux 默认每 60 秒 GC 并归还空闲堆页；0 关闭")
    parser.add_argument("--drive", action="store_true", help="打开串口并使能，按寻线速度行驶")
    parser.add_argument("--culvert-stop", action="store_true", help="启用已标定的涵洞中央停车与本次任务去重")
    parser.add_argument("--culvert-estimated-camera", action="store_true",
                        help="涵洞试运行：使用 --config 中原相机高度、倾角及内参估算距离，精度尚未实测验证")
    parser.add_argument("--culvert-config", type=Path, default=ROOT / "config" / "culvert.yaml")
    parser.add_argument("--culvert-inspect", action="store_true", help="显式启用涵洞两侧识别和 PWM；替换 5 秒占位任务")
    parser.add_argument("--inspection-config", type=Path, default=ROOT / "config" / "culvert_inspection.json")
    parser.add_argument("--speech-config", type=Path, default=ROOT / "config" / "speech.json",
                        help="32 条音轨的已测占音时长；缺项时阻止新识别播报")
    parser.add_argument("--inspection-web", action="store_true", help="导航期间推流侧视相机并允许网页持久化调参")
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
        help="启用拓扑任务；开口按地图和驶入方向判断，FORWARD_DONE 后由 Agent 决定转向",
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
    args.inspection_settings = None
    if args.inspection_web and not args.culvert_inspect:
        parser.error("--inspection-web 必须与 --culvert-inspect 一起使用")
    if args.culvert_inspect and not args.culvert_stop:
        parser.error("--culvert-inspect 必须与 --culvert-stop 一起使用")
    if args.npu_contexts is None:
        # 两个分割实例分别处理不同帧，核 0 留给目标检测；保留桌面与显式单核覆盖。
        args.npu_contexts = 2 if args.model.suffix == '.rknn' and args.npu_core_mask is None else 1
    if args.async_record_video is None:
        args.async_record_video = args.record_video is not None
    for value in (args.opencv_threads, args.blas_threads):
        if value is not None and value < 1:
            parser.error("CPU 线程数必须大于零")
    if args.culvert_estimated_camera and not args.culvert_stop:
        parser.error("--culvert-estimated-camera 必须与 --culvert-stop 同时使用")
    # 在打开模型、相机或串口之前验证已选标定；整个进程保持同一份快照。
    from vision.ipm_proto.manual_ground import runtime_activation
    try:
        active_ground = runtime_activation()
        if active_ground is not None:
            ground_cfg = _load_config(args.config)
            capture = ground_cfg.get("capture", {}) or {}
            make_ipm(ground_cfg, (int(capture["height"]), int(capture["width"])))
            print(f"地面投影：手动标定 {active_ground['calibration_file']}；仅重启导航后切换", file=sys.stderr)
    except (OSError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))
    if args.culvert_stop:
        # 在频率保护、模型初始化、相机或串口打开之前拒绝不完整配置。
        if not args.drive or not (args.left_at_junction or args.turn_at_junction):
            parser.error("--culvert-stop 需要 --drive 和拓扑任务；离线观察请用 road_follow.culvert_replay")
        if args.turn_at_rfid or args.backup_on_obstacle:
            parser.error("涵洞停车只用于拓扑任务，不能组合独立RFID/倒车测试")
        from road_follow.culvert_runtime import validate_culvert_config
        try:
            nav_cfg = _load_config(args.config)
            camera = nav_cfg.get("capture", {}) or {}
            args.culvert_setup = validate_culvert_config(args.culvert_config, ROOT, drive=True,
                image_size=(int(camera.get("width",1280)),int(camera.get("height",720))),
                near_speed=float((nav_cfg.get("follow") or {}).get("near_mps",.05)),
                estimated_camera=args.culvert_estimated_camera, nav_config=nav_cfg)
            if args.culvert_estimated_camera and active_ground is None:
                print(f"涵洞估算试运行：{args.culvert_setup[2].metadata['camera']}；中央停车精度待实测确认", file=sys.stderr)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    if args.culvert_inspect:
        from road_follow.inspection_config import Settings, preflight
        try:
            args.inspection_settings = Settings(args.inspection_config)
            capture_cfg = _load_config(args.config).get("capture", {}) or {}
            preflight(args.inspection_settings, ROOT, capture_cfg.get("device", "/dev/video0"))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    if needs_frequency_guard(args.model):
        # 父保护进程先保存频率，再以当前普通用户重启同一入口；退出/中断负责恢复。
        # 不把密码写入程序，也不要求额外优化参数。sudo 按现有系统策略认证。
        arguments = list(argv) if argv is not None else sys.argv[1:]
        return run_guarded([sys.executable, '-m', 'road_follow', *arguments])
    # 线程预算覆盖完整运行，worker 全部结束后才恢复，避免在途矩阵运算受影响。
    with cpu_thread_budget(args.opencv_threads, args.blas_threads):
        with HeapReclaimer(args.heap_trim_interval):
            return _run(args, parser)


def _run(args, parser) -> int:
    """解析与运行分开，保证模型、采集和异常退出均位于线程预算作用域内。"""
    if args.async_record_video and args.record_video is None:
        parser.error("--async-record-video 需要 --record-video")
    if args.latest_result_order != "completion" and not args.latest_frame:
        parser.error("--latest-result-order capture 需要 --latest-frame")
    if (args.npu_contexts > 1 or args.latest_frame) and (args.npu_core_mask is not None or args.model.suffix != ".rknn"):
        parser.error("多 context 需要 RKNN 模型，并且不能与 --npu-core-mask 同时使用")

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
    record_path = None
    if args.record_video is not None:
        record_path = (
            ROOT / "data" / "road" / f"drive_{datetime.now():%Y%m%d_%H%M%S_%f}.avi"
            if args.record_video == ""
            else Path(args.record_video)
        )
        if record_path.suffix.lower() != ".avi" or record_path.exists():
            parser.error("录像必须使用新的 .avi 路径，不能覆盖已有文件")

    cfg = _load_config(args.config)
    from road_follow.speech import SpeechConfig
    try:
        anchor_enabled = heading_anchor_enabled_from_mapping(cfg)
        speech_config = SpeechConfig.load(args.speech_config)
    except (OSError, ValueError) as exc:
        _event("错误", f"航向/播报配置无效：{exc}")
        return 1
    if turn_side is not None:
        _event("航向校正", "已启用路口参考同步，旧固件将拒绝启动" if anchor_enabled
               else "已显式关闭中途参考校正，当前模式不更新下位机动作航向参考")
    if args.culvert_inspect and not speech_config.ready:
        _event("播报", f"识别播报已阻止：缺少已测音轨时长 {','.join(map(str, speech_config.missing_ids))}；UID 原流程保留")
    capture_cfg = cfg.get("capture", {}) or {}
    uart_cfg = cfg.get("uart", {}) or {}
    device = str(capture_cfg.get("device", "/dev/video0"))
    width = int(capture_cfg.get("width", 1280))
    height = int(capture_cfg.get("height", 720))

    uart_bin = args.uart_bin or (ROOT / "build" / "uart" / "uart_vel")
    if args.drive and not uart_bin.is_file():
        print(f"找不到 {uart_bin}，先编译 uart_vel", file=sys.stderr)
        return 1
    segmenter, segment_stream = None, None
    if args.npu_contexts > 1 or args.latest_frame:
        stream_type = LatestSegmentStream if args.latest_frame else OrderedSegmentStream
        stream_options = {"result_order": args.latest_result_order} if args.latest_frame else {}
        cores = (2,4) if args.npu_contexts == 2 else (1,2,4)[:args.npu_contexts]
        segment_stream = stream_type(args.model, cores=cores,
            correct_nms=args.correct_nms, reuse_buffers=args.fps_opt, backend=args.rknn_backend,
            factory=RoadSegmenter,
            **stream_options)
        segment_stream.warmup(np.zeros((height, width, 3), dtype=np.uint8))
    else:
        segmenter = RoadSegmenter(args.model, correct_nms=args.correct_nms,
                                  reuse_buffers=args.fps_opt, core_mask=args.npu_core_mask,
                                  backend=args.rknn_backend)
        try:
            segmenter.mask(np.zeros((height, width, 3), dtype=np.uint8))
        except BaseException:
            segmenter.close()
            raise
    smoother = temporal_from_mapping(cfg)
    junctions = JunctionTracker()
    projector = BevProjector() if args.fps_opt else None
    bridge: subprocess.Popen | None = None
    if args.drive:
        serial = str(uart_cfg.get("device", "/dev/ttyS6"))
        baud = str(int(uart_cfg.get("baud", 921600)))
        bridge = subprocess.Popen(
            [str(uart_bin), "--device", serial, "--baud", baud]
            + (["--require-heading-anchor"] if turn_side is not None and anchor_enabled else [])
            + speech_config.bridge_args(),
            stdin=subprocess.PIPE,
            # 始终由监视线程消费 ODOM 等高频内部消息，避免直接刷满终端。
            stdout=subprocess.PIPE,
            text=True,
        )
    action_notes: queue.Queue[str] = queue.Queue()
    rfid_events: queue.Queue[tuple[int, int]] = queue.Queue()
    odom_samples: queue.Queue[tuple[float, float, float, float]] = queue.Queue()
    map_rfid_events = queue.Queue() if args.culvert_stop else None
    anchor_notes: queue.Queue[tuple[str, float]] = queue.Queue()
    speech_notes: queue.Queue[str] = queue.Queue()
    if bridge is not None:
        threading.Thread(
            target=_watch_uart_notes,
            args=(
                bridge,
                action_notes,
                rfid_events,
                odom_samples,
                turn_side is None,
                map_rfid_events,
                anchor_notes,
                speech_notes,
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
        try:
            route_agent, node_xy, first = _start_route()
        except (KeyError, ValueError) as exc:
            _event("错误", f"拓扑任务加载失败：{exc}")
            _stop_bridge(bridge)
            if segment_stream is not None:
                segment_stream.close()
            elif segmenter is not None:
                segmenter.close()
            return 1
        if type(first).__name__ != "FollowEdge":
            print(f"Agent 没有给出第一条路: {first}", file=sys.stderr)
            _stop_bridge(bridge)
            if segment_stream is not None:
                segment_stream.close()
            elif segmenter is not None:
                segmenter.close()
            return 1
        _event("路线", f"开始路段 {first.edge_id}：{first.from_node} → {first.to_node}")
    detector = None
    obstacle_judge = None
    obstacle_edge_key = None
    obstacle_recovery = ObstacleRecovery() if route_agent is not None else None
    if args.backup_on_obstacle or obstacle_recovery is not None:
        from vision.obstacle.blockage import HardBlockageJudge, hard_block_config_from_mapping
        from vision.obstacle.detect import ObstacleDetector

        try:
            detector = ObstacleDetector(args.culvert_config if args.culvert_stop else ROOT / "config" / "obstacle.yaml", root=ROOT)
            obstacle_judge = HardBlockageJudge(
                hard_block_config_from_mapping(detector.config)
            )
        except (OSError, RuntimeError, ValueError, ImportError) as exc:
            _event("错误", f"障碍模型加载失败：{exc}")
            _stop_bridge(bridge)
            if segment_stream is not None:
                segment_stream.close()
            elif segmenter is not None:
                segmenter.close()
            return 1
    rfid_turn = RfidTurn(side=args.turn_at_rfid or "none")
    rfid_turn_cfg = rfid_turn_config_from_mapping(cfg)
    rfid_arrival = RfidArrival()
    rfid_arrival_cfg = rfid_arrival_config_from_mapping(cfg)
    arrival_policy = None
    arrival_key = None
    if route_agent is not None:
        try:
            # 启动相机和运动前检查所有可行驶方向，错误配置不能在途中才发现。
            for route_edge in route_agent.graph.edges.values():
                pairs = [(route_edge.u, route_edge.v)]
                if route_edge.bidirectional:
                    pairs.append((route_edge.v, route_edge.u))
                for source, destination in pairs:
                    resolve_arrival_policy(
                        route_agent.graph, source, destination, route_edge.length_m,
                        junction_turn_cfg, rfid_arrival_cfg,
                    )
        except ValueError as exc:
            _event("错误", f"到点策略配置无效：{exc}")
            _stop_bridge(bridge)
            if segment_stream is not None:
                segment_stream.close()
            elif segmenter is not None:
                segmenter.close()
            return 1

    culvert_runtime = None
    if args.culvert_stop:
        from road_follow.culvert_runtime import CulvertRuntime, culvert_eligible
        try:
            culvert_runtime = CulvertRuntime(*args.culvert_setup, nav_config=cfg, agent=route_agent,
                send=lambda line: write_velocity(bridge, line), event=_event, root=ROOT,
                inspection_settings=args.inspection_settings, inspection_web=args.inspection_web,
                speech_config=speech_config)
        except BaseException as exc:
            _event("错误", f"涵洞初始化失败：{exc}")
            _stop_bridge(bridge)
            _finish_resources(None, time.monotonic(), detector, None, segment_stream, segmenter, False)
            if not isinstance(exc, (OSError, ValueError)):
                raise
            return 1

    stopping = False
    last_status_signature: tuple[object, ...] | None = None
    last_status_s = 0.0
    last_camera_warning_s = 0.0
    last_arrival_reason = ""
    recording_started = False

    def _reset_progress():
        if culvert_runtime is not None and culvert_runtime.controller.owns:
            culvert_runtime.controller.fault("progress_reset_during_culvert", time.monotonic())
            return
        progress.reset()
        if obstacle_judge is not None:
            obstacle_judge.reset()
        if culvert_runtime is not None:
            culvert_runtime.reset_progress_origin()

    def _record_frame(frame, captured_s):
        nonlocal recording_started
        if culvert_runtime is not None and culvert_runtime.first_capture_s is None:
            culvert_runtime.first_capture_s = captured_s
        if recorder is not None:
            recorder.write(frame, captured_s)
            if not recording_started:
                _event("录像", f"开始录制：{recorder.path}")
                recording_started = True

    def _request_stop(signum, _frame) -> None:
        del signum
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    capture: cv2.VideoCapture | None = None
    frames = 0
    try:
        # 所有前置初始化成功后才启动编码线程，避免模型/串口初始化失败时线程滞留。
        if record_path is not None:
            recorder = (AsyncVideoRecorder(record_path) if args.async_record_video
                        else VideoRecorder(record_path))
        capture = _open_camera(device, width, height)
        while not stopping:
            culvert_outcome = None
            while True:
                try:
                    speech_note = speech_notes.get_nowait()
                except queue.Empty:
                    break
                if culvert_runtime is not None:
                    culvert_runtime.handle_speech_note(speech_note)
                else:
                    _event("播报回执", speech_note)
            if culvert_runtime is not None:
                while True:
                    try:
                        sample = odom_samples.get_nowait()
                    except queue.Empty:
                        break
                    progress.update(*sample)
                    culvert_runtime.record_odom(sample, progress)
                culvert_runtime.records.set_vehicle(culvert_runtime.edge_key(), progress.s_m, culvert_runtime.controller.phase)
                while True:
                    try:
                        card, generation, received_s = map_rfid_events.get_nowait()
                    except queue.Empty:
                        break
                    culvert_runtime.records.mark_rfid(card, generation, received_s)
                    culvert_runtime.event("topology_rfid", card_number=card, generation=generation, received_s=received_s)
            if segment_stream is not None:
                try:
                    result = segment_stream.read(capture, _record_frame)
                    ok, frame = True, result.image
                except CameraReadError:
                    segment_stream.invalidate()
                    ok, frame = False, None
            else:
                ok, frame = capture.read()
                captured_s = time.monotonic()
            if not ok:
                if culvert_runtime is not None:
                    culvert_runtime.unsafe_frame(time.monotonic(), "camera_failed")
                now_s = time.monotonic()
                if now_s - last_camera_warning_s >= STATUS_LOG_INTERVAL_S:
                    _event("警告", "相机读取失败，已发送停车指令")
                    last_camera_warning_s = now_s
                if bridge is not None and not (culvert_runtime is not None and culvert_runtime.controller.owns) and not write_velocity(bridge, "0.000 0.000"):
                    break
                time.sleep(0.05)
                continue

            if segment_stream is not None:
                mask, infer_s = result.mask, result.inference_s
                captured_s = result.captured_s
            else:
                _record_frame(frame, captured_s)
                t0 = time.monotonic()
                mask = segmenter.mask(frame)
                infer_s = time.monotonic() - t0
            turn_states = (entrance, junction_turn, rfid_turn)
            turn_phases_before = tuple(state.phase for state in turn_states)
            # 投影仅当前帧复用；状态机和 EMA 仍在原来的单线程中按原帧序更新。
            projection = projector.project(mask, cfg) if projector else None
            command, follow_diag = command_from_mask_with_diagnostics(
                mask, cfg, smoother, projection=projection, lazy_raw=args.lazy_raw_centerline
            )
            opening, junction = _junction_read(mask, cfg, junctions, projection=projection)
            if turn_side is not None:
                while True:
                    try:
                        sample = odom_samples.get_nowait()
                    except queue.Empty:
                        break
                    progress.update(*sample)
                    if culvert_runtime is not None:
                        culvert_runtime.record_odom(sample, progress)
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
                        _reset_progress()
            elif turn_side is not None and bridge is not None:
                recovery_owns = False
                detections = []
                if obstacle_recovery is not None and route_agent is not None:
                    current_key = (route_agent.state.current_edge, route_agent.state.from_node, route_agent.state.to_node)
                    if current_key != obstacle_edge_key:
                        if obstacle_judge is not None:
                            obstacle_judge.reset()
                        obstacle_edge_key = current_key
                    armed = detection_armed(
                        entrance_done=True,
                        agent_phase=route_agent.state.phase,
                        junction_phase=junction_turn.phase,
                        recovery_phase=obstacle_recovery.phase,
                    )
                    if culvert_runtime is not None:
                        armed = armed or culvert_runtime.controller.owns or culvert_eligible(
                            entrance, route_agent, junction_turn, obstacle_recovery, rfid_arrival)
                    confirmed = False
                    if armed and detector is not None and obstacle_judge is not None:
                        detections, _elapsed_ms = detector.detect(frame)
                        if culvert_runtime is not None:
                            hard_detections, _, _ = culvert_runtime.split(detections)
                        else:
                            hard_detections = detections
                        # 检测结束后补齐 ODOM，再使用采集时刻的当前边进度判断目标归属。
                        while True:
                            try:
                                sample = odom_samples.get_nowait()
                            except queue.Empty:
                                break
                            progress.update(*sample)
                            if culvert_runtime is not None:
                                culvert_runtime.record_odom(sample, progress)
                        from road_follow.obstacle_edge import filter_current_edge, OBSTACLE_DEFAULTS
                        edge_settings = OBSTACLE_DEFAULTS
                        capture_progress = progress.s_m
                        obstacle_ipm = projection[0] if projection else make_ipm(cfg, frame.shape)
                        if culvert_runtime is not None:
                            edge_settings = culvert_runtime.obstacle_settings(obstacle_judge)
                            capture_progress = culvert_runtime.history.progress_at(captured_s)
                            obstacle_ipm = culvert_runtime.calibration.ipm
                        hard_detections, obstacle_details = filter_current_edge(
                            hard_detections, graph=route_agent.graph,
                            edge_key=(route_agent.state.current_edge, route_agent.state.from_node, route_agent.state.to_node),
                            progress_m=capture_progress, ipm=obstacle_ipm, image_shape=frame.shape,
                            settings=edge_settings, lane_x_m=follow_diag.near_x_m)
                        observation = obstacle_judge.update(hard_detections, frame.shape)
                        confirmed = observation.just_confirmed
                        if culvert_runtime is not None:
                            culvert_runtime.observe_obstacles(obstacle_details, observation)
                        if confirmed:
                            _event(
                                "障碍物",
                                f"确认道路被阻挡；置信度={observation.score:.2f}，"
                                f"连续={observation.stable_frames} 帧，"
                                f"路段进度={progress.s_m:.2f} m",
                            )
                    if obstacle_recovery.phase != "idle" or confirmed:
                        if confirmed and culvert_runtime is not None:
                            culvert_runtime.cancel_for_obstacle(time.monotonic(), action_notes)
                        recovery_owns = True
                        recovery_before = obstacle_recovery.phase
                        obstacle_recovery, backup, outcome = step_obstacle_route(
                            obstacle_recovery,
                            backup,
                            armed=armed,
                            confirmed=confirmed,
                            agent=route_agent,
                            node_xy=node_xy,
                            progress_s_m=progress.s_m,
                            near_x_m=follow_diag.near_x_m,
                            command=command,
                            now_s=time.monotonic(),
                            notes=action_notes,
                            send=lambda line: write_velocity(bridge, line),
                            backup_cfg=backup_cfg,
                            stop_settle_s=junction_turn_cfg.stop_settle_s,
                            reacquire_frames=junction_turn_cfg.reacquire_frames,
                        )
                        command = outcome.command
                        if culvert_runtime is not None:
                            culvert_runtime.event("obstacle_backup_control", phase=obstacle_recovery.phase,
                                progress_m=progress.s_m, near_x_m=follow_diag.near_x_m,
                                lane_heading_rad=follow_diag.lane_heading_rad,
                                command={"v_mps":command.v_mps,"omega_radps":command.omega_radps,"reason":command.reason})
                        if outcome.reset_smoother:
                            smoother.reset()
                            if segment_stream is not None:
                                segment_stream.invalidate()
                        if outcome.reset_progress:
                            _reset_progress()
                            junction_turn.phase = "follow"
                            junction_turn.suppress_cue = True
                            junction_turn.departure = "none"
                            junction_turn.side = "none"
                            junction_turn.clear = 0
                            junction_turn.branch_latched = False
                            junction_turn.branch_votes.clear()
                            rfid_arrival = RfidArrival()
                            arrival_key = None
                            _event(
                                "路线",
                                "障碍边已封闭，下一路段从 "
                                f"{route_agent.state.from_node} 重新计里程",
                            )
                        if obstacle_recovery.phase != recovery_before:
                            turn = ""
                            if obstacle_recovery.side in ("left", "right"):
                                times = "两次" if obstacle_recovery.quarters_remaining > 1 else ""
                                turn = (
                                    f"，转向={times}"
                                    f"{DIRECTION_NAMES[obstacle_recovery.side]}"
                                )
                            _event(
                                "障碍回退",
                                f"{_phase_name(recovery_before)} → "
                                f"{_phase_name(obstacle_recovery.phase)}；"
                                f"路段进度={progress.s_m:.2f} m{turn}",
                            )
                if not recovery_owns and culvert_runtime is not None:
                    # 补入YOLO期间到达的ODOM，再按采集时刻插值，不拿处理结束的里程加旧距离。
                    while True:
                        try:
                            sample = odom_samples.get_nowait()
                        except queue.Empty:
                            break
                        progress.update(*sample)
                        culvert_runtime.record_odom(sample, progress)
                    was_inspecting = culvert_runtime.controller.phase == "task"
                    culvert_outcome = culvert_runtime.update(detections=detections, frame=frame, mask=mask,
                        captured_s=captured_s, sequence=result.sequence if segment_stream is not None else frames,
                        source=result.source if segment_stream is not None else 0,
                        now=time.monotonic(), progress=progress, command=command,
                        eligible=culvert_eligible(entrance, route_agent, junction_turn, obstacle_recovery, rfid_arrival),
                        notes=action_notes, navigation_diagnostics=follow_diag, inference_s=infer_s)
                    command = culvert_outcome.command
                    if was_inspecting and culvert_runtime.controller.phase == "reacquire":
                        # 识别结束后丢弃旧分割任务，再用新画面重新取路。
                        smoother.reset()
                        if segment_stream is not None:
                            segment_stream.invalidate()
                    recovery_owns = culvert_outcome.owns
                    if culvert_outcome.resumed:
                        culvert_runtime.resume(junction_turn, rfid_arrival, junctions, smoother, segment_stream)
                        junctions = JunctionTracker()
                if not recovery_owns:
                    previous_phase = junction_turn.phase
                    visual_command = command
                    target = None
                    if route_agent is not None and route_agent.state.current_edge:
                        edge = route_agent.graph.edges[route_agent.state.current_edge]
                        target = route_agent.graph.nodes.get(route_agent.state.to_node or "")
                        key = (edge.id, route_agent.state.from_node, target.id)
                        if key != arrival_key:
                            arrival_policy = resolve_arrival_policy(
                                route_agent.graph, route_agent.state.from_node, target.id,
                                edge.length_m, junction_turn_cfg, rfid_arrival_cfg,
                            )
                            arrival_key = key
                            rfid_arrival = RfidArrival()
                            guard_label = (
                                "禁用（尽头巡检点固定距离保护）"
                                if (
                                    target.role == "patrol_slot"
                                    and arrival_policy.mode == "visual_end"
                                )
                                else f"{arrival_policy.guard_progress_m:.2f} m"
                            )
                            _event(
                                "到点策略",
                                f"驶入={arrival_policy.from_node} → {target.id}，"
                                f"策略={arrival_policy.mode}，"
                                f"预期开口={','.join(arrival_policy.expected_openings) or '无'}，"
                                f"交接={arrival_policy.handoff_progress_m:.2f} m，"
                                f"定距={arrival_policy.final_forward_m:.2f} m，"
                                f"保护={guard_label}",
                            )
                    if target is not None and arrival_policy is not None:
                        previous_rfid_phase = rfid_arrival.phase
                        follow_cfg = cfg.get("follow", {}) or {}
                        visual_safe = (
                            command.reason != "stop_camera"
                            and follow_diag.bev_road_pixels
                            >= int(follow_cfg.get("min_road_pixels", 200))
                            and follow_diag.output_points
                            >= int(follow_cfg.get("min_points", 8))
                        )
                        junction_turn, rfid_arrival, command, trigger = step_route_arrival(
                            arrival_policy,
                            patrol=target.role == "patrol_slot",
                            state=junction_turn, patrol_state=rfid_arrival,
                            reading=junction, opening=opening, command=command,
                            progress_m=progress.s_m,
                            odom_valid=progress.is_fresh(time.monotonic()),
                            notes=action_notes,
                            send=lambda line: write_velocity(bridge, line),
                            junction_cfg=junction_turn_cfg,
                            patrol_cfg=rfid_arrival_cfg,
                            visual_safe=visual_safe,
                            near_x_m=follow_diag.near_x_m,
                            lane_heading_rad=follow_diag.lane_heading_rad,
                            yaw_rad=progress.yaw_rad,
                            centerline_points=follow_diag.centerline_points,
                            road_pixels=follow_diag.bev_road_pixels,
                            follow=follow_config_from_mapping(cfg),
                            frame_captured_s=captured_s,
                            odom_received_s=progress.last_sample_s,
                            anchor_notes=anchor_notes,
                        )
                        if trigger and (
                            trigger not in ("guard", "odom_stale")
                            or previous_phase != junction_turn.phase
                            or command.reason != last_arrival_reason
                        ):
                            _event(
                                "安全停车" if trigger in ("guard", "odom_stale") else "到点交接",
                                f"目标={target.id}，策略={arrival_policy.mode}，"
                                f"原因={trigger}，进度={progress.s_m:.2f} m，"
                                f"检测带={junction.forward_band_ratio:.2f}",
                            )
                        last_arrival_reason = command.reason
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
                            rfid_arrival = RfidArrival()
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
                    else:
                        # 完成/故障时 Agent 没有下一目标，不能回落到视觉前进指令。
                        command = VelocityCommand(
                            0.0, 0.0, "done" if junction_turn.phase == "done" else "stop_action_fail"
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
                                _reset_progress()
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
                        # 贯通点和尽头点确认到达后都清沿边里程，下一条从 0 计。
                        _reset_progress()
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
                        _reset_progress()
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

            if segment_stream is not None and _invalidate_turn_frames(
                    turn_phases_before, turn_states, segment_stream, smoother):
                junctions = JunctionTracker()
                command = VelocityCommand(0.0, 0.0, "reacquire")
            line = f"{command.v_mps:.3f} {command.omega_radps:.3f}"
            status_signature = _status_signature(
                command,
                entrance,
                junction_turn,
                rfid_turn,
                rfid_arrival,
                backup,
                "idle" if obstacle_recovery is None else obstacle_recovery.phase,
            )
            now_s = time.monotonic()
            if (
                status_signature != last_status_signature
                or now_s - last_status_s >= STATUS_LOG_INTERVAL_S
            ):
                details = [
                    COMMAND_NAMES.get(command.reason, command.reason),
                    f"速度={command.v_mps:.3f} m/s",
                    f"路段进度={progress.s_m:.2f} m",
                    f"路口={JUNCTION_NAMES.get(opening, opening)}",
                ]
                _event("状态", "；".join(details))
                last_status_signature = status_signature
                last_status_s = now_s
            if (culvert_outcome is not None and not culvert_outcome.send_velocity) or _finite_action_active(entrance, junction_turn, rfid_turn, rfid_arrival, backup, obstacle_recovery):
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
        if culvert_runtime is not None:
            culvert_runtime.close()
        try:
            # latest 的录像回调运行在读取线程；先结束生产者再关闭编码器。
            if args.latest_frame and segment_stream is not None:
                segment_stream.close()
        finally:
            _finish_resources(recorder, recording_stopped_s, detector, capture,
                              segment_stream, segmenter, args.latest_frame)
    return 0


def _finish_resources(recorder, stopped_s, detector, capture, stream, segmenter, latest):
    """保持异常清理顺序，任何一路释放失败也继续释放其余资源。"""
    try:
        if recorder is not None:
            recorder.close(stopped_s)
            if recorder.frames_written:
                _event("录像", f"已保存：{recorder.path}（{recorder.frames_written} 帧）")
    finally:
        # 编码线程异常也要释放相机和 NPU；否则下一次启动会残留资源。
        try:
            if detector is not None:
                detector.close()
        finally:
            if stream is not None:
                try:
                    stream.close()
                finally:
                    # latest 模式由读取线程释放句柄，不能和 read 并发释放。
                    if capture is not None and not (latest and getattr(stream, 'owns_capture', False)):
                        capture.release()
            elif segmenter is not None:
                try:
                    segmenter.close()
                finally:
                    if capture is not None:
                        capture.release()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
