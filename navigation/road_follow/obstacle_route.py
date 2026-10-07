"""拓扑循迹中的硬堵塞：用本段沿边进度倒回入口，再按车头转向。"""

from __future__ import annotations

import math
import queue
from dataclasses import dataclass

from road_follow.backup import Backup, BackupConfig, step_backup
from road_follow.control import VelocityCommand, is_visual_follow
from road_follow.departure import NodeXY


def detection_armed(
    *,
    entrance_done: bool,
    agent_phase: str,
    junction_phase: str,
    recovery_phase: str,
) -> bool:
    """只在当前边的循迹阶段看障碍。交接、转弯和倒车期间不确认。"""
    return (
        entrance_done
        and agent_phase == "following"
        and junction_phase == "follow"
        and recovery_phase == "idle"
    )


@dataclass(frozen=True)
class RecoveryTurn:
    """回到入口后的原地转向。身后出口是两次同向 90°。"""

    side: str
    quarters: int


def recovery_maneuver(
    node_xy: NodeXY,
    at_node: str,
    facing_node: str,
    depart_to: str,
) -> RecoveryTurn:
    """车头仍指向被封边的终点。约 0° 表示新路仍是这条被封走廊。"""
    if depart_to == facing_node:
        raise ValueError("下一条路仍指向被封走廊")
    ax, ay = node_xy[at_node]
    fx, fy = node_xy[facing_node]
    dx, dy = node_xy[depart_to]
    hx, hy = fx - ax, fy - ay
    ox, oy = dx - ax, dy - ay
    if math.hypot(hx, hy) < 1e-6 or math.hypot(ox, oy) < 1e-6:
        raise ValueError("路口相邻节点重合，无法判断转向")
    angle = math.atan2(hx * oy - hy * ox, hx * ox + hy * oy)
    if abs(angle) <= math.radians(45):
        raise ValueError("下一条路与被封走廊同向")
    if abs(angle) >= math.radians(135):
        return RecoveryTurn("left", 2)
    if angle > 0:
        return RecoveryTurn("left", 1)
    return RecoveryTurn("right", 1)


REVERSE_HOLD_S = 1.0


@dataclass
class ObstacleRecovery:
    """idle → holding → backing → stopping → turning → reacquire → idle。"""

    phase: str = "idle"
    facing_node: str | None = None
    side: str = "none"
    quarters_remaining: int = 0
    stop_started_s: float = 0.0
    stop_settle_s: float = 2.0
    hold_started_s: float | None = None
    reverse_hold_s: float = REVERSE_HOLD_S
    clear: int = 0
    reacquire_frames: int = 2


@dataclass(frozen=True)
class ObstacleRouteOutcome:
    command: VelocityCommand
    reset_progress: bool = False
    reset_smoother: bool = False


def _stop(reason: str) -> ObstacleRouteOutcome:
    return ObstacleRouteOutcome(VelocityCommand(0.0, 0.0, reason))


def _fail(recovery: ObstacleRecovery, reason: str) -> ObstacleRouteOutcome:
    recovery.phase = "fault"
    return _stop(reason)


def step_obstacle_route(
    recovery: ObstacleRecovery,
    backup: Backup,
    *,
    armed: bool,
    confirmed: bool = False,
    agent,
    node_xy: NodeXY,
    progress_s_m: float,
    near_x_m: float | None,
    command: VelocityCommand,
    now_s: float,
    notes: queue.Queue[str],
    send,
    backup_cfg: BackupConfig | None = None,
    stop_settle_s: float = 2.0,
    reverse_hold_s: float = REVERSE_HOLD_S,
    reacquire_frames: int = 2,
) -> tuple[ObstacleRecovery, Backup, ObstacleRouteOutcome]:
    """确认后先停车停稳，再倒到沿边进度为 0。新边进度要等转向结束才清零。"""
    if recovery.phase == "fault":
        return recovery, backup, _stop("stop_action_fail")
    if recovery.phase == "done":
        return recovery, backup, _stop("done")
    if recovery.phase == "idle" and not (armed and confirmed):
        return recovery, backup, ObstacleRouteOutcome(command)

    if recovery.phase == "idle":
        edge_id = agent.state.current_edge
        if not edge_id:
            return recovery, backup, _fail(recovery, "stop_action_fail")
        confirmed_cmd = agent.obstacle_confirmed(edge_id, progress_s_m)
        if type(confirmed_cmd).__name__ != "StartBackup":
            return recovery, backup, _fail(recovery, "stop_action_fail")
        recovery.facing_node = agent.state.to_node
        recovery.stop_settle_s = max(0.0, stop_settle_s)
        recovery.reverse_hold_s = max(0.0, reverse_hold_s)
        recovery.reacquire_frames = max(1, reacquire_frames)
        cfg = backup_cfg or BackupConfig()
        if progress_s_m <= cfg.done_progress_m:
            return _begin_turn(recovery, backup, agent, node_xy, send)
        if not send("stop"):
            return _fail_backup(recovery, backup, agent, "stop_fail")
        recovery.phase = "holding"
        recovery.hold_started_s = None
        return recovery, backup, _stop("backup_hold")

    if recovery.phase == "holding":
        _consume_hold_notes(recovery, notes, now_s)
        if recovery.phase == "fault":
            return _fail_backup(recovery, backup, agent, "stop_fail")
        if (
            recovery.hold_started_s is None
            or now_s - recovery.hold_started_s < recovery.reverse_hold_s
        ):
            return recovery, backup, _stop("backup_hold")
        return _start_reverse(
            recovery, backup, agent, node_xy, near_x_m, progress_s_m, command, now_s,
            backup_cfg, send,
        )

    if recovery.phase == "backing":
        backup, velocity = step_backup(
            backup, False, near_x_m, progress_s_m, command, now_s, backup_cfg
        )
        if backup.phase == "done":
            return _begin_turn(recovery, backup, agent, node_xy, send)
        if backup.phase == "fault":
            return _fail_backup(recovery, backup, agent, velocity.reason)
        return recovery, backup, ObstacleRouteOutcome(velocity)

    before = recovery.phase
    _consume_turn_notes(recovery, notes, send, now_s)
    if recovery.phase == "fault":
        return recovery, backup, _stop("stop_action_fail")
    if before != "reacquire" and recovery.phase == "reacquire":
        return recovery, backup, ObstacleRouteOutcome(
            VelocityCommand(0.0, 0.0, "reacquire"),
            reset_smoother=True,
        )
    if recovery.phase == "stopping":
        return recovery, backup, _stop("stopping")
    if recovery.phase == "stopped":
        if now_s - recovery.stop_started_s < recovery.stop_settle_s:
            return recovery, backup, _stop("junction_stop_settle")
        if not send(f"turn {recovery.side}"):
            return recovery, backup, _fail(recovery, "stop_action_fail")
        recovery.phase = "turning"
        return recovery, backup, _stop("turning")
    if recovery.phase == "turning":
        return recovery, backup, _stop("turning")
    if recovery.phase == "reacquire":
        if is_visual_follow(command):
            recovery.clear += 1
        else:
            recovery.clear = 0
        if recovery.clear >= recovery.reacquire_frames:
            recovery.phase = "idle"
            recovery.side = "none"
            recovery.quarters_remaining = 0
            recovery.clear = 0
            return recovery, backup, ObstacleRouteOutcome(command, reset_progress=True)
        return recovery, backup, _stop("reacquire")
    return recovery, backup, _fail(recovery, "stop_action_fail")


def _start_reverse(
    recovery: ObstacleRecovery,
    backup: Backup,
    agent,
    node_xy: NodeXY,
    near_x_m: float | None,
    progress_s_m: float,
    command: VelocityCommand,
    now_s: float,
    backup_cfg: BackupConfig | None,
    send,
) -> tuple[ObstacleRecovery, Backup, ObstacleRouteOutcome]:
    """停稳结束后才开始计倒车时间和距离。"""
    backup, velocity = step_backup(
        backup, True, near_x_m, progress_s_m, command, now_s, backup_cfg
    )
    recovery.phase = "backing"
    if backup.phase == "done":
        return _begin_turn(recovery, backup, agent, node_xy, send)
    if backup.phase == "fault":
        return _fail_backup(recovery, backup, agent, velocity.reason)
    return recovery, backup, ObstacleRouteOutcome(velocity)


def _consume_hold_notes(
    recovery: ObstacleRecovery, notes: queue.Queue[str], now_s: float
) -> None:
    while recovery.phase == "holding":
        try:
            note = notes.get_nowait()
        except queue.Empty:
            return
        if note == "STOP_DONE" and recovery.hold_started_s is None:
            recovery.hold_started_s = now_s
        elif note == "STOP_FAIL":
            recovery.phase = "fault"


def _begin_turn(
    recovery: ObstacleRecovery,
    backup: Backup,
    agent,
    node_xy: NodeXY,
    send,
) -> tuple[ObstacleRecovery, Backup, ObstacleRouteOutcome]:
    """倒车完成只重规划。贴脸入口也走这里，不把 backup_done_at_entry 当成故障。"""
    facing = recovery.facing_node
    edge_id = agent.state.current_edge
    from_node = agent.state.from_node
    if not facing or not edge_id or not from_node:
        return recovery, backup, _fail(recovery, "stop_action_fail")
    resumed = agent.backup_done(edge_id, from_node)
    if type(resumed).__name__ == "MissionComplete":
        recovery.phase = "done"
        return recovery, Backup(), _stop("done")
    if type(resumed).__name__ != "FollowEdge":
        return recovery, backup, _fail(recovery, "stop_action_fail")
    try:
        maneuver = recovery_maneuver(node_xy, from_node, facing, resumed.to_node)
    except (KeyError, ValueError):
        return recovery, Backup(), _fail(recovery, "stop_action_fail")
    recovery.side = maneuver.side
    recovery.quarters_remaining = maneuver.quarters
    backup = Backup()
    if not send("stop"):
        return recovery, backup, _fail(recovery, "stop_action_fail")
    recovery.phase = "stopping"
    return recovery, backup, _stop("stopping")


def _fail_backup(
    recovery: ObstacleRecovery,
    backup: Backup,
    agent,
    reason: str,
) -> tuple[ObstacleRecovery, Backup, ObstacleRouteOutcome]:
    edge_id = agent.state.current_edge or ""
    agent.backup_failed(edge_id, reason)
    return recovery, backup, _fail(recovery, "stop_backup_fault")


def _consume_turn_notes(
    recovery: ObstacleRecovery,
    notes: queue.Queue[str],
    send,
    now_s: float,
) -> None:
    while True:
        try:
            note = notes.get_nowait()
        except queue.Empty:
            return
        if recovery.phase == "stopping":
            if note == "STOP_DONE":
                recovery.phase = "stopped"
                recovery.stop_started_s = now_s
            elif note == "STOP_FAIL":
                recovery.phase = "fault"
        elif recovery.phase == "turning" and note == "TURN_DONE":
            recovery.quarters_remaining -= 1
            if recovery.quarters_remaining > 0:
                if not send(f"turn {recovery.side}"):
                    recovery.phase = "fault"
            else:
                recovery.phase = "reacquire"
                recovery.clear = 0
        elif recovery.phase == "turning" and note == "TURN_FAIL":
            recovery.phase = "fault"

