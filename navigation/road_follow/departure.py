"""到点之后，用入边和出边决定直行、转向或倒车。"""

from __future__ import annotations

import math
from collections.abc import Mapping

from road_follow.backup import Backup, BackupConfig, step_backup
from road_follow.control import VelocityCommand, align_settle_command
from road_follow.junction_turn import JunctionTurn, JunctionTurnConfig

NodeXY = Mapping[str, tuple[float, float]]


def departure_maneuver(
    node_xy: NodeXY,
    arrived_from: str,
    arrived_at: str,
    depart_to: str,
    *,
    reverse_arrival: bool = False,
) -> str:
    """坐标 x 向右、y 向上；倒车到点时，车头与入边移动方向相反。"""
    if depart_to == arrived_from and not reverse_arrival:
        return "backup"
    ax, ay = node_xy[arrived_from]
    bx, by = node_xy[arrived_at]
    cx, cy = node_xy[depart_to]
    ix, iy = bx - ax, by - ay
    if reverse_arrival:
        ix, iy = -ix, -iy
    ox, oy = cx - bx, cy - by
    if math.hypot(ix, iy) < 1e-6 or math.hypot(ox, oy) < 1e-6:
        raise ValueError("路口相邻节点重合，无法判断转向")
    angle = math.atan2(ix * oy - iy * ox, ix * ox + iy * oy)
    if abs(angle) <= math.radians(45):
        return "straight"
    if abs(angle) >= math.radians(135):
        if reverse_arrival:
            # 倒车到点后，下一条边与当前车头相反时需要原地掉头。
            return "uturn"
        raise ValueError("下一条路与来路反向，但终点不是上一个节点")
    if angle > 0:
        return "left"
    return "right"


def handoff_entrance(agent, node_xy: NodeXY):
    """15 cm 动作到达 0_J 后推进首边，并返回 Agent 选定的原地转向。"""
    arrived_from = agent.state.from_node
    arrived_at = agent.state.to_node
    if not arrived_from or not arrived_at:
        raise ValueError("Agent 没有正在执行的出发区路段")

    expected_junction = str(agent.graph.meta.get("start_junction", ""))
    if expected_junction and arrived_at != expected_junction:
        raise ValueError(
            f"出发定距终点应为 {expected_junction}，实际规划为 {arrived_at}"
        )

    command = agent.node_reached(arrived_at)
    if type(command).__name__ != "FollowEdge" or command.from_node != arrived_at:
        raise ValueError(f"到达 {arrived_at} 后 Agent 没有给出下一条路")
    maneuver = departure_maneuver(
        node_xy, arrived_from, arrived_at, command.to_node
    )
    if maneuver not in ("left", "right"):
        raise ValueError(
            f"出发区 {arrived_at} 的下一动作不是原地转弯: {maneuver}"
        )
    return maneuver, command


def step_planned_backup(
    state: Backup,
    *,
    lane_heading_rad: float | None,
    frame_captured_s: float,
    progress_s_m: float,
    command: VelocityCommand,
    now_s: float,
    yaw_rad: float | None,
    odom_valid: bool,
    backup_cfg: BackupConfig,
    align_cfg: JunctionTurnConfig,
    near_x_m: float | None = None,
    visual_steering_gain: float = 0.0,
) -> tuple[Backup, VelocityCommand]:
    """规划倒车先视觉摆正，再融合 IMU 航向、视觉横向和里程退回入口。"""
    can_align = (
        odom_valid and math.isfinite(progress_s_m)
        and progress_s_m > backup_cfg.done_progress_m
        and yaw_rad is not None and math.isfinite(yaw_rad)
    )
    if state.phase in ("idle", "align") and can_align:
        if state.phase == "idle":
            state.phase = "align"
            state.clear_frames = 0
            state.last_frame_s = None
        if state.last_frame_s is not None and frame_captured_s <= state.last_frame_s:
            return state, VelocityCommand(0.0, 0.0, "align")
        state.last_frame_s = frame_captured_s
        aligned = (
            lane_heading_rad is not None
            and math.isfinite(lane_heading_rad)
            and abs(lane_heading_rad) <= align_cfg.align_max_abs_heading_rad
        )
        state.clear_frames = state.clear_frames + 1 if aligned else 0
        if state.clear_frames < align_cfg.align_stable_frames:
            return state, align_settle_command(
                lane_heading_rad, align_cfg.align_gain,
                align_cfg.align_max_abs_omega, align_cfg.align_max_abs_heading_rad,
            )
        state.phase = "idle"
    elif state.phase == "align":
        # 沿用原倒车的起点和 ODOM/IMU 检查，不在无效输入上继续摆正。
        state.phase = "idle"

    return step_backup(
        state, state.phase == "idle", near_x_m, progress_s_m, command, now_s,
        backup_cfg, yaw_rad=yaw_rad, odom_valid=odom_valid,
        frame_captured_s=frame_captured_s, heading_only=True,
        visual_steering_gain=visual_steering_gain,
    )


def apply_departure(
    state: JunctionTurn, maneuver: str, send, stop_settle_s: float = 2.0
) -> JunctionTurn:
    """只在 ``arrived`` 之后提交动作。转弯前必须先明确停车。"""
    if state.phase != "arrived":
        state.phase = "fault"
        state.departure = "none"
        return state
    state.departure = maneuver
    if maneuver in ("left", "right", "straight", "backup", "uturn"):
        state.stop_settle_s = max(0.0, stop_settle_s)
        state.side = maneuver if maneuver in ("left", "right") else (
            "left" if maneuver == "uturn" else "none"
        )
        state.quarters_remaining = 2 if maneuver == "uturn" else 0
        if send("stop"):
            state.phase = "stopping"
        else:
            state.phase = "fault"
        return state
    state.phase = "fault"
    return state


def handoff_arrival(
    state: JunctionTurn,
    agent,
    node_xy: NodeXY,
    send,
    stop_settle_s: float = 2.0,
    *,
    reverse_arrival: bool = False,
) -> JunctionTurn:
    """到点后上报当前目标节点；按实际车头方向交接 Agent 的下一条边。"""
    if state.phase != "arrived":
        return state
    arrived_from = agent.state.from_node
    arrived_at = agent.state.to_node
    if not arrived_from or not arrived_at:
        state.phase = "fault"
        state.departure = "none"
        return state
    command = agent.node_reached(arrived_at)
    kind = type(command).__name__
    if kind == "FollowEdge":
        if command.from_node != arrived_at:
            state.phase = "fault"
            state.departure = "none"
            return state
        try:
            maneuver = departure_maneuver(
                node_xy, arrived_from, arrived_at, command.to_node,
                reverse_arrival=reverse_arrival,
            )
        except (KeyError, ValueError):
            state.phase = "fault"
            state.departure = "none"
            return state
        return apply_departure(state, maneuver, send, stop_settle_s)
    if kind == "MissionComplete":
        state.phase = "done"
        state.departure = "none"
        return state
    state.phase = "fault"
    state.departure = "none"
    return state
