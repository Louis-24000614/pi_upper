"""到点之后，用入边和出边决定直行、转向或倒车。"""

from __future__ import annotations

import math
from collections.abc import Mapping

from road_follow.junction_turn import JunctionTurn

NodeXY = Mapping[str, tuple[float, float]]


def departure_maneuver(
    node_xy: NodeXY,
    arrived_from: str,
    arrived_at: str,
    depart_to: str,
) -> str:
    """坐标 x 向右、y 向上。正的叉积是左转。原路返回是倒车，不是 180°。"""
    if depart_to == arrived_from:
        return "backup"
    ax, ay = node_xy[arrived_from]
    bx, by = node_xy[arrived_at]
    cx, cy = node_xy[depart_to]
    ix, iy = bx - ax, by - ay
    ox, oy = cx - bx, cy - by
    if math.hypot(ix, iy) < 1e-6 or math.hypot(ox, oy) < 1e-6:
        raise ValueError("路口相邻节点重合，无法判断转向")
    angle = math.atan2(ix * oy - iy * ox, ix * ox + iy * oy)
    if abs(angle) <= math.radians(45):
        return "straight"
    if abs(angle) >= math.radians(135):
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


def apply_departure(
    state: JunctionTurn, maneuver: str, send, stop_settle_s: float = 2.0
) -> JunctionTurn:
    """只在 ``arrived`` 之后提交动作。转弯前必须先明确停车。"""
    if state.phase != "arrived":
        state.phase = "fault"
        state.departure = "none"
        return state
    state.departure = maneuver
    if maneuver in ("left", "right", "straight", "backup"):
        state.stop_settle_s = max(0.0, stop_settle_s)
        state.side = maneuver if maneuver in ("left", "right") else "none"
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
) -> JunctionTurn:
    """``FORWARD_DONE`` 之后上报当前目标节点，再按 Agent 的下一条边执行。"""
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
                node_xy, arrived_from, arrived_at, command.to_node
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
