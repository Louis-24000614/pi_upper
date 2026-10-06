"""地图指定的正向到点策略；动作完成后仍由 departure 推进 Agent。"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from ipm_proto.junction import KIND_CORNER, KIND_CROSS, KIND_T
from road_follow.control import VelocityCommand, is_visual_follow
from road_follow.junction_turn import (
    JunctionCue, road_end_turn_cue, step_junction_turn,
)
from road_follow.rfid_arrival import step_rfid_arrival

if TYPE_CHECKING:
    from navigation.topo_proto.graph import TopologyGraph
    from road_follow.junction_turn import JunctionTurnConfig
    from road_follow.rfid_arrival import RfidArrivalConfig


@dataclass(frozen=True)
class ArrivalPolicy:
    from_node: str
    to_node: str
    mode: str
    handoff_progress_m: float
    final_forward_m: float
    guard_progress_m: float
    expected_openings: tuple[str, ...]


def resolve_arrival_policy(
    graph: TopologyGraph,
    from_node: str,
    to_node: str,
    edge_length_m: float,
    junction_cfg: JunctionTurnConfig,
    patrol_cfg: RfidArrivalConfig,
) -> ArrivalPolicy:
    """有向驶入覆盖 > 节点配置 > 旧 role 默认值。"""
    target = graph.nodes[to_node]
    override = graph.arrival_overrides.get((from_node, to_node))

    def setting(name, default):
        value = getattr(override, name, None)
        if value is None:
            value = getattr(target.arrival, name)
        return default if value is None else value

    mode = setting("mode", "visual_odom" if target.role == "junction" else "visual_end")
    final = setting(
        "final_forward_m",
        patrol_cfg.step_distance_mm / 1000.0
        if target.role == "patrol_slot" else junction_cfg.turn_forward_m,
    )
    handoff = setting("handoff_progress_m", max(0.0, edge_length_m - final))
    guard = setting("guard_progress_m", max(0.0, edge_length_m - junction_cfg.odom_stop_margin_m))
    if (
        not math.isfinite(edge_length_m) or edge_length_m <= 0
        or not all(math.isfinite(v) for v in (handoff, final, guard))
        or not 0 <= handoff < guard <= edge_length_m
        or not junction_cfg.min_forward_mm <= round(final * 1000) <= junction_cfg.max_forward_mm
    ):
        raise ValueError(f"不安全的到点距离配置: {from_node} -> {to_node}")

    a, b = graph.nodes[from_node], target
    ix, iy = b.x - a.x, b.y - a.y
    if math.hypot(ix, iy) < 1e-6:
        raise ValueError(f"驶入节点坐标重合: {from_node} -> {to_node}")
    directions = set()
    # 使用物理连边，即使道路临时封闭，侧向开口的几何形状仍然存在。
    for edge in graph.edges.values():
        neighbor = edge.v if edge.u == to_node else edge.u if edge.v == to_node else None
        if neighbor is None or neighbor == from_node:
            continue
        c = graph.nodes[neighbor]
        ox, oy = c.x - b.x, c.y - b.y
        angle = math.atan2(ix * oy - iy * ox, ix * ox + iy * oy)
        if abs(angle) <= math.pi / 4:
            directions.add("forward")
        elif abs(angle) < 3 * math.pi / 4:
            directions.add("left" if angle > 0 else "right")
    return ArrivalPolicy(
        from_node, to_node, mode, handoff, final, guard,
        tuple(side for side in ("forward", "left", "right") if side in directions),
    )


def expected_side_cue(policy, reading, opening) -> JunctionCue:
    """任一符合地图的侧向开口即可；距离/多帧证据由状态机继续检查。"""
    for side in ("left", "right"):
        if side in policy.expected_openings and getattr(reading, side):
            return JunctionCue(
                opening in (KIND_T, KIND_CROSS, KIND_CORNER),
                side, reading.junction_y_m, "side_branch",
            )
    return JunctionCue(False)


def step_route_arrival(
    policy, *, patrol, state, patrol_state, reading, opening, command,
    progress_m, odom_valid, notes, send, junction_cfg, patrol_cfg,
    visual_safe=True,
):
    """共享地图选择和 ODOM 新鲜度检查；距离保护只用于普通路口。"""
    approaching = state.phase in ("follow", "approach") and (
        not patrol or patrol_state.phase == "follow"
    )
    if approaching:
        # 普通路口在有限动作前检查距离上限；巡检点继续等待视觉交接，
        # 不因地图边长与实测视觉触发位置的偏差提前锁定停车。
        if not patrol and odom_valid and progress_m + 1e-6 >= policy.guard_progress_m:
            state.phase = "odom_wait"
            return state, patrol_state, VelocityCommand(0.0, 0.0, "stop_arrival_guard"), "guard"
        if not odom_valid:
            state.arm = 0
            patrol_state.odom_handoff_frames = 0
            patrol_state.road_end_missing_frames = 0
            return state, patrol_state, VelocityCommand(0.0, 0.0, "stop_odom_stale"), "odom_stale"

    side_cue = expected_side_cue(policy, reading, opening)
    if approaching and state.suppress_cue:
        if side_cue.detected:
            return state, patrol_state, command, ""
        state.suppress_cue = False

    final_mm = int(round(policy.final_forward_m * 1000))
    if patrol and state.phase == "follow":
        before = patrol_state.phase
        side_valid = (
            side_cue.detected and side_cue.distance_m is not None
            and junction_cfg.branch_observe_min_distance_m <= side_cue.distance_m
            <= junction_cfg.branch_observe_max_distance_m
        )
        patrol_state, command = step_rfid_arrival(
            patrol_state, None, command, notes, send,
            replace(patrol_cfg, step_distance_mm=final_mm),
            edge_left_visible=side_valid and "left" in policy.expected_openings and reading.left,
            edge_right_visible=side_valid and "right" in policy.expected_openings and reading.right,
            forward_band_ratio=reading.forward_band_ratio,
            visual_safe=visual_safe,
            arrival_mode=policy.mode,
            odom_handoff=(progress_m + 1e-6 >= policy.handoff_progress_m and is_visual_follow(command)),
            odom_stable_frames=junction_cfg.stable_frames,
        )
        source = policy.mode if before == "follow" and patrol_state.phase == "blind_forward" else ""
        return state, patrol_state, command, source

    cfg = replace(junction_cfg, turn_forward_m=policy.final_forward_m)
    cue = side_cue
    if policy.mode == "visual_odom":
        if (
            state.branch_latched and odom_valid and is_visual_follow(command)
            and progress_m + 1e-6 >= policy.handoff_progress_m
            and progress_m < policy.guard_progress_m
        ):
            cue = JunctionCue(True, state.side, policy.final_forward_m, "odom_handoff")
    else:
        end_cue = road_end_turn_cue(
            side=state.side, stable_blocked=False, raw_blocked=False,
            command=command, road_end_y_m=reading.corridor_end_y_m,
            lane_x_m=reading.lane_x_m, lane_width_m=reading.lane_width_m,
            cfg=cfg, approach_latched=state.branch_latched,
            forward_band_ratio=reading.forward_band_ratio,
        )
        if end_cue.detected:
            cue = end_cue
    before = state.phase
    state, command = step_junction_turn(state, cue, command, notes, send, cfg)
    source = cue.source if before in ("follow", "approach") and state.phase == "forward" else ""
    return state, patrol_state, command, source
