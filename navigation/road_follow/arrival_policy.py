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
from road_follow.rfid_arrival import step_rfid_arrival, update_visual_heading_lock

if TYPE_CHECKING:
    from navigation.topo_proto.graph import TopologyGraph
    from road_follow.junction_turn import JunctionTurnConfig
    from road_follow.rfid_arrival import RfidArrivalConfig


# 标准 0.97 m 格网边统一在 0.57 m 交接，锁住航向后再走 0.40 m。
# 短边不使用这组数。
_GRID_LENGTH_M = 0.97
_GRID_HANDOFF_M = 0.57
_GRID_HANDOFF_LATEST_M = 0.60
_GRID_FINAL_M = 0.40
_GRID_ARRIVAL_M = _GRID_HANDOFF_M + _GRID_FINAL_M
_GRID_ALIGN_TIMEOUT_S = 3.0
_GRID_ALIGN_STABLE_FRAMES = 8
_IMU_FULL_EDGES = frozenset({("0_J", "1_2")})
_IMU_FULL_EDGE_EXTRA_M = 0.07


@dataclass(frozen=True)
class ArrivalPolicy:
    from_node: str
    to_node: str
    mode: str
    handoff_progress_m: float
    final_forward_m: float
    guard_progress_m: float
    latest_handoff_m: float
    expected_openings: tuple[str, ...]
    align_timeout_s: float
    align_stable_frames: int
    imu_full_edge: bool = False
    early_heading_lock: bool = False
    edge_length_m: float = 0.0


def _exit_directions(graph, from_node: str, to_node: str) -> set[str]:
    """驶入方向上，目标点其余连边相对车头的开口。正前约 45° 内算 forward。"""
    a, b = graph.nodes[from_node], graph.nodes[to_node]
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
    return directions


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

    directions = _exit_directions(graph, from_node, to_node)
    default_mode = (
        "visual_odom"
        if target.role == "junction" or "forward" in directions
        else "visual_end"
    )
    mode = setting("mode", default_mode)
    grid = math.isfinite(edge_length_m) and math.isclose(
        edge_length_m, _GRID_LENGTH_M, abs_tol=1e-4
    )
    final = setting(
        "final_forward_m",
        _GRID_FINAL_M if grid else (
            patrol_cfg.step_distance_mm / 1000.0
            if target.role == "patrol_slot" else junction_cfg.turn_forward_m
        ),
    )
    handoff = setting(
        "handoff_progress_m",
        _GRID_HANDOFF_M if grid else max(0.0, edge_length_m - final),
    )
    guard = setting("guard_progress_m", max(0.0, edge_length_m - junction_cfg.odom_stop_margin_m))
    align_timeout_s = _GRID_ALIGN_TIMEOUT_S if grid else junction_cfg.align_timeout_s
    align_stable_frames = _GRID_ALIGN_STABLE_FRAMES if grid else junction_cfg.align_stable_frames
    if (
        not math.isfinite(edge_length_m) or edge_length_m <= 0
        or not all(math.isfinite(v) for v in (handoff, final, guard))
        or not 0 <= handoff < guard <= edge_length_m
        or not junction_cfg.min_forward_mm <= round(final * 1000) <= junction_cfg.max_forward_mm
    ):
        raise ValueError(f"不安全的到点距离配置: {from_node} -> {to_node}")

    directed_edge = (from_node, to_node)
    imu_full_edge = directed_edge in _IMU_FULL_EDGES
    arrival_distance_m = (
        edge_length_m + _IMU_FULL_EDGE_EXTRA_M
        if imu_full_edge
        else _GRID_ARRIVAL_M if grid else edge_length_m
    )
    return ArrivalPolicy(
        from_node, to_node, mode, handoff, final, guard,
        _GRID_HANDOFF_LATEST_M if grid else handoff,
        tuple(side for side in ("forward", "left", "right") if side in directions),
        align_timeout_s, align_stable_frames,
        imu_full_edge,
        grid,
        arrival_distance_m,
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
    near_x_m=None,
    lane_heading_rad=None,
    yaw_rad=None,
    centerline_points=None,
    road_pixels=800,
    follow=None,
    frame_captured_s=None,
    odom_received_s=None,
    anchor_notes=None,
    now_s=None,
    remaining_edge_m=None,
    forward_depth_m=None,
    trusted_odom_handoff=False,
):
    """共享地图选择和 ODOM 新鲜度检查。贯通点按边长保护，尽头巡检点仍等墙。"""
    approaching = state.phase in ("follow", "approach") and (
        not patrol or patrol_state.phase == "follow"
    )
    if approaching:
        # 尽头巡检点的墙可能晚于边长，不能用固定距离截断。
        # 贯通点正前方还有路，超过边长保护仍未交接就停车。
        if (
            (not patrol or policy.mode == "visual_odom")
            and odom_valid
            and not trusted_odom_handoff
            and progress_m + 1e-6 >= policy.guard_progress_m
        ):
            if patrol:
                patrol_state.phase = "odom_wait"
            else:
                state.phase = "odom_wait"
            return state, patrol_state, VelocityCommand(0.0, 0.0, "stop_arrival_guard"), "guard"
        if not odom_valid:
            state.arm = 0
            patrol_state.odom_handoff_frames = 0
            patrol_state.road_end_missing_frames = 0
            return state, patrol_state, VelocityCommand(0.0, 0.0, "stop_odom_stale"), "odom_stale"

    side_cue = expected_side_cue(policy, reading, opening)

    # 标准格网边在干净直路段持续更新绝对道路航向。普通路口仍由原来的
    # JunctionTurn 保存支路证据；RfidArrival 仅作为本边的航向锁存状态。
    if (
        policy.early_heading_lock and not patrol
        and state.phase in ("follow", "approach", "align", "anchor_wait")
    ):
        update_visual_heading_lock(
            patrol_state,
            patrol_cfg,
            visual=command,
            visual_safe=visual_safe,
            odom_valid=odom_valid,
            yaw_rad=yaw_rad,
            lane_heading_rad=lane_heading_rad,
            forward_depth_m=forward_depth_m,
            progress_m=progress_m,
        )
        vision_blocked = command.reason in (
            "stop_road", "stop_centerline", "stop_lookahead", "stop_forward_strip",
        )
        patrol_state.early_lock_blocked_count = (
            patrol_state.early_lock_blocked_count + 1 if vision_blocked else 0
        )
    if (
        approaching and state.suppress_cue and not policy.imu_full_edge
        and not trusted_odom_handoff
    ):
        if side_cue.detected:
            return state, patrol_state, command, ""
        state.suppress_cue = False

    final_mm = int(round(policy.final_forward_m * 1000))
    if patrol and state.phase == "follow":
        if remaining_edge_m is None and math.isfinite(progress_m):
            remaining_edge_m = max(0.0, policy.edge_length_m - progress_m)
        before = patrol_state.phase
        side_valid = (
            side_cue.detected and side_cue.distance_m is not None
            and junction_cfg.branch_observe_min_distance_m <= side_cue.distance_m
            <= junction_cfg.branch_observe_max_distance_m
        )
        patrol_state, command = step_rfid_arrival(
            patrol_state, None, command, notes, send,
            replace(
                patrol_cfg, step_distance_mm=final_mm,
                align_timeout_s=policy.align_timeout_s,
                align_stable_frames=policy.align_stable_frames,
            ),
            edge_left_visible=side_valid and "left" in policy.expected_openings and reading.left,
            edge_right_visible=side_valid and "right" in policy.expected_openings and reading.right,
            forward_band_ratio=reading.forward_band_ratio,
            visual_safe=visual_safe,
            arrival_mode=policy.mode,
            odom_handoff=(progress_m + 1e-6 >= policy.handoff_progress_m and is_visual_follow(command)),
            odom_stable_frames=junction_cfg.stable_frames,
            near_x_m=near_x_m,
            lane_heading_rad=lane_heading_rad,
            yaw_rad=yaw_rad,
            progress_m=progress_m,
            odom_valid=odom_valid,
            centerline_points=centerline_points,
            road_pixels=road_pixels,
            follow=follow,
            frame_captured_s=frame_captured_s,
            odom_received_s=odom_received_s,
            anchor_notes=anchor_notes,
            now_s=now_s,
            remaining_edge_m=remaining_edge_m,
            forward_depth_m=forward_depth_m,
            imu_full_edge=policy.imu_full_edge,
            early_heading_lock=policy.early_heading_lock,
            handoff_start_m=policy.handoff_progress_m,
            handoff_latest_m=policy.latest_handoff_m,
        )
        if before in ("follow", "align", "anchor_wait") and patrol_state.phase == "heading_hold":
            if policy.imu_full_edge:
                source = "imu_full_edge"
            elif policy.early_heading_lock:
                source = "grid_imu"
            else:
                source = policy.mode
        else:
            source = ""
        return state, patrol_state, command, source

    cfg = replace(
        junction_cfg, turn_forward_m=policy.final_forward_m,
        align_timeout_s=policy.align_timeout_s,
        align_stable_frames=policy.align_stable_frames,
    )
    if state.phase == "heading_hold" and state.forward_mm > 0:
        cfg = replace(cfg, turn_forward_m=state.forward_mm / 1000.0)
    cue = side_cue
    if policy.mode == "visual_odom":
        if (
            policy.early_heading_lock
            and state.phase in ("follow", "approach")
            and progress_m + 1e-6 >= policy.latest_handoff_m
            and not trusted_odom_handoff
        ):
            if patrol_state.locked_heading_yaw_rad is None:
                state.phase = "fault"
                state.fault_reason = "stop_heading_lock_invalid"
                return state, patrol_state, VelocityCommand(
                    0.0, 0.0, state.fault_reason
                ), "heading_lock_invalid"
            if not state.branch_latched and not trusted_odom_handoff:
                state.phase = "fault"
                state.fault_reason = "stop_junction_unconfirmed"
                return state, patrol_state, VelocityCommand(
                    0.0, 0.0, state.fault_reason
                ), "junction_unconfirmed"
        blocked_handoff = (
            policy.early_heading_lock
            and patrol_state.locked_heading_yaw_rad is not None
            and patrol_state.early_lock_blocked_count
            >= patrol_cfg.early_lock_blocked_frames
            and progress_m + 1e-6 >= patrol_cfg.early_lock_fallback_min_m
        )
        normal_handoff = (
            state.branch_latched and odom_valid
            and (
                (
                    is_visual_follow(command)
                    and progress_m + 1e-6 >= policy.handoff_progress_m
                )
                or blocked_handoff
            )
            and (
                not policy.early_heading_lock
                or patrol_state.locked_heading_yaw_rad is not None
            )
        )
        trusted_handoff = (
            trusted_odom_handoff and odom_valid and is_visual_follow(command)
            and progress_m + 1e-6 >= policy.handoff_progress_m
            and progress_m < policy.edge_length_m
        )
        if (normal_handoff or trusted_handoff) and state.phase in (
            "follow", "approach", "align", "anchor_wait",
        ):
            side = state.side if state.side in ("left", "right") else next(
                (item for item in policy.expected_openings if item in ("left", "right")),
                "left",
            )
            cue = JunctionCue(True, side, policy.final_forward_m, "odom_handoff")
            # 按实际交接位置补齐到拓扑边末端，不能从晚交接位置再走固定 0.40 m。
            remaining_m = max(
                junction_cfg.min_forward_mm / 1000.0,
                policy.edge_length_m - progress_m,
            )
            cfg = replace(cfg, turn_forward_m=remaining_m)
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
    target_yaw_rad = (
        patrol_state.locked_heading_yaw_rad
        if policy.early_heading_lock else None
    )
    alignment_heading_rad = lane_heading_rad
    if (
        target_yaw_rad is not None
        and yaw_rad is not None and math.isfinite(yaw_rad)
    ):
        alignment_heading_rad = math.atan2(
            math.sin(target_yaw_rad - yaw_rad),
            math.cos(target_yaw_rad - yaw_rad),
        )
    state, command = step_junction_turn(
        state, cue, command, notes, send, cfg, near_x_m=near_x_m,
        progress_m=progress_m, odom_valid=odom_valid,
        lane_heading_rad=alignment_heading_rad, yaw_rad=yaw_rad,
        target_yaw_rad=target_yaw_rad,
        frame_captured_s=frame_captured_s, odom_received_s=odom_received_s,
        anchor_notes=anchor_notes, now_s=now_s, visual_safe=visual_safe,
    )
    if (
        cue.source == "odom_handoff"
        and normal_handoff
        and policy.early_heading_lock
        and before in ("follow", "approach")
        and state.phase in ("align", "anchor_wait", "heading_hold")
    ):
        # 真正进入交接状态时冻结；确认帧期间仍可更新视觉候选航向。
        patrol_state.heading_lock_frozen = True
    if before in ("follow", "approach", "align", "anchor_wait") and state.phase == "heading_hold":
        source = "culvert_odom_handoff" if trusted_odom_handoff else cue.source
    else:
        source = ""
    return state, patrol_state, command, source
