"""把道路覆盖规划推进成可由 Navigation 执行的边级命令。"""

from __future__ import annotations

from dataclasses import dataclass

from navigation.topo_proto.graph import TopologyGraph

from .coverage.planner import CoveragePlan, plan_remaining
from .state import AgentState


@dataclass(frozen=True)
class FollowEdge:
    edge_id: str
    from_node: str
    to_node: str


@dataclass(frozen=True)
class StartBackup:
    edge_id: str
    from_node: str
    progress_m: float


@dataclass(frozen=True)
class Stop:
    reason: str


@dataclass(frozen=True)
class MissionComplete:
    home: str


AgentCommand = FollowEdge | StartBackup | Stop | MissionComplete


class RouteAgent:
    """持有唯一拓扑状态，并在确认事件到达后推进或重新规划。"""

    def __init__(self, graph: TopologyGraph, home: str = "0_0") -> None:
        if home not in graph.nodes:
            raise ValueError(f"未知出发区: {home}")
        if not all(edge.bidirectional for edge in graph.edges.values()):
            raise ValueError("当前道路覆盖 Agent 只支持双向道路")
        self.graph = graph
        self.all_edges = frozenset(graph.edges)
        self.state = AgentState(home=home, current_node=home)

    def start(self) -> AgentCommand:
        """开始新一轮任务；调用方应为每轮比赛传入新加载的拓扑图。"""
        if self.state.phase != "idle":
            return self._fail("任务已经开始")
        self.state.phase = "replanning"
        return self._replan()

    def node_reached(self, node_id: str) -> AgentCommand:
        """Navigation 确认完整通过当前边并到达目标节点。"""
        state = self.state
        if state.phase != "following" or state.current_edge is None:
            return self._fail("非行驶状态收到 NODE_REACHED")
        if node_id != state.to_node:
            return self._fail(
                f"到达节点不匹配: expected={state.to_node} actual={node_id}"
            )
        if state.current_edge in state.blocked_edges:
            return self._fail("封闭道路不能标记为已覆盖")

        state.covered_edges.add(state.current_edge)
        state.current_node = node_id
        state.current_edge = None
        state.from_node = None
        state.to_node = None
        state.route_index += 1
        state.phase = "at_node"
        return self._dispatch_next()

    def obstacle_confirmed(self, edge_id: str, progress_m: float) -> AgentCommand:
        """当前边确认硬障碍：立即封边，只允许 Navigation 倒回入口。"""
        state = self.state
        if state.phase != "following" or state.current_edge is None:
            return self._fail("非行驶状态收到 OBSTACLE_CONFIRMED")
        if edge_id != state.current_edge:
            return self._fail(
                f"障碍道路不匹配: expected={state.current_edge} actual={edge_id}"
            )
        if progress_m < 0.0:
            return self._fail("沿边进度不能为负数")

        state.blocked_edges.add(edge_id)
        state.covered_edges.discard(edge_id)
        self.graph.set_edge_blocked(edge_id)
        state.route_nodes = ()
        state.route_edges = ()
        state.route_index = 0
        state.phase = "backing"
        assert state.from_node is not None
        return StartBackup(edge_id, state.from_node, float(progress_m))

    def backup_done(self, edge_id: str, node_id: str) -> AgentCommand:
        """Navigation 已用视觉和 ODOM 倒回入口，才允许从该节点重规划。"""
        state = self.state
        if state.phase != "backing" or state.current_edge is None:
            return self._fail("非倒车状态收到 BACKUP_DONE")
        if edge_id != state.current_edge or node_id != state.from_node:
            return self._fail(
                "倒车完成位置不匹配: "
                f"edge={edge_id} node={node_id} expected={state.current_edge}/{state.from_node}"
            )

        state.current_node = node_id
        state.current_edge = None
        state.from_node = None
        state.to_node = None
        state.phase = "replanning"
        return self._replan()

    def backup_failed(self, edge_id: str, reason: str) -> AgentCommand:
        """倒车不能安全完成时停车，不尝试执行候选新路线。"""
        state = self.state
        if state.phase != "backing" or edge_id != state.current_edge:
            return self._fail("倒车失败事件与当前道路不匹配")
        return self._fail(f"倒车失败 {edge_id}: {reason}")

    def _replan(self) -> AgentCommand:
        state = self.state
        required = set(self.all_edges) - state.covered_edges - state.blocked_edges
        try:
            plan = plan_remaining(
                self.graph,
                required,
                current=state.current_node,
                home=state.home,
            )
            plan = self._prefer_initial_exit(plan)
        except (ValueError, AssertionError) as exc:
            return self._fail(f"重新规划失败: {exc}")
        return self._accept_plan(plan)

    def _prefer_initial_exit(self, plan: CoveragePlan) -> CoveragePlan:
        """初次闭环覆盖可整体反向，借此固定出发区的第一条支路。"""
        state = self.state
        junction = str(self.graph.meta.get("start_junction", ""))
        exit_node = str(self.graph.meta.get("start_exit", ""))
        if (
            not junction
            or not exit_node
            or state.covered_edges
            or state.blocked_edges
            or state.current_node != state.home
        ):
            return plan

        wanted = (state.home, junction, exit_node)
        if plan.nodes[:3] == wanted:
            return plan
        reversed_nodes = tuple(reversed(plan.nodes))
        if reversed_nodes[:3] != wanted:
            raise ValueError(
                "配置的出发路线不在覆盖闭环中: "
                f"{state.home} -> {junction} -> {exit_node}"
            )
        return CoveragePlan(
            nodes=reversed_nodes,
            edges=tuple(reversed(plan.edges)),
            length_m=plan.length_m,
            unreachable_edges=plan.unreachable_edges,
        )

    def _accept_plan(self, plan: CoveragePlan) -> AgentCommand:
        state = self.state
        if any(edge_id in state.blocked_edges for edge_id in plan.edges):
            return self._fail("规划结果包含已封闭道路")
        state.route_nodes = plan.nodes
        state.route_edges = plan.edges
        state.route_index = 0
        state.unreachable_edges = plan.unreachable_edges
        state.phase = "at_node"
        return self._dispatch_next()

    def _dispatch_next(self) -> AgentCommand:
        state = self.state
        if state.route_index >= len(state.route_edges):
            if state.current_node != state.home:
                return self._fail("路线结束但车辆未回到出发区")
            if state.unreachable_edges:
                return self._fail(
                    "仍有不可达道路: " + ", ".join(state.unreachable_edges)
                )
            state.phase = "done"
            return MissionComplete(state.home)

        index = state.route_index
        if index + 1 >= len(state.route_nodes):
            return self._fail("规划节点序列与边序列长度不一致")
        from_node = state.route_nodes[index]
        to_node = state.route_nodes[index + 1]
        edge_id = state.route_edges[index]
        if state.current_node != from_node:
            return self._fail(
                f"规划起点不匹配: current={state.current_node} route={from_node}"
            )
        if edge_id in state.blocked_edges:
            return self._fail(f"即将执行封闭道路: {edge_id}")

        state.current_edge = edge_id
        state.from_node = from_node
        state.to_node = to_node
        state.phase = "following"
        return FollowEdge(edge_id, from_node, to_node)

    def _fail(self, reason: str) -> Stop:
        self.state.phase = "fault"
        self.state.failure_reason = reason
        return Stop(reason)
