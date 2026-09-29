"""演示状态机：未知路障只有驶入相应道路后才会被发现。"""

from __future__ import annotations

from dataclasses import dataclass

from navigation.topo_proto.graph import TopologyGraph
from .planner import CoveragePlan, plan_remaining


@dataclass(frozen=True)
class MissionEvent:
    kind: str
    message: str
    edge_id: str | None = None


class PostmanMission:
    """一次 step 表示驶过一段路，或完成一次发现/掉头动作。"""

    def __init__(
        self,
        graph: TopologyGraph,
        obstacles: set[str],
        home: str = "0_0",
    ) -> None:
        if len(obstacles) != 3 or not obstacles <= graph.edges.keys():
            raise ValueError("启动前必须指定 3 条不同且存在的障碍道路")
        if not all(edge.bidirectional for edge in graph.edges.values()):
            raise ValueError("本演示的无向邮递员算法仅支持双向道路")
        if home not in graph.nodes:
            raise ValueError(f"未知出发区: {home}")
        self.graph = graph
        self.obstacles = frozenset(obstacles)  # 仅仿真器知道，规划器不读取。
        self.home = home
        self.current = home
        self.covered: set[str] = set()
        self.discovered: set[str] = set()
        self.visited_patrol: set[str] = set()
        self.plan: CoveragePlan = plan_remaining(graph, set(graph.edges), home, home)
        self.plan_index = 0
        self.phase = "running"
        self.entry_node: str | None = None
        self.entered_edge: str | None = None
        self.failure_reason: str | None = None
        self.events: list[MissionEvent] = []

    def robot_position(self) -> tuple[float, float]:
        """发现路障时画在道路内侧 25%，表示先进入再看见并停车。"""
        if self.phase == "backing" and self.entry_node and self.entered_edge:
            edge = self.graph.edges[self.entered_edge]
            far = edge.v if edge.u == self.entry_node else edge.u
            near_node = self.graph.nodes[self.entry_node]
            far_node = self.graph.nodes[far]
            return (near_node.x + 0.25 * (far_node.x - near_node.x),
                    near_node.y + 0.25 * (far_node.y - near_node.y))
        node = self.graph.nodes[self.current]
        return node.x, node.y

    def step(self) -> MissionEvent:
        if self.phase in {"done", "stalled"}:
            return MissionEvent(self.phase, "任务已结束")

        if self.phase == "backing":
            # 规划已在发现瞬间完成；车先按原路倒回入口，再执行新路线。
            assert self.entry_node is not None
            self.current = self.entry_node
            self.entry_node = None
            self.entered_edge = None
            self.phase = "stalled" if self.failure_reason else "running"
            message = f"掉头返回节点 {self.current}"
            if self.failure_reason:
                message += f"；无法继续：{self.failure_reason}"
            event = MissionEvent("uturn", message)
            self.events.append(event)
            return event

        if self.plan_index >= len(self.plan.edges):
            self.phase = "done"
            unreachable = self.plan.unreachable_edges
            suffix = f"；不可达道路 {len(unreachable)} 条" if unreachable else ""
            event = MissionEvent(
                "done",
                f"回到出发区；已覆盖 {len(self.covered)} 条道路，"
                f"已确认封闭 {len(self.discovered)} 条{suffix}",
            )
            self.events.append(event)
            return event

        edge_id = self.plan.edges[self.plan_index]
        assert self.current == self.plan.nodes[self.plan_index]
        if edge_id in self.obstacles and edge_id not in self.discovered:
            # 障碍预设位置不可传给规划器；只有驶入这条路才封边。
            self.discovered.add(edge_id)
            self.graph.set_edge_blocked(edge_id)
            self.entry_node = self.current
            self.entered_edge = edge_id
            try:
                self.plan = plan_remaining(
                    self.graph, set(self.graph.edges) - self.covered - self.discovered,
                    self.current, self.home,
                )
            except ValueError as exc:
                # 两条出发通道都封住等情况会使返航无解；仍须先退回安全节点。
                self.failure_reason = str(exc)
            self.plan_index = 0
            self.phase = "backing"
            event = MissionEvent(
                "discover",
                f"驶入 {edge_id} 后发现路中间障碍，停车、封边并重规划，下一步掉头",
                edge_id,
            )
            self.events.append(event)
            return event

        self.current = self.plan.nodes[self.plan_index + 1]
        self.plan_index += 1
        self.covered.add(edge_id)
        if self.graph.nodes[self.current].role == "patrol_slot":
            self.visited_patrol.add(self.current)
        event = MissionEvent("move", f"通过 {edge_id}，到达 {self.current}", edge_id)
        self.events.append(event)
        return event

    def run_all(self, max_steps: int = 1000) -> list[MissionEvent]:
        """给命令行和测试使用；上限防止异常规划造成无限循环。"""
        while self.phase not in {"done", "stalled"} and len(self.events) < max_steps:
            self.step()
        if self.phase not in {"done", "stalled"}:
            raise RuntimeError(f"超过 {max_steps} 步，任务未完成")
        return self.events
