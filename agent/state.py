"""Agent 的边级任务状态；不包含相机、UART 或电机对象。"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class AgentState:
    home: str = "0_0"
    current_node: str = "0_0"
    current_edge: str | None = None
    from_node: str | None = None
    to_node: str | None = None
    covered_edges: set[str] = field(default_factory=set)
    blocked_edges: set[str] = field(default_factory=set)
    route_nodes: tuple[str, ...] = ()
    route_edges: tuple[str, ...] = ()
    route_index: int = 0
    unreachable_edges: tuple[str, ...] = ()
    phase: str = "idle"
    failure_reason: str | None = None

