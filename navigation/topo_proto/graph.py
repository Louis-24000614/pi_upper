"""加载 config/nav_topology.yaml，并在边上做 Dijkstra 最短路。"""

from __future__ import annotations

from dataclasses import dataclass, field
from heapq import heappop, heappush
import math
from pathlib import Path
from typing import Iterable

import yaml

# 仓库根：navigation/topo_proto/graph.py → parents[2]
_REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TOPOLOGY = _REPO_ROOT / "config" / "nav_topology.yaml"


@dataclass(frozen=True)
class ArrivalSettings:
    """节点默认值或有向驶入覆盖；None 表示继承。"""

    mode: str | None = None
    handoff_progress_m: float | None = None
    final_forward_m: float | None = None
    guard_progress_m: float | None = None


def arrival_settings_from_mapping(raw: dict | None) -> ArrivalSettings:
    if raw is None:
        return ArrivalSettings()
    if not isinstance(raw, dict):
        raise ValueError("arrival 必须是 mapping")
    unknown = set(raw) - {"mode", "handoff_progress_m", "final_forward_m", "guard_progress_m"}
    if unknown:
        raise ValueError(f"未知 arrival 字段: {sorted(unknown)}")
    mode = raw.get("mode")
    if mode is not None and mode not in ("visual_end", "visual_odom"):
        raise ValueError(f"未知到点策略: {mode}")
    distances = {}
    for name in ("handoff_progress_m", "final_forward_m", "guard_progress_m"):
        value = raw.get(name)
        if value is not None:
            if isinstance(value, bool):
                raise ValueError(f"{name} 必须是距离")
            value = float(value)
            if not math.isfinite(value) or value < 0 or (name == "final_forward_m" and value == 0):
                raise ValueError(f"{name} 必须是有限非负距离，最后前进距离必须大于0")
        distances[name] = value
    return ArrivalSettings(mode=mode, **distances)


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float
    role: str = ""
    arrival: ArrivalSettings = field(default_factory=ArrivalSettings)


@dataclass(frozen=True)
class Edge:
    id: str
    u: str
    v: str
    length_m: float
    bidirectional: bool = True
    tunnel: bool = False


@dataclass
class TopologyGraph:
    """可变图：blocked 边在搜路时被跳过。"""

    meta: dict
    nodes: dict[str, Node]
    edges: dict[str, Edge]
    blocked: set[str] = field(default_factory=set)
    arrival_overrides: dict[tuple[str, str], ArrivalSettings] = field(default_factory=dict)

    def set_edge_blocked(self, edge_id: str, blocked: bool = True) -> None:
        if edge_id not in self.edges:
            raise KeyError(f"unknown edge id: {edge_id}")
        if blocked:
            self.blocked.add(edge_id)
        else:
            self.blocked.discard(edge_id)

    def clear_blocked(self) -> None:
        self.blocked.clear()

    def neighbors(self, node_id: str) -> list[tuple[str, float, str]]:
        """返回 (neighbor_id, length_m, edge_id)。"""
        if node_id not in self.nodes:
            raise KeyError(f"unknown node id: {node_id}")
        out: list[tuple[str, float, str]] = []
        for edge in self.edges.values():
            if edge.id in self.blocked:
                continue
            if edge.u == node_id:
                out.append((edge.v, edge.length_m, edge.id))
            elif edge.bidirectional and edge.v == node_id:
                out.append((edge.u, edge.length_m, edge.id))
        return out


def load_topology(path: Path | str | None = None) -> TopologyGraph:
    """从 YAML 构建拓扑图。默认读仓库 config/nav_topology.yaml。"""
    yaml_path = Path(path) if path is not None else DEFAULT_TOPOLOGY
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"topology root must be a mapping: {yaml_path}")

    raw_nodes = data.get("nodes") or {}
    nodes: dict[str, Node] = {}
    for node_id, info in raw_nodes.items():
        info = info or {}
        nodes[str(node_id)] = Node(
            id=str(node_id),
            x=float(info["x"]),
            y=float(info["y"]),
            role=str(info.get("role", "")),
            arrival=arrival_settings_from_mapping(info.get("arrival")),
        )

    edges: dict[str, Edge] = {}
    for item in data.get("edges") or []:
        edge_id = str(item["id"])
        u = str(item["u"])
        v = str(item["v"])
        if u not in nodes or v not in nodes:
            raise ValueError(f"edge {edge_id} references missing node ({u}, {v})")
        edges[edge_id] = Edge(
            id=edge_id,
            u=u,
            v=v,
            length_m=float(item["length_m"]),
            bidirectional=bool(item.get("bidirectional", True)),
            tunnel=bool(item.get("tunnel", False)),
        )

    overrides: dict[tuple[str, str], ArrivalSettings] = {}
    raw_overrides = data.get("arrival_overrides") or []
    if not isinstance(raw_overrides, list):
        raise ValueError("arrival_overrides 必须是列表")
    for item in raw_overrides:
        if not isinstance(item, dict):
            raise ValueError("arrival_overrides 条目必须是 mapping")
        source, target = str(item.get("from_node", "")), str(item.get("to_node", ""))
        if not any(
            (edge.u == source and edge.v == target)
            or (edge.bidirectional and edge.v == source and edge.u == target)
            for edge in edges.values()
        ):
            raise ValueError(f"到点覆盖引用不存在的有向道路: {source} -> {target}")
        key = (source, target)
        if key in overrides:
            raise ValueError(f"重复到点覆盖: {source} -> {target}")
        overrides[key] = arrival_settings_from_mapping(
            {name: value for name, value in item.items() if name not in ("from_node", "to_node")}
        )

    return TopologyGraph(
        meta=dict(data.get("meta") or {}),
        nodes=nodes,
        edges=edges,
        arrival_overrides=overrides,
    )


@dataclass(frozen=True)
class PathResult:
    node_ids: list[str]
    edge_ids: list[str]
    length_m: float

    def format_nodes(self) -> str:
        return " -> ".join(self.node_ids)

    def format_edges(self) -> str:
        return " -> ".join(self.edge_ids) if self.edge_ids else "(empty)"


def shortest_path(
    graph: TopologyGraph,
    start: str,
    goal: str,
) -> PathResult | None:
    """Dijkstra：节点代价为累计 length_m。不可达返回 None。"""
    if start not in graph.nodes or goal not in graph.nodes:
        raise KeyError(f"start/goal must be graph nodes: {start!r} -> {goal!r}")
    if start == goal:
        return PathResult(node_ids=[start], edge_ids=[], length_m=0.0)

    dist: dict[str, float] = {start: 0.0}
    prev: dict[str, tuple[str, str]] = {}  # node -> (prev_node, edge_id)
    heap: list[tuple[float, str]] = [(0.0, start)]

    while heap:
        cost, node = heappop(heap)
        if cost > dist.get(node, float("inf")):
            continue
        if node == goal:
            break
        for nxt, length, edge_id in graph.neighbors(node):
            new_cost = cost + length
            if new_cost < dist.get(nxt, float("inf")):
                dist[nxt] = new_cost
                prev[nxt] = (node, edge_id)
                heappush(heap, (new_cost, nxt))

    if goal not in dist:
        return None

    nodes_rev: list[str] = [goal]
    edges_rev: list[str] = []
    cur = goal
    while cur != start:
        parent, edge_id = prev[cur]
        edges_rev.append(edge_id)
        nodes_rev.append(parent)
        cur = parent
    nodes_rev.reverse()
    edges_rev.reverse()
    return PathResult(node_ids=nodes_rev, edge_ids=edges_rev, length_m=dist[goal])


def block_many(graph: TopologyGraph, edge_ids: Iterable[str]) -> None:
    for edge_id in edge_ids:
        graph.set_edge_blocked(edge_id, True)
