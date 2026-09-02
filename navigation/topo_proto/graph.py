"""加载 config/nav_topology.yaml，并在边上做 Dijkstra 最短路。"""

from __future__ import annotations

from dataclasses import dataclass, field
from heapq import heappop, heappush
from pathlib import Path
from typing import Iterable

import yaml

# 仓库根：navigation/topo_proto/graph.py → parents[2]
_REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TOPOLOGY = _REPO_ROOT / "config" / "nav_topology.yaml"


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float
    role: str = ""


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

    return TopologyGraph(
        meta=dict(data.get("meta") or {}),
        nodes=nodes,
        edges=edges,
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
