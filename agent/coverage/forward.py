"""带驶入方向的道路覆盖规划：正常巡航仅前进，允许少覆盖后返航。"""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import count
import math

from navigation.topo_proto.graph import TopologyGraph

from .planner import CoveragePlan, _reachable_nodes


class ForwardReturnUnavailable(ValueError):
    """当前车头方向没有仅前进的返航路线，可由遇障恢复逻辑处理。"""


def forward_maneuver(
    graph: TopologyGraph,
    heading: tuple[str, str] | None,
    source: str,
    target: str,
) -> str:
    """仅允许直行或不超过 90° 的左右转，不用原地掉头代替倒车。"""
    a, b = graph.nodes[source], graph.nodes[target]
    ox, oy = b.x - a.x, b.y - a.y
    if math.hypot(ox, oy) < 1e-9:
        raise ValueError("相邻节点坐标重合，无法判断前进方向")
    if heading is None:
        return "straight"
    a, b = (graph.nodes[node] for node in heading)
    ix, iy = b.x - a.x, b.y - a.y
    if math.hypot(ix, iy) < 1e-9:
        raise ValueError("车头方向无效")
    angle = math.atan2(ix * oy - iy * ox, ix * ox + iy * oy)
    if abs(angle) > math.pi / 2 + 1e-9:
        return "reverse"
    if abs(angle) < 1e-9:
        return "straight"
    return "left" if angle > 0 else "right"


def plan_forward_remaining(
    graph: TopologyGraph,
    required_edges: set[str],
    current: str,
    home: str,
    *,
    heading: tuple[str, str] | None = None,
) -> CoveragePlan:
    """在“节点 + 驶入方向”状态图中贪心覆盖，所有正常动作仅前进。

    每次选择最近的剩余道路，但只使用仍能仅前进返航的状态。
    出发区只作为起点和最终终点，不能借它重置方向或中途折返。
    无法满足方向/返航约束的道路记为 skipped，不强求全覆盖或最短。
    heading 表示车头方向的两个端点。实车遇障倒回入口后，由既有
    恢复状态机按首条出边转向；仅在这次重规划时传入 heading=None。
    """
    if current not in graph.nodes or home not in graph.nodes:
        raise ValueError("current/home 不在拓扑节点中")
    unknown = required_edges - graph.edges.keys()
    if unknown:
        raise ValueError(f"未知道路: {sorted(unknown)}")
    if heading is not None and any(node not in graph.nodes for node in heading):
        raise ValueError("车头方向引用未知节点")

    # (入边起点, 当前节点)；None 为独立的起始状态，其方向由 heading 提供。
    State = tuple[str | None, str]
    root: State = (None, current)
    adjacency: dict[State, list[tuple[State, float, str]]] = {}
    parents: dict[State, set[State]] = {}
    stack = [root]
    while stack:
        state = stack.pop()
        if state in adjacency:
            continue
        previous, node = state
        adjacency[state] = []
        if node == home and state != root:
            continue
        facing = heading if previous is None else (previous, node)
        for neighbor, length, edge_id in sorted(graph.neighbors(node)):
            if forward_maneuver(graph, facing, node, neighbor) == "reverse":
                continue
            nxt: State = (node, neighbor)
            adjacency[state].append((nxt, length, edge_id))
            parents.setdefault(nxt, set()).add(state)
            stack.append(nxt)

    # 反向可达性：进入每一个保留状态后，都必须还有不倒车的返航路线。
    safe = {state for state in adjacency if state[1] == home}
    stack = list(safe)
    while stack:
        for parent in parents.get(stack.pop(), ()):
            if parent not in safe:
                safe.add(parent)
                stack.append(parent)
    if root not in safe:
        raise ForwardReturnUnavailable("按当前车头方向无法仅前进返回出发区")

    reachable = _reachable_nodes(graph, current)
    unreachable = tuple(sorted(
        edge_id for edge_id in required_edges
        if edge_id in graph.blocked
        or graph.edges[edge_id].u not in reachable
        or graph.edges[edge_id].v not in reachable
    ))
    pending = required_edges - set(unreachable)
    nodes = [current]
    edges: list[str] = []
    state = root

    def search(start: State, *, returning: bool = False):
        distances = {start: 0.0}
        previous: dict[State, tuple[State, str]] = {}
        order = count()
        heap = [(0.0, next(order), start)]
        while heap:
            cost, _, source = heappop(heap)
            if cost > distances[source]:
                continue
            for target, length, edge_id in adjacency[source]:
                if target not in safe or (target[1] == home and not returning):
                    continue
                candidate = cost + length
                if candidate < distances.get(target, float("inf")):
                    distances[target] = candidate
                    previous[target] = (source, edge_id)
                    heappush(heap, (candidate, next(order), target))
        return distances, previous

    def append_path(target: State, previous: dict[State, tuple[State, str]]) -> None:
        nonlocal state
        moves = []
        cursor = target
        while cursor != state:
            parent, edge_id = previous[cursor]
            moves.append((cursor[1], edge_id))
            cursor = parent
        for node, edge_id in reversed(moves):
            nodes.append(node)
            edges.append(edge_id)
            pending.discard(edge_id)
        state = target

    while pending:
        distances, previous = search(state)
        best = None
        for source, cost in distances.items():
            for target, length, edge_id in adjacency[source]:
                if edge_id not in pending or target not in safe or target[1] == home:
                    continue
                key = (cost + length, edge_id, source[1], source[0] or "")
                if best is None or key < best[0]:
                    best = (key, source, target, edge_id)
        if best is None:
            break
        _, source, target, edge_id = best
        append_path(source, previous)
        nodes.append(target[1])
        edges.append(edge_id)
        pending.discard(edge_id)
        state = target

    distances, previous = search(state, returning=True)
    goals = [target for target in distances if target[1] == home]
    if not goals:
        raise AssertionError("仅前进规划丢失返航路径")
    target = min(goals, key=lambda item: (distances[item], item[0] or ""))
    append_path(target, previous)
    skipped = tuple(sorted(required_edges - set(edges) - set(unreachable)))
    return CoveragePlan(tuple(nodes), tuple(edges),
                        sum(graph.edges[edge_id].length_m for edge_id in edges),
                        unreachable, skipped)

