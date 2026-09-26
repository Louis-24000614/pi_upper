"""在当前已知路网中，规划覆盖剩余道路并回到出发区的步行路线。"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

from navigation.topo_proto.graph import PathResult, TopologyGraph, shortest_path


@dataclass(frozen=True)
class CoveragePlan:
    nodes: tuple[str, ...]
    edges: tuple[str, ...]
    length_m: float
    unreachable_edges: tuple[str, ...] = ()


def _reachable_nodes(graph: TopologyGraph, start: str) -> set[str]:
    """只规划当前分量；路障可能把部分道路与车辆完全隔开。"""
    seen = {start}
    stack = [start]
    while stack:
        for neighbor, _, _ in graph.neighbors(stack.pop()):
            if neighbor not in seen:
                seen.add(neighbor)
                stack.append(neighbor)
    return seen


def plan_remaining(
    graph: TopologyGraph,
    required_edges: set[str],
    current: str,
    home: str,
) -> CoveragePlan:
    """规划未覆盖边的开放式邮递员路线，终点固定为出发区。

    初始全边必经时，这是无向中国邮递员问题的奇点匹配解。
    重规划后必经边可能分成数块：先用最短路贪心连接这些块，
    再对奇度节点做精确最小权匹配。连接块这一步是近似的，
    因此重规划结果保证覆盖与可行性，但不保证全局最短。
    """
    if current not in graph.nodes or home not in graph.nodes:
        raise ValueError("current/home 不在拓扑节点中")
    unknown = required_edges - graph.edges.keys()
    if unknown:
        raise ValueError(f"未知道路: {sorted(unknown)}")

    reachable = _reachable_nodes(graph, current)
    if home not in reachable:
        raise ValueError("出发区与当前位置不连通，无法返回")

    unreachable = tuple(sorted(
        edge_id for edge_id in required_edges
        if edge_id in graph.blocked
        or graph.edges[edge_id].u not in reachable
        or graph.edges[edge_id].v not in reachable
    ))
    pending = sorted(required_edges - set(unreachable))

    if not pending:
        direct = shortest_path(graph, current, home)
        assert direct is not None
        return CoveragePlan(tuple(direct.node_ids), tuple(direct.edge_ids),
                            direct.length_m, unreachable)

    # 同一条物理道路可能因补路而走多次，故用实例序号表示多重图中的边。
    instances: list[tuple[str, str, str]] = []

    def add_edge(edge_id: str) -> None:
        edge = graph.edges[edge_id]
        instances.append((edge.u, edge.v, edge_id))

    def add_path(path: PathResult) -> None:
        for edge_id in path.edge_ids:
            add_edge(edge_id)

    for edge_id in pending:
        add_edge(edge_id)

    @lru_cache(maxsize=None)
    def route(a: str, b: str) -> PathResult | None:
        return shortest_path(graph, a, b)

    def components() -> list[set[str]]:
        # 当前位置与终点也必须并入多重图，否则欧拉路径无法从车的位置开始。
        nodes = {current, home}
        adjacency: dict[str, set[str]] = {}
        for u, v, _ in instances:
            nodes.update((u, v))
            adjacency.setdefault(u, set()).add(v)
            adjacency.setdefault(v, set()).add(u)
        result = []
        while nodes:
            root = min(nodes)
            component = {root}
            stack = [root]
            nodes.remove(root)
            while stack:
                for nxt in adjacency.get(stack.pop(), ()):
                    if nxt in nodes:
                        nodes.remove(nxt)
                        component.add(nxt)
                        stack.append(nxt)
            result.append(component)
        return result

    # 剩余必经边可不连通；已走过的道路仍可作为连接路段再次通行。
    while True:
        parts = components()
        if len(parts) == 1:
            break
        best: tuple[float, str, str, PathResult] | None = None
        for i, left in enumerate(parts):
            for right in parts[i + 1:]:
                for u in sorted(left):
                    for v in sorted(right):
                        path = route(u, v)
                        if path is None:
                            continue
                        candidate = (path.length_m, u, v, path)
                        if best is None or candidate[:3] < best[:3]:
                            best = candidate
        if best is None:
            raise ValueError("剩余道路无法连接")
        add_path(best[3])

    degree: dict[str, int] = {}
    for u, v, _ in instances:
        degree[u] = degree.get(u, 0) + 1
        degree[v] = degree.get(v, 0) + 1
    odd = {node for node, count in degree.items() if count % 2}
    # 开放路线需要 current、home 为奇点；若同点出发并结束，则都为偶点。
    if current != home:
        odd.symmetric_difference_update((current, home))
    odd_nodes = sorted(odd)
    if len(odd_nodes) % 2:
        raise AssertionError("奇点数量必须为偶数")

    @lru_cache(maxsize=None)
    def match(mask: int) -> tuple[float, tuple[tuple[int, int], ...]]:
        """位掩码动态规划求奇点的最小权完美匹配。"""
        if mask == 0:
            return 0.0, ()
        first_bit = mask & -mask
        i = first_bit.bit_length() - 1
        rest = mask ^ first_bit
        best: tuple[float, tuple[tuple[int, int], ...]] | None = None
        bits = rest
        while bits:
            bit = bits & -bits
            j = bit.bit_length() - 1
            path = route(odd_nodes[i], odd_nodes[j])
            assert path is not None
            tail_cost, tail_pairs = match(rest ^ bit)
            candidate = (path.length_m + tail_cost, ((i, j),) + tail_pairs)
            if best is None or candidate < best:
                best = candidate
            bits ^= bit
        assert best is not None
        return best

    _, pairs = match((1 << len(odd_nodes)) - 1)
    for i, j in pairs:
        path = route(odd_nodes[i], odd_nodes[j])
        assert path is not None
        add_path(path)

    # Hierholzer：每个“道路实例”恰好使用一次，得到连续的欧拉行程。
    adjacency: dict[str, list[tuple[int, str]]] = {}
    for index, (u, v, _) in enumerate(instances):
        adjacency.setdefault(u, []).append((index, v))
        adjacency.setdefault(v, []).append((index, u))
    used: set[int] = set()
    stack: list[tuple[str, int | None]] = [(current, None)]
    reversed_walk: list[tuple[str, int | None]] = []
    while stack:
        node = stack[-1][0]
        while adjacency.get(node) and adjacency[node][-1][0] in used:
            adjacency[node].pop()
        if not adjacency.get(node):
            reversed_walk.append(stack.pop())
            continue
        index, nxt = adjacency[node].pop()
        used.add(index)
        stack.append((nxt, index))

    walk = list(reversed(reversed_walk))
    nodes = tuple(node for node, _ in walk)
    edges = tuple(instances[index][2] for _, index in walk[1:] if index is not None)
    if len(used) != len(instances) or nodes[-1] != home:
        raise AssertionError("欧拉行程没有完整覆盖或未回到出发区")
    length = sum(graph.edges[edge_id].length_m for edge_id in edges)
    return CoveragePlan(nodes, edges, length, unreachable)
