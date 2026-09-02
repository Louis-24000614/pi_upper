"""命令行：加载拓扑、Dijkstra、模拟封边对比路径。"""

from __future__ import annotations

import argparse
from pathlib import Path

from .graph import DEFAULT_TOPOLOGY, block_many, load_topology, shortest_path


def _print_path(title: str, result) -> None:
    print(title)
    if result is None:
        print("  (unreachable)")
        return
    print(f"  nodes ({len(result.node_ids)}): {result.format_nodes()}")
    print(f"  edges ({len(result.edge_ids)}): {result.format_edges()}")
    print(f"  length_m: {result.length_m:.3f}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Load nav_topology.yaml and run Dijkstra (optional blocked edges).",
    )
    parser.add_argument(
        "--topology",
        type=Path,
        default=DEFAULT_TOPOLOGY,
        help=f"YAML path (default: {DEFAULT_TOPOLOGY})",
    )
    parser.add_argument("--start", default="0_0", help="start node id")
    parser.add_argument("--goal", default="5_2", help="goal node id")
    parser.add_argument(
        "--block",
        action="append",
        default=[],
        metavar="EDGE_ID",
        help="block an edge id (repeatable); after first path, replan with these blocked",
    )
    parser.add_argument(
        "--list-tunnels",
        action="store_true",
        help="list edges marked tunnel:true and exit",
    )
    args = parser.parse_args(argv)

    graph = load_topology(args.topology)
    print(f"loaded {args.topology}")
    print(
        f"meta.name={graph.meta.get('name')} "
        f"nodes={len(graph.nodes)} edges={len(graph.edges)}"
    )

    if args.list_tunnels:
        tunnels = sorted(e.id for e in graph.edges.values() if e.tunnel)
        print("tunnels:")
        for edge_id in tunnels:
            print(f"  {edge_id}")
        return 0

    clear = shortest_path(graph, args.start, args.goal)
    _print_path(f"path {args.start} -> {args.goal} (no blocks):", clear)

    if args.block:
        block_many(graph, args.block)
        print(f"blocked: {', '.join(args.block)}")
        blocked_path = shortest_path(graph, args.start, args.goal)
        _print_path(f"path {args.start} -> {args.goal} (after block):", blocked_path)
        if clear and blocked_path and clear.edge_ids != blocked_path.edge_ids:
            print("OK: path changed after blocking.")
        elif clear and blocked_path:
            print("NOTE: path unchanged (blocked edges were not on the shortest path).")
        elif clear and blocked_path is None:
            print("OK: goal unreachable after blocking.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
