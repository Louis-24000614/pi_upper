"""运行入口：默认图形界面；--headless 输出完整行驶日志。"""

from __future__ import annotations

import argparse
from pathlib import Path

from navigation.topo_proto.graph import DEFAULT_TOPOLOGY, load_topology

from .mission import PostmanMission


def main() -> int:
    parser = argparse.ArgumentParser(description="未知路障下的道路覆盖与在线重规划演示")
    parser.add_argument("--topology", type=Path, default=DEFAULT_TOPOLOGY,
                        help="道路拓扑 YAML，默认读取 config/nav_topology.yaml")
    parser.add_argument("--headless", action="store_true", help="不打开窗口，直接打印运行过程")
    parser.add_argument("--obstacles", nargs=3, metavar=("EDGE1", "EDGE2", "EDGE3"),
                        help="预设 3 条路障道路；仅在 --headless 模式使用")
    parser.add_argument("--list-edges", action="store_true", help="列出可选择的道路 ID")
    args = parser.parse_args()
    graph = load_topology(args.topology)
    if args.list_edges:
        for edge in graph.edges.values():
            print(f"{edge.id:18} {edge.u} -> {edge.v}")
        return 0
    if args.headless:
        if not args.obstacles:
            parser.error("--headless 需要 --obstacles EDGE1 EDGE2 EDGE3")
        try:
            mission = PostmanMission(graph, set(args.obstacles))
        except ValueError as exc:
            parser.error(str(exc))
        print(f"初始规划：{len(mission.plan.edges)} 段行程，"
              f"覆盖 {len(graph.edges)} 条道路；3 个障碍对车辆未知。")
        for event in mission.run_all():
            print(event.message)
        print(f"汇总：实际覆盖 {len(mission.covered)} 条，"
              f"发现路障 {len(mission.discovered)} 个，"
              f"到达巡逻点 {len(mission.visited_patrol)}/12，"
              f"剩余不可达道路 {len(mission.plan.unreachable_edges)} 条。")
        return 0 if mission.phase == "done" and not mission.plan.unreachable_edges else 2
    if args.obstacles:
        parser.error("图形界面请直接点击道路中段选择障碍；--obstacles 只用于 --headless")
    from .gui import launch
    launch(args.topology)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
