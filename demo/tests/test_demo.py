"""检查道路覆盖、发现时机和封边后的路线连续性。"""

from __future__ import annotations

import unittest

from navigation.topo_proto.graph import Edge, Node, TopologyGraph, load_topology
from demo.mission import PostmanMission
from demo.planner import plan_remaining


class PlannerTests(unittest.TestCase):
    def test_closed_postman_covers_cycle_once(self) -> None:
        nodes = {name: Node(name, x, y) for name, x, y in
                 (("A", 0, 0), ("B", 1, 0), ("C", 1, 1), ("D", 0, 1))}
        edges = {f"{u}{v}": Edge(f"{u}{v}", u, v, 1.0) for u, v in
                 (("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"))}
        graph = TopologyGraph({}, nodes, edges)
        plan = plan_remaining(graph, set(edges), "A", "A")
        self.assertEqual(len(plan.edges), 4)
        self.assertEqual(set(plan.edges), set(edges))
        self.assertEqual(plan.nodes[0], plan.nodes[-1])

    def test_remaining_disconnected_edges_use_old_roads_as_connectors(self) -> None:
        nodes = {name: Node(name, index, 0) for index, name in enumerate("ABCD")}
        edges = {f"{u}{v}": Edge(f"{u}{v}", u, v, 1.0) for u, v in
                 (("A", "B"), ("B", "C"), ("C", "D"))}
        graph = TopologyGraph({}, nodes, edges)
        plan = plan_remaining(graph, {"AB", "CD"}, "A", "A")
        self.assertEqual(plan.length_m, 6.0)
        self.assertEqual(plan.nodes[0], "A")
        self.assertEqual(plan.nodes[-1], "A")
        self.assertTrue({"AB", "CD"} <= set(plan.edges))


class MissionTests(unittest.TestCase):
    def test_hidden_obstacles_trigger_only_on_entry_then_uturn(self) -> None:
        graph = load_topology()
        obstacles = {"1_2__2_2", "2_1__2_2", "3_1__3_2"}
        mission = PostmanMission(graph, obstacles)
        self.assertFalse(mission.discovered)
        self.assertFalse(graph.blocked)

        while mission.phase == "running" and not mission.discovered:
            event = mission.step()
        self.assertEqual(event.kind, "discover")
        self.assertEqual(mission.current, mission.entry_node)
        self.assertEqual(mission.phase, "backing")
        self.assertIn(event.edge_id, graph.blocked)
        self.assertNotIn(event.edge_id, mission.covered)
        self.assertEqual(mission.step().kind, "uturn")

        mission.run_all()
        self.assertEqual(mission.phase, "done")
        self.assertEqual(mission.current, mission.home)
        self.assertEqual(mission.discovered, obstacles)
        self.assertEqual(mission.covered, set(graph.edges) - obstacles)
        self.assertEqual(len(mission.visited_patrol), 12)
        self.assertFalse(mission.plan.unreachable_edges)


if __name__ == "__main__":
    unittest.main()
