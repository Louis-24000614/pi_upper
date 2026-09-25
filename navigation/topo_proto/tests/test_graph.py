"""拓扑加载与 Dijkstra 封边改道断言。"""

from __future__ import annotations

import unittest

from topo_proto.graph import load_topology, shortest_path


class TopologyGraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.graph = load_topology()

    def test_loads_figure3_v2(self) -> None:
        self.assertEqual(self.graph.meta.get("name"), "figure3_rowcol_v2")
        self.assertIn("0_0", self.graph.nodes)
        self.assertIn("5_2", self.graph.nodes)
        self.assertGreaterEqual(len(self.graph.edges), 20)

    def test_start_to_goal_reachable(self) -> None:
        path = shortest_path(self.graph, "0_0", "5_2")
        self.assertIsNotNone(path)
        assert path is not None
        self.assertEqual(path.node_ids[0], "0_0")
        self.assertEqual(path.node_ids[-1], "5_2")
        self.assertGreater(path.length_m, 0.0)
        self.assertEqual(len(path.edge_ids), len(path.node_ids) - 1)

    def test_block_forces_detour_or_unreachable(self) -> None:
        self.graph.clear_blocked()
        before = shortest_path(self.graph, "0_0", "5_2")
        self.assertIsNotNone(before)
        assert before is not None

        # 封掉原最短路上的第一条边，应改道或不可达
        edge0 = before.edge_ids[0]
        self.graph.set_edge_blocked(edge0, True)
        after = shortest_path(self.graph, "0_0", "5_2")
        self.assertTrue(
            after is None or after.edge_ids != before.edge_ids,
            msg=f"expected detour after blocking {edge0}",
        )
        self.graph.clear_blocked()

    def test_block_edge_on_shortest_path_to_2_4(self) -> None:
        """封掉右出口下去的竖边后，去 2_4 应改走左出口。"""
        self.graph.clear_blocked()
        before = shortest_path(self.graph, "0_0", "2_4")
        self.assertIsNotNone(before)
        assert before is not None
        self.assertIn("1_3__2_3", before.edge_ids)
        self.graph.set_edge_blocked("1_3__2_3", True)
        after = shortest_path(self.graph, "0_0", "2_4")
        self.assertIsNotNone(after)
        assert after is not None
        self.assertNotEqual(before.edge_ids, after.edge_ids)
        self.assertNotIn("1_3__2_3", after.edge_ids)
        self.assertIn("0_0__1_2", after.edge_ids)
        self.graph.clear_blocked()


if __name__ == "__main__":
    unittest.main()
