"""Agent 道路覆盖规划测试。"""

from __future__ import annotations

import unittest

from agent.coverage.planner import plan_remaining
from demo.planner import plan_remaining as demo_plan_remaining
from navigation.topo_proto.graph import load_topology


class CoveragePlannerTests(unittest.TestCase):
    def test_initial_plan_matches_validated_demo(self) -> None:
        agent_graph = load_topology()
        demo_graph = load_topology()

        actual = plan_remaining(
            agent_graph, set(agent_graph.edges), current="0_0", home="0_0"
        )
        expected = demo_plan_remaining(
            demo_graph, set(demo_graph.edges), current="0_0", home="0_0"
        )

        # 两个模块各自定义 CoveragePlan，类型身份不同，因此逐字段比较。
        self.assertEqual(actual.nodes, expected.nodes)
        self.assertEqual(actual.edges, expected.edges)
        self.assertEqual(actual.length_m, expected.length_m)
        self.assertEqual(actual.unreachable_edges, expected.unreachable_edges)
        self.assertEqual(len(actual.edges), 33)
        # 出发区三段保持 0.15 / 0.40 m，其余格网边为 0.97 m。
        # 29 次格网通行各缩短 0.03 m：30.1 - 29 * 0.03 = 29.23。
        self.assertAlmostEqual(actual.length_m, 29.23)

    def test_blocked_edge_is_not_reused(self) -> None:
        graph = load_topology()
        blocked = "2_1__2_2"
        graph.set_edge_blocked(blocked)
        required = set(graph.edges) - {blocked}

        plan = plan_remaining(graph, required, current="0_0", home="0_0")

        self.assertNotIn(blocked, plan.edges)
        self.assertFalse(plan.unreachable_edges)
        self.assertTrue(required <= set(plan.edges))


if __name__ == "__main__":
    unittest.main()
