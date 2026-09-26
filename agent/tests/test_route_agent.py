"""RouteAgent 的纯事件状态机测试，不打开相机或串口。"""

from __future__ import annotations

import unittest

from agent.route_agent import (
    FollowEdge,
    MissionComplete,
    RouteAgent,
    StartBackup,
    Stop,
)
from navigation.topo_proto.graph import load_topology


class RouteAgentTests(unittest.TestCase):
    def setUp(self) -> None:
        self.graph = load_topology()
        self.agent = RouteAgent(self.graph)

    def _finish_without_new_obstacles(self, command):
        steps = 0
        while isinstance(command, FollowEdge):
            command = self.agent.node_reached(command.to_node)
            steps += 1
            self.assertLess(steps, 200)
        return command

    def test_no_obstacle_covers_all_edges_and_returns_home(self) -> None:
        result = self._finish_without_new_obstacles(self.agent.start())

        self.assertIsInstance(result, MissionComplete)
        self.assertEqual(self.agent.state.phase, "done")
        self.assertEqual(self.agent.state.current_node, "0_0")
        self.assertEqual(self.agent.state.covered_edges, set(self.graph.edges))

    def test_initial_route_reaches_0_j_then_uses_configured_right_exit(self) -> None:
        first = self.agent.start()
        self.assertEqual(first, FollowEdge("0_0__0_J", "0_0", "0_J"))

        second = self.agent.node_reached("0_J")

        self.assertEqual(second, FollowEdge("0_J__1_2", "0_J", "1_2"))

    def test_obstacle_requires_backup_before_replan(self) -> None:
        entrance = self.agent.start()
        self.assertEqual(entrance, FollowEdge("0_0__0_J", "0_0", "0_J"))
        command = self.agent.node_reached("0_J")
        self.assertEqual(command, FollowEdge("0_J__1_2", "0_J", "1_2"))
        assert isinstance(command, FollowEdge)

        backup = self.agent.obstacle_confirmed(command.edge_id, 0.35)
        self.assertEqual(backup, StartBackup(command.edge_id, command.from_node, 0.35))
        self.assertEqual(self.agent.state.phase, "backing")
        self.assertIn(command.edge_id, self.agent.state.blocked_edges)
        self.assertNotIn(command.edge_id, self.agent.state.covered_edges)
        self.assertFalse(self.agent.state.route_edges)

        resumed = self.agent.backup_done(command.edge_id, command.from_node)
        self.assertIsInstance(resumed, FollowEdge)
        self.assertNotIn(command.edge_id, self.agent.state.route_edges)
        result = self._finish_without_new_obstacles(resumed)
        self.assertIsInstance(result, MissionComplete)
        self.assertEqual(
            self.agent.state.covered_edges,
            set(self.graph.edges) - {command.edge_id},
        )

    def test_backup_failure_stops_mission(self) -> None:
        command = self.agent.start()
        assert isinstance(command, FollowEdge)
        self.agent.obstacle_confirmed(command.edge_id, 0.30)

        result = self.agent.backup_failed(command.edge_id, "vision lost")

        self.assertIsInstance(result, Stop)
        self.assertEqual(self.agent.state.phase, "fault")
        self.assertIn("vision lost", result.reason)

    def test_three_known_obstacles_cover_every_other_edge(self) -> None:
        obstacles = {"1_2__2_2", "2_1__2_2", "3_1__3_2"}
        command = self.agent.start()
        steps = 0
        while isinstance(command, FollowEdge):
            if command.edge_id in obstacles and command.edge_id not in self.agent.state.blocked_edges:
                backup = self.agent.obstacle_confirmed(command.edge_id, 0.40)
                self.assertIsInstance(backup, StartBackup)
                command = self.agent.backup_done(command.edge_id, command.from_node)
            else:
                command = self.agent.node_reached(command.to_node)
            steps += 1
            self.assertLess(steps, 300)

        self.assertIsInstance(command, MissionComplete)
        self.assertEqual(self.agent.state.blocked_edges, obstacles)
        self.assertEqual(
            self.agent.state.covered_edges,
            set(self.graph.edges) - obstacles,
        )
        self.assertEqual(self.agent.state.current_node, "0_0")

    def test_wrong_node_event_enters_fault(self) -> None:
        command = self.agent.start()
        assert isinstance(command, FollowEdge)

        result = self.agent.node_reached("5_4")

        self.assertIsInstance(result, Stop)
        self.assertEqual(self.agent.state.phase, "fault")


if __name__ == "__main__":
    unittest.main()
