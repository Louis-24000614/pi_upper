"""Navigation 到 Agent 的事件适配测试。"""

from __future__ import annotations

import unittest

from agent.route_agent import FollowEdge, RouteAgent, StartBackup
from navigation.agent_bridge import AgentBridge
from navigation.road_follow.backup import Backup
from navigation.topo_proto.graph import load_topology
from vision.obstacle.blockage import ObstacleObservation


class AgentBridgeTests(unittest.TestCase):
    def test_obstacle_and_backup_events_preserve_topology_order(self) -> None:
        agent = RouteAgent(load_topology())
        bridge = AgentBridge(agent)
        entrance = bridge.start()
        assert isinstance(entrance, FollowEdge)
        self.assertEqual(entrance.edge_id, "0_0__0_J")
        command = bridge.node_reached("0_J")
        assert isinstance(command, FollowEdge)
        self.assertEqual(command.edge_id, "0_J__1_2")

        bridge.update_odometry(0.0, 0.0, 0.0)
        bridge.update_odometry(0.32, 0.0, 0.0)
        observation = ObstacleObservation(
            detected=True,
            candidate=True,
            hard_blocked=True,
            just_confirmed=True,
            stable_frames=3,
            reason="confirmed",
        )
        backup_command = bridge.obstacle_observed(observation)
        self.assertEqual(
            backup_command,
            StartBackup(command.edge_id, command.from_node, 0.32),
        )

        self.assertIsNone(bridge.backup_updated(Backup(phase="backing")))
        resumed = bridge.backup_updated(Backup(phase="done"))
        self.assertIsInstance(resumed, FollowEdge)
        self.assertEqual(bridge.edge_progress.s_m, 0.0)
        self.assertNotIn(command.edge_id, agent.state.route_edges)

    def test_node_reached_resets_edge_progress(self) -> None:
        agent = RouteAgent(load_topology())
        bridge = AgentBridge(agent)
        command = bridge.start()
        assert isinstance(command, FollowEdge)
        bridge.update_odometry(0.0, 0.0, 0.0)
        bridge.update_odometry(0.25, 0.0, 0.0)
        self.assertAlmostEqual(bridge.edge_progress.s_m, 0.25)

        next_command = bridge.node_reached(command.to_node)

        self.assertIsInstance(next_command, FollowEdge)
        self.assertEqual(bridge.edge_progress.s_m, 0.0)


if __name__ == "__main__":
    unittest.main()
