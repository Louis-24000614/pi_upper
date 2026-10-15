"""仅前进规划与真实到点/遇障恢复接口的离线集成验证。"""

import unittest

from agent.coverage.forward import plan_forward_remaining
from agent.route_agent import FollowEdge, MissionComplete, RouteAgent, StartBackup, Stop
from navigation.topo_proto.graph import Edge, Node, TopologyGraph, load_topology
from road_follow.departure import departure_maneuver, handoff_arrival, handoff_entrance
from road_follow.junction_turn import JunctionTurn
from road_follow.obstacle_route import recovery_maneuver


class ForwardRouteTests(unittest.TestCase):
    def test_forward_mission_uses_real_departure_handoffs_and_returns_home(self):
        graph = load_topology()
        agent = RouteAgent(graph, no_planned_reverse=True)
        xy = {key: (node.x, node.y) for key, node in graph.nodes.items()}
        first = agent.start()
        self.assertEqual((first.from_node, first.to_node), ("0_0", "0_J"))
        side, _command = handoff_entrance(agent, xy)
        self.assertEqual(side, "right")
        self.assertNotIn("0_0", agent.state.route_nodes[1:-1])
        departures = []
        for _ in range(200):
            state = handoff_arrival(
                JunctionTurn(phase="arrived"), agent, xy, lambda line: True,
            )
            if state.phase == "done":
                break
            self.assertEqual(state.phase, "stopping")
            self.assertIn(state.departure, ("straight", "left", "right"))
            departures.append(state.departure)
        else:
            self.fail("仅前进任务未结束")
        self.assertTrue(departures)
        self.assertEqual(agent.state.phase, "done")
        self.assertEqual(agent.state.current_node, "0_0")
        self.assertEqual(agent.state.covered_edges, set(graph.edges))
        self.assertEqual(agent.state.skipped_edges, ())

    def test_default_route_keeps_original_planning_and_completes(self):
        agent = RouteAgent(load_topology())
        command = agent.start()
        original = agent.state.route_nodes
        self.assertEqual(len(agent.state.route_edges), 33)
        self.assertTrue(any(a == c for a, c in zip(original, original[2:])))
        for _ in range(200):
            if not isinstance(command, FollowEdge):
                break
            command = agent.node_reached(command.to_node)
        self.assertIsInstance(command, MissionComplete)
        self.assertEqual(agent.state.covered_edges, set(agent.graph.edges))

    def test_bottom_edge_does_not_reverse_to_previous_node(self):
        graph = load_topology()
        plan = plan_forward_remaining(
            graph, {"5_2__5_3"}, "5_3", "0_0", heading=("5_2", "5_3"),
        )
        self.assertNotEqual(plan.nodes[1], "5_2")
        self.assertEqual(plan.nodes[-1], "0_0")

    def test_obstacles_allow_backup_then_only_forward_departures(self):
        obstacles = {"1_2__2_2", "2_1__2_2", "3_1__3_2"}
        agent = RouteAgent(load_topology(), no_planned_reverse=True)
        xy = {key: (node.x, node.y) for key, node in agent.graph.nodes.items()}
        command = agent.start()
        arrived_from = None
        backups = 0
        for _ in range(300):
            if not isinstance(command, FollowEdge):
                break
            self.assertNotIn(command.edge_id, agent.state.blocked_edges)
            if arrived_from is not None:
                self.assertIn(departure_maneuver(
                    xy, arrived_from, command.from_node, command.to_node,
                ), ("straight", "left", "right"))
            if command.edge_id in obstacles:
                failed = command
                result = agent.obstacle_confirmed(failed.edge_id, 0.4)
                self.assertIsInstance(result, StartBackup)
                self.assertEqual(agent.state.phase, "backing")
                self.assertEqual(agent.state.route_edges, ())
                command = agent.backup_done(failed.edge_id, failed.from_node)
                if isinstance(command, FollowEdge):
                    turn = recovery_maneuver(
                        xy, failed.from_node, failed.to_node, command.to_node,
                    )
                    self.assertIn(turn.quarters, (1, 2))
                arrived_from = None  # 既有遇障恢复已按下一条边完成转向。
                backups += 1
            else:
                arrived_from = command.from_node
                command = agent.node_reached(command.to_node)
        self.assertIsInstance(command, MissionComplete)
        self.assertEqual(backups, 3)
        self.assertEqual(agent.state.blocked_edges, obstacles)
        self.assertEqual(agent.state.current_node, "0_0")
        self.assertTrue(agent.state.skipped_edges)
        self.assertEqual(
            agent.state.covered_edges | set(agent.state.skipped_edges)
            | set(agent.state.unreachable_edges) | obstacles,
            set(agent.graph.edges),
        )

    def test_skipped_and_disconnected_roads_do_not_prevent_forward_completion(self):
        nodes = {name: Node(name, x, y) for name, x, y in (
            ("A", 0, 0), ("B", 1, 0), ("C", 1, 1), ("D", 0, 1),
            ("E", 2, 0), ("F", 3, 0), ("G", 4, 0),
        )}
        edges = {u + v: Edge(u + v, u, v, 1) for u, v in (
            ("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"),
            ("B", "E"), ("F", "G"),
        )}
        agent = RouteAgent(TopologyGraph({}, nodes, edges), home="A", no_planned_reverse=True)
        command = agent.start()
        for _ in range(20):
            if not isinstance(command, FollowEdge):
                break
            command = agent.node_reached(command.to_node)
        self.assertIsInstance(command, MissionComplete)
        self.assertEqual(agent.state.skipped_edges, ("BE",))
        self.assertEqual(agent.state.unreachable_edges, ("FG",))

    def test_obstacle_recovery_can_turn_around_then_return_along_a_straight_road(self):
        nodes = {name: Node(name, i, 0) for i, name in enumerate("ABCD")}
        edges = {u + v: Edge(u + v, u, v, 1) for u, v in (("A", "B"), ("B", "C"), ("C", "D"))}
        graph = TopologyGraph({}, nodes, edges)
        agent = RouteAgent(graph, home="A", no_planned_reverse=True)
        # 模拟倒回 C，既有遇障恢复可掉头朝 B，此后应一直前进到 A。
        agent.state.phase = "following"
        agent.state.current_node = agent.state.from_node = "C"
        agent.state.current_edge, agent.state.to_node = "CD", "D"
        self.assertIsInstance(agent.obstacle_confirmed("CD", 0.3), StartBackup)
        result = agent.backup_done("CD", "C")
        self.assertIsInstance(result, FollowEdge)
        self.assertEqual(result.to_node, "B")
        # 转向后 C -> B -> A 一直前进，不能进入其它死胡同补覆盖。
        result = agent.node_reached("B")
        self.assertEqual(result.to_node, "A")
        self.assertIsInstance(agent.node_reached("A"), MissionComplete)

    def test_disconnected_home_stops_without_falling_back_to_original_planner(self):
        agent = RouteAgent(load_topology(), no_planned_reverse=True)
        agent.start()
        command = agent.node_reached("0_J")
        agent.graph.set_edge_blocked("0_0__0_J")
        agent.obstacle_confirmed(command.edge_id, 0.3)
        result = agent.backup_done(command.edge_id, command.from_node)
        self.assertIsInstance(result, Stop)
        self.assertIn("无法仅前进返回", result.reason)

    def test_guard_refuses_accidental_reverse_at_normal_arrival(self):
        agent = RouteAgent(load_topology(), no_planned_reverse=True)
        first = agent.start()
        agent.state.route_nodes = (first.from_node, first.to_node, first.from_node)
        agent.state.route_edges = (first.edge_id, first.edge_id)
        result = agent.node_reached(first.to_node)
        self.assertIsInstance(result, Stop)
        self.assertIn("拒绝普通反向", result.reason)


if __name__ == "__main__":
    unittest.main()
