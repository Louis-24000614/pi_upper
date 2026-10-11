"""FORWARD_DONE 之后的动作来自下一条边，不来自启动参数。"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from road_follow.departure import (
    apply_departure,
    departure_maneuver,
    handoff_arrival,
    handoff_entrance,
)
from road_follow.junction_turn import JunctionTurn


NODES = {
    "3_1": (0.40, 2.00),
    "3_2": (1.20, 2.00),
    "3_3": (2.00, 2.00),
    "2_2": (1.20, 2.80),
    "4_2": (1.20, 1.20),
    "3_4": (2.80, 2.00),
    "4_4": (2.80, 1.20),
    "4_3": (2.00, 1.20),
    "4_5": (3.60, 1.20),
    "5_4": (2.80, 0.40),
}


class FollowEdge:
    def __init__(self, edge_id: str, from_node: str, to_node: str) -> None:
        self.edge_id = edge_id
        self.from_node = from_node
        self.to_node = to_node


class MissionComplete:
    def __init__(self, home: str) -> None:
        self.home = home


class Stop:
    def __init__(self, reason: str) -> None:
        self.reason = reason


class DepartureTest(unittest.TestCase):
    def test_live_entrance_handoff_reaches_0_j_and_turns_right(self) -> None:
        import sys
        from pathlib import Path

        root = Path(__file__).resolve().parents[3]
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from agent.route_agent import RouteAgent
        from topo_proto.graph import load_topology

        graph = load_topology()
        agent = RouteAgent(graph)
        first = agent.start()
        self.assertEqual(
            (first.edge_id, first.from_node, first.to_node),
            ("0_0__0_J", "0_0", "0_J"),
        )
        node_xy = {node_id: (node.x, node.y) for node_id, node in graph.nodes.items()}

        side, second = handoff_entrance(agent, node_xy)

        self.assertEqual(side, "right")
        self.assertEqual(
            (second.edge_id, second.from_node, second.to_node),
            ("0_J__1_2", "0_J", "1_2"),
        )
        self.assertEqual(agent.state.current_node, "0_J")
        self.assertIn("0_0__0_J", agent.state.covered_edges)

    def test_live_agent_reports_the_expected_node_before_turning(self) -> None:
        import sys
        from pathlib import Path

        root = Path(__file__).resolve().parents[3]
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from agent.route_agent import RouteAgent
        from topo_proto.graph import load_topology

        graph = load_topology()
        agent = RouteAgent(graph)
        first = agent.start()
        self.assertEqual(type(first).__name__, "FollowEdge")
        node_xy = {node_id: (node.x, node.y) for node_id, node in graph.nodes.items()}
        sent: list[str] = []
        state = handoff_arrival(
            JunctionTurn(phase="arrived", side="left"),
            agent,
            node_xy,
            lambda line: sent.append(line) or True,
        )

        self.assertIn(state.departure, {"straight", "left", "right", "backup"})
        self.assertEqual(agent.state.current_node, first.to_node)
        self.assertIn(first.edge_id, agent.state.covered_edges)
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(sent, ["stop"])
    def test_axis_aligned_departure(self) -> None:
        self.assertEqual(departure_maneuver(NODES, "3_1", "3_2", "3_3"), "straight")
        self.assertEqual(departure_maneuver(NODES, "3_1", "3_2", "2_2"), "left")
        self.assertEqual(departure_maneuver(NODES, "3_1", "3_2", "4_2"), "right")
        self.assertEqual(departure_maneuver(NODES, "3_1", "3_2", "3_1"), "backup")

    def test_straight_resumes_follow_without_a_turn(self) -> None:
        sent: list[str] = []
        state = apply_departure(
            JunctionTurn(phase="arrived", side="left"),
            "straight",
            lambda line: sent.append(line) or True,
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(state.departure, "straight")
        self.assertEqual(sent, ["stop"])

    def test_handoff_uses_the_next_edge_not_the_latched_side(self) -> None:
        sent: list[str] = []
        agent = SimpleNamespace(
            state=SimpleNamespace(from_node="3_1", to_node="3_2"),
            node_reached=lambda node_id: FollowEdge("3_2__4_2", "3_2", "4_2"),
        )
        state = handoff_arrival(
            JunctionTurn(phase="arrived", side="left"),
            agent,
            NODES,
            lambda line: sent.append(line) or True,
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(state.departure, "right")
        self.assertEqual(sent, ["stop"])

    def test_reverse_edge_is_backup(self) -> None:
        sent: list[str] = []
        agent = SimpleNamespace(
            state=SimpleNamespace(from_node="3_1", to_node="3_2"),
            node_reached=lambda node_id: FollowEdge("3_1__3_2", "3_2", "3_1"),
        )
        state = handoff_arrival(
            JunctionTurn(phase="arrived", side="left"),
            agent,
            NODES,
            lambda line: sent.append(line) or True,
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(state.departure, "backup")
        self.assertEqual(sent, ["stop"])

    def test_reverse_arrival_uses_heading_instead_of_travel_direction(self) -> None:
        for target, expected in (("4_3", "left"), ("4_5", "right"), ("3_4", "straight")):
            with self.subTest(target=target):
                self.assertEqual(
                    departure_maneuver(
                        NODES, "3_4", "4_4", target, reverse_arrival=True
                    ),
                    expected,
                )

    def test_reverse_arrival_does_not_start_another_backup_from_zero_progress(self) -> None:
        with self.assertRaisesRegex(ValueError, "仍朝车尾"):
            departure_maneuver(
                NODES, "3_4", "4_4", "5_4", reverse_arrival=True
            )

    def test_reverse_arrival_faults_on_invalid_route_geometry(self) -> None:
        for target, nodes in (
            ("missing", NODES),
            ("4_3", NODES | {"3_4": NODES["4_4"]}),
            ("4_3", NODES | {"4_3": NODES["4_4"]}),
            ("5_4", NODES),
        ):
            with self.subTest(target=target, nodes=nodes):
                agent = SimpleNamespace(
                    state=SimpleNamespace(from_node="3_4", to_node="4_4"),
                    node_reached=lambda node_id: FollowEdge("next", node_id, target),
                )
                sent = []
                state = handoff_arrival(
                    JunctionTurn(phase="arrived"), agent, nodes,
                    lambda line: sent.append(line) or True, reverse_arrival=True,
                )
                self.assertEqual(state.phase, "fault")
                self.assertEqual(state.departure, "none")
                self.assertEqual(sent, [])

    def test_reverse_arrival_without_expected_nodes_does_not_advance_agent(self) -> None:
        from unittest.mock import Mock

        for from_node, to_node in ((None, "4_4"), ("3_4", None)):
            with self.subTest(from_node=from_node, to_node=to_node):
                agent = SimpleNamespace(
                    state=SimpleNamespace(from_node=from_node, to_node=to_node),
                    node_reached=Mock(),
                )
                state = handoff_arrival(
                    JunctionTurn(phase="arrived"), agent, NODES,
                    lambda line: True, reverse_arrival=True,
                )
                self.assertEqual(state.phase, "fault")
                agent.node_reached.assert_not_called()

    def test_mission_complete_does_not_turn(self) -> None:
        sent: list[str] = []
        agent = SimpleNamespace(
            state=SimpleNamespace(from_node="1_2", to_node="0_0"),
            node_reached=lambda node_id: MissionComplete("0_0"),
        )
        state = handoff_arrival(
            JunctionTurn(phase="arrived"),
            agent,
            NODES,
            lambda line: sent.append(line) or True,
        )
        self.assertEqual(state.phase, "done")
        self.assertEqual(sent, [])

    def test_stop_command_faults_without_turning(self) -> None:
        sent: list[str] = []
        agent = SimpleNamespace(
            state=SimpleNamespace(from_node="3_1", to_node="3_2"),
            node_reached=lambda node_id: Stop("到达节点不匹配"),
        )
        state = handoff_arrival(
            JunctionTurn(phase="arrived", side="right"),
            agent,
            NODES,
            lambda line: sent.append(line) or True,
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(sent, [])


if __name__ == "__main__":
    unittest.main()
