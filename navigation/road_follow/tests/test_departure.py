"""FORWARD_DONE 之后的动作来自下一条边，不来自启动参数。"""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from road_follow.backup import Backup, BackupConfig
from road_follow.control import VelocityCommand
from road_follow.departure import (
    apply_departure,
    departure_maneuver,
    handoff_arrival,
    handoff_entrance,
    step_planned_backup,
)
from road_follow.junction_turn import JunctionTurn, JunctionTurnConfig


NODES = {
    "3_1": (0.40, 2.00),
    "3_2": (1.20, 2.00),
    "3_3": (2.00, 2.00),
    "2_2": (1.20, 2.80),
    "4_2": (1.20, 1.20),
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

    def test_reverse_arrival_opposite_edge_plans_uturn(self) -> None:
        sent: list[str] = []
        agent = SimpleNamespace(
            state=SimpleNamespace(from_node="3_1", to_node="3_2"),
            node_reached=lambda node_id: FollowEdge("3_2__3_3", "3_2", "3_3"),
        )
        self.assertEqual(
            departure_maneuver(
                NODES, "3_1", "3_2", "3_3", reverse_arrival=True
            ),
            "uturn",
        )
        state = handoff_arrival(
            JunctionTurn(phase="arrived"),
            agent,
            NODES,
            lambda line: sent.append(line) or True,
            reverse_arrival=True,
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(state.departure, "uturn")
        self.assertEqual(state.side, "left")
        self.assertEqual(state.quarters_remaining, 2)
        self.assertEqual(sent, ["stop"])

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


class PlannedBackupTest(unittest.TestCase):
    def setUp(self) -> None:
        self.state = Backup()
        self.backup_cfg = BackupConfig()
        self.align_cfg = JunctionTurnConfig(align_stable_frames=3)

    def step(self, heading=0.0, now=1.0, yaw=0.5, progress=0.6, captured=None, odom_valid=True):
        self.state, command = step_planned_backup(
            self.state, lane_heading_rad=heading,
            frame_captured_s=now if captured is None else captured,
            progress_s_m=progress, command=VelocityCommand(0.08, 0.0, "follow"),
            now_s=now, yaw_rad=yaw, odom_valid=odom_valid,
            backup_cfg=self.backup_cfg, align_cfg=self.align_cfg,
        )
        return command

    def test_visual_alignment_turns_in_place_without_latching_imu(self):
        for heading in (0.2, -0.2):
            with self.subTest(heading=heading):
                self.state = Backup()
                command = self.step(heading=heading)
                self.assertEqual(self.state.phase, "align")
                self.assertIsNone(self.state.hold_yaw_rad)
                self.assertEqual(command.v_mps, 0.0)
                self.assertLess(command.omega_radps * heading, 0.0)
                self.assertLessEqual(abs(command.omega_radps), self.align_cfg.align_max_abs_omega)

    def test_locks_current_imu_only_after_alignment_then_reverses(self):
        self.step(heading=0.2, now=1.0, yaw=0.5)
        for now, yaw in ((20.0, 0.3), (20.1, 0.2)):
            command = self.step(now=now, yaw=yaw)
            self.assertEqual(command.v_mps, 0.0)
            self.assertIsNone(self.state.hold_yaw_rad)
        command = self.step(now=20.2, yaw=0.15, progress=0.58)
        self.assertEqual(self.state.phase, "backing")
        self.assertEqual(command.v_mps, 0.0)
        self.assertAlmostEqual(self.state.hold_yaw_rad, 0.15)
        self.assertAlmostEqual(self.state.started_s, 20.2)
        self.assertAlmostEqual(self.state.start_progress_m, 0.58)
        command = self.step(heading=-0.3, now=20.3, yaw=0.2, progress=0.57)
        self.assertLess(command.v_mps, 0.0)
        self.assertEqual(command.reason, "heading_backup")
        self.assertLess(command.omega_radps, 0.0)
        self.assertAlmostEqual(self.state.hold_yaw_rad, 0.15)
        command = self.step(heading=None, now=20.4, yaw=0.15, progress=0.56)
        self.assertLess(command.v_mps, 0.0)
        self.assertEqual(command.omega_radps, 0.0)

    def test_repeated_or_older_frame_does_not_complete_alignment(self):
        self.step(now=1.0)
        self.step(now=1.1, captured=1.0)
        self.step(now=1.2, captured=0.9)
        self.assertEqual(self.state.clear_frames, 1)
        self.assertIsNone(self.state.hold_yaw_rad)
        self.step(now=1.3)
        self.assertEqual(self.state.phase, "align")
        self.step(now=1.4)
        self.assertEqual(self.state.phase, "backing")

    def test_missing_or_unaligned_measurement_restarts_stable_count(self):
        for heading in (None, float("nan"), float("inf"), 0.2):
            with self.subTest(heading=heading):
                self.state = Backup()
                self.step(now=1.0)
                self.step(now=1.1)
                self.step(heading=heading, now=1.2)
                self.assertEqual(self.state.clear_frames, 0)
                self.assertIsNone(self.state.hold_yaw_rad)
                self.step(now=1.3)
                self.step(now=1.4)
                self.assertEqual(self.state.phase, "align")
                self.step(now=1.5, yaw=0.1)
                self.assertEqual(self.state.phase, "backing")
                self.assertAlmostEqual(self.state.hold_yaw_rad, 0.1)

    def test_existing_backup_completion_and_fault_results_are_preserved(self):
        for progress, valid, reason in (
            (0.0, True, "backup_done_at_entry"),
            (0.6, False, "stop_backup_odom_stale"),
        ):
            with self.subTest(reason=reason):
                self.state = Backup()
                command = self.step(progress=progress, odom_valid=valid)
                self.assertEqual(command.reason, reason)
                self.assertEqual(command.v_mps, 0.0)

    def test_existing_odom_fault_still_applies_during_alignment(self):
        self.step(heading=0.2)
        command = self.step(heading=0.2, now=1.1, odom_valid=False)
        self.assertEqual(self.state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_odom_stale")
        self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
