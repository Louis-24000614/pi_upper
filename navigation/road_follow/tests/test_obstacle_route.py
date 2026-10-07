"""拓扑循迹中的硬堵塞：封边、沿边进度倒车，再按车头转向。"""

from __future__ import annotations

import queue
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

from road_follow.backup import Backup, BackupConfig, EdgeProgress
from road_follow.control import VelocityCommand
from road_follow.obstacle_route import (
    ObstacleRecovery,
    detection_armed,
    recovery_maneuver,
    step_obstacle_route,
)


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


class StartBackup:
    def __init__(self, edge_id: str, from_node: str, progress_m: float) -> None:
        self.edge_id = edge_id
        self.from_node = from_node
        self.progress_m = progress_m


class Stop:
    def __init__(self, reason: str) -> None:
        self.reason = reason


class FakeAgent:
    """只记录封边和倒车完成，下一条边由测试指定。"""

    def __init__(self, depart_to: str) -> None:
        self.depart_to = depart_to
        self.state = SimpleNamespace(
            phase="following",
            current_edge="3_2__3_3",
            from_node="3_2",
            to_node="3_3",
            blocked_edges=set(),
            covered_edges=set(),
        )
        self.failed = None

    def obstacle_confirmed(self, edge_id: str, progress_m: float):
        self.state.blocked_edges.add(edge_id)
        self.state.covered_edges.discard(edge_id)
        self.state.phase = "backing"
        return StartBackup(edge_id, self.state.from_node, progress_m)

    def backup_done(self, edge_id: str, node_id: str):
        self.state.current_node = node_id
        self.state.current_edge = f"{node_id}__{self.depart_to}"
        self.state.from_node = node_id
        self.state.to_node = self.depart_to
        self.state.phase = "following"
        return FollowEdge(self.state.current_edge, node_id, self.depart_to)

    def backup_failed(self, edge_id: str, reason: str):
        self.failed = (edge_id, reason)
        self.state.phase = "fault"
        return Stop(reason)


class ObstacleRouteTest(unittest.TestCase):
    def setUp(self) -> None:
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.0, "follow")
        self.backup_cfg = BackupConfig(max_missing_frames=2, max_duration_s=5.0)

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

    def step(self, recovery, backup, agent, progress, **kwargs):
        return step_obstacle_route(
            recovery,
            backup,
            agent=agent,
            node_xy=NODES,
            progress_s_m=progress.s_m,
            near_x_m=kwargs.pop("near_x_m", 0.0),
            command=kwargs.pop("command", self.follow),
            now_s=kwargs.pop("now_s", 1.0),
            notes=self.notes,
            send=self.send,
            backup_cfg=self.backup_cfg,
            stop_settle_s=kwargs.pop("stop_settle_s", 0.0),
            reverse_hold_s=kwargs.pop("reverse_hold_s", 0.0),
            reacquire_frames=kwargs.pop("reacquire_frames", 2),
            **kwargs,
        )

    def test_visual_backup_still_sends_velocity(self) -> None:
        from road_follow.__main__ import _finite_action_active

        entrance = SimpleNamespace(phase="done")
        junction = SimpleNamespace(phase="follow")
        rfid_turn = SimpleNamespace(phase="idle")
        patrol = SimpleNamespace(phase="follow")
        self.assertFalse(
            _finite_action_active(
                entrance, junction, rfid_turn, patrol, Backup(phase="backing"),
                ObstacleRecovery(phase="backing"),
            )
        )
        for phase in ("heading_hold",):
            self.assertFalse(
                _finite_action_active(
                    entrance, SimpleNamespace(phase=phase), rfid_turn,
                    SimpleNamespace(phase=phase), Backup(),
                    ObstacleRecovery(),
                ),
                phase,
            )
        for phase in ("holding", "stopping", "stopped", "turning"):
            self.assertTrue(
                _finite_action_active(
                    entrance, junction, rfid_turn, patrol, Backup(),
                    ObstacleRecovery(phase=phase),
                ),
                phase,
            )

    def test_detection_is_armed_only_while_following_the_edge(self) -> None:
        self.assertTrue(
            detection_armed(
                entrance_done=True,
                agent_phase="following",
                junction_phase="follow",
                recovery_phase="idle",
            )
        )
        for phase in ("approach", "align", "forward", "turning", "backup", "stopping"):
            self.assertFalse(
                detection_armed(
                    entrance_done=True,
                    agent_phase="following",
                    junction_phase=phase,
                    recovery_phase="idle",
                ),
                phase,
            )
        self.assertFalse(
            detection_armed(
                entrance_done=False,
                agent_phase="following",
                junction_phase="follow",
                recovery_phase="idle",
            )
        )
        self.assertFalse(
            detection_armed(
                entrance_done=True,
                agent_phase="backing",
                junction_phase="follow",
                recovery_phase="idle",
            )
        )
        self.assertFalse(
            detection_armed(
                entrance_done=True,
                agent_phase="following",
                junction_phase="follow",
                recovery_phase="backing",
            )
        )

    def test_unarmed_confirmation_does_not_block_the_edge(self) -> None:
        agent = FakeAgent("2_2")
        progress = self._progress(0.40)
        recovery, _backup, outcome = self.step(
            ObstacleRecovery(),
            Backup(),
            agent,
            progress,
            armed=False,
            confirmed=True,
        )
        self.assertEqual(recovery.phase, "idle")
        self.assertEqual(outcome.command.reason, "follow")
        self.assertEqual(outcome.command.v_mps, 0.10)
        self.assertFalse(agent.state.blocked_edges)
        self.assertFalse(outcome.reset_progress)

    def test_recovery_turn_is_left_right_or_two_quarters(self) -> None:
        self.assertEqual(recovery_maneuver(NODES, "3_2", "3_3", "2_2").side, "left")
        self.assertEqual(recovery_maneuver(NODES, "3_2", "3_3", "2_2").quarters, 1)
        self.assertEqual(recovery_maneuver(NODES, "3_2", "3_3", "4_2").side, "right")
        self.assertEqual(recovery_maneuver(NODES, "3_2", "3_3", "4_2").quarters, 1)
        around = recovery_maneuver(NODES, "3_2", "3_3", "3_1")
        self.assertEqual((around.side, around.quarters), ("left", 2))
        with self.assertRaises(ValueError):
            recovery_maneuver(NODES, "3_2", "3_3", "3_3")

    def test_confirm_holds_one_second_before_reverse(self) -> None:
        agent = FakeAgent("2_2")
        progress = self._progress(0.40)
        recovery, backup, outcome = self.step(
            ObstacleRecovery(), Backup(), agent, progress,
            armed=True, confirmed=True, now_s=10.0, reverse_hold_s=1.0,
        )
        self.assertEqual(recovery.phase, "holding")
        self.assertEqual(outcome.command.reason, "backup_hold")
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertAlmostEqual(progress.s_m, 0.40)
        self.assertEqual(backup.phase, "idle")
        self.assertEqual(self.sent, ["stop"])

        self.notes.put("STOP_DONE")
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=10.0, reverse_hold_s=1.0,
        )
        self.assertEqual(recovery.phase, "holding")
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertEqual(backup.phase, "idle")
        self.assertAlmostEqual(progress.s_m, 0.40)

        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=10.9, reverse_hold_s=1.0,
        )
        self.assertEqual(recovery.phase, "holding")
        self.assertGreaterEqual(outcome.command.v_mps, 0.0)
        self.assertNotEqual(outcome.command.reason, "visual_backup")

        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=11.0, reverse_hold_s=1.0,
        )
        self.assertEqual(recovery.phase, "backing")
        self.assertEqual(outcome.command.reason, "stop_backup")
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertAlmostEqual(progress.s_m, 0.40)

    def test_confirm_backs_up_on_s_then_turns_without_covering_the_edge(self) -> None:
        agent = FakeAgent("2_2")
        progress = self._progress(0.40)
        recovery, backup, outcome = self.step(
            ObstacleRecovery(), Backup(), agent, progress, armed=True, confirmed=True
        )
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertEqual(outcome.command.reason, "backup_hold")
        self.assertAlmostEqual(progress.s_m, 0.40)
        self.assertFalse(outcome.reset_progress)
        self.assertIn("3_2__3_3", agent.state.blocked_edges)
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)
        self.assertEqual(self.sent, ["stop"])

        self.notes.put("STOP_DONE")
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=1.0
        )
        self.assertEqual(outcome.command.reason, "stop_backup")
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertAlmostEqual(progress.s_m, 0.40)

        progress.update(0.20, 0.0, 0.0)
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, confirmed=False, now_s=1.2
        )
        self.assertLess(outcome.command.v_mps, 0.0)
        self.assertEqual(outcome.command.reason, "visual_backup")
        self.assertAlmostEqual(progress.s_m, 0.20)
        self.assertFalse(outcome.reset_progress)
        self.assertNotIn("turn", " ".join(self.sent))

        progress.update(0.0, 0.0, 0.0)
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, confirmed=False, now_s=1.4
        )
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertNotEqual(outcome.command.reason, "backup_done_at_entry")
        self.assertAlmostEqual(progress.s_m, 0.0)
        self.assertFalse(outcome.reset_progress)
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)
        self.assertEqual(backup.phase, "idle")
        self.assertEqual(self.sent, ["stop", "stop"])

        self.notes.put("STOP_DONE")
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=1.5
        )
        self.assertEqual(self.sent, ["stop", "stop", "turn left"])
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertNotEqual(outcome.command.reason, "visual_backup")

        self.notes.put("TURN_DONE")
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=2.0
        )
        self.assertEqual(self.sent, ["stop", "stop", "turn left"])
        self.assertEqual(recovery.phase, "reacquire")
        self.assertTrue(outcome.reset_smoother)
        self.assertFalse(outcome.reset_progress)

        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=2.1
        )
        self.assertFalse(outcome.reset_progress)
        recovery, backup, outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=2.2
        )
        self.assertTrue(outcome.reset_progress)
        self.assertEqual(recovery.phase, "idle")
        self.assertEqual(outcome.command.reason, "follow")
        progress.reset()
        self.assertEqual(progress.s_m, 0.0)
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)

    def test_right_and_behind_use_turn_actions_not_visual_backup(self) -> None:
        right = self._finish_backup(FakeAgent("4_2"), 0.40)
        self.assertEqual(right, ["stop", "stop", "turn right"])

        behind = self._finish_backup(FakeAgent("3_1"), 0.40)
        self.assertEqual(behind, ["stop", "stop", "turn left", "turn left"])
        self.assertNotIn("visual_backup", behind)

    def test_confirm_at_entry_blocks_and_turns_without_entry_fault(self) -> None:
        agent = FakeAgent("4_2")
        progress = self._progress(0.0)
        recovery, backup, outcome = self.step(
            ObstacleRecovery(), Backup(), agent, progress, armed=True, confirmed=True
        )
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertNotEqual(outcome.command.reason, "backup_done_at_entry")
        self.assertEqual(recovery.phase, "stopping")
        self.assertEqual(backup.phase, "idle")
        self.assertIn("3_2__3_3", agent.state.blocked_edges)
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)
        self.assertEqual(self.sent, ["stop"])
        self.assertFalse(outcome.reset_progress)

    def test_backup_fault_does_not_cover_or_replan(self) -> None:
        agent = FakeAgent("2_2")
        progress = self._progress(0.50)
        recovery, backup, _outcome = self.step(
            ObstacleRecovery(), Backup(), agent, progress, armed=True, confirmed=True
        )
        self.notes.put("STOP_DONE")
        recovery, backup, _outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=1.0
        )
        for frame in range(3):
            recovery, backup, outcome = self.step(
                recovery,
                backup,
                agent,
                progress,
                armed=False,
                near_x_m=None,
                now_s=1.1 + frame * 0.1,
            )
        self.assertEqual(recovery.phase, "fault")
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertEqual(agent.failed, ("3_2__3_3", "stop_backup_road_lost"))
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)
        self.assertEqual(self.sent, ["stop"])

    def test_departing_back_into_the_blocked_corridor_stops(self) -> None:
        agent = FakeAgent("3_3")
        progress = self._progress(0.0)
        recovery, _backup, outcome = self.step(
            ObstacleRecovery(), Backup(), agent, progress, armed=True, confirmed=True
        )
        self.assertEqual(recovery.phase, "fault")
        self.assertEqual(outcome.command.v_mps, 0.0)
        self.assertNotIn("turn", " ".join(self.sent))
        self.assertIn("3_2__3_3", agent.state.blocked_edges)
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)

    def test_live_agent_blocks_the_edge_and_turns_away_from_it(self) -> None:
        root = Path(__file__).resolve().parents[3]
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from agent.route_agent import RouteAgent
        from topo_proto.graph import load_topology

        graph = load_topology()
        agent = RouteAgent(graph)
        first = agent.start()
        self.assertEqual(first.to_node, "0_J")
        edge = agent.node_reached("0_J")
        self.assertEqual((edge.from_node, edge.to_node), ("0_J", "1_2"))
        node_xy = {node_id: (node.x, node.y) for node_id, node in graph.nodes.items()}
        progress = self._progress(0.25)
        recovery = ObstacleRecovery()
        backup = Backup()
        recovery, backup, outcome = step_obstacle_route(
            recovery,
            backup,
            armed=True,
            confirmed=True,
            agent=agent,
            node_xy=node_xy,
            progress_s_m=progress.s_m,
            near_x_m=0.0,
            command=self.follow,
            now_s=1.0,
            notes=self.notes,
            send=self.send,
            backup_cfg=self.backup_cfg,
            stop_settle_s=0.0,
            reverse_hold_s=0.0,
            reacquire_frames=2,
        )
        self.assertAlmostEqual(progress.s_m, 0.25)
        self.assertIn(edge.edge_id, agent.state.blocked_edges)
        self.assertNotIn(edge.edge_id, agent.state.covered_edges)
        self.assertEqual(outcome.command.reason, "backup_hold")

        self.notes.put("STOP_DONE")
        recovery, backup, outcome = step_obstacle_route(
            recovery,
            backup,
            armed=False,
            confirmed=False,
            agent=agent,
            node_xy=node_xy,
            progress_s_m=progress.s_m,
            near_x_m=0.0,
            command=self.follow,
            now_s=1.05,
            notes=self.notes,
            send=self.send,
            backup_cfg=self.backup_cfg,
            stop_settle_s=0.0,
            reverse_hold_s=0.0,
            reacquire_frames=2,
        )
        progress.update(0.0, 0.0, 0.0)
        recovery, backup, outcome = step_obstacle_route(
            recovery,
            backup,
            armed=False,
            confirmed=False,
            agent=agent,
            node_xy=node_xy,
            progress_s_m=progress.s_m,
            near_x_m=0.0,
            command=self.follow,
            now_s=1.4,
            notes=self.notes,
            send=self.send,
            backup_cfg=self.backup_cfg,
            stop_settle_s=0.0,
            reverse_hold_s=0.0,
            reacquire_frames=2,
        )
        self.assertEqual(recovery.phase, "stopping")
        self.assertNotIn(edge.edge_id, agent.state.covered_edges)
        self.assertNotEqual(agent.state.to_node, "1_2")
        expected = recovery_maneuver(node_xy, "0_J", "1_2", agent.state.to_node)
        self.notes.put("STOP_DONE")
        now_s = 1.5
        while len([line for line in self.sent if line.startswith("turn ")]) < expected.quarters:
            recovery, backup, _outcome = step_obstacle_route(
                recovery,
                backup,
                armed=False,
                confirmed=False,
                agent=agent,
                node_xy=node_xy,
                progress_s_m=progress.s_m,
                near_x_m=0.0,
                command=self.follow,
                now_s=now_s,
                notes=self.notes,
                send=self.send,
                backup_cfg=self.backup_cfg,
                stop_settle_s=0.0,
                reverse_hold_s=0.0,
                reacquire_frames=2,
            )
            now_s += 0.5
            if recovery.phase == "turning" and len(
                [line for line in self.sent if line.startswith("turn ")]
            ) < expected.quarters:
                self.notes.put("TURN_DONE")
        self.assertEqual(
            self.sent,
            ["stop", "stop", *[f"turn {expected.side}"] * expected.quarters],
        )
        self.assertFalse(any(line.startswith("backward") for line in self.sent))
        self.assertNotIn(edge.edge_id, agent.state.covered_edges)

    def _finish_backup(self, agent: FakeAgent, start_s: float) -> list[str]:
        self.sent.clear()
        progress = self._progress(start_s)
        recovery, backup, _outcome = self.step(
            ObstacleRecovery(), Backup(), agent, progress, armed=True, confirmed=True
        )
        self.notes.put("STOP_DONE")
        recovery, backup, _outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=1.0
        )
        progress.update(0.0, 0.0, 0.0)
        recovery, backup, _outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=1.4
        )
        self.notes.put("STOP_DONE")
        recovery, backup, _outcome = self.step(
            recovery, backup, agent, progress, armed=False, now_s=1.5
        )
        if recovery.phase == "turning" and recovery.quarters_remaining:
            self.notes.put("TURN_DONE")
            recovery, backup, _outcome = self.step(
                recovery, backup, agent, progress, armed=False, now_s=2.0
            )
        self.assertNotEqual(recovery.phase, "backing")
        self.assertNotIn("3_2__3_3", agent.state.covered_edges)
        return list(self.sent)

    def _progress(self, meters: float) -> EdgeProgress:
        progress = EdgeProgress()
        progress.update(0.0, 0.0, 0.0)
        progress.update(meters, 0.0, 0.0)
        return progress
