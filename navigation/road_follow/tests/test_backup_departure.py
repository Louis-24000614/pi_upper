"""离线驱动真实导航入口：倒车到点不能直接恢复正向循迹。"""

from __future__ import annotations

import sys
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from road_follow import __main__ as entry
from road_follow.backup import Backup, EdgeProgress
from road_follow.control import VelocityCommand
from road_follow.entrance import EntranceDeparture
from road_follow.junction_turn import JunctionTurn

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from agent.route_agent import FollowEdge, MissionComplete, RouteAgent, Stop
from navigation.topo_proto.graph import Edge, Node, TopologyGraph


class BackupDepartureRuntimeTest(unittest.TestCase):
    def run_frames(
        self, frames, *, target="4_3", forward_arrival=False,
        reply=None, stop_ok=True, at_entry=False,
    ):
        """只替换相机/NPU/UART/时间；路线交接和有限动作状态机使用真实实现。"""
        node_xy = {
            "3_4": (2.80, 2.00), "4_4": (2.80, 1.20),
            "4_3": (2.00, 1.20), "4_5": (3.60, 1.20),
        }
        graph = TopologyGraph(
            meta={},
            nodes={key: Node(key, *xy, role="junction") for key, xy in node_xy.items()},
            edges={
                "3_4__4_4": Edge("3_4__4_4", "3_4", "4_4", 0.80),
                "4_3__4_4": Edge("4_3__4_4", "4_3", "4_4", 0.80),
                "4_4__4_5": Edge("4_4__4_5", "4_4", "4_5", 0.80),
            },
        )
        agent = RouteAgent(graph, home=target)
        nodes = ("3_4", "4_4", target)
        edges = ("3_4__4_4", next(
            edge.id for edge in graph.edges.values()
            if {edge.u, edge.v} == {"4_4", target}
        ))
        if forward_arrival:
            nodes, edges = ("4_4",) + nodes, ("3_4__4_4",) + edges
        if reply is not None:
            nodes, edges = nodes[:2], edges[:1]
        agent.state.current_node = nodes[0]
        agent.state.route_nodes, agent.state.route_edges = nodes, edges
        agent.state.from_node, agent.state.to_node = nodes[:2]
        agent.state.current_edge, agent.state.phase = edges[0], "following"
        first = FollowEdge(edges[0], nodes[0], nodes[1])
        agent.node_reached = Mock(wraps=agent.node_reached)
        if reply is not None:
            agent.node_reached.side_effect = lambda node_id: reply
        junction = JunctionTurn(phase="arrived" if forward_arrival else "backup")
        backup = Backup() if forward_arrival or at_entry else Backup(
            phase="backing", started_s=0.0, start_progress_m=0.98,
        )
        progress = EdgeProgress()
        capture, stream, smoother = Mock(), Mock(), Mock()
        uart_thread = Mock()
        writes, snapshots = [], []
        frame_index, now_s, visual_reason = -1, 0.0, "follow"
        image = np.zeros((8, 8, 3), np.uint8)
        mask = np.zeros((8, 8), np.uint8)

        def snapshot():
            snapshots.append(SimpleNamespace(
                phase=junction.phase, departure=junction.departure,
                clear=junction.clear, progress=progress.s_m,
                node=agent.state.current_node, route_index=agent.state.route_index,
                edge=(agent.state.from_node, agent.state.to_node),
            ))

        def read(*args):
            nonlocal frame_index, now_s, visual_reason
            if frame_index >= 0:
                snapshot()
            frame_index += 1
            now_s, progress.s_m, note, visual_reason = frames[frame_index]
            if note:
                uart_thread.call_args.kwargs["args"][1].put(note)
            return SimpleNamespace(
                image=image, mask=mask, inference_s=0.001, captured_s=now_s,
            )

        def send(bridge, line):
            writes.append((frame_index, line))
            return stop_ok or line != "stop"

        def visual(*args, **kwargs):
            velocity = 0.10 if visual_reason == "follow" else 0.0
            return VelocityCommand(velocity, 0.0, visual_reason), SimpleNamespace(
                near_x_m=0.0, bev_road_pixels=1000, output_points=20,
                y_min_m=0.20, y_max_m=1.0,
            )

        stream.read.side_effect = read
        junction_read = SimpleNamespace(
            forward=True, left=False, right=False, kind=entry.KIND_UNKNOWN,
            junction_y_m=None, corridor_end_y_m=None, lane_x_m=0.0,
            lane_width_m=0.20, forward_band_ratio=1.0,
        )
        replacements = {
            "_load_config": Mock(return_value={}),
            "needs_frequency_guard": Mock(return_value=False),
            "OrderedSegmentStream": Mock(return_value=stream),
            "_open_camera": Mock(return_value=capture),
            "_start_route": Mock(return_value=(agent, node_xy, first)),
            "JunctionTurn": Mock(return_value=junction),
            "EntranceDeparture": Mock(return_value=EntranceDeparture(phase="done")),
            "Backup": Mock(side_effect=[backup, Backup()]),
            "EdgeProgress": Mock(return_value=progress),
            "temporal_from_mapping": Mock(return_value=smoother),
            "command_from_mask_with_diagnostics": Mock(side_effect=visual),
            "_junction_read": Mock(return_value=(entry.KIND_UNKNOWN, junction_read)),
            "write_velocity": Mock(side_effect=send),
            "_stop_bridge": Mock(),
            "_event": Mock(),
        }
        with ExitStack() as stack:
            for name, replacement in replacements.items():
                stack.enter_context(patch.object(entry, name, replacement))
            stack.enter_context(patch.object(entry.subprocess, "Popen", return_value=Mock()))
            stack.enter_context(patch.object(entry.threading, "Thread", uart_thread))
            stack.enter_context(patch.object(entry.signal, "signal"))
            stack.enter_context(patch.object(entry.time, "monotonic", side_effect=lambda: now_s))
            self.assertEqual(entry.main([
                "--drive", "--turn-at-junction", "left", "--model", "fake.rknn",
                "--uart-bin", str(ROOT / "navigation/road_follow/__main__.py"),
                "--no-fps-opt", "--frames", str(len(frames)),
            ]), 0)
        snapshot()
        capture.release.assert_called_once()
        stream.close.assert_called_once()
        return agent, writes, snapshots, stream, smoother

    def test_logged_reverse_route_stops_turns_left_and_reacquires_new_frames(self):
        frames = [
            (0.0, 0.98, None, "follow"),
            (0.1, 0.98, "STOP_DONE", "follow"),
            (2.1, 0.98, None, "follow"),
            (2.2, 0.50, None, "follow"),
            (2.3, 0.04, None, "follow"),
            (2.4, 0.0, "STOP_DONE", "follow"),
            (4.3, 0.0, None, "follow"),
            (4.4, 0.0, None, "follow"),
            (4.5, 0.0, "TURN_DONE", "follow"),
            (4.6, 0.0, None, "follow"),
            (4.7, 0.0, None, "stop_road"),
            (4.8, 0.0, None, "follow"),
            (4.9, 0.0, None, "follow"),
            (5.0, 0.0, None, "follow"),
        ]
        agent, writes, states, stream, smoother = self.run_frames(
            frames, forward_arrival=True,
        )
        self.assertEqual(agent.node_reached.call_args_list, [(("3_4",),), (("4_4",),)])
        self.assertEqual([state.phase for state in states], [
            "stopping", "stopped", "backup", "backup", "stopping", "stopped",
            "stopped", "turning", "reacquire", "reacquire", "reacquire",
            "reacquire", "reacquire", "follow",
        ])
        self.assertEqual(states[4].departure, "left")
        self.assertEqual(states[4].edge, ("4_4", "4_3"))
        self.assertEqual(states[4].node, "4_4")
        self.assertEqual(states[4].route_index, 2)
        self.assertEqual(states[4].progress, 0.0)
        self.assertEqual(states[8].clear, 0)
        self.assertEqual(states[10].clear, 0)
        self.assertEqual([row for row in writes if row[1] in ("stop", "turn left")],
                         [(0, "stop"), (4, "stop"), (7, "turn left")])
        self.assertEqual([frame for frame, line in writes if line == "0.100 0.000"], [13])
        stream.invalidate.assert_called_once()
        self.assertGreaterEqual(smoother.reset.call_count, 1)

    def test_reverse_arrival_turns_right_in_mirrored_route(self):
        agent, writes, states, _, _ = self.run_frames([
            (1.0, 0.04, None, "follow"),
            (1.1, 0.0, "STOP_DONE", "follow"),
            (3.1, 0.0, None, "follow"),
        ], target="4_5")
        self.assertEqual(states[0].departure, "right")
        self.assertIn((2, "turn right"), writes)
        agent.node_reached.assert_called_once_with("4_4")

    def test_forward_to_backup_start_waits_for_stop_and_then_goes_straight(self):
        _, writes, states, _, _ = self.run_frames([
            (1.0, 0.04, None, "follow"),
            (1.1, 0.0, "STOP_DONE", "follow"),
            (3.0, 0.0, None, "follow"),
            (3.1, 0.0, None, "follow"),
        ], target="3_4")
        self.assertEqual(states[0].departure, "straight")
        self.assertEqual([state.phase for state in states],
                         ["stopping", "stopped", "stopped", "follow"])
        self.assertEqual([frame for frame, line in writes if line == "0.100 0.000"], [3])
        self.assertFalse(any(line.startswith("turn ") for _, line in writes))

    def test_complete_or_rejected_route_never_resumes_forward(self):
        for reply, phase in (
            (MissionComplete("4_4"), "done"),
            (Stop("到达节点不匹配"), "fault"),
            (FollowEdge("invalid", "3_4", "4_3"), "fault"),
        ):
            with self.subTest(reply=reply):
                agent, writes, states, _, _ = self.run_frames([
                    (1.0, 0.04, None, "follow"), (1.1, 0.0, None, "follow"),
                ], reply=reply)
                self.assertEqual([state.phase for state in states], [phase, phase])
                self.assertTrue(all(line == "0.000 0.000" for _, line in writes))
                agent.node_reached.assert_called_once_with("4_4")

    def test_failed_stop_send_latches_fault(self):
        _, writes, states, _, _ = self.run_frames([
            (1.0, 0.04, None, "follow"), (1.1, 0.0, None, "follow"),
        ], stop_ok=False)
        self.assertEqual([state.phase for state in states], ["fault", "fault"])
        self.assertEqual(writes[0], (0, "stop"))
        self.assertTrue(all(line in ("stop", "0.000 0.000") for _, line in writes))

    def test_failed_stop_or_turn_confirmation_latches_fault(self):
        for frames in (
            [(1.0, 0.04, None, "follow"), (1.1, 0.0, "STOP_FAIL", "follow")],
            [(1.0, 0.04, None, "follow"), (1.1, 0.0, "STOP_DONE", "follow"),
             (3.1, 0.0, None, "follow"), (3.2, 0.0, "TURN_FAIL", "follow")],
        ):
            with self.subTest(frames=frames):
                _, writes, states, _, _ = self.run_frames(frames)
                self.assertEqual(states[-1].phase, "fault")
                self.assertFalse(any(line == "0.100 0.000" for _, line in writes))

    def test_backup_triggered_at_entry_does_not_report_false_arrival(self):
        agent, writes, states, _, _ = self.run_frames([
            (1.0, 0.04, None, "follow"), (1.1, 0.04, None, "follow"),
        ], at_entry=True)
        self.assertEqual([state.phase for state in states], ["fault", "fault"])
        agent.node_reached.assert_not_called()
        self.assertTrue(all(line == "0.000 0.000" for _, line in writes))


if __name__ == "__main__":
    unittest.main()
