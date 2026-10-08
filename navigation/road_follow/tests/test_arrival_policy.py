"""按有向地图选择到点策略及两类状态机的保护/交接行为。"""

from __future__ import annotations

import queue
import unittest
from dataclasses import replace

from ipm_proto.junction import KIND_CROSS, KIND_STRAIGHT, KIND_T, JunctionRead
from navigation.topo_proto.graph import ArrivalSettings, load_topology
from road_follow.arrival_policy import resolve_arrival_policy, step_route_arrival
from road_follow.backup import EdgeProgress
from road_follow.control import VelocityCommand
from road_follow.departure import handoff_arrival
from road_follow.junction_turn import JunctionTurn, JunctionTurnConfig
from road_follow.rfid_arrival import RfidArrival, RfidArrivalConfig
from agent.route_agent import RouteAgent


class ArrivalPolicyTest(unittest.TestCase):
    def setUp(self):
        self.graph = load_topology()
        self.cfg = JunctionTurnConfig(branch_observe_min_distance_m=0.18)
        self.patrol_cfg = RfidArrivalConfig()
        self.state = JunctionTurn()
        self.patrol_state = RfidArrival()
        self.notes = queue.Queue()
        self.sent = []
        self.command = VelocityCommand(0.08, 0, "follow")
        self.reading = JunctionRead(KIND_CROSS, True, True, True, 0, 0.20, 0.50, 1.0, 0.85)

    def policy(self, source="1_2", target="2_2", length=0.97):
        return resolve_arrival_policy(self.graph, source, target, length, self.cfg, self.patrol_cfg)

    def send(self, line):
        self.sent.append(line)
        return True

    def step(self, policy, progress, *, patrol=False, odom_valid=True, reading=None, command=None,
             visual_safe=True, near_x_m=None, lane_heading_rad=None, yaw_rad=None):
        self.state, self.patrol_state, cmd, trigger = step_route_arrival(
            policy, patrol=patrol, state=self.state, patrol_state=self.patrol_state,
            reading=reading or self.reading, opening=(reading or self.reading).kind,
            command=command or self.command, progress_m=progress, odom_valid=odom_valid,
            notes=self.notes, send=self.send, junction_cfg=self.cfg,
            patrol_cfg=self.patrol_cfg, visual_safe=visual_safe, near_x_m=near_x_m,
            lane_heading_rad=lane_heading_rad, yaw_rad=yaw_rad,
        )
        return cmd, trigger

    def test_directed_override_and_map_side_rotate_with_incoming_edge(self):
        incoming = self.policy()
        self.assertEqual(incoming.mode, "visual_odom")
        self.assertEqual(incoming.handoff_progress_m, 0.80)
        self.assertAlmostEqual(incoming.final_forward_m, 0.17)
        self.assertEqual(incoming.guard_progress_m, 0.92)
        self.assertEqual(incoming.align_timeout_s, 3.0)
        self.assertEqual(incoming.align_stable_frames, 8)
        self.assertEqual(incoming.expected_openings, ("forward", "left", "right"))
        reverse = self.policy("2_2", "1_2", 0.97)
        self.assertEqual(reverse.mode, "visual_end")
        self.assertAlmostEqual(reverse.final_forward_m, 0.17)
        self.assertAlmostEqual(reverse.guard_progress_m, 0.92)
        self.assertEqual(reverse.align_timeout_s, 3.0)
        self.assertEqual(reverse.expected_openings, ("right",))
        left_corner = self.policy("0_J", "1_2", 0.4)
        self.assertEqual(left_corner.expected_openings, ("left",))
        self.assertAlmostEqual(left_corner.final_forward_m, 0.20)
        self.assertEqual(left_corner.align_timeout_s, 1.5)
        self.assertEqual(left_corner.align_stable_frames, 5)

    def test_through_patrol_uses_edge_length_and_dead_end_still_watches_the_wall(self):
        through = self.policy("2_1", "3_1", 0.97)
        self.assertEqual(through.mode, "visual_odom")
        self.assertEqual(through.expected_openings, ("forward", "left"))
        self.assertAlmostEqual(through.handoff_progress_m, 0.80)
        self.assertAlmostEqual(through.final_forward_m, 0.17)
        self.assertAlmostEqual(through.guard_progress_m, 0.92)
        self.assertEqual(through.align_stable_frames, 8)
        dead_end = self.policy("3_2", "3_1", 1.0)
        self.assertEqual(dead_end.mode, "visual_end")
        self.assertNotIn("forward", dead_end.expected_openings)

        self.graph.arrival_overrides[("2_1", "3_1")] = ArrivalSettings(mode="visual_end")
        self.assertEqual(self.policy("2_1", "3_1", 1.0).mode, "visual_end")

    def test_through_patrol_hands_off_at_edge_length_while_the_band_stays_high(self):
        policy = self.policy("2_1", "3_1", 0.97)
        for _ in range(2):
            self.step(policy, 0.40, patrol=True)
        self.assertTrue(self.patrol_state.edge_latched)
        high = replace(self.reading, forward_band_ratio=0.72)
        self.step(policy, 0.85, patrol=True, reading=high)
        cmd, trigger = self.step(policy, 0.86, patrol=True, reading=high)
        self.assertEqual(trigger, "visual_odom")
        self.assertEqual(self.sent, [])
        self.assertEqual(cmd.reason, "heading_hold")
        self.assertEqual(cmd.omega_radps, 0.0)
        self.assertAlmostEqual(cmd.v_mps, 0.05)
        self.assertEqual(self.patrol_state.phase, "heading_hold")

    def test_through_patrol_guard_stops_when_the_edge_length_is_passed(self):
        policy = self.policy("2_1", "3_1", 0.97)
        for _ in range(2):
            self.step(policy, 0.40, patrol=True)
        cmd, trigger = self.step(policy, 0.95, patrol=True)
        self.assertEqual(trigger, "guard")
        self.assertEqual(self.patrol_state.phase, "odom_wait")
        self.assertEqual(self.sent, [])
        self.assertEqual(cmd.reason, "stop_arrival_guard")
        cmd, _ = self.step(policy, 1.10, patrol=True)
        self.assertEqual(self.patrol_state.phase, "odom_wait")
        self.assertEqual(cmd.reason, "stop_arrival_guard")
        self.assertEqual(self.sent, [])

    def test_near_offset_aligns_before_junction_and_patrol_forward(self):
        policy = self.policy()
        for _ in range(2):
            self.step(policy, 0.50)
        self.step(policy, 0.80)
        cmd, trigger = self.step(policy, 0.81, near_x_m=0.08, lane_heading_rad=0.20)
        self.assertEqual(self.state.phase, "align")
        self.assertEqual(cmd.reason, "align")
        self.assertLess(cmd.omega_radps, 0.0)
        self.assertEqual(cmd.v_mps, 0.0)
        self.assertEqual(self.sent, [])
        self.assertEqual(trigger, "")
        for _ in range(7):
            self.step(policy, 0.81, near_x_m=0.08, lane_heading_rad=0.0)
        cmd, trigger = self.step(policy, 0.81, near_x_m=0.08, lane_heading_rad=0.0)
        self.assertEqual(self.sent, [])
        self.assertEqual(trigger, "odom_handoff")
        self.assertEqual(self.state.phase, "heading_hold")
        self.assertEqual(cmd.reason, "heading_hold")
        self.assertEqual(cmd.omega_radps, 0.0)

        self.state, self.patrol_state, self.sent = JunctionTurn(), RfidArrival(), []
        patrol = self.policy("2_2", "2_1", 0.80)
        for _ in range(2):
            self.step(patrol, 0.70, patrol=True)
        for _ in range(2):
            self.step(
                patrol, 0.77, patrol=True,
                reading=replace(self.reading, forward_band_ratio=0.0),
                near_x_m=0.08, lane_heading_rad=0.20,
            )
        cmd, trigger = self.step(
            patrol, 0.77, patrol=True,
            reading=replace(self.reading, forward_band_ratio=0.0),
            near_x_m=0.08, lane_heading_rad=0.20,
        )
        self.assertEqual(self.patrol_state.phase, "align")
        self.assertEqual(cmd.reason, "align")
        self.assertEqual(self.sent, [])
        self.assertEqual(trigger, "")
        cmd, trigger = self.step(
            patrol, 0.77, patrol=True,
            reading=replace(self.reading, forward_band_ratio=0.0),
            lane_heading_rad=None,
        )
        self.assertEqual(self.sent, [])
        self.assertEqual(self.patrol_state.phase, "align")
        self.assertEqual((cmd.v_mps, cmd.omega_radps), (0.0, 0.0))
        self.assertEqual(trigger, "")
        for _ in range(4):
            self.step(
                patrol, 0.77, patrol=True,
                reading=replace(self.reading, forward_band_ratio=0.0),
                lane_heading_rad=0.0,
            )
        cmd, _ = self.step(
            patrol, 0.77, patrol=True,
            reading=replace(self.reading, forward_band_ratio=0.0),
            lane_heading_rad=0.0,
        )
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.assertEqual(cmd.reason, "heading_hold")

    def test_high_band_cross_hands_off_once_and_consumes_done_without_odom(self):
        policy = self.policy()
        for _ in range(2):
            self.step(policy, 0.50)
        self.assertTrue(self.state.branch_latched)
        self.step(policy, 0.79)
        self.assertEqual(self.sent, [])
        self.step(policy, 0.80)
        cmd, trigger = self.step(policy, 0.81)
        self.assertEqual(self.sent, [])
        self.assertEqual(trigger, "odom_handoff")
        self.assertEqual(self.state.phase, "heading_hold")
        self.assertEqual(self.state.hold_start_m, 0.81)
        cmd, _ = self.step(policy, 0.90, command=VelocityCommand(0.08, 0.4, "follow"))
        self.assertEqual(self.state.phase, "heading_hold")
        self.assertEqual(cmd.omega_radps, 0.0)
        self.assertEqual(self.state.hold_start_m, 0.81)
        cmd, _ = self.step(policy, 0.90, odom_valid=False)
        self.assertEqual(self.state.phase, "heading_hold")
        self.assertEqual(cmd.reason, "stop_odom_stale")
        self.assertEqual(self.state.hold_start_m, 0.81)
        self.step(policy, 1.01)
        self.assertEqual(self.state.phase, "arrived")

    def test_no_branch_or_wrong_map_side_cannot_fake_arrival(self):
        policy = self.policy("3_2", "3_1", 0.80)
        # 向西驶入3_1只有右侧向北和左侧向南，两侧都关闭时不能锁存。
        no_side = replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT)
        for _ in range(4):
            self.step(policy, 0.62, patrol=True, reading=no_side)
        self.assertEqual(self.sent, [])
        cmd, _ = self.step(policy, 0.75, patrol=True, reading=no_side)
        self.assertEqual(cmd, self.command)
        self.assertEqual(self.state.phase, "follow")
        self.assertEqual(self.patrol_state.phase, "follow")
        for _ in range(3):
            self.step(policy, 0.90, patrol=True,
                      reading=replace(no_side, forward_band_ratio=0))
        self.assertFalse(self.patrol_state.edge_latched)
        self.assertEqual(self.sent, [])

        self.state, self.patrol_state = JunctionTurn(), RfidArrival()
        policy = self.policy("0_J", "1_2", 0.40)
        wrong = replace(self.reading, left=False, right=True, kind=KIND_T, forward_band_ratio=0.0)
        for _ in range(5):
            self.step(policy, 0.20, patrol=True, reading=wrong)
        self.assertFalse(self.patrol_state.edge_latched)
        self.assertEqual(self.sent, [])

    def test_patrol_late_visual_end_can_handoff_after_old_guard_position(self):
        policy = self.policy("2_2", "2_1", 0.80)
        for _ in range(2):
            self.step(policy, 0.70, patrol=True)
        self.assertTrue(self.patrol_state.edge_latched)
        cmd, trigger = self.step(
            policy, 0.75, patrol=True,
            reading=replace(self.reading, forward_band_ratio=0.38),
        )
        self.assertEqual(cmd, self.command)
        self.assertEqual(trigger, "")
        self.assertEqual(self.state.phase, "follow")
        self.assertEqual(self.patrol_state.phase, "follow")
        self.assertEqual(self.sent, [])
        for _ in range(2):
            self.step(policy, 0.77, patrol=True,
                      reading=replace(self.reading, forward_band_ratio=0.07))
        self.assertEqual(self.sent, [])
        _, trigger = self.step(policy, 0.77, patrol=True,
                               reading=replace(self.reading, forward_band_ratio=0))
        self.assertEqual(trigger, "visual_end")
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.assertEqual(self.patrol_state.hold_start_m, 0.77)
        self.assertEqual(self.sent, [])
        self.step(policy, 0.90, patrol=True)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.step(policy, 0.97, patrol=True)
        self.assertEqual(self.patrol_state.phase, "arrived")
        self.assertEqual(self.sent, [])

    def test_junction_guard_is_latched_and_does_not_advance_agent(self):
        agent = RouteAgent(self.graph)
        agent.start()
        agent.node_reached("0_J")
        agent.node_reached("1_2")
        policy = self.policy()
        for _ in range(2):
            self.step(policy, 0.50)
        self.assertTrue(self.state.branch_latched)
        cmd, trigger = self.step(policy, 0.95)
        self.assertEqual((cmd.v_mps, cmd.omega_radps), (0, 0))
        self.assertEqual(trigger, "guard")
        self.step(policy, 0.96, reading=replace(self.reading, forward_band_ratio=0))
        self.assertEqual(self.state.phase, "odom_wait")
        xy = {key: (node.x, node.y) for key, node in self.graph.nodes.items()}
        handoff_arrival(self.state, agent, xy, self.send)
        self.assertEqual(agent.state.to_node, "2_2")
        self.assertEqual(agent.state.covered_edges, {"0_0__0_J", "0_J__1_2"})
        self.assertEqual(self.sent, [])

    def test_patrol_stale_odom_still_stops_beyond_old_guard(self):
        policy = self.policy("2_2", "2_1", 0.80)
        for _ in range(2):
            self.step(policy, 0.70, patrol=True)
        self.patrol_state.road_end_missing_frames = 2
        cmd, trigger = self.step(policy, 0.77, patrol=True, odom_valid=False)
        self.assertEqual((cmd.v_mps, cmd.omega_radps), (0, 0))
        self.assertEqual(trigger, "odom_stale")
        self.assertEqual(self.patrol_state.road_end_missing_frames, 0)
        self.assertEqual(self.sent, [])

    def test_patrol_visual_dropout_returns_to_follow_when_the_road_returns(self):
        policy = self.policy("2_2", "2_1", 0.80)
        stopped = VelocityCommand(0.0, 0.0, "stop_road")
        for _ in range(2):
            self.step(policy, 0.70, patrol=True)
        for _ in range(3):
            cmd, _ = self.step(
                policy, 0.77, patrol=True,
                reading=replace(self.reading, forward_band_ratio=0),
                visual_safe=False, command=stopped,
            )
        self.assertEqual(cmd, stopped)
        self.assertEqual(self.patrol_state.phase, "follow")
        self.assertTrue(self.patrol_state.edge_latched)
        self.assertEqual(self.patrol_state.road_end_missing_frames, 0)
        self.assertEqual(self.sent, [])
        cmd, _ = self.step(
            policy, 0.77, patrol=True,
            reading=replace(self.reading, forward_band_ratio=0.70),
            visual_safe=True,
        )
        self.assertEqual(self.patrol_state.phase, "follow")
        self.assertEqual(self.patrol_state.road_end_missing_frames, 0)
        self.assertEqual(cmd.reason, "follow")

    def test_patrol_visual_end_remains_available(self):
        policy = self.policy("0_J", "1_2", 0.40)
        for _ in range(2):
            self.step(policy, 0.15, patrol=True)
        for _ in range(3):
            self.step(policy, 0.20, patrol=True, reading=replace(self.reading, forward_band_ratio=0.05))
        self.assertEqual(self.sent, [])
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.assertEqual(self.patrol_state.hold_start_m, 0.20)
        self.step(policy, 0.40, patrol=True)
        self.assertEqual(self.patrol_state.phase, "arrived")

    def test_ordinary_visual_end_requires_band_and_uses_override_distance(self):
        self.graph.arrival_overrides[("1_3", "0_J")] = ArrivalSettings(
            mode="visual_end", final_forward_m=0.15,
        )
        policy = self.policy("1_3", "0_J", 0.40)
        for _ in range(2):
            self.step(policy, 0.10)
        self.assertTrue(self.state.branch_latched)
        self.assertEqual(self.sent, [])
        end = replace(self.reading, forward_band_ratio=0.05, corridor_end_y_m=0.35)
        for _ in range(3):
            self.step(policy, 0.20, reading=end)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.state.phase, "heading_hold")
        self.assertAlmostEqual(self.state.hold_start_m, 0.20)

    def test_special_patrol_can_use_odom_with_high_band_and_custom_distance(self):
        self.graph.arrival_overrides[("0_J", "1_2")] = ArrivalSettings(
            mode="visual_odom", handoff_progress_m=0.25, final_forward_m=0.10,
            guard_progress_m=0.35,
        )
        policy = self.policy("0_J", "1_2", 0.40)
        for _ in range(2):
            self.step(policy, 0.20, patrol=True)
        for _ in range(2):
            self.step(policy, 0.30, patrol=True)
        self.assertEqual(self.sent, [])
        self.step(policy, 0.30, patrol=True)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.assertEqual(self.patrol_state.hold_start_m, 0.30)
        self.step(policy, 0.39, patrol=True)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.step(policy, 0.40, patrol=True)
        self.assertEqual(self.patrol_state.phase, "arrived")

    def test_guard_has_priority_over_late_visual_confirmation(self):
        policy = self.policy()
        self.state.branch_latched = True
        self.state.side = "right"
        self.state.arm = 1
        self.step(policy, 0.96)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.state.phase, "odom_wait")

    def test_stale_odom_stops_and_resets_stability_votes(self):
        policy = self.policy()
        self.state.branch_latched, self.state.side = True, "right"
        self.step(policy, 0.80)
        self.step(policy, 0.81, odom_valid=False)
        self.assertEqual(self.sent, [])
        self.step(policy, 0.81)
        self.assertEqual(self.sent, [])
        self.step(policy, 0.82)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.state.phase, "heading_hold")
        self.assertEqual(self.state.hold_start_m, 0.82)

    def test_received_timestamp_detects_buffered_or_invalid_odom(self):
        progress = EdgeProgress()
        progress.update(0, 0, 0, received_s=10.0)
        self.assertTrue(progress.is_fresh(10.4))
        self.assertFalse(progress.is_fresh(10.6))
        progress.update(float("nan"), 0, 0, received_s=10.7)
        self.assertFalse(progress.is_fresh(10.7))
        self.assertEqual(progress.s_m, 0)
        progress.reset()
        self.assertFalse(progress.is_fresh(10.7))

    def test_new_leg_has_no_old_patrol_evidence_and_suppresses_old_opening(self):
        policy = self.policy("2_2", "2_1", 0.80)
        self.state.suppress_cue = True
        self.step(policy, 0.10, patrol=True)
        self.assertFalse(self.patrol_state.edge_latched)
        self.step(policy, 0.12, patrol=True, reading=replace(self.reading, left=False, right=False))
        self.assertFalse(self.state.suppress_cue)
        self.assertFalse(self.patrol_state.edge_latched)

    def test_all_map_directions_validate_and_invalid_windows_are_rejected(self):
        for edge in self.graph.edges.values():
            for source, target in ((edge.u, edge.v), (edge.v, edge.u)):
                self.policy(source, target, edge.length_m)
        for override in (
            ArrivalSettings(handoff_progress_m=0.96, guard_progress_m=0.95),
            ArrivalSettings(final_forward_m=0.80),
            ArrivalSettings(guard_progress_m=1.5),
        ):
            self.graph.arrival_overrides[("1_2", "2_2")] = override
            with self.subTest(override=override), self.assertRaises(ValueError):
                self.policy()


if __name__ == "__main__":
    unittest.main()
