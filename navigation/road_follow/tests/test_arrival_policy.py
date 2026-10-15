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
        self.cfg = JunctionTurnConfig(heading_anchor_enabled=False, branch_observe_min_distance_m=0.18)
        self.patrol_cfg = RfidArrivalConfig(
            heading_anchor_enabled=False, early_lock_frames=2,
        )
        self.state = JunctionTurn()
        self.patrol_state = RfidArrival()
        self.notes = queue.Queue()
        self.sent = []
        self.command = VelocityCommand(0.08, 0, "follow")
        self.reading = JunctionRead(KIND_CROSS, True, True, True, 0, 0.20, 0.50, 1.0, 0.85)

    def policy(self, source="1_2", target="2_2", length=0.99):
        return resolve_arrival_policy(self.graph, source, target, length, self.cfg, self.patrol_cfg)

    def send(self, line):
        self.sent.append(line)
        return True

    def step(self, policy, progress, *, patrol=False, odom_valid=True, reading=None, command=None,
             visual_safe=True, near_x_m=None, lane_heading_rad=None, yaw_rad=None,
             trusted_odom_handoff=False, now_s=None):
        if policy.early_heading_lock:
            lane_heading_rad = 0.0 if lane_heading_rad is None else lane_heading_rad
            yaw_rad = 0.0 if yaw_rad is None else yaw_rad
        elif policy.imu_full_edge and yaw_rad is None:
            yaw_rad = 0.0
        self.state, self.patrol_state, cmd, trigger = step_route_arrival(
            policy, patrol=patrol, state=self.state, patrol_state=self.patrol_state,
            reading=reading or self.reading, opening=(reading or self.reading).kind,
            command=command or self.command, progress_m=progress, odom_valid=odom_valid,
            notes=self.notes, send=self.send, junction_cfg=self.cfg,
            patrol_cfg=self.patrol_cfg, visual_safe=visual_safe, near_x_m=near_x_m,
            lane_heading_rad=lane_heading_rad, yaw_rad=yaw_rad,
            forward_depth_m=0.45,
            trusted_odom_handoff=trusted_odom_handoff,
            now_s=now_s,
        )
        return cmd, trigger

    def _finish_live_align(self, policy, progress, *, patrol, yaw, heading, now=1.0,
                           stable_heading=0.0):
        """停稳后按可见走廊摆正；走廊消失时保持停车时的航向。"""
        active = self.patrol_state if patrol else self.state
        cmd, trigger = self.step(
            policy, progress, patrol=patrol, yaw_rad=yaw,
            lane_heading_rad=heading, now_s=now,
        )
        self.assertEqual(active.phase, "heading_lock_stop")
        self.assertEqual(cmd.reason, "stop_heading_lock_wait")
        self.assertEqual(self.sent[-1], "stop")
        self.notes.put("STOP_DONE")
        self.step(
            policy, progress, patrol=patrol, yaw_rad=yaw,
            lane_heading_rad=heading, now_s=now + 0.05,
        )
        self.assertEqual(active.phase, "heading_lock_stopped")
        cmd, trigger = self.step(
            policy, progress, patrol=patrol, yaw_rad=yaw,
            lane_heading_rad=stable_heading, now_s=now + 0.40,
        )
        if active.phase == "heading_hold":
            return cmd, trigger
        self.assertEqual(active.phase, "align")
        for index in range(policy.align_stable_frames):
            cmd, trigger = self.step(
                policy, progress, patrol=patrol, yaw_rad=yaw,
                lane_heading_rad=stable_heading, now_s=now + 0.50 + index * 0.03,
            )
            if active.phase == "heading_hold":
                return cmd, trigger
        self.fail("摆正没有在稳定帧内结束")

    def test_directed_override_and_map_side_rotate_with_incoming_edge(self):
        incoming = self.policy()
        self.assertEqual(incoming.mode, "visual_odom")
        self.assertEqual(incoming.handoff_progress_m, 0.68)
        self.assertAlmostEqual(incoming.final_forward_m, 0.31)
        self.assertEqual(incoming.guard_progress_m, 0.94)
        self.assertEqual(incoming.align_timeout_s, 3.0)
        self.assertEqual(incoming.align_stable_frames, 8)
        self.assertEqual(incoming.expected_openings, ("forward", "left", "right"))
        reverse = self.policy("2_2", "1_2", 0.99)
        self.assertEqual(reverse.mode, "visual_end")
        self.assertAlmostEqual(reverse.final_forward_m, 0.31)
        self.assertAlmostEqual(reverse.guard_progress_m, 0.94)
        self.assertEqual(reverse.align_timeout_s, 3.0)
        self.assertEqual(reverse.expected_openings, ("right",))
        left_corner = self.policy("0_J", "1_2", 0.4)
        self.assertEqual(left_corner.expected_openings, ("left",))
        self.assertAlmostEqual(left_corner.final_forward_m, 0.20)
        self.assertEqual(left_corner.align_timeout_s, 1.5)
        self.assertEqual(left_corner.align_stable_frames, 5)
        self.assertTrue(left_corner.imu_full_edge)
        self.assertFalse(left_corner.early_heading_lock)
        self.assertAlmostEqual(left_corner.edge_length_m, 0.47)
        for source, target in (
            ("2_2", "2_1"), ("2_3", "2_4"),
            ("4_1", "5_1"), ("4_4", "5_4"),
        ):
            corner = self.policy(source, target, 0.99)
            self.assertFalse(corner.imu_full_edge)
            self.assertTrue(corner.early_heading_lock)

    def test_through_patrol_uses_edge_length_and_dead_end_still_watches_the_wall(self):
        through = self.policy("2_1", "3_1", 0.99)
        self.assertEqual(through.mode, "visual_odom")
        self.assertEqual(through.expected_openings, ("forward", "left"))
        self.assertAlmostEqual(through.handoff_progress_m, 0.68)
        self.assertAlmostEqual(through.final_forward_m, 0.31)
        self.assertAlmostEqual(through.guard_progress_m, 0.94)
        self.assertEqual(through.align_stable_frames, 8)
        dead_end = self.policy("3_2", "3_1", 1.0)
        self.assertEqual(dead_end.mode, "visual_end")
        self.assertNotIn("forward", dead_end.expected_openings)

        self.graph.arrival_overrides[("2_1", "3_1")] = ArrivalSettings(mode="visual_end")
        self.assertEqual(self.policy("2_1", "3_1", 1.0).mode, "visual_end")

    def test_every_standard_grid_edge_uses_68cm_handoff(self):
        for edge in self.graph.edges.values():
            if abs(edge.length_m - 0.99) > 1e-6:
                continue
            directions = [(edge.u, edge.v)]
            if edge.bidirectional:
                directions.append((edge.v, edge.u))
            for source, target in directions:
                with self.subTest(edge=edge.id, source=source, target=target):
                    policy = self.policy(source, target, edge.length_m)
                    self.assertAlmostEqual(policy.handoff_progress_m, 0.68)
                    self.assertAlmostEqual(policy.final_forward_m, 0.31)
                    self.assertAlmostEqual(policy.edge_length_m, 0.99)
                    self.assertTrue(policy.early_heading_lock)

    def test_through_patrol_aligns_live_and_drives_the_remainder(self):
        policy = self.policy("2_1", "3_1", 0.99)
        closed = replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT)
        for progress in (0.40, 0.50):
            self.step(
                policy, progress, patrol=True, reading=closed,
                yaw_rad=0.10, lane_heading_rad=0.0,
            )
        self.assertFalse(self.patrol_state.edge_latched)
        self.assertIsNotNone(self.patrol_state.locked_heading_yaw_rad)
        cmd, _ = self.step(
            policy, 0.70, patrol=True, reading=closed,
            yaw_rad=0.25, lane_heading_rad=0.20, now_s=1.0,
        )
        self.assertEqual(self.patrol_state.phase, "heading_lock_stop")
        self.assertEqual(cmd.reason, "stop_heading_lock_wait")
        self.notes.put("STOP_DONE")
        self.step(
            policy, 0.70, patrol=True, reading=closed,
            yaw_rad=0.25, lane_heading_rad=0.20, now_s=1.1,
        )
        cmd, _ = self.step(
            policy, 0.70, patrol=True, reading=closed,
            yaw_rad=0.25, lane_heading_rad=0.20, now_s=1.5,
        )
        self.assertEqual((self.patrol_state.phase, cmd.reason, cmd.v_mps), ("align", "align", 0.0))
        cmd, _ = self.step(
            policy, 0.70, patrol=True, reading=closed,
            yaw_rad=0.25, lane_heading_rad=0.20, now_s=1.53,
        )
        self.assertLess(cmd.omega_radps, 0.0)
        cmd, trigger = None, ""
        for index in range(policy.align_stable_frames):
            cmd, trigger = self.step(
                policy, 0.70, patrol=True, reading=closed,
                yaw_rad=0.25, lane_heading_rad=0.0, now_s=2.0 + index * 0.03,
            )
        self.assertEqual(trigger, "grid_imu")
        self.assertEqual(cmd.reason, "heading_hold")
        self.assertEqual(cmd.omega_radps, 0.0)
        self.assertAlmostEqual(cmd.v_mps, 0.05)
        self.assertEqual(self.patrol_state.active_step_mm, 290)
        self.assertAlmostEqual(self.patrol_state.hold_yaw_rad, 0.25)
        self.assertNotAlmostEqual(self.patrol_state.hold_yaw_rad, 0.10, places=2)
        self.step(policy, 0.99, patrol=True, reading=closed, yaw_rad=0.25)
        self.assertEqual(self.patrol_state.phase, "arrived")

    def test_through_patrol_past_68cm_stops_to_align_instead_of_guarding(self):
        policy = self.policy("2_1", "3_1", 0.99)
        cmd, trigger = self.step(policy, 0.95, patrol=True, now_s=1.0)
        self.assertEqual(trigger, "")
        self.assertEqual(self.patrol_state.phase, "heading_lock_stop")
        self.assertEqual(self.sent, ["stop"])
        self.assertEqual(cmd.reason, "stop_heading_lock_wait")

    def test_near_offset_aligns_before_junction_and_patrol_forward(self):
        policy = self.policy()
        cmd, trigger = self.step(
            policy, 0.68, near_x_m=0.08, lane_heading_rad=0.20, yaw_rad=0.30,
        )
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        self.assertEqual(self.sent, [])
        cmd, trigger = self.step(policy, 0.99, near_x_m=0.08, yaw_rad=0.30)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))

        self.state, self.patrol_state, self.sent = JunctionTurn(), RfidArrival(), []
        patrol = self.policy("3_2", "3_1", 0.80)
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
        self.assertLess(cmd.omega_radps, 0.0)

    def test_corner_stops_at_68cm_and_holds_the_aligned_yaw(self):
        policy = self.policy("2_2", "2_1", 0.99)
        cmd, trigger = self.step(
            policy, 0.67, patrol=True, yaw_rad=0.40, lane_heading_rad=0.20,
        )
        self.assertEqual((self.patrol_state.phase, cmd.reason, trigger), ("follow", "follow", ""))

        cmd, trigger = self._finish_live_align(
            policy, 0.68, patrol=True, yaw=0.40, heading=0.20,
        )
        self.assertEqual((self.patrol_state.phase, cmd.reason, trigger), (
            "heading_hold", "heading_hold", "grid_imu",
        ))
        self.assertAlmostEqual(self.patrol_state.hold_start_m, 0.68)
        self.assertAlmostEqual(self.patrol_state.hold_yaw_rad, 0.40)
        self.assertEqual(self.patrol_state.active_step_mm, 310)

        self.step(policy, 0.96, patrol=True, yaw_rad=0.40)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.step(policy, 0.99, patrol=True, yaw_rad=0.40)
        self.assertEqual(self.patrol_state.phase, "arrived")

    def test_live_align_timeout_keeps_the_yaw_at_the_stop(self):
        policy = self.policy("2_2", "2_1", 0.99)
        self.step(policy, 0.68, patrol=True, yaw_rad=0.33, lane_heading_rad=0.40, now_s=1.0)
        self.assertEqual(self.sent, ["stop"])
        self.notes.put("STOP_DONE")
        self.step(policy, 0.68, patrol=True, yaw_rad=0.33, lane_heading_rad=0.40, now_s=1.1)
        self.step(policy, 0.68, patrol=True, yaw_rad=0.33, lane_heading_rad=0.40, now_s=1.5)
        self.assertEqual(self.patrol_state.phase, "align")
        cmd, _ = self.step(
            policy, 0.68, patrol=True, yaw_rad=0.33, lane_heading_rad=0.40, now_s=4.5,
        )
        self.assertEqual((self.patrol_state.phase, cmd.reason), ("heading_hold", "heading_hold"))
        self.assertAlmostEqual(self.patrol_state.hold_yaw_rad, 0.33)
        self.assertEqual(self.patrol_state.active_step_mm, 310)

    def test_cross_follows_visually_until_the_node(self):
        policy = self.policy()
        cmd, trigger = self.step(policy, 0.68)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        self.assertEqual(self.sent, [])
        self.assertFalse(self.state.branch_latched)
        cmd, _ = self.step(policy, 0.90, command=VelocityCommand(0.08, 0.4, "follow"))
        self.assertEqual((self.state.phase, cmd.reason), ("follow", "follow"))
        cmd, trigger = self.step(policy, 0.94)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        cmd, trigger = self.step(policy, 0.99)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))

    def test_cross_does_not_freeze_a_heading_at_70cm(self):
        policy = self.policy()
        cmd, trigger = self.step(policy, 0.70)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        self.assertEqual(self.sent, [])
        self.assertFalse(self.patrol_state.heading_lock_frozen)
        cmd, trigger = self.step(policy, 0.99)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))

    def test_cross_visual_dropout_does_not_switch_to_locked_heading(self):
        policy = self.policy()
        stopped = VelocityCommand(0.0, 0.0, "stop_road")
        for _ in range(self.patrol_cfg.early_lock_blocked_frames + 1):
            cmd, trigger = self.step(policy, 0.80, command=stopped, visual_safe=False)
            self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "stop_road", ""))
        self.assertEqual(self.sent, [])
        cmd, trigger = self.step(policy, 0.99, command=stopped)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))

    def test_cross_without_a_latched_branch_still_arrives_on_odometry(self):
        policy = self.policy()
        unsafe = VelocityCommand(0.08, 0.0, "follow")
        cmd, trigger = self.step(policy, 0.68, command=unsafe, visual_safe=False, now_s=1.0)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        self.assertEqual(self.sent, [])
        cmd, trigger = self.step(policy, 0.99, command=unsafe, visual_safe=False)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))

    def test_tee_aligns_to_the_live_lane_without_a_branch_or_locked_heading(self):
        policy = self.policy("0_0", "0_J", 0.99)
        self.assertNotEqual(set(policy.expected_openings), {"forward", "left", "right"})
        self.assertTrue(policy.early_heading_lock)
        closed = replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT)
        for progress, heading in ((0.40, 0.0), (0.50, 0.0)):
            self.step(
                policy, progress, reading=closed, yaw_rad=0.10, lane_heading_rad=heading,
            )
        self.assertFalse(self.state.branch_latched)
        locked = self.patrol_state.locked_heading_yaw_rad
        cmd, trigger = self._finish_live_align(
            policy, 0.68, patrol=False, yaw=0.22, heading=0.18,
        )
        self.assertEqual((self.state.phase, cmd.reason, trigger), (
            "heading_hold", "heading_hold", "live_align",
        ))
        self.assertAlmostEqual(self.state.hold_yaw_rad, 0.22)
        self.assertEqual(self.state.forward_mm, 310)
        if locked is not None:
            self.assertNotAlmostEqual(self.state.hold_yaw_rad, locked, places=2)
        self.step(policy, 0.99, yaw_rad=0.22)
        self.assertEqual(self.state.phase, "arrived")

    def test_tee_with_no_visible_lane_holds_the_stop_yaw(self):
        policy = self.policy("0_0", "0_J", 0.99)
        self.step(policy, 0.68, yaw_rad=0.17, lane_heading_rad=float("nan"), now_s=1.0)
        self.assertEqual(self.state.phase, "heading_lock_stop")
        self.assertEqual(self.sent, ["stop"])
        self.notes.put("STOP_DONE")
        self.step(policy, 0.68, yaw_rad=0.17, lane_heading_rad=float("nan"), now_s=1.1)
        cmd, trigger = self.step(
            policy, 0.68, yaw_rad=0.17, lane_heading_rad=float("nan"), now_s=1.5,
        )
        self.assertEqual((self.state.phase, cmd.reason, trigger), (
            "heading_hold", "heading_hold", "live_align",
        ))
        self.assertAlmostEqual(self.state.hold_yaw_rad, 0.17)
        self.assertNotEqual(self.state.phase, "fault")

    def test_culvert_resume_uses_route_and_odom_without_relocking_branch(self):
        policy = self.policy("1_2", "2_2", 0.99)
        self.assertFalse(self.state.branch_latched)
        cmd, trigger = self.step(
            policy, 0.80, trusted_odom_handoff=True,
            reading=replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT),
        )
        self.assertEqual((self.state.phase, trigger), ("follow", ""))
        self.assertEqual(cmd.reason, "follow")

        cmd, trigger = self.step(
            policy, 0.80, trusted_odom_handoff=True,
            reading=replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT),
        )
        self.assertEqual((self.state.phase, trigger), ("heading_hold", "culvert_odom_handoff"))
        self.assertEqual(cmd.reason, "heading_hold")
        self.assertFalse(self.state.branch_latched)
        self.assertEqual(self.state.forward_mm, 190)
        self.assertAlmostEqual(self.state.hold_start_m, 0.80)

        self.step(policy, 0.96, trusted_odom_handoff=True)
        self.assertEqual(self.state.phase, "heading_hold")
        self.step(policy, 0.99, trusted_odom_handoff=True)
        self.assertEqual(self.state.phase, "arrived")

    def _culvert_patrol_without_side_evidence(self):
        policy = self.policy("2_1", "3_1", 0.99)
        closed = replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT)
        for progress in (0.40, 0.50):
            self.step(
                policy, progress, patrol=True, reading=closed,
                trusted_odom_handoff=True,
            )
        self.assertIsNotNone(self.patrol_state.locked_heading_yaw_rad)
        self.assertFalse(self.patrol_state.edge_latched)
        return policy, closed

    def test_culvert_patrol_reaches_node_without_late_side_evidence(self):
        for first_at_node in (0.99, 1.01):
            with self.subTest(first_at_node=first_at_node):
                self.setUp()
                policy, closed = self._culvert_patrol_without_side_evidence()
                self.assertEqual(policy.expected_openings, ("forward", "left"))
                for progress in (0.55, 0.68, 0.94, 0.989):
                    cmd, trigger = self.step(
                        policy, progress, patrol=True, reading=closed,
                        trusted_odom_handoff=True,
                    )
                    self.assertEqual((self.state.phase, self.patrol_state.phase), ("follow", "follow"))
                    self.assertEqual((cmd, trigger), (self.command, ""))
                cmd, trigger = self.step(
                    policy, first_at_node, patrol=True, reading=closed,
                    trusted_odom_handoff=True,
                )
                self.assertEqual((self.state.phase, self.patrol_state.phase), ("arrived", "arrived"))
                self.assertEqual((cmd.v_mps, cmd.omega_radps), (0.0, 0.0))
                self.assertEqual(trigger, "culvert_odom_handoff")
                self.assertFalse(self.patrol_state.edge_latched)
                self.assertEqual(self.sent, [])

                agent = RouteAgent(self.graph)
                agent.start()
                for _ in range(len(agent.state.route_edges)):
                    if (agent.state.from_node, agent.state.to_node) == ("2_1", "3_1"):
                        break
                    agent.node_reached(agent.state.to_node)
                self.assertEqual((agent.state.from_node, agent.state.to_node), ("2_1", "3_1"))
                xy = {key: (node.x, node.y) for key, node in self.graph.nodes.items()}
                handoff_arrival(self.state, agent, xy, self.send)
                self.assertEqual((agent.state.from_node, agent.state.to_node), ("3_1", "3_2"))
                self.assertIn("2_1__3_1", agent.state.covered_edges)
                self.assertEqual((self.state.phase, self.state.departure), ("stopping", "left"))
                self.assertEqual(self.sent, ["stop"])

    def test_culvert_patrol_stale_odometry_stops_and_recovers_at_node(self):
        for progress in (0.94, 0.99, 1.01):
            with self.subTest(progress=progress):
                self.setUp()
                policy, closed = self._culvert_patrol_without_side_evidence()
                cmd, trigger = self.step(
                    policy, progress, patrol=True, reading=closed,
                    trusted_odom_handoff=True, odom_valid=False,
                )
                self.assertEqual((self.state.phase, self.patrol_state.phase), ("follow", "follow"))
                self.assertEqual((cmd.v_mps, cmd.omega_radps, trigger), (0.0, 0.0, "odom_stale"))
                cmd, trigger = self.step(
                    policy, progress, patrol=True, reading=closed,
                    trusted_odom_handoff=True,
                )
                if progress < policy.edge_length_m:
                    self.assertEqual((self.state.phase, self.patrol_state.phase), ("follow", "follow"))
                    self.assertEqual((cmd, trigger), (self.command, ""))
                else:
                    self.assertEqual((self.state.phase, self.patrol_state.phase), ("arrived", "arrived"))
                    self.assertEqual((cmd.v_mps, cmd.omega_radps, trigger), (0.0, 0.0, "culvert_odom_handoff"))
                self.assertEqual(self.sent, [])

    def test_culvert_patrol_at_node_does_not_require_visual_follow(self):
        stopped = VelocityCommand(0.0, 0.0, "stop_road")
        for progress in (0.99, 1.01):
            with self.subTest(progress=progress):
                self.setUp()
                policy, closed = self._culvert_patrol_without_side_evidence()
                cmd, trigger = self.step(
                    policy, progress, patrol=True, reading=closed,
                    trusted_odom_handoff=True, command=stopped, visual_safe=False,
                )
                self.assertEqual((self.state.phase, self.patrol_state.phase), ("arrived", "arrived"))
                self.assertEqual((cmd.v_mps, cmd.omega_radps, trigger), (0.0, 0.0, "culvert_odom_handoff"))
                self.assertEqual(self.sent, [])

    def test_culvert_handoff_preserves_normal_alignment_and_visual_end(self):
        policy = self.policy("2_1", "3_1", 0.99)
        closed = replace(self.reading, left=False, right=False, kind=KIND_STRAIGHT)
        cmd, trigger = self.step(
            policy, 0.68, patrol=True, reading=closed, trusted_odom_handoff=False,
        )
        self.assertEqual((self.patrol_state.phase, cmd.reason, trigger), ("heading_lock_stop", "stop_heading_lock_wait", ""))
        self.assertEqual(self.sent, ["stop"])

        self.setUp()
        dead_end = self.policy("3_2", "3_1", 0.80)
        self.assertEqual(dead_end.mode, "visual_end")
        for progress in (0.79, 0.80, 1.01):
            cmd, trigger = self.step(
                dead_end, progress, patrol=True, reading=closed,
                trusted_odom_handoff=True,
            )
            self.assertEqual((self.state.phase, self.patrol_state.phase), ("follow", "follow"))
            self.assertEqual((cmd, trigger), (self.command, ""))
        self.assertEqual(self.sent, [])

    def test_culvert_patrol_distance_handoff_does_not_interrupt_departure(self):
        for phase in ("turning", "reacquire"):
            with self.subTest(phase=phase):
                self.setUp()
                policy = self.policy("2_1", "3_1", 0.99)
                self.state.phase = phase
                cmd, trigger = self.step(
                    policy, 1.01, patrol=True, trusted_odom_handoff=True,
                    command=VelocityCommand(0.0, 0.0, "stop_road"), visual_safe=False,
                )
                self.assertEqual((self.state.phase, self.patrol_state.phase), (phase, "follow"))
                self.assertEqual((cmd.v_mps, cmd.omega_radps, trigger), (0.0, 0.0, ""))
                self.assertEqual(self.sent, [])

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
        policy = self.policy("3_2", "3_1", 0.80)
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
        self.assertEqual(self.patrol_state.phase, "arrived")
        self.assertEqual(self.sent, [])

    def test_cross_guard_does_not_stop_short_of_the_node(self):
        agent = RouteAgent(self.graph)
        agent.start()
        agent.node_reached("0_J")
        agent.node_reached("1_2")
        policy = self.policy()
        cmd, trigger = self.step(policy, 0.95)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        xy = {key: (node.x, node.y) for key, node in self.graph.nodes.items()}
        handoff_arrival(self.state, agent, xy, self.send)
        self.assertEqual(agent.state.to_node, "2_2")
        self.assertEqual(agent.state.covered_edges, {"0_0__0_J", "0_J__1_2"})
        cmd, trigger = self.step(policy, 0.99)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))
        handoff_arrival(self.state, agent, xy, self.send)
        self.assertIn("1_2__2_2", agent.state.covered_edges)
        self.assertNotEqual(agent.state.to_node, "2_2")

    def test_patrol_stale_odom_still_stops_beyond_old_guard(self):
        policy = self.policy("3_2", "3_1", 0.80)
        for _ in range(2):
            self.step(policy, 0.70, patrol=True)
        self.patrol_state.road_end_missing_frames = 2
        cmd, trigger = self.step(policy, 0.77, patrol=True, odom_valid=False)
        self.assertEqual((cmd.v_mps, cmd.omega_radps), (0, 0))
        self.assertEqual(trigger, "odom_stale")
        self.assertEqual(self.patrol_state.road_end_missing_frames, 0)
        self.assertEqual(self.sent, [])

    def test_patrol_visual_dropout_returns_to_follow_when_the_road_returns(self):
        policy = self.policy("3_2", "3_1", 0.80)
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
        self.assertEqual(self.patrol_state.hold_start_m, 0.15)
        self.step(policy, 0.40, patrol=True)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.step(policy, 0.47, patrol=True)
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
        self.assertEqual(self.patrol_state.hold_start_m, 0.20)
        self.step(policy, 0.39, patrol=True)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.step(policy, 0.40, patrol=True)
        self.assertEqual(self.patrol_state.phase, "heading_hold")
        self.step(policy, 0.47, patrol=True)
        self.assertEqual(self.patrol_state.phase, "arrived")

    def test_cross_does_not_stop_at_the_arrival_guard(self):
        policy = self.policy()
        self.state.branch_latched = True
        self.state.side = "right"
        self.state.arm = 1
        self.step(policy, 0.96)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.state.phase, "follow")

    def test_stale_odom_stops_and_cross_resumes_visual_follow(self):
        policy = self.policy()
        self.state.branch_latched, self.state.side = True, "right"
        cmd, trigger = self.step(policy, 0.68, odom_valid=False)
        self.assertEqual((cmd.reason, trigger), ("stop_odom_stale", "odom_stale"))
        self.assertEqual(self.sent, [])
        cmd, trigger = self.step(policy, 0.68)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("follow", "follow", ""))
        cmd, trigger = self.step(policy, 0.99)
        self.assertEqual((self.state.phase, cmd.reason, trigger), ("arrived", "arrived", "grid_visual"))

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
        policy = self.policy("3_2", "3_1", 0.80)
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
