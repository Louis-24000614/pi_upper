"""视觉交接、定距直行、转弯和重新捕获。"""

from __future__ import annotations

import queue
import unittest

from road_follow.control import VelocityCommand
from road_follow.departure import apply_departure
from road_follow.junction_turn import (
    JunctionCue,
    JunctionTurn,
    JunctionTurnConfig,
    odom_handoff_turn_cue,
    road_end_turn_cue,
    should_stop_at_expected_junction,
    step_junction_turn,
)


class JunctionTurnTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = JunctionTurnConfig(
            stable_frames=2, reacquire_frames=2, road_end_missing_frames=2
        )
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.0, "follow")

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

    def test_visual_cue_hands_off_then_waits_for_planned_turn(self) -> None:
        state = JunctionTurn()
        cue = JunctionCue(True, "left", 0.32)
        state, command = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "follow")
        state, command = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 150 50"])

        self.notes.put("FORWARD_DONE")
        state, command = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "arrived")
        self.assertEqual(command.reason, "arrived")
        self.assertEqual(self.sent, ["forward 150 50"])

        state = apply_departure(state, "right", self.send)
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(state.departure, "right")
        self.assertEqual(self.sent[-1], "stop")

        self.notes.put("STOP_DONE")
        state, _ = step_junction_turn(
            state,
            JunctionCue(False),
            self.follow,
            self.notes,
            self.send,
            self.cfg,
            now_s=10.0,
        )
        self.assertEqual(state.phase, "stopped")
        state, _ = step_junction_turn(
            state,
            JunctionCue(False),
            self.follow,
            self.notes,
            self.send,
            self.cfg,
            now_s=11.9,
        )
        self.assertEqual(state.phase, "stopped")
        self.assertEqual(self.sent[-1], "stop")
        state, _ = step_junction_turn(
            state,
            JunctionCue(False),
            self.follow,
            self.notes,
            self.send,
            self.cfg,
            now_s=12.0,
        )
        self.assertEqual(state.phase, "turning")
        self.assertEqual(self.sent[-1], "turn right")

        self.notes.put("TURN_DONE")
        state, _ = step_junction_turn(
            state, JunctionCue(False), self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "reacquire")
        state, command = step_junction_turn(
            state, JunctionCue(False), self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "follow")

    def test_out_of_window_does_not_start_action(self) -> None:
        state = JunctionTurn()
        cue = JunctionCue(True, "right", 0.80)
        for _ in range(4):
            state, command = step_junction_turn(
                state, cue, self.follow, self.notes, self.send, self.cfg
            )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "follow")
        self.assertEqual(self.sent, [])

    def test_side_branch_only_latches_and_blue_band_triggers_forward(self) -> None:
        cfg = JunctionTurnConfig(
            stable_frames=2,
            branch_observe_min_distance_m=0.18,
            road_end_missing_frames=2,
        )
        state = JunctionTurn()
        side = JunctionCue(True, "right", 0.32, "side_branch")
        for _ in range(4):
            state, command = step_junction_turn(
                state, side, self.follow, self.notes, self.send, cfg
            )
        self.assertEqual(state.phase, "approach")
        self.assertTrue(state.branch_latched)
        self.assertEqual(command.reason, "follow")
        self.assertEqual(self.sent, [])

        road_end = road_end_turn_cue(
            side="right",
            stable_blocked=False,
            raw_blocked=False,
            command=self.follow,
            road_end_y_m=0.30,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=cfg,
            approach_latched=True,
            forward_band_ratio=0.05,
        )
        self.assertTrue(road_end.detected)
        state, _ = step_junction_turn(
            state, road_end, self.follow, self.notes, self.send, cfg
        )
        self.assertEqual(state.phase, "approach")
        state, command = step_junction_turn(
            state, road_end, self.follow, self.notes, self.send, cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 150 50"])

    def test_finite_action_failure_latches_stop(self) -> None:
        state = JunctionTurn(phase="forward", side="right", forward_mm=200)
        self.notes.put("FORWARD_FAIL")
        state, command = step_junction_turn(
            state, JunctionCue(False), self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_action_fail")
        state, command = step_junction_turn(
            state, JunctionCue(False), self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.v_mps, 0.0)

    def test_blocked_road_end_becomes_turn_cue(self) -> None:
        cue = road_end_turn_cue(
            side="left",
            stable_blocked=True,
            raw_blocked=True,
            command=self.follow,
            road_end_y_m=0.24,
            lane_x_m=-0.02,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=True,
        )
        self.assertTrue(cue.detected)
        self.assertEqual(cue.source, "road_end")
        self.assertAlmostEqual(cue.distance_m or 0.0, 0.15)

    def test_road_end_cue_requires_safe_follow_window(self) -> None:
        cases = (
            {"road_end_y_m": 0.52},
            {"road_end_y_m": 0.60},
            {"lane_x_m": 0.12},
            {"lane_width_m": 0.40},
            {"forward_band_ratio": 0.50},
        )
        defaults = {
            "side": "right",
            "stable_blocked": True,
            "raw_blocked": True,
            "command": self.follow,
            "road_end_y_m": 0.24,
            "lane_x_m": 0.0,
            "lane_width_m": 0.20,
            "cfg": self.cfg,
            "approach_latched": True,
        }
        for changed in cases:
            with self.subTest(changed=changed):
                cue = road_end_turn_cue(**(defaults | changed))
                self.assertFalse(cue.detected)

    def test_road_end_cue_uses_existing_stability_gate(self) -> None:
        state = JunctionTurn()
        cue = road_end_turn_cue(
            side="right",
            stable_blocked=True,
            raw_blocked=True,
            command=self.follow,
            road_end_y_m=0.24,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=True,
        )
        state, _ = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "follow")
        state, command = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 150 50"])

    def test_far_branch_is_latched_until_hidden_road_end(self) -> None:
        state = JunctionTurn()
        far_left = JunctionCue(True, "left", 0.82, "side_branch")
        state, command = step_junction_turn(
            state, far_left, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "follow")
        state, command = step_junction_turn(
            state, far_left, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "approach")
        self.assertTrue(state.branch_latched)
        self.assertEqual(state.side, "left")
        self.assertEqual(command.reason, "follow")
        self.assertEqual(self.sent, [])

        # 支路随后被墙遮住，锁存方向不能因此清除。
        state, command = step_junction_turn(
            state, JunctionCue(False), self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "approach")
        self.assertTrue(state.branch_latched)
        self.assertEqual(command.reason, "follow")

        road_end = road_end_turn_cue(
            side="left",
            stable_blocked=False,
            raw_blocked=True,
            command=self.follow,
            road_end_y_m=0.24,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=state.branch_latched,
        )
        self.assertTrue(road_end.detected)
        state, command = step_junction_turn(
            state, road_end, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "approach")
        state, command = step_junction_turn(
            state, road_end, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 150 50"])

    def test_latched_branch_handoffs_after_lookahead_stops(self) -> None:
        state = JunctionTurn()
        far_left = JunctionCue(True, "left", 0.54, "side_branch")
        for _ in range(2):
            state, _ = step_junction_turn(
                state, far_left, self.follow, self.notes, self.send, self.cfg
            )
        self.assertTrue(state.branch_latched)
        stopped = VelocityCommand(0.0, 0.0, "stop_lookahead")
        too_far = road_end_turn_cue(
            side="left",
            stable_blocked=True,
            raw_blocked=True,
            command=stopped,
            road_end_y_m=0.58,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=True,
        )
        self.assertFalse(too_far.detected)
        cue = road_end_turn_cue(
            side="left",
            stable_blocked=True,
            raw_blocked=True,
            command=stopped,
            road_end_y_m=0.42,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=True,
        )
        self.assertTrue(cue.detected)
        state, command = step_junction_turn(
            state, cue, stopped, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "approach")
        state, command = step_junction_turn(
            state, cue, stopped, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 150 50"])

    def test_lookahead_stop_without_branch_latch_does_not_blind_move(self) -> None:
        stopped = VelocityCommand(0.0, 0.0, "stop_lookahead")
        cue = road_end_turn_cue(
            side="left",
            stable_blocked=True,
            raw_blocked=True,
            command=stopped,
            road_end_y_m=0.27,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=False,
        )
        self.assertFalse(cue.detected)
        state, command = step_junction_turn(
            JunctionTurn(), cue, stopped, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "stop_lookahead")
        self.assertEqual(self.sent, [])

    def test_far_branch_latch_survives_long_occlusion(self) -> None:
        cfg = JunctionTurnConfig(branch_vote_window=8, branch_vote_min=2)
        state = JunctionTurn()
        cue = JunctionCue(True, "right", 0.80, "side_branch")
        for _ in range(2):
            state, _ = step_junction_turn(
                state, cue, self.follow, self.notes, self.send, cfg
            )
        self.assertEqual(state.phase, "approach")
        for _ in range(300):
            state, _ = step_junction_turn(
                state,
                JunctionCue(False),
                self.follow,
                self.notes,
                self.send,
                cfg,
            )
        self.assertEqual(state.phase, "approach")
        self.assertTrue(state.branch_latched)
        self.assertEqual(state.side, "right")

    def test_odom_at_expected_junction_stops_without_visual_turn(self) -> None:
        state = JunctionTurn()
        cfg = JunctionTurnConfig(odom_stop_margin_m=0.05)
        self.assertFalse(
            should_stop_at_expected_junction(0.70, 0.80, "junction", state, cfg)
        )
        self.assertTrue(
            should_stop_at_expected_junction(0.75, 0.80, "junction", state, cfg)
        )
        state.branch_latched = True
        self.assertTrue(
            should_stop_at_expected_junction(0.80, 0.80, "junction", state, cfg)
        )

    def test_latched_cross_uses_odom_for_last_twenty_centimeters(self) -> None:
        """十字路口前方仍有道路时，也必须在边末端完成到点交接。"""
        cfg = JunctionTurnConfig(
            stable_frames=2,
            turn_forward_m=0.20,
            road_end_missing_frames=3,
        )
        state = JunctionTurn(
            phase="approach",
            side="right",
            branch_latched=True,
        )

        before = odom_handoff_turn_cue(
            side=state.side,
            progress_m=0.59,
            edge_length_m=0.80,
            target_role="junction",
            state=state,
            command=self.follow,
            cfg=cfg,
        )
        self.assertFalse(before.detected)

        cue = odom_handoff_turn_cue(
            side=state.side,
            progress_m=0.60,
            edge_length_m=0.80,
            target_role="junction",
            state=state,
            command=self.follow,
            cfg=cfg,
        )
        self.assertTrue(cue.detected)
        self.assertEqual(cue.source, "odom_handoff")
        self.assertAlmostEqual(cue.distance_m or 0.0, 0.20)

        state, _ = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, cfg
        )
        self.assertEqual(state.phase, "approach")
        state, command = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 200 50"])

    def test_odom_handoff_requires_a_latched_junction_and_safe_follow(self) -> None:
        cfg = JunctionTurnConfig(turn_forward_m=0.20)
        cases = (
            (JunctionTurn(phase="approach", side="right"), self.follow, "junction"),
            (
                JunctionTurn(phase="approach", side="right", branch_latched=True),
                VelocityCommand(0.0, 0.0, "stop_no_road"),
                "junction",
            ),
            (
                JunctionTurn(phase="approach", side="right", branch_latched=True),
                self.follow,
                "patrol_slot",
            ),
        )
        for state, command, role in cases:
            with self.subTest(role=role, reason=command.reason):
                cue = odom_handoff_turn_cue(
                    side=state.side,
                    progress_m=0.65,
                    edge_length_m=0.80,
                    target_role=role,
                    state=state,
                    command=command,
                    cfg=cfg,
                )
                self.assertFalse(cue.detected)


if __name__ == "__main__":
    unittest.main()
