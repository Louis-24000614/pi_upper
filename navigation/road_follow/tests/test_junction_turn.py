"""视觉交接、定距直行、转弯和重新捕获。"""

from __future__ import annotations

import queue
import unittest

from road_follow.control import VelocityCommand
from road_follow.junction_turn import (
    JunctionCue,
    JunctionTurn,
    JunctionTurnConfig,
    road_end_turn_cue,
    step_junction_turn,
)


class JunctionTurnTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = JunctionTurnConfig(stable_frames=2, reacquire_frames=2)
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.0, "follow")

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

    def test_visual_cue_hands_off_to_finite_forward_then_turn(self) -> None:
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
        self.assertEqual(self.sent, ["forward 320 50"])

        self.notes.put("FORWARD_DONE")
        state, command = step_junction_turn(
            state, cue, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "turning")
        self.assertEqual(self.sent[-1], "turn left")

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
            road_end_y_m=0.52,
            lane_x_m=-0.02,
            lane_width_m=0.20,
            cfg=self.cfg,
        )
        self.assertTrue(cue.detected)
        self.assertEqual(cue.source, "road_end")
        self.assertAlmostEqual(cue.distance_m or 0.0, 0.42)

    def test_road_end_cue_requires_safe_follow_window(self) -> None:
        cases = (
            {"road_end_y_m": 0.60},
            {"command": VelocityCommand(0.0, 0.0, "stop_lookahead")},
            {"stable_blocked": False},
            {"lane_x_m": 0.12},
            {"lane_width_m": 0.40},
        )
        defaults = {
            "side": "right",
            "stable_blocked": True,
            "raw_blocked": True,
            "command": self.follow,
            "road_end_y_m": 0.52,
            "lane_x_m": 0.0,
            "lane_width_m": 0.20,
            "cfg": self.cfg,
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
            road_end_y_m=0.52,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
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
        self.assertEqual(self.sent, ["forward 420 50"])

    def test_far_branch_is_latched_until_hidden_road_end(self) -> None:
        state = JunctionTurn()
        far_left = JunctionCue(True, "left", 0.82, "side_branch")
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
            road_end_y_m=0.52,
            lane_x_m=0.0,
            lane_width_m=0.20,
            cfg=self.cfg,
            approach_latched=state.branch_latched,
        )
        self.assertTrue(road_end.detected)
        state, command = step_junction_turn(
            state, road_end, self.follow, self.notes, self.send, self.cfg
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "blind_forward")
        self.assertEqual(self.sent, ["forward 420 50"])

    def test_far_branch_latch_expires(self) -> None:
        cfg = JunctionTurnConfig(branch_latch_max_frames=2)
        state = JunctionTurn()
        state, _ = step_junction_turn(
            state,
            JunctionCue(True, "right", 0.80, "side_branch"),
            self.follow,
            self.notes,
            self.send,
            cfg,
        )
        self.assertEqual(state.phase, "approach")
        for _ in range(3):
            state, _ = step_junction_turn(
                state,
                JunctionCue(False),
                self.follow,
                self.notes,
                self.send,
                cfg,
            )
        self.assertEqual(state.phase, "follow")
        self.assertFalse(state.branch_latched)


if __name__ == "__main__":
    unittest.main()
