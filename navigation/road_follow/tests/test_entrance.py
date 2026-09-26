"""出发区先前进 15 cm 到 0_J，再按 Agent 方向转弯并交回视觉速度。"""

from __future__ import annotations

import queue
import unittest

from road_follow.control import VelocityCommand
from road_follow.entrance import EntranceConfig, EntranceDeparture, step_entrance_departure


class EntranceDepartureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.0, "follow")

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

    def test_forward_then_right_then_visual(self) -> None:
        state = EntranceDeparture()
        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=10.0
        )
        self.assertEqual(state.phase, "forward")
        self.assertEqual(command.reason, "entrance_forward")
        self.assertEqual(self.sent, ["forward 150 50"])

        self.notes.put("FORWARD_DONE")
        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=11.0
        )
        self.assertEqual(state.phase, "stopping_wait")
        self.assertEqual(self.sent[-1], "stop")
        self.assertEqual(command.v_mps, 0.0)
        # 主循环在收到 FORWARD_DONE 后，用拓扑的下一条边确定这个方向。
        state.turn_side = "right"

        self.notes.put("STOP_DONE")
        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=11.1
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(command.reason, "entrance_stop_settle")

        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=13.0
        )
        self.assertEqual(state.phase, "stopping")

        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=13.1
        )
        self.assertEqual(state.phase, "turning")
        self.assertEqual(self.sent[-1], "turn right")
        self.assertEqual(command.v_mps, 0.0)

        self.notes.put("TURN_DONE")
        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=14.0
        )
        self.assertEqual(state.phase, "reacquire")
        self.assertEqual(command.reason, "entrance_reacquire")
        self.assertEqual(command.v_mps, 0.0)

        cfg = EntranceConfig(reacquire_frames=3, recovery_frames=2, recovery_mps=0.06)
        for expected_clear in (1, 2):
            state, command = step_entrance_departure(
                state, self.notes, self.send, self.follow, cfg, now_s=14.1
            )
            self.assertEqual(state.phase, "reacquire")
            self.assertEqual(state.clear, expected_clear)
            self.assertEqual(command.v_mps, 0.0)

        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, cfg, now_s=14.2
        )
        self.assertEqual(state.phase, "recovery")
        self.assertEqual(command.reason, "entrance_recovery")
        self.assertAlmostEqual(command.v_mps, 0.06)

        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, cfg, now_s=14.3
        )
        self.assertEqual(state.phase, "done")
        self.assertEqual(command.reason, "entrance_recovery")
        self.assertAlmostEqual(command.v_mps, 0.06)

    def test_lost_road_during_recovery_returns_to_reacquire(self) -> None:
        state = EntranceDeparture(phase="recovery", clear=3, recovery=2)
        stopped = VelocityCommand(0.0, 0.0, "stop_lookahead")
        state, command = step_entrance_departure(
            state, self.notes, self.send, stopped, EntranceConfig()
        )
        self.assertEqual(state.phase, "reacquire")
        self.assertEqual(state.clear, 0)
        self.assertEqual(state.recovery, 0)
        self.assertEqual(command.v_mps, 0.0)

    def test_forward_failure_stops(self) -> None:
        state = EntranceDeparture(phase="forward")
        self.notes.put("FORWARD_FAIL")
        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_entrance_fail")
        self.assertEqual(self.sent, [])

    def test_agent_can_select_left_turn(self) -> None:
        state = EntranceDeparture(
            phase="stopping", turn_side="left", stop_started_s=10.0
        )

        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=12.1
        )

        self.assertEqual(state.phase, "turning")
        self.assertEqual(self.sent, ["turn left"])
        self.assertEqual(command.reason, "entrance_turn")

    def test_missing_agent_turn_direction_fails_safe(self) -> None:
        state = EntranceDeparture(phase="stopping", stop_started_s=10.0)

        state, command = step_entrance_departure(
            state, self.notes, self.send, self.follow, now_s=12.1
        )

        self.assertEqual(state.phase, "fault")
        self.assertEqual(self.sent, [])
        self.assertEqual(command.reason, "stop_entrance_fail")


if __name__ == "__main__":
    unittest.main()
