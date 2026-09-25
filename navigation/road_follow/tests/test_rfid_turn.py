"""RFID 到点停车、转弯和视觉重新接管。"""

from __future__ import annotations

import queue
import unittest

from road_follow.control import VelocityCommand
from road_follow.rfid_turn import RfidTurn, RfidTurnConfig, step_rfid_turn


class RfidTurnTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = RfidTurnConfig(stop_settle_s=0.30, reacquire_frames=2)
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.01, "follow")

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

    def test_detection_stops_then_turns_and_returns_to_vision(self) -> None:
        state = RfidTurn(side="right")
        state, command = step_rfid_turn(
            state, (7, 3), self.follow, self.notes, self.send, 10.0, self.cfg
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(state.card_number, 7)
        self.assertEqual(command.reason, "stop_rfid")
        self.assertEqual(self.sent, [])

        state, command = step_rfid_turn(
            state, None, self.follow, self.notes, self.send, 10.2, self.cfg
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(command.reason, "stop_rfid_settle")
        self.assertEqual(self.sent, [])

        state, command = step_rfid_turn(
            state, None, self.follow, self.notes, self.send, 10.3, self.cfg
        )
        self.assertEqual(state.phase, "turning")
        self.assertEqual(command.reason, "rfid_turning")
        self.assertEqual(self.sent, ["turn right"])

        self.notes.put("TURN_DONE")
        state, command = step_rfid_turn(
            state, None, self.follow, self.notes, self.send, 11.0, self.cfg
        )
        self.assertEqual(state.phase, "reacquire")
        self.assertEqual(command.reason, "rfid_reacquire")
        state, command = step_rfid_turn(
            state, None, self.follow, self.notes, self.send, 11.1, self.cfg
        )
        self.assertEqual(state.phase, "complete")
        self.assertEqual(command.reason, "follow")

    def test_completed_test_ignores_later_cards(self) -> None:
        state = RfidTurn(phase="complete", side="left", card_number=2)
        state, command = step_rfid_turn(
            state, (9, 4), self.follow, self.notes, self.send, 20.0, self.cfg
        )
        self.assertEqual(state.phase, "complete")
        self.assertEqual(state.card_number, 2)
        self.assertEqual(command.reason, "follow")
        self.assertEqual(self.sent, [])

    def test_lookahead_loss_starts_bounded_imu_search_until_card(self) -> None:
        state = RfidTurn(side="right")
        stopped = VelocityCommand(0.0, 0.0, "stop_lookahead")
        state, command = step_rfid_turn(
            state, None, stopped, self.notes, self.send, 40.0, self.cfg
        )
        self.assertEqual(state.phase, "searching")
        self.assertEqual(command.reason, "rfid_searching")
        self.assertEqual(self.sent, ["forward 350 50"])

        state, command = step_rfid_turn(
            state, (6, 2), stopped, self.notes, self.send, 41.0, self.cfg
        )
        self.assertEqual(state.phase, "stopping_wait")
        self.assertEqual(command.reason, "stop_rfid_action")
        self.assertEqual(self.sent[-1], "stop")

        self.notes.put("STOP_DONE")
        state, command = step_rfid_turn(
            state, None, stopped, self.notes, self.send, 41.1, self.cfg
        )
        self.assertEqual(state.phase, "stopping")
        self.assertEqual(command.reason, "stop_rfid_settle")

    def test_search_limit_without_card_latches_stop(self) -> None:
        state = RfidTurn(phase="searching", side="right")
        self.notes.put("FORWARD_DONE")
        state, command = step_rfid_turn(
            state, None, self.follow, self.notes, self.send, 50.0, self.cfg
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_rfid_not_found")
        self.assertEqual(command.v_mps, 0.0)

    def test_turn_failure_latches_stop(self) -> None:
        state = RfidTurn(phase="turning", side="right", card_number=3)
        self.notes.put("TURN_FAIL")
        state, command = step_rfid_turn(
            state, None, self.follow, self.notes, self.send, 30.0, self.cfg
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_rfid_turn_fail")
        self.assertEqual(command.v_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
