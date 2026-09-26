"""Agent patrol_slot 的分段 RFID 搜索。"""

from __future__ import annotations

import queue
import unittest

from road_follow.control import VelocityCommand
from road_follow.rfid_arrival import RfidArrival, RfidArrivalConfig, step_rfid_arrival


class RfidArrivalTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = RfidArrivalConfig(
            step_distance_mm=200,
            search_speed_mmps=50,
            edge_visible_frames=2,
            road_end_missing_frames=3,
            road_end_band_max_ratio=0.10,
        )
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.0, "follow")
        self.blocked = VelocityCommand(0.0, 0.0, "stop_lookahead")

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

    def step(
        self,
        state,
        detection=None,
        visual=None,
        edge_visible=False,
        edge_left_visible=False,
        edge_right_visible=False,
        forward_band_ratio=1.0,
        visual_safe=True,
    ):
        return step_rfid_arrival(
            state,
            detection,
            visual or self.follow,
            self.notes,
            self.send,
            self.cfg,
            edge_visible=edge_visible,
            edge_left_visible=edge_left_visible,
            edge_right_visible=edge_right_visible,
            forward_band_ratio=forward_band_ratio,
            visual_safe=visual_safe,
        )

    def test_saves_left_right_and_both_edge_directions(self) -> None:
        left = RfidArrival()
        for _ in range(2):
            left, _ = self.step(left, edge_left_visible=True)
        self.assertTrue(left.edge_latched)
        self.assertTrue(left.edge_left_seen)
        self.assertFalse(left.edge_right_seen)

        right = RfidArrival()
        for _ in range(2):
            right, _ = self.step(right, edge_right_visible=True)
        self.assertTrue(right.edge_latched)
        self.assertFalse(right.edge_left_seen)
        self.assertTrue(right.edge_right_seen)

        both = RfidArrival()
        both, _ = self.step(both, edge_left_visible=True)
        both, _ = self.step(both, edge_right_visible=True)
        self.assertTrue(both.edge_latched)
        self.assertTrue(both.edge_left_seen)
        self.assertTrue(both.edge_right_seen)

        # 锁存后画面中的侧路消失，历史方向仍保留到本节点动作结束。
        both, _ = self.step(both)
        self.assertTrue(both.edge_left_seen)
        self.assertTrue(both.edge_right_seen)

    def test_visual_follows_until_forward_band_disappears_then_moves_once(self) -> None:
        state = RfidArrival()
        for _ in range(2):
            state, command = self.step(state, edge_visible=True)
            self.assertEqual(command.reason, "follow")
        self.assertTrue(state.edge_latched)
        self.assertEqual(self.sent, [])

        # 侧边角消失本身不触发；蓝色检测带还有路就继续视觉。
        for _ in range(3):
            state, command = self.step(
                state, edge_visible=False, forward_band_ratio=0.70
            )
            self.assertEqual(command.reason, "follow")
        self.assertEqual(self.sent, [])

        # 正前方检测带连续 3 帧无 road mask 后，才走最后 20 cm。
        for _ in range(2):
            state, command = self.step(state, forward_band_ratio=0.05)
            self.assertEqual(command.reason, "follow")
        state, command = self.step(state, forward_band_ratio=0.05)
        self.assertEqual((state.phase, command.reason), ("blind_forward", "rfid_searching"))
        self.assertEqual(self.sent, ["forward 200 50"])

        state, command = self.step(state, detection=(6, 2))
        self.assertEqual((state.phase, command.reason), ("stopping_wait", "stop_rfid_action"))
        self.assertEqual(self.sent[-1], "stop")

        self.notes.put("STOP_DONE")
        state, command = self.step(state, visual=self.blocked)
        self.assertEqual((state.phase, command.reason), ("arrived", "rfid_arrived"))
        self.assertEqual((state.card_number, state.generation), (6, 2))

    def test_blind_forward_is_not_repeated_without_a_card(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, _ = self.step(state, forward_band_ratio=0.0)
        self.assertEqual(state.phase, "blind_forward")
        self.notes.put("FORWARD_DONE")
        state, command = self.step(state)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_rfid_not_found")
        self.assertEqual(state.searched_mm, 200)
        self.assertEqual(self.sent, ["forward 200 50"])

    def test_does_not_blind_move_without_seeing_edge_first(self) -> None:
        state, command = self.step(
            RfidArrival(), visual=VelocityCommand(0.0, 0.0, "stop_camera")
        )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "stop_camera")
        self.assertEqual(self.sent, [])

    def test_unsafe_frame_stops_instead_of_blind_forward(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(
            state, forward_band_ratio=0.0, visual_safe=False
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_rfid_unsafe")
        self.assertEqual(self.sent, [])

    def test_card_while_following_requests_explicit_stop(self) -> None:
        state, command = self.step(RfidArrival(), detection=(2, 9))
        self.assertEqual(state.phase, "stopping_wait")
        self.assertEqual(command.reason, "stop_rfid_action")
        self.assertEqual(self.sent, ["stop"])


if __name__ == "__main__":
    unittest.main()
