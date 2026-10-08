"""Agent patrol_slot 的分段 RFID 搜索。"""

from __future__ import annotations

import queue
import unittest

from road_follow.control import FollowConfig, VelocityCommand
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
        near_x_m=None,
        lane_heading_rad=None,
        yaw_rad=None,
        now_s=None,
        progress_m=0.80,
        odom_valid=True,
        centerline_points=None,
        road_pixels=800,
        follow=None,
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
            near_x_m=near_x_m,
            lane_heading_rad=lane_heading_rad,
            yaw_rad=yaw_rad,
            now_s=now_s,
            progress_m=progress_m,
            odom_valid=odom_valid,
            centerline_points=centerline_points,
            road_pixels=road_pixels,
            follow=follow,
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
        state, command = self.step(state, forward_band_ratio=0.05, progress_m=0.80)
        self.assertEqual((state.phase, command.reason), ("heading_hold", "heading_hold"))
        self.assertEqual(command.omega_radps, 0.0)
        self.assertAlmostEqual(command.v_mps, 0.05)
        self.assertEqual(state.hold_start_m, 0.80)
        self.assertEqual(self.sent, [])

        state, command = self.step(state, detection=(6, 2))
        self.assertEqual((state.phase, command.reason), ("stopping_wait", "stop_rfid_action"))
        self.assertEqual(self.sent[-1], "stop")

        self.notes.put("STOP_DONE")
        state, command = self.step(state, visual=self.blocked)
        self.assertEqual((state.phase, command.reason), ("arrived", "rfid_arrived"))
        self.assertEqual((state.card_number, state.generation), (6, 2))

    def test_heading_hold_confirms_arrival_without_a_card(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(state, forward_band_ratio=0.0, progress_m=0.80)
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.omega_radps, 0.0)
        state, command = self.step(state, progress_m=0.90, odom_valid=False)
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "stop_odom_stale")
        self.assertEqual(state.hold_start_m, 0.80)
        state, command = self.step(state, progress_m=1.00)
        self.assertEqual(state.phase, "arrived")
        self.assertEqual(command.reason, "rfid_arrived")
        self.assertEqual(state.searched_mm, 200)
        self.assertEqual(self.sent, [])

    def test_does_not_blind_move_without_seeing_edge_first(self) -> None:
        state, command = self.step(
            RfidArrival(), visual=VelocityCommand(0.0, 0.0, "stop_camera")
        )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "stop_camera")
        self.assertEqual(self.sent, [])

    def test_unsafe_frame_stays_on_follow_and_resumes_when_the_road_returns(self) -> None:
        stopped = VelocityCommand(0.0, 0.0, "stop_road")
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(
            state, visual=stopped, forward_band_ratio=0.0, visual_safe=False,
        )
        self.assertEqual(state.phase, "follow")
        self.assertTrue(state.edge_latched)
        self.assertEqual(state.road_end_missing_frames, 0)
        self.assertEqual(command, stopped)
        self.assertEqual(self.sent, [])

        state, command = self.step(state, forward_band_ratio=0.70, visual_safe=True)
        self.assertEqual(state.phase, "follow")
        self.assertEqual(state.road_end_missing_frames, 0)
        self.assertEqual(command.reason, "follow")
        self.assertEqual(self.sent, [])

        for _ in range(2):
            state, command = self.step(
                state, forward_band_ratio=0.0, visual_safe=True, lane_heading_rad=0.0,
            )
            self.assertEqual(state.phase, "follow")
        state, command = self.step(
            state, forward_band_ratio=0.0, visual_safe=True, lane_heading_rad=0.0,
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "heading_hold")

    def test_parallel_offset_locks_yaw_for_the_last_step(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(
            state, forward_band_ratio=0.0, near_x_m=0.08,
            lane_heading_rad=0.0, yaw_rad=0.25, now_s=5.0, progress_m=0.80,
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(state.hold_yaw_rad, 0.25)
        self.assertEqual(command.omega_radps, 0.0)

        state, command = self.step(
            state, forward_band_ratio=0.0, yaw_rad=0.05,
            progress_m=0.90, odom_valid=True,
        )
        self.assertGreater(command.omega_radps, 0.0)
        self.assertEqual(state.hold_yaw_rad, 0.25)
        self.assertAlmostEqual(command.v_mps, 0.05)

        state, command = self.step(
            state, yaw_rad=1.0, progress_m=0.90, odom_valid=False,
        )
        self.assertEqual(command.reason, "stop_odom_stale")
        self.assertEqual(state.hold_yaw_rad, 0.25)

    def test_offset_near_centerline_aligns_before_blind_forward(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(
            state, forward_band_ratio=0.0, lane_heading_rad=0.20,
            near_x_m=0.08, now_s=5.0,
        )
        self.assertEqual(state.phase, "align")
        self.assertEqual(command.reason, "align")
        self.assertEqual(command.v_mps, 0.0)
        self.assertLess(command.omega_radps, 0.0)
        self.assertEqual(self.sent, [])

        state, command = self.step(
            state, forward_band_ratio=0.0, lane_heading_rad=0.0,
            near_x_m=0.08, now_s=5.1,
        )
        self.assertEqual(state.phase, "align")
        self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))
        self.assertEqual(self.sent, [])
        for now_s in (5.2, 5.3, 5.4):
            state, _ = self.step(
                state, forward_band_ratio=0.0, lane_heading_rad=0.0,
                near_x_m=0.08, now_s=now_s,
            )
            self.assertEqual(state.phase, "align")
        state, command = self.step(
            state, forward_band_ratio=0.0, lane_heading_rad=0.0,
            near_x_m=0.08, now_s=5.5,
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "heading_hold")
        self.assertEqual(command.omega_radps, 0.0)
        self.assertAlmostEqual(command.v_mps, 0.05)
        self.assertEqual(self.sent, [])

    def test_visual_end_align_uses_quarter_the_follow_gain(self) -> None:
        follow = FollowConfig(steering_gain=2.0)
        state, command = self.step(
            RfidArrival(edge_latched=True, road_end_missing_frames=2),
            forward_band_ratio=0.0,
            lane_heading_rad=0.20,
            now_s=5.0,
            follow=follow,
        )
        self.assertEqual(state.phase, "align")
        self.assertAlmostEqual(command.omega_radps, -0.10)
        state, command = self.step(
            state,
            forward_band_ratio=0.0,
            lane_heading_rad=0.20,
            now_s=5.1,
            follow=follow,
        )
        self.assertEqual(state.phase, "align")
        self.assertEqual(command.reason, "align")
        self.assertAlmostEqual(command.omega_radps, -0.10)

    def test_visual_end_align_finishes_when_the_band_returns(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(
            state, forward_band_ratio=0.06, lane_heading_rad=0.20, now_s=5.0
        )
        self.assertEqual(state.phase, "align")
        self.assertLess(command.omega_radps, 0.0)
        state, command = self.step(
            state, forward_band_ratio=0.29, lane_heading_rad=0.20, now_s=5.1
        )
        self.assertEqual(state.phase, "align")
        self.assertEqual(command.reason, "align")
        self.assertLess(command.omega_radps, 0.0)
        state, command = self.step(
            state, forward_band_ratio=0.29, lane_heading_rad=0.20, now_s=6.6
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "heading_hold")
        self.assertEqual(self.sent, [])

    def test_latched_visual_end_follows_the_forward_strip_not_the_opening(self) -> None:
        ahead = [(0.0, 0.20 + i * 0.01) for i in range(8)]
        opening = [(-0.35, 0.30 + i * 0.02) for i in range(6)]
        chasing = VelocityCommand(0.08, 0.30, "follow")
        follow = FollowConfig(
            lookahead_m=0.30, min_lookahead_m=0.24, near_mps=0.05, min_points=8,
        )
        state, command = self.step(
            RfidArrival(edge_latched=True),
            visual=chasing,
            forward_band_ratio=0.80,
            centerline_points=ahead + opening,
            follow=follow,
        )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "follow_near")
        self.assertAlmostEqual(command.v_mps, 0.05)
        self.assertAlmostEqual(command.omega_radps, 0.0)
        self.assertNotEqual(command.omega_radps, chasing.omega_radps)

    def test_latched_visual_end_stops_when_only_the_opening_is_visible(self) -> None:
        opening = [(-0.35, 0.20 + i * 0.03) for i in range(10)]
        chasing = VelocityCommand(0.08, 0.30, "follow")
        state, command = self.step(
            RfidArrival(edge_latched=True),
            visual=chasing,
            forward_band_ratio=0.80,
            centerline_points=opening,
        )
        self.assertEqual(state.phase, "follow")
        self.assertEqual(command.reason, "stop_forward_strip")
        self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))

    def test_latched_visual_end_does_not_align_into_the_opening(self) -> None:
        ahead = [(0.0, 0.20 + i * 0.02) for i in range(8)]
        opening = [(-0.40, 0.40 + i * 0.02) for i in range(4)]
        state, command = self.step(
            RfidArrival(edge_latched=True, road_end_missing_frames=2),
            visual=VelocityCommand(0.08, 0.50, "follow"),
            forward_band_ratio=0.0,
            lane_heading_rad=-0.50,
            centerline_points=ahead + opening,
            now_s=5.0,
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "heading_hold")
        self.assertNotEqual(command.omega_radps, 0.50)

    def test_visual_end_align_still_creeps_when_the_band_stays_low(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, _ = self.step(
            state, forward_band_ratio=0.06, lane_heading_rad=0.20, now_s=5.0
        )
        self.assertEqual(state.phase, "align")
        state, command = self.step(
            state, forward_band_ratio=0.06, lane_heading_rad=0.0, now_s=5.1
        )
        self.assertEqual(state.phase, "align")
        self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))
        for now_s in (5.2, 5.3, 5.4):
            state, _ = self.step(
                state, forward_band_ratio=0.06, lane_heading_rad=0.0, now_s=now_s
            )
        state, command = self.step(
            state, forward_band_ratio=0.06, lane_heading_rad=0.0, now_s=5.5
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "heading_hold")
        self.assertEqual(command.omega_radps, 0.0)
        self.assertAlmostEqual(command.v_mps, 0.05)
        self.assertEqual(self.sent, [])

    def test_missing_near_centerline_skips_align(self) -> None:
        state = RfidArrival(edge_latched=True, road_end_missing_frames=2)
        state, command = self.step(
            state, forward_band_ratio=0.0, near_x_m=None, now_s=5.0
        )
        self.assertEqual(state.phase, "heading_hold")
        self.assertEqual(command.reason, "heading_hold")
        self.assertEqual(command.omega_radps, 0.0)
        self.assertAlmostEqual(command.v_mps, 0.05)
        self.assertEqual(self.sent, [])

    def test_card_while_following_requests_explicit_stop(self) -> None:
        state, command = self.step(RfidArrival(), detection=(2, 9))
        self.assertEqual(state.phase, "stopping_wait")
        self.assertEqual(command.reason, "stop_rfid_action")
        self.assertEqual(self.sent, ["stop"])


if __name__ == "__main__":
    unittest.main()
