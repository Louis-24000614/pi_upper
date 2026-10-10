"""遇障后的视觉闭环倒车。"""

from __future__ import annotations

import math
import unittest

from road_follow.backup import (
    Backup,
    BackupConfig,
    EdgeProgress,
    forward_lane_heading,
    near_lane_heading,
    near_lane_x,
    reverse_omega,
    step_backup,
)
from road_follow.control import VelocityCommand


class BackupTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = BackupConfig(max_missing_frames=2, max_duration_s=5.0)
        self.follow = VelocityCommand(0.10, 0.0, "follow")

    def step(
        self,
        state: Backup,
        *,
        triggered: bool = False,
        near_x_m: float | None = 0.0,
        progress_s_m: float = 0.5,
        now_s: float = 1.0,
        yaw_rad: float | None = 0.0,
        odom_valid: bool = True,
        frame_captured_s: float | None = None,
        cfg: BackupConfig | None = None,
    ) -> tuple[Backup, VelocityCommand]:
        return step_backup(
            state,
            triggered,
            near_x_m,
            progress_s_m,
            self.follow,
            now_s,
            cfg or self.cfg,
            yaw_rad=yaw_rad,
            odom_valid=odom_valid,
            frame_captured_s=now_s if frame_captured_s is None else frame_captured_s,
        )

    def test_progress_grows_forward_and_shrinks_in_reverse(self) -> None:
        progress = EdgeProgress()
        self.assertEqual(progress.update(0.0, 0.0, 0.0), 0.0)
        self.assertAlmostEqual(progress.update(0.30, 0.0, 0.0), 0.30)
        self.assertAlmostEqual(progress.update(0.10, 0.0, 0.0), 0.10)
        progress.reset()
        self.assertEqual(progress.update(1.0, 1.0, 0.0), 0.0)

    def test_near_lane_ignores_the_far_centerline(self) -> None:
        points = [(0.20, 0.60), (0.04, 0.25), (0.02, 0.32)]
        self.assertAlmostEqual(near_lane_x(points), 0.03)

    def test_trigger_stops_before_visual_reverse(self) -> None:
        state, command = self.step(Backup(), triggered=True, progress_s_m=0.50)
        self.assertEqual(state.phase, "backing")
        self.assertEqual(command.v_mps, 0.0)
        self.assertEqual(command.omega_radps, 0.0)
        self.assertEqual(command.reason, "stop_backup")

        state, command = self.step(
            state, near_x_m=0.04, progress_s_m=0.49, now_s=1.1
        )
        self.assertEqual(command.reason, "visual_backup")
        self.assertLess(command.v_mps, 0.0)
        self.assertGreater(command.omega_radps, 0.0)

    def test_reverse_steering_sign_is_opposite_to_forward(self) -> None:
        self.assertGreater(reverse_omega(0.05, self.cfg), 0.0)
        self.assertLess(reverse_omega(-0.05, self.cfg), 0.0)
        self.assertEqual(reverse_omega(0.0, self.cfg), 0.0)

    def test_reverse_omega_deadband_ignores_small_offset(self) -> None:
        self.assertEqual(reverse_omega(0.02, self.cfg), 0.0)
        self.assertEqual(reverse_omega(-0.029, self.cfg), 0.0)
        self.assertNotEqual(reverse_omega(0.04, self.cfg), 0.0)

    def test_visual_omega_changes_are_rate_limited(self) -> None:
        cfg = BackupConfig(omega_deadband_x_m=0.0, max_omega_step_radps=0.15)
        state, command = self.step(Backup(), triggered=True, progress_s_m=0.90, cfg=cfg)
        self.assertEqual(command.reason, "stop_backup")
        state, command = self.step(state, near_x_m=0.30, progress_s_m=0.89, now_s=1.1, cfg=cfg)
        self.assertEqual(command.reason, "visual_backup")
        self.assertAlmostEqual(command.omega_radps, 0.15)
        state, command = self.step(state, near_x_m=-0.30, progress_s_m=0.88, now_s=1.2, cfg=cfg)
        self.assertAlmostEqual(command.omega_radps, 0.0)
        state, command = self.step(state, near_x_m=-0.30, progress_s_m=0.87, now_s=1.3, cfg=cfg)
        self.assertAlmostEqual(command.omega_radps, -0.15)

    def test_omega_ramp_restarts_after_no_vision_stop(self) -> None:
        cfg = BackupConfig(omega_deadband_x_m=0.0, max_omega_step_radps=0.15)
        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.90, yaw_rad=None, cfg=cfg)
        state, command = self.step(state, near_x_m=0.30, progress_s_m=0.89, now_s=1.1, cfg=cfg)
        self.assertAlmostEqual(command.omega_radps, 0.15)
        state, command = self.step(
            state, near_x_m=None, progress_s_m=0.88, now_s=1.2, yaw_rad=None, cfg=cfg
        )
        self.assertEqual(command.reason, "stop_backup_no_vision")
        state, command = self.step(state, near_x_m=-0.30, progress_s_m=0.87, now_s=1.3, cfg=cfg)
        self.assertEqual(command.reason, "visual_backup")
        self.assertAlmostEqual(command.omega_radps, -0.15)

    def test_heading_backup_omega_is_rate_limited(self) -> None:
        cfg = BackupConfig(max_omega_step_radps=0.15)
        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.90, yaw_rad=0.0, cfg=cfg)
        state, command = self.step(
            state, near_x_m=None, progress_s_m=0.88, now_s=1.1, yaw_rad=-0.06, cfg=cfg
        )
        self.assertEqual(command.reason, "heading_backup")
        self.assertAlmostEqual(command.omega_radps, 0.15)

    def test_odometry_progress_completes_backup(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.55)
        state, command = self.step(
            state, near_x_m=-0.02, progress_s_m=0.0, now_s=3.0
        )
        self.assertEqual(state.phase, "done")
        self.assertEqual(command.reason, "backup_done")
        self.assertEqual(command.v_mps, 0.0)

    def test_trigger_at_entry_needs_no_reverse(self) -> None:
        state, command = self.step(Backup(), triggered=True, progress_s_m=0.0)
        self.assertEqual(state.phase, "done")
        self.assertEqual(command.reason, "backup_done_at_entry")

    def test_short_of_the_junction_still_reverses(self) -> None:
        state, command = self.step(Backup(), triggered=True, progress_s_m=0.10)
        self.assertEqual(state.phase, "backing")
        self.assertEqual(command.reason, "stop_backup")

    def test_missing_vision_without_heading_reference_stops_then_faults(self) -> None:
        state, _ = self.step(Backup(), triggered=True, yaw_rad=None)
        for frame in range(2):
            state, command = self.step(
                state, near_x_m=None, now_s=1.1 + frame * 0.1
            )
            self.assertEqual(state.phase, "backing")
            self.assertEqual(command.reason, "stop_backup_no_vision")
        state, command = self.step(state, near_x_m=None, now_s=1.3)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_road_lost")

    def test_no_vision_reverses_with_heading_until_fifteen_cm(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=.42)
        for t, progress in ((1.1,.42),(1.2,.40),(1.3,.38),(1.4,.36),(1.5,.34),(1.6,.30),(1.7,.271)):
            state, command = self.step(state, near_x_m=None, progress_s_m=progress, now_s=t)
            self.assertEqual(state.phase,"backing")
            self.assertEqual(command.reason,"heading_backup")
            self.assertAlmostEqual(command.v_mps,-.08)
        state, command = self.step(state, near_x_m=None, progress_s_m=.27, now_s=1.8)
        self.assertEqual(state.phase,"fault")
        self.assertEqual(command.reason,"stop_backup_no_vision_distance")
        self.assertEqual(command.v_mps,0)
        self.assertAlmostEqual(state.no_vision_segment_m,.15)

    def test_three_fresh_frames_restore_visual_reverse(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=.42)
        self.step(state, near_x_m=None, progress_s_m=.42, now_s=1.1)
        for t, progress in ((1.2,.41),(1.3,.40)):
            _, command = self.step(state, near_x_m=.02, progress_s_m=progress, now_s=t)
            self.assertEqual(command.reason,"heading_backup")
        _, command = self.step(state, near_x_m=.02, progress_s_m=.39, now_s=1.4)
        self.assertEqual(command.reason,"visual_backup")
        self.assertAlmostEqual(state.no_vision_distance_m,.03)
        self.assertIsNone(state.no_vision_started_s)
        _, command = self.step(state, progress_s_m=.20, now_s=1.5)
        self.assertEqual(command.reason,"visual_backup")

    def test_duplicate_and_stale_frames_do_not_restore_vision(self) -> None:
        state, _ = self.step(Backup(), triggered=True)
        self.step(state, near_x_m=None, now_s=1.1)
        self.step(state, now_s=1.2, frame_captured_s=1.2)
        for t in (1.21,1.22,1.23):
            _, command = self.step(state, now_s=t, frame_captured_s=1.2)
            self.assertEqual(command.reason,"heading_backup")
        self.assertEqual(state.clear_frames,1)
        self.step(state, now_s=1.5, frame_captured_s=1.2)
        self.assertEqual(state.clear_frames,0)

    def test_repeated_vision_loss_does_not_reset_fifteen_cm_budget(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=.42)
        self.step(state, near_x_m=None, progress_s_m=.42, now_s=1.1)
        for t, progress in ((1.2,.38),(1.3,.37),(1.4,.36)):
            self.step(state, progress_s_m=progress, now_s=t)
        self.assertAlmostEqual(state.no_vision_distance_m,.06)
        self.step(state, progress_s_m=.34, now_s=1.5)
        self.step(state, near_x_m=None, progress_s_m=.34, now_s=1.6)
        _, command = self.step(state, near_x_m=None, progress_s_m=.251, now_s=1.7)
        self.assertEqual(command.reason,"heading_backup")
        _, command = self.step(state, near_x_m=None, progress_s_m=.25, now_s=1.8)
        self.assertEqual(command.reason,"stop_backup_no_vision_distance")

    def test_short_edge_stops_at_node_before_fifteen_cm(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=.04)
        self.step(state, near_x_m=None, progress_s_m=.04, now_s=1.1)
        _, command = self.step(state, near_x_m=None, progress_s_m=.01, now_s=1.2)
        self.assertEqual(command.reason,"heading_backup")
        _, command = self.step(state, near_x_m=None, progress_s_m=0, now_s=1.3)
        self.assertEqual(state.phase,"done")
        self.assertEqual(command.reason,"backup_done")

    def test_no_vision_with_no_progress_times_out(self) -> None:
        state, _ = self.step(Backup(), triggered=True)
        self.step(state, near_x_m=None, now_s=1.1)
        _, command = self.step(state, near_x_m=None, now_s=3.11)
        self.assertEqual(state.phase,"fault")
        self.assertEqual(command.reason,"stop_backup_no_vision_timeout")

    def test_stale_odom_cannot_reverse_or_fake_node_arrival(self) -> None:
        for triggered, progress in ((True,.42),(False,.42),(False,0)):
            with self.subTest(triggered=triggered,progress=progress):
                state=Backup()
                if not triggered:
                    self.step(state,triggered=True,progress_s_m=.42)
                _, command=self.step(state,triggered=triggered,progress_s_m=progress,
                                     near_x_m=None,odom_valid=False)
                self.assertEqual(state.phase,"fault")
                self.assertEqual(command.reason,"stop_backup_odom_stale")
                self.assertEqual(command.v_mps,0)

    def test_no_vision_heading_correction_and_deviation_guard(self) -> None:
        state, _ = self.step(Backup(),triggered=True,yaw_rad=math.pi-.01)
        _, command=self.step(state,near_x_m=None,yaw_rad=-math.pi+.01,now_s=1.1)
        self.assertEqual(command.reason,"heading_backup")
        self.assertAlmostEqual(command.omega_radps,-.08)
        _, command=self.step(state,near_x_m=None,yaw_rad=-math.pi+.15,now_s=1.2)
        self.assertEqual(state.phase,"fault")
        self.assertEqual(command.reason,"stop_backup_heading_error")

    def test_invalid_current_heading_stops_no_vision_reverse(self) -> None:
        for yaw in (None,float("nan"),float("inf")):
            with self.subTest(yaw=yaw):
                state, _=self.step(Backup(),triggered=True)
                _, command=self.step(state,near_x_m=None,yaw_rad=yaw,now_s=1.1)
                self.assertEqual(state.phase,"fault")
                self.assertEqual(command.reason,"stop_backup_heading_invalid")

    def test_nan_lane_offset_is_missing_vision(self) -> None:
        state, _=self.step(Backup(),triggered=True)
        _, command=self.step(state,near_x_m=float("nan"),now_s=1.1)
        self.assertEqual(command.reason,"heading_backup")
        self.assertTrue(math.isfinite(command.omega_radps))

    def test_timeout_and_distance_limit_do_not_fake_arrival(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=1.30)
        state, command = self.step(state, progress_s_m=0.10, now_s=2.0)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_distance_limit")

        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.50)
        state, command = self.step(state, progress_s_m=0.40, now_s=6.0)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_timeout")

    def test_near_lane_heading_uses_direction_not_offset(self) -> None:
        parallel = [(0.06, 0.22), (0.06, 0.28), (0.06, 0.34)]
        self.assertAlmostEqual(near_lane_heading(parallel), 0.0)
        slanted = [(0.00, 0.22), (0.04, 0.28), (0.08, 0.34)]
        self.assertGreater(near_lane_heading(slanted), 0.2)
        self.assertIsNone(near_lane_heading([(0.02, 0.22)]))

    def test_forward_lane_heading_ignores_side_opening(self) -> None:
        forward = [(0.0, 0.14), (0.001, 0.18), (0.002, 0.22), (0.003, 0.26)]
        opening_left = [(-0.16, 0.22), (-0.22, 0.26), (-0.28, 0.28)]
        heading = forward_lane_heading(forward + opening_left)
        self.assertIsNotNone(heading)
        self.assertLess(abs(heading), 0.05)
        # 只有开口点时方向未知，绝不能把开口当道路。
        self.assertIsNone(forward_lane_heading(opening_left))

    def test_forward_lane_heading_survives_wall_end_shallow_road(self) -> None:
        wall_end = [(0.0, 0.14), (0.001, 0.16), (0.002, 0.19), (0.002, 0.21)]
        heading = forward_lane_heading(wall_end)
        self.assertIsNotNone(heading)
        self.assertLess(abs(heading), 0.05)

    def test_forward_lane_heading_needs_min_points(self) -> None:
        two_points = [(0.0, 0.15), (0.001, 0.18)]
        self.assertIsNone(forward_lane_heading(two_points))
        self.assertIsNone(forward_lane_heading([]))

    def test_forward_lane_heading_detects_real_skew(self) -> None:
        slanted = [(0.06, 0.15), (0.045, 0.18), (0.03, 0.21), (0.015, 0.24), (0.0, 0.27)]
        heading = forward_lane_heading(slanted)
        self.assertIsNotNone(heading)
        self.assertLess(heading, -0.1)

    def test_idle_preserves_forward_command(self) -> None:
        state, command = self.step(Backup(), triggered=False)
        self.assertEqual(state.phase, "idle")
        self.assertEqual(command, self.follow)


if __name__ == "__main__":
    unittest.main()
