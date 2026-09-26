"""遇障后的视觉闭环倒车。"""

from __future__ import annotations

import unittest

from road_follow.backup import (
    Backup,
    BackupConfig,
    EdgeProgress,
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
    ) -> tuple[Backup, VelocityCommand]:
        return step_backup(
            state,
            triggered,
            near_x_m,
            progress_s_m,
            self.follow,
            now_s,
            self.cfg,
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

    def test_odometry_progress_completes_backup(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.55)
        state, command = self.step(
            state, near_x_m=-0.02, progress_s_m=0.14, now_s=3.0
        )
        self.assertEqual(state.phase, "done")
        self.assertEqual(command.reason, "backup_done")
        self.assertEqual(command.v_mps, 0.0)

    def test_trigger_at_entry_needs_no_reverse(self) -> None:
        state, command = self.step(Backup(), triggered=True, progress_s_m=0.10)
        self.assertEqual(state.phase, "done")
        self.assertEqual(command.reason, "backup_done_at_entry")

    def test_missing_vision_stops_then_faults(self) -> None:
        state, _ = self.step(Backup(), triggered=True)
        for frame in range(2):
            state, command = self.step(
                state, near_x_m=None, now_s=1.1 + frame * 0.1
            )
            self.assertEqual(state.phase, "backing")
            self.assertEqual(command.reason, "stop_backup_no_vision")
        state, command = self.step(state, near_x_m=None, now_s=1.3)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_road_lost")

    def test_timeout_and_distance_limit_do_not_fake_arrival(self) -> None:
        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.90)
        state, command = self.step(state, progress_s_m=0.19, now_s=2.0)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_distance_limit")

        state, _ = self.step(Backup(), triggered=True, progress_s_m=0.50)
        state, command = self.step(state, progress_s_m=0.40, now_s=6.0)
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_timeout")

    def test_idle_preserves_forward_command(self) -> None:
        state, command = self.step(Backup(), triggered=False)
        self.assertEqual(state.phase, "idle")
        self.assertEqual(command, self.follow)


if __name__ == "__main__":
    unittest.main()
