"""遇障摆正，并按沿边进度发出定距倒车。"""

from __future__ import annotations

import queue
import unittest

from road_follow.backup import (
    Backup,
    BackupConfig,
    EdgeProgress,
    align_omega,
    backup_distance_mm,
    near_lane_x,
    step_backup,
)
from road_follow.control import VelocityCommand


class BackupTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = BackupConfig(align_frames=2, align_timeout_s=1.0)
        self.notes: queue.Queue[str] = queue.Queue()
        self.sent: list[str] = []
        self.follow = VelocityCommand(0.10, 0.0, "follow")

    def send(self, line: str) -> bool:
        self.sent.append(line)
        return True

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

    def test_distance_is_capped_at_one_cell(self) -> None:
        self.assertEqual(backup_distance_mm(0.63, BackupConfig()), 630)
        self.assertEqual(backup_distance_mm(0.90, BackupConfig()), 700)
        self.assertEqual(backup_distance_mm(0.0, BackupConfig()), 0)

    def test_align_turns_toward_the_near_centerline_then_backs_up(self) -> None:
        state = Backup()
        state, command = step_backup(
            state, True, 0.08, 0.42, self.follow, self.notes, self.send, 1.0, self.cfg
        )
        self.assertEqual(state.phase, "align")
        self.assertEqual(state.distance_mm, 420)
        self.assertEqual(command.reason, "stop_backup")

        state, command = step_backup(
            state, False, 0.08, 0.42, self.follow, self.notes, self.send, 1.1, self.cfg
        )
        self.assertEqual(command.v_mps, 0.0)
        self.assertLess(command.omega_radps, 0.0)
        self.assertEqual(command.reason, "backup_align")
        self.assertEqual(self.sent, [])

        state, command = step_backup(
            state, False, 0.01, 0.42, self.follow, self.notes, self.send, 1.2, self.cfg
        )
        self.assertEqual(state.phase, "align")
        state, command = step_backup(
            state, False, -0.01, 0.42, self.follow, self.notes, self.send, 1.3, self.cfg
        )
        self.assertEqual(state.phase, "backing")
        self.assertEqual(self.sent, ["backward 420 100"])
        self.assertEqual(command.reason, "backup_backing")

        self.notes.put("BACKWARD_DONE")
        state, command = step_backup(
            state, False, None, 0.42, self.follow, self.notes, self.send, 2.0, self.cfg
        )
        self.assertEqual(state.phase, "done")
        self.assertEqual(command.reason, "backup_done")

    def test_right_of_center_commands_a_right_turn(self) -> None:
        self.assertLess(align_omega(0.05, BackupConfig()), 0.0)
        self.assertGreater(align_omega(-0.05, BackupConfig()), 0.0)

    def test_blocked_near_road_reverses_with_the_distance_already_driven(self) -> None:
        state = Backup()
        state, _command = step_backup(
            state, True, -0.03, 0.35, self.follow, self.notes, self.send, 1.0, self.cfg
        )
        state, command = step_backup(
            state, False, None, 0.35, self.follow, self.notes, self.send, 1.1, self.cfg
        )
        self.assertEqual(state.phase, "backing")
        self.assertEqual(self.sent, ["backward 350 100"])
        self.assertEqual(command.reason, "backup_backing")

    def test_failure_and_missing_distance_stop(self) -> None:
        state = Backup()
        state, command = step_backup(
            state, True, 0.0, 0.0, self.follow, self.notes, self.send, 1.0, self.cfg
        )
        state, command = step_backup(
            state, False, 0.0, 0.0, self.follow, self.notes, self.send, 1.1, self.cfg
        )
        state, command = step_backup(
            state, False, 0.0, 0.0, self.follow, self.notes, self.send, 1.2, self.cfg
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_no_distance")
        self.assertEqual(self.sent, [])

        state = Backup(phase="backing")
        self.notes.put("BACKWARD_FAIL")
        state, command = step_backup(
            state, False, None, 0.4, self.follow, self.notes, self.send, 3.0, self.cfg
        )
        self.assertEqual(state.phase, "fault")
        self.assertEqual(command.reason, "stop_backup_fail")


if __name__ == "__main__":
    unittest.main()
