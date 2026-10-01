"""检测框到硬堵塞事件的纯逻辑测试。"""

from __future__ import annotations

import unittest

from vision.obstacle.blockage import HardBlockConfig, HardBlockageJudge
from vision.obstacle.detect import Detection


def detection(
    x1: float = 450,
    y1: float = 260,
    x2: float = 820,
    y2: float = 650,
    score: float = 0.9,
) -> Detection:
    return Detection(0, "obstacle", score, x1, y1, x2, y2)


class HardBlockageJudgeTests(unittest.TestCase):
    shape = (720, 1280, 3)

    def test_three_stable_frames_confirm_and_latch(self) -> None:
        judge = HardBlockageJudge(HardBlockConfig(confirm_frames=3))

        first = judge.update([detection()], self.shape)
        second = judge.update([detection(x1=455, x2=825)], self.shape)
        third = judge.update([detection(x1=460, x2=830)], self.shape)
        latched = judge.update([], self.shape)

        self.assertFalse(first.hard_blocked)
        self.assertEqual(second.stable_frames, 2)
        self.assertTrue(third.hard_blocked)
        self.assertTrue(third.just_confirmed)
        self.assertTrue(latched.hard_blocked)
        self.assertEqual(latched.reason, "latched")

    def test_unstable_boxes_restart_confirmation(self) -> None:
        judge = HardBlockageJudge(HardBlockConfig(confirm_frames=2, min_track_iou=0.3))

        judge.update([detection(x1=300, x2=500)], self.shape)
        moved = judge.update([detection(x1=780, x2=1000)], self.shape)

        self.assertFalse(moved.hard_blocked)
        self.assertEqual(moved.stable_frames, 1)

    def test_filters_irrelevant_detections(self) -> None:
        judge = HardBlockageJudge()

        outside = judge.update([detection(x1=0, x2=100)], self.shape)
        far = judge.update([detection(y1=20, y2=100)], self.shape)
        small = judge.update([detection(x1=630, y1=300, x2=640, y2=310)], self.shape)
        weak = judge.update([detection(score=0.2)], self.shape)

        self.assertEqual(outside.reason, "outside_corridor")
        self.assertEqual(far.reason, "too_far")
        self.assertEqual(small.reason, "too_small")
        self.assertEqual(weak.reason, "low_score")
        self.assertFalse(any(item.hard_blocked for item in (outside, far, small, weak)))

    def test_reset_starts_a_new_edge(self) -> None:
        judge = HardBlockageJudge(HardBlockConfig(confirm_frames=1))
        self.assertTrue(judge.update([detection()], self.shape).hard_blocked)

        judge.reset()
        clear = judge.update([], self.shape)

        self.assertFalse(clear.hard_blocked)
        self.assertEqual(clear.reason, "no_detection")


if __name__ == "__main__":
    unittest.main()

