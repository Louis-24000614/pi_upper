"""前墙横缝：左边墙只剩车头一截时判左转；直路和还没到头时不转。"""

from __future__ import annotations

import queue
import unittest

import cv2
import numpy as np

from road_follow.control import VelocityCommand
from road_follow.wall_end import WallEnd, WallTurn, read_wall_end, step_wall_turn


def _scene(*, left_stub: bool, right_wall: bool, front: bool = True) -> np.ndarray:
    image = np.full((720, 1280, 3), 230, np.uint8)
    if front:
        cv2.line(image, (30, 280), (1250, 280), (40, 40, 40), 3)
    if left_stub:
        cv2.line(image, (70, 540), (160, 710), (40, 40, 40), 3)
    if right_wall:
        cv2.line(image, (1120, 292), (1220, 700), (40, 40, 40), 3)
    return image


class WallEndTest(unittest.TestCase):
    def test_left_stub_means_turn_left(self) -> None:
        reading = read_wall_end(_scene(left_stub=True, right_wall=True))
        self.assertTrue(reading.arrived)
        self.assertEqual(reading.side, "left")

    def test_right_stub_means_turn_right(self) -> None:
        image = _scene(left_stub=False, right_wall=False)
        cv2.line(image, (80, 292), (180, 700), (40, 40, 40), 3)
        cv2.line(image, (1120, 540), (1220, 710), (40, 40, 40), 3)
        reading = read_wall_end(image)
        self.assertTrue(reading.arrived)
        self.assertEqual(reading.side, "right")

    def test_no_front_wall_keeps_following(self) -> None:
        reading = read_wall_end(_scene(left_stub=True, right_wall=True, front=False))
        self.assertFalse(reading.arrived)
        self.assertEqual(reading.side, "none")

    def test_creep_then_turn_left(self) -> None:
        state = WallTurn()
        sent: list[str] = []

        def send(line: str) -> bool:
            sent.append(line)
            return True

        notes: queue.Queue[str] = queue.Queue()
        follow = VelocityCommand(0.10, 0.0, "follow")
        wall = WallEnd(True, "left")
        for _ in range(2):
            state, command = step_wall_turn(state, wall, follow, 0.1, notes, send)
            self.assertEqual(state.phase, "follow")
            self.assertEqual(command.reason, "follow")
        state, command = step_wall_turn(state, wall, follow, 0.1, notes, send)
        self.assertEqual(state.phase, "creep")
        self.assertEqual(command.reason, "creep")
        self.assertAlmostEqual(command.v_mps, 0.08, places=2)
        state.creep_m = 0.14
        state, command = step_wall_turn(state, wall, follow, 0.2, notes, send)
        self.assertEqual(state.phase, "turning")
        self.assertEqual(sent, ["0.000 0.000", "turn left"])
        self.assertEqual(command.reason, "turn_left")


if __name__ == "__main__":
    unittest.main()
