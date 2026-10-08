"""单侧调试不转舵机、不伪造第二侧；离线合成帧。"""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np

from road_follow.inspection_config import Settings
from road_follow.inspection_debug import DisabledServo, SingleSideDebug
from road_follow.inspection_io import FrameHub
from road_follow.tests.test_inspection import Pool, accepted


class DebugTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name)/"settings.json"
        path.write_bytes((Path(__file__).with_name("fixtures")/"culvert_inspection.json").read_bytes())
        self.settings, self.hub, self.servo = Settings(path), FrameHub(), DisabledServo()
        recognizer = Mock()
        recognizer.recognize.return_value = accepted()
        self.task = SingleSideDebug(self.settings, self.hub, self.servo, recognizer, pool=Pool())
        self.addCleanup(self.task.close)
        self.task.start(0, None)

    def test_confirm_only_current_side_and_restart_without_turning(self):
        for now in (0, 1.3, 2.6):
            self.hub.publish(np.zeros((20, 30, 3), np.uint8), now)
            self.task.step(now)
            outcome = self.task.step(now+.01)
        self.assertEqual(outcome, "done")
        self.assertEqual(self.task.result["status"], "debug_confirmed")
        self.assertEqual(len(self.task.sides), 1)
        self.assertTrue(self.task.result["debug_only"])
        state = self.task.status()
        self.assertFalse(state["pwm_enabled"])
        self.assertFalse(state["drive_enabled"])
        self.assertIn("PWM 禁用", state["phase"])
        self.task.start(10, None)
        self.assertEqual(self.task.side, "A")
        self.assertEqual(self.task.count, 0)

    def test_timeout_is_debug_result_without_second_side(self):
        self.assertEqual(self.task.step(45), "done")
        self.assertEqual(self.task.result["status"], "debug_unconfirmed")
        self.assertEqual(len(self.task.sides), 1)
        self.assertEqual(self.task.side, "A")

    def test_disabled_servo_rejects_any_rotation(self):
        with self.assertRaises(RuntimeError):
            self.servo.request(180)


if __name__ == "__main__":
    unittest.main()
