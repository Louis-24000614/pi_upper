"""转弯后原地修正的离线回归，不连接相机或底盘。"""

import unittest
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from road_follow.post_turn import (
    PostTurnAlignment,
    PostTurnConfig,
    post_turn_config_from_mapping,
    turn_completed,
)


class PostTurnTest(unittest.TestCase):
    def setUp(self):
        self.state = PostTurnAlignment()
        self.cfg = PostTurnConfig()

    def step(self, heading=0.1, captured=10.1, now=10.2, safe=True):
        return self.state.step(
            now_s=now, frame_captured_s=captured,
            lane_heading_rad=heading, visual_safe=safe, cfg=self.cfg,
        )

    def test_inactive_and_one_second_window(self):
        self.assertIsNone(self.step())
        self.state.start(10.0)
        self.assertEqual(self.step().v_mps, 0.0)
        self.assertIsNone(self.step(now=11.0))
        self.assertIsNone(self.step(now=11.1))

    def test_direction_gain_and_limit(self):
        self.state.start(10.0)
        self.assertAlmostEqual(self.step(heading=0.1).omega_radps, -0.2)
        self.assertAlmostEqual(self.step(heading=-0.1).omega_radps, 0.2)
        self.assertEqual(self.step(heading=1.0).omega_radps, -0.5)
        self.assertEqual(self.step(heading=-1.0).omega_radps, 0.5)

    def test_aligned_waits_without_rotation(self):
        self.state.start(10.0)
        command = self.step(heading=0.02)
        self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))
        self.assertEqual(command.reason, "post_turn_align")

    def test_missing_stale_and_pre_turn_measurements_do_not_rotate(self):
        self.state.start(10.0)
        for kwargs in ({"heading": None}, {"heading": float("nan")},
                       {"captured": 9.99}, {"captured": 10.0},
                       {"captured": 10.01, "now": 10.5},
                       {"captured": 10.3, "now": 10.2}, {"safe": False}):
            with self.subTest(kwargs=kwargs):
                command = self.step(**kwargs)
                self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))

    def test_only_final_successful_turn_arms_alignment(self):
        for phase in ("reacquire", "follow", "done", "idle"):
            self.assertTrue(turn_completed(("turning",), [SimpleNamespace(phase=phase)]))
        for before, after in (("turning", "turning"), ("turning", "fault"),
                              ("follow", "reacquire"), ("stopped", "follow")):
            self.assertFalse(turn_completed((before,), [SimpleNamespace(phase=after)]))

    def test_config_keeps_stationary_and_moving_gains_separate(self):
        cfg = post_turn_config_from_mapping({"follow": {"steering_gain": 1.5},
                                            "post_turn": {"gain": 2.0, "duration_s": 1.0}})
        self.assertEqual((cfg.gain, cfg.duration_s), (2.0, 1.0))


class PostTurnMainTest(unittest.TestCase):
    def test_turn_completion_pauses_navigation_and_sends_stationary_correction(self):
        import road_follow.__main__ as entry
        from road_follow.control import VelocityCommand
        from road_follow.parallel_segment import SegmentedFrame

        lines = []
        sink = SimpleNamespace(write=lines.append, flush=lambda: None, close=lambda: None)
        proc = SimpleNamespace(stdin=sink, stdout=[], poll=lambda: None, wait=lambda timeout: 0)
        capture = SimpleNamespace(release=lambda: None)
        stream = Mock()
        stream.read.side_effect = lambda *_: SegmentedFrame(
            0, 0, time.monotonic(), np.zeros((64, 64, 3), np.uint8),
            np.zeros((64, 64), np.uint8), .01,
        )
        diag = SimpleNamespace(
            forward_depth_m=.4, lane_heading_rad=.1, bev_road_pixels=300,
            output_points=10, near_x_m=.02, centerline_points=(),
        )
        turn = entry.RfidTurn(phase="turning")

        def finish_turn(state, *args):
            state.phase = "reacquire"
            return state, VelocityCommand(0, 0, "rfid_reacquire")

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config, uart = root / "config.yaml", root / "fake_uart"
            config.write_text("{}\n")
            uart.touch()
            with patch.object(entry, "OrderedSegmentStream", return_value=stream), \
                    patch.object(entry, "_open_camera", return_value=capture), \
                    patch.object(entry.subprocess, "Popen", return_value=proc), \
                    patch.object(entry.signal, "signal"), patch.object(entry, "_event"), \
                    patch.object(entry, "RfidTurn", return_value=turn), \
                    patch.object(entry, "_junction_read", return_value=("unknown", None)), \
                    patch.object(entry, "command_from_mask_with_diagnostics",
                                 return_value=(VelocityCommand(.08, .3, "follow"), diag)), \
                    patch.object(entry, "step_rfid_turn", side_effect=finish_turn) as advance:
                result = entry.main([
                    "--config", str(config), "--model", str(root / "fake.rknn"),
                    "--drive", "--uart-bin", str(uart), "--turn-at-rfid", "right",
                    "--npu-contexts", "3", "--frames", "3",
                ])
        self.assertEqual(result, 0)
        self.assertEqual(advance.call_count, 1)
        self.assertEqual(lines, ["0.000 0.000\n", "0.000 -0.200\n", "0.000 -0.200\n", "0 0\n"])
        stream.invalidate.assert_called_once()


if __name__ == "__main__":
    unittest.main()
