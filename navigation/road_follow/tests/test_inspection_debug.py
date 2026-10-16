"""Complete debug task tests: synthetic camera, fake PWM and mocked speech process."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from road_follow.inspection_config import Settings
from road_follow.inspection_debug import FullInspectionDebug, DebugSpeechTransport
from road_follow.inspection_io import FrameHub
from road_follow.tests.test_inspection import Pool, Servo
from road_follow.tests.test_inspection_pairs import candidate


class DebugTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name)/"settings.json"
        path.write_bytes((Path(__file__).with_name("fixtures")/"culvert_inspection.json").read_bytes())
        self.settings, self.hub, self.servo = Settings(path), FrameHub(), Servo()
        self.recognizer = Mock()
        self.transport = Mock()
        self.transport.send.return_value = True
        self.factory = Mock(return_value=self.servo)
        self.task = FullInspectionDebug(self.settings, self.hub, self.recognizer,
                                        servo_factory=self.factory, transport=self.transport, pool=Pool())
        self.addCleanup(self.task.close)

    def begin(self, now=0):
        self.task.request_start()
        self.task.step(now)
        self.assertEqual(self.task.phase, "settling")
        self.task.step(now+3.01)
        self.assertEqual(self.task.phase, "recognizing")

    def confirm(self, identity, now):
        self.recognizer.recognize.return_value = candidate(identity)
        for delta in (0, .05, .10):
            t=now+delta
            self.hub.publish(np.zeros((20, 30, 3), np.uint8), t)
            self.task.step(t)
            self.task.step(t+.001)

    def test_waits_for_button_without_creating_servo_or_sending_speech(self):
        self.task.step(0)
        self.assertEqual(self.task.phase, "idle")
        self.factory.assert_not_called()
        self.transport.send.assert_not_called()
        self.task.request_start()
        self.factory.assert_not_called()
        self.assertEqual(self.servo.angles, [])

    def test_one_complete_two_side_task_with_speech_then_waits_for_next_button(self):
        self.begin()
        self.assertEqual(self.servo.angles, [0])
        self.confirm("suspect_08", 3.1)
        self.assertEqual(self.servo.angles, [0, 180])
        self.task.step(3.22)
        self.task.step(6.23)
        self.confirm("knife_07", 6.3)
        self.assertEqual(self.task.phase, "done")
        self.assertEqual(len(self.task.sides), 2)
        self.assertEqual(len(self.transport.send.call_args_list), 2)
        self.assertEqual(self.task.step(100), "done")
        self.assertEqual(self.task.run_number, 1)
        self.assertEqual(self.servo.angles, [0, 180])
        self.assertFalse(self.task.status()["drive_enabled"])

    def test_restart_during_turning_settling_and_recognizing_starts_at_initial_side(self):
        for phase in ("turning", "settling", "recognizing"):
            with self.subTest(phase=phase):
                self.task.phase = phase
                self.task.sides = [{"status": "old"}]
                self.task.pending_announcement = (100, {"identity": "knife_07"})
                previous = self.task.run_number
                self.task.request_start()
                self.task.step(10+previous)
                self.assertEqual(self.task.run_number, previous+1)
                self.assertEqual(self.task.phase, "settling")
                self.assertEqual(self.task.sides, [])
                self.assertIsNone(self.task.pending_announcement)
                self.assertEqual(self.servo.angles[-1], 0)
                self.transport.reset.assert_called()

    def test_restart_after_done_uses_new_scope_so_both_sides_can_speak_again(self):
        self.begin()
        self.confirm("suspect_08", 3.1)
        self.task.step(3.22)
        self.task.step(6.23)
        self.confirm("knife_07", 6.3)
        old_scope = self.task.target.edge_id
        self.begin(10)
        self.confirm("suspect_08", 13.1)
        self.task.step(13.22)
        self.task.step(16.23)
        self.confirm("knife_07", 16.3)
        self.assertNotEqual(self.task.target.edge_id, old_scope)
        self.assertEqual(self.transport.send.call_count, 4)

    def test_pair_mode_on_debug_uses_same_delay_and_clears_pending_on_restart(self):
        self.task.pair_fallback = True
        self.begin()
        self.task.step(self.task.deadline_s)
        self.task.step(18.02)
        self.task.step(21.03)
        self.confirm("suspect_08", 21.1)
        self.assertEqual(self.task.phase, "announcing")
        self.assertEqual(self.transport.send.call_count, 1)
        self.task.request_start()
        self.task.step(22)
        self.assertIsNone(self.task.pending_announcement)
        self.assertEqual(self.transport.send.call_count, 1)

    def test_servo_failure_retains_web_state_and_allows_button_retry_with_new_bridge(self):
        self.servo.state = "failed"
        self.task.request_start()
        self.assertEqual(self.task.step(0), "failed")
        self.assertTrue(self.servo.closed)
        self.assertEqual(self.task.phase, "failed")
        replacement = Servo()
        self.factory.return_value = replacement
        self.task.request_start()
        self.task.step(1)
        self.assertEqual(replacement.angles, [0])
        self.assertEqual(self.task.phase, "settling")

    def test_closing_idle_task_closes_transport_without_initializing_hardware(self):
        self.task.close()
        self.factory.assert_not_called()
        self.transport.close.assert_called()
        with self.assertRaises(ValueError):
            self.task.request_start()


    def test_debug_cli_passes_pair_flag_and_preserves_original_arguments(self):
        from road_follow.inspection_debug import main
        stop = Mock()
        stop.wait.side_effect = [False, True]
        task = Mock()
        task.phase = "idle"
        with patch("road_follow.inspection_debug.create_debug", return_value=task) as create, patch(
                "road_follow.inspection_debug.threading.Event", return_value=stop), patch(
                "road_follow.inspection_debug.signal.signal"), patch("builtins.print"):
            result = main(["--config", str(self.settings.path), "--camera", "/fake/camera", "-b"])
        self.assertEqual(result, 0)
        self.assertTrue(create.call_args.kwargs["pair_fallback"])
        self.assertEqual(create.call_args.args[1], "/fake/camera")
        task.close.assert_called_once()


class TransportTest(unittest.TestCase):
    def setUp(self):
        self.transport = DebugSpeechTransport(Path("/fake/uart_speak"),
                                              {"device": "/fake/tty", "baud": 921600})
        self.addCleanup(self.transport.close)
        self.speech = Mock()

    def test_queued_speech_launches_only_uart_speak_and_remains_asynchronous(self):
        process=Mock()
        process.poll.return_value=None
        process.communicate.return_value=("ACK_OK\n", None)
        process.returncode=0
        with patch("road_follow.inspection_debug.subprocess.Popen", return_value=process) as popen:
            self.assertTrue(self.transport.send("speech event-1 20"))
            popen.assert_not_called()
            self.transport.poll(0, self.speech)
            self.assertEqual(popen.call_args.args[0],
                             ["/fake/uart_speak", "20", "--device", "/fake/tty", "--baud", "921600"])
            self.transport.poll(.1, self.speech)
            self.speech.handle_note.assert_not_called()
            process.poll.return_value=0
            self.transport.poll(.2, self.speech)
            self.speech.handle_note.assert_called_once_with("SPEECH_RESULT event-1 20 ACK_OK")

    def test_reset_drops_queued_speech_and_terminates_inflight_process(self):
        process=Mock()
        process.poll.return_value=None
        process.communicate.return_value=("", None)
        with patch("road_follow.inspection_debug.subprocess.Popen", return_value=process) as popen:
            self.transport.send("speech old-1 20")
            self.transport.send("speech old-2 29")
            self.transport.poll(0,self.speech)
            self.transport.reset()
            process.terminate.assert_called_once()
            self.transport.poll(1,self.speech)
            self.assertEqual(popen.call_count,1)
            self.assertIsNone(self.transport.process)
            self.assertEqual(len(self.transport.queue),0)

    def test_invalid_or_motion_commands_never_enter_speech_queue(self):
        for line in ("arm", "vel 0 0", "speech e 0", "speech e 33", "speech e bad"):
            self.assertFalse(self.transport.send(line))
        self.assertEqual(len(self.transport.queue),0)

    def test_timeout_is_reported_and_queue_can_continue(self):
        process=Mock()
        process.poll.return_value=None
        process.communicate.return_value=("", None)
        with patch("road_follow.inspection_debug.subprocess.Popen", return_value=process):
            self.transport.send("speech event-1 20")
            self.transport.poll(0,self.speech)
            self.transport.poll(7,self.speech)
        self.speech.handle_note.assert_called_once_with("SPEECH_RESULT event-1 20 ACK_TIMEOUT")

    def test_process_start_failure_is_reported(self):
        with patch("road_follow.inspection_debug.subprocess.Popen", side_effect=OSError("test")):
            self.transport.send("speech event-1 20")
            self.transport.poll(0,self.speech)
        self.speech.handle_note.assert_called_once_with("SPEECH_RESULT event-1 20 FAILED")


if __name__ == "__main__":
    unittest.main()
