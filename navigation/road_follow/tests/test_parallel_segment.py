"""相机复用内存、模型异常与来源切换的回归，不使用真实 NPU。"""
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
from road_follow.parallel_segment import OrderedSegmentStream, CameraReadError, SegmentedFrame


class Capture:
    def __init__(self):
        self.buffer = np.zeros((8,8,3), np.uint8)
        self.count = 0
        self.failed = False

    def read(self):
        if self.failed:
            return False, None
        self.count += 1
        self.buffer.fill(self.count)
        return True, self.buffer


class Segment:
    def __init__(self, model, core_mask, **kwargs):
        self.closed = False

    def mask(self, image):
        time.sleep(.01 if int(image[0,0,0]) % 3 == 1 else .003)
        return image[:,:,0].copy()

    def close(self):
        self.closed = True


class ParallelSegmentTest(unittest.TestCase):
    def test_order_identity_and_capture_memory_ownership(self):
        stream = OrderedSegmentStream(Path("fake.rknn"), factory=Segment)
        capture = Capture()
        retained = []
        try:
            for i in range(9):
                result = stream.read(capture)
                self.assertEqual(result.sequence, i)
                self.assertEqual(result.source, 0)
                self.assertTrue(np.all(result.image == i+1))
                self.assertTrue(np.all(result.mask == i+1))
                retained.append(result)
            for i, result in enumerate(retained):
                self.assertTrue(np.all(result.image == i+1))
                self.assertTrue(np.all(result.mask == i+1))
        finally:
            stream.close()
        self.assertTrue(all(s.closed for s in stream.segments))


class MainFrameGateTest(unittest.TestCase):
    def test_turn_completion_requires_new_generation_evidence(self):
        from unittest.mock import Mock
        from road_follow.__main__ import _invalidate_turn_frames
        stream, smoother = Mock(), Mock()
        states = [SimpleNamespace(phase="follow", clear=2),
                  SimpleNamespace(phase="follow", clear=3),
                  SimpleNamespace(phase="reacquire", clear=1)]
        self.assertTrue(_invalidate_turn_frames(("follow", "turning", "turning"),
                                                states, stream, smoother))
        self.assertEqual([(s.phase,s.clear) for s in states],
                         [("follow",2), ("reacquire",0), ("reacquire",0)])
        stream.invalidate.assert_called_once()
        smoother.reset.assert_called_once()

    def check_main(self, active=False):
        import road_follow.__main__ as entry
        from road_follow.control import VelocityCommand

        class Sink:
            def __init__(self):
                self.lines = []
            def write(self, value):
                self.lines.append(value)
            def flush(self):
                pass
            def close(self):
                pass

        class Stream:
            def __init__(self):
                self.count = 0
                self.closed = False
            def warmup(self, image):
                pass
            def read(self, capture, on_capture):
                sequence = self.count
                self.count += 1
                image = np.zeros((64,64,3), np.uint8)
                return SegmentedFrame(sequence, 0,
                    time.monotonic()-(.3 if sequence == 0 else 0), image,
                    np.zeros((64,64), np.uint8), .01)
            def close(self):
                self.closed = True

        stream, sink = Stream(), Sink()
        proc = SimpleNamespace(stdin=sink, stdout=[], poll=lambda: None, wait=lambda timeout: 0)
        capture = SimpleNamespace(release=lambda: None)
        original_command = entry.command_from_mask_with_diagnostics
        def command(*args, **kwargs):
            _, diag = original_command(*args, **kwargs)
            return VelocityCommand(.1, .2, "follow"), diag
        original_entrance = entry.EntranceDeparture
        def entrance():
            state = original_entrance()
            if active:
                state.phase = "forward"
            return state

        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            config, uart = root/"config.yaml", root/"fake_uart"
            config.write_text("{}\n")
            uart.write_text("")
            with patch.object(entry, "OrderedSegmentStream", return_value=stream), \
                    patch.object(entry, "_open_camera", return_value=capture), \
                    patch.object(entry.subprocess, "Popen", return_value=proc), \
                    patch.object(entry.signal, "signal"), patch.object(entry, "_event"), \
                    patch.object(entry, "EntranceDeparture", side_effect=entrance), \
                    patch.object(entry, "command_from_mask_with_diagnostics", side_effect=command) as nav:
                result = entry.main(["--model", str(root/"fake.rknn"), "--config", str(config),
                    "--npu-contexts", "3", "--frames", "1" if active else "2",
                    "--drive", "--uart-bin", str(uart)])
                self.assertEqual(result, 0)
                self.assertEqual(nav.call_count, 0 if active else 1)
        self.assertTrue(stream.closed)
        return sink.lines

    def test_expired_frame_does_not_advance_navigation_and_sends_zero(self):
        self.assertEqual(self.check_main(), ["0.000 0.000\n", "0.100 0.200\n", "0 0\n"])

    def test_expired_frame_does_not_interrupt_finite_action(self):
        # 结束程序时仍有一次停车；运行中的过期帧不能插入 CMD_VEL 覆盖有限动作。
        self.assertEqual(self.check_main(active=True), ["0 0\n"])


class ParallelFaultTest(unittest.TestCase):
    def test_source_switch_discards_inflight_old_frames(self):
        stream = OrderedSegmentStream(Path("fake.rknn"), factory=Segment)
        capture = Capture()
        try:
            first = stream.read(capture)
            stream.invalidate()
            new = stream.read(capture)
            self.assertEqual(new.source, 1)
            self.assertGreater(new.sequence, first.sequence+1)
            self.assertEqual(int(new.mask[0,0]), new.sequence+1)
        finally:
            stream.close()

    def test_camera_failure_is_distinct_from_inference_failure(self):
        stream = OrderedSegmentStream(Path("fake.rknn"), factory=Segment)
        capture = Capture()
        capture.failed = True
        try:
            with self.assertRaises(CameraReadError):
                stream.read(capture)
        finally:
            stream.close()

        class Failed(Segment):
            def mask(self, image):
                raise ValueError("injected model failure")
        stream = OrderedSegmentStream(Path("fake.rknn"), factory=Failed)
        try:
            with self.assertRaisesRegex(ValueError, "injected model failure"):
                stream.read(Capture())
        finally:
            stream.close()
