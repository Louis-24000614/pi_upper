"""用事件控制读帧/推理竞争，验证最新帧策略的所有权和异常边界。"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeoutError
import queue
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from road_follow.latest_segment import LatestSegmentStream
from road_follow.parallel_segment import CameraReadError
from road_follow.tests import test_parallel_segment as gate_tests


def wait_for(predicate):
    deadline = time.monotonic()+3
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("事件等待超时")
        time.sleep(.001)


class Capture:
    def __init__(self):
        self.inputs = queue.Queue()
        self.buffer = np.zeros((8,8,3), np.uint8)
        self.released = False
        self.reading = threading.Event()
        self.calls = 0

    def read(self):
        self.calls += 1
        self.reading.set()
        item = self.inputs.get()
        if item is None:
            return False, None
        if isinstance(item, BaseException):
            raise item
        self.buffer.fill(item)
        return True, self.buffer

    def stop_reading(self):
        self.inputs.put(None)

    def release(self):
        self.released = True


class Segment:
    def __init__(self, model, core_mask, **kwargs):
        self.closed = False
        self.block = threading.Event()
        self.block.set()
        self.began = threading.Event()

    def mask(self, image):
        self.began.set()
        if not self.block.wait(3):
            raise AssertionError("推理阻塞未解除")
        return image[:,:,0].copy()

    def close(self):
        self.closed = True


class LatestSegmentTest(unittest.TestCase):
    def make_stream(self, cores=(1,), result_order="completion"):
        capture = Capture()
        stream = LatestSegmentStream(Path("fake.rknn"), cores, factory=Segment,
                                     result_order=result_order)
        self.addCleanup(stream.close)
        stream.start(capture)
        return stream, capture

    def test_single_waiting_slot_and_running_frame_not_replaced(self):
        stream, capture = self.make_stream()
        segment = stream.segments[0]
        segment.block.clear()
        capture.inputs.put(1)
        self.assertTrue(segment.began.wait(3))
        for value in range(2, 11):
            capture.inputs.put(value)
        wait_for(lambda: stream.stats['captured'] == 10)
        self.assertEqual(stream.stats['started'], 1)
        self.assertEqual(stream.stats['replaced_input'], 8)
        segment.block.set()
        first, second = stream.read(capture), stream.read(capture)
        self.assertEqual((first.sequence, second.sequence), (0, 9))
        np.testing.assert_array_equal(first.image, np.ones((8,8,3)))
        np.testing.assert_array_equal(first.mask, np.ones((8,8)))
        self.assertTrue(np.all(second.mask == 10))
        self.assertEqual(stream.stats['max_waiting'], 1)
        self.assertEqual(stream.stats['max_results'], 1)
        stream.close()
        self.assertTrue(capture.released)
        self.assertTrue(segment.closed)

    def test_reader_and_idle_workers_start_without_consumer(self):
        stream, capture = self.make_stream((1,2,4))
        for segment in stream.segments:
            segment.block.clear()
        for value in range(1,4):
            capture.inputs.put(value)
            wait_for(lambda: stream.stats['started'] == value)
        self.assertTrue(all(segment.began.is_set() for segment in stream.segments))
        self.assertEqual(stream.stats['started'], 3)
        for segment in stream.segments:
            segment.block.set()
        wait_for(lambda: stream.stats['completed'] == 3)
        self.assertLessEqual(stream.stats['max_results'], 3)
        # 下游不消费时每个模型只保留一份结果，不能无界推理/积压结果。
        capture.inputs.put(4)
        wait_for(lambda: stream.stats['captured'] == 4)
        self.assertEqual(stream.stats['started'], 3)

    def test_late_completed_frame_cannot_move_navigation_backwards(self):
        stream, capture = self.make_stream((1,2))
        for segment in stream.segments:
            segment.block.clear()
        capture.inputs.put(1)
        wait_for(lambda: stream.stats['started'] == 1)
        old = next(segment for segment in stream.segments if segment.began.is_set())
        capture.inputs.put(2)
        wait_for(lambda: stream.stats['started'] == 2)
        new = next(segment for segment in stream.segments if segment is not old)
        new.block.set()
        self.assertEqual(stream.read(capture).sequence, 1)
        old.block.set()
        wait_for(lambda: stream.stats['completed'] == 2)
        capture.inputs.put(3)
        self.assertEqual(stream.read(capture).sequence, 2)
        self.assertEqual(stream.stats['out_of_order'], 1)

    def test_capture_order_waits_for_running_older_frame_without_discard(self):
        stream, capture = self.make_stream((1,2), result_order="capture")
        for segment in stream.segments:
            segment.block.clear()
        capture.inputs.put(1)
        wait_for(lambda: stream.stats['started'] == 1)
        old = next(segment for segment in stream.segments if segment.began.is_set())
        capture.inputs.put(2)
        wait_for(lambda: stream.stats['started'] == 2)
        new = next(segment for segment in stream.segments if segment is not old)
        new.block.set()
        wait_for(lambda: stream.stats['completed'] == 1)
        # 快帧已经完成仍不能先交给导航；用受控慢帧验证等待，不靠真实 NPU 耗时。
        entered = threading.Event()
        def consume():
            entered.set()
            return stream.read(capture)
        with ThreadPoolExecutor(max_workers=1) as consumer:
            future = consumer.submit(consume)
            try:
                self.assertTrue(entered.wait(3))
                with self.assertRaises(FutureTimeoutError):
                    future.result(timeout=.03)
            finally:
                old.block.set()
            first = future.result(timeout=3)
        second = stream.read(capture)
        self.assertEqual((first.sequence, second.sequence), (0,1))
        self.assertEqual(stream.stats['out_of_order'], 0)
        self.assertEqual(stream.stats['max_results'], 2)
        np.testing.assert_array_equal(first.mask, np.ones((8,8)))
        np.testing.assert_array_equal(second.mask, np.full((8,8),2))

    def test_capture_order_skips_replaced_sequence_numbers(self):
        stream, capture = self.make_stream(result_order="capture")
        segment = stream.segments[0]
        segment.block.clear()
        capture.inputs.put(1)
        self.assertTrue(segment.began.wait(3))
        for value in range(2,11):
            capture.inputs.put(value)
        wait_for(lambda: stream.stats['captured'] == 10)
        segment.block.set()
        self.assertEqual(stream.read(capture).sequence, 0)
        # 单帧替换会造成输入序号有空洞，排序只等实际开工的帧，不能永久等 1..8。
        self.assertEqual(stream.read(capture).sequence, 9)
        self.assertEqual(stream.stats['out_of_order'], 0)

    def test_capture_order_does_not_wait_for_inflight_previous_generation(self):
        stream, capture = self.make_stream((1,2), result_order="capture")
        for segment in stream.segments:
            segment.block.clear()
        capture.inputs.put(1)
        wait_for(lambda: stream.stats['started'] == 1)
        old = next(segment for segment in stream.segments if segment.began.is_set())
        capture.inputs.put(2)
        wait_for(lambda: stream.stats['started'] == 2)
        new = next(segment for segment in stream.segments if segment is not old)
        new.block.set()
        wait_for(lambda: stream.stats['completed'] == 1)
        wait_for(lambda: capture.calls == 3)
        stream.invalidate()
        # read 已经跨过转弯边界，先舍弃这一帧，再发布新方向的帧。
        capture.inputs.put(3)
        wait_for(lambda: stream.stats['captured'] == 3)
        capture.inputs.put(4)
        try:
            result = stream.read(capture)
            self.assertEqual((result.source, int(result.mask[0,0])), (1,4))
            self.assertFalse(old.block.is_set())
        finally:
            old.block.set()

    def test_invalidate_fences_read_started_before_turn_and_inflight_results(self):
        stream, capture = self.make_stream()
        segment = stream.segments[0]
        segment.block.clear()
        capture.inputs.put(1)
        self.assertTrue(segment.began.wait(3))
        wait_for(lambda: stream.stats['captured'] == 1)
        wait_for(lambda: capture.calls == 2)
        # 此刻第二个 read 已经开始，转弯后返回的这一帧也必须丢弃。
        stream.invalidate()
        capture.inputs.put(2)
        wait_for(lambda: stream.stats['captured'] == 2)
        capture.inputs.put(3)
        wait_for(lambda: stream.stats['captured'] == 3)
        segment.block.set()
        result = stream.read(capture)
        self.assertEqual((result.source, int(result.mask[0,0])), (1,3))
        self.assertGreaterEqual(stream.stats['old_source'], 2)

    def test_camera_failure_can_retry_and_model_failure_propagates(self):
        stream, capture = self.make_stream()
        capture.inputs.put(None)
        with self.assertRaises(CameraReadError):
            stream.read(capture)
        stream.invalidate()
        capture.inputs.put(5)
        self.assertEqual(stream.read(capture).source, 1)
        stream.close()
        stream, capture = self.make_stream()
        with patch.object(stream.segments[0], 'mask', side_effect=ValueError('model failed')):
            capture.inputs.put(1)
            with self.assertRaisesRegex(ValueError, 'model failed'):
                stream.read(capture)

    def test_callback_error_is_fatal_and_close_releases_reader(self):
        capture = Capture()
        stream = LatestSegmentStream(Path('fake.rknn'), (1,), factory=Segment)
        self.addCleanup(stream.close)
        def record(*args):
            raise OSError('writer failed')
        stream.start(capture, record)
        capture.inputs.put(1)
        with self.assertRaisesRegex(OSError, 'writer failed'):
            stream.read(capture)
        stream.close()
        self.assertTrue(capture.released)


class LatestMainGateTest(unittest.TestCase):
    def test_capture_order_option_reaches_stream(self):
        import road_follow.__main__ as entry
        import tempfile
        capture = Capture()
        capture.inputs.put(1)
        options = []
        def factory(*args, **kwargs):
            options.append(kwargs['result_order'])
            kwargs['factory'] = Segment
            return LatestSegmentStream(*args, **kwargs)
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)/'config.yaml'
            config.write_text('{}\n')
            with patch.object(entry, 'LatestSegmentStream', side_effect=factory), \
                    patch.object(entry, '_open_camera', return_value=capture), \
                    patch.object(entry.signal, 'signal'), patch.object(entry, '_event'):
                self.assertEqual(entry.main(['--model','fake.rknn','--config',str(config),
                    '--latest-frame','--latest-result-order','capture','--frames','1']),0)
        self.assertEqual(options, ['capture'])

    def test_reader_stops_before_recording_is_closed(self):
        import road_follow.__main__ as entry
        import tempfile
        capture, streams, events = Capture(), [], []
        capture.inputs.put(1)
        def factory(*args, **kwargs):
            kwargs['factory'] = Segment
            stream = LatestSegmentStream(*args, **kwargs)
            streams.append(stream)
            return stream
        class Recorder:
            path = Path('fake.avi')
            frames_written = 1
            closed = False
            def write(self, image, timestamp):
                self.assert_open()
                events.append('write')
            def assert_open(self):
                if self.closed:
                    raise AssertionError('生产者向已关闭的编码器写入')
            def close(self, stopped_s):
                if not streams[0].closed or not capture.released:
                    raise AssertionError('编码器先于采集线程关闭')
                self.closed = True
                events.append('close')
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)/'config.yaml'
            config.write_text('{}\n')
            with patch.object(entry, 'LatestSegmentStream', side_effect=factory), \
                    patch.object(entry, '_open_camera', return_value=capture), \
                    patch.object(entry, 'AsyncVideoRecorder', return_value=Recorder()), \
                    patch.object(entry.signal, 'signal'), patch.object(entry, '_event'):
                self.assertEqual(entry.main(['--model','fake.rknn','--config',str(config),
                    '--latest-frame','--npu-contexts','3','--frames','1',
                    '--record-video',str(Path(folder)/'new.avi'),'--async-record-video']),0)
        self.assertEqual(events, ['write','close'])

    def test_latest_path_keeps_expiry_and_action_mutex(self):
        # 同一入口回归切到 latest 工厂：过期帧不更新导航，也不覆盖有限动作。
        import road_follow.__main__ as entry
        original_main = entry.main
        with patch.object(entry, 'LatestSegmentStream') as factory:
            # 原测试 mock 的 Ordered 工厂返回受控结果；转交给 latest 入口验证接线。
            factory.side_effect = lambda *a, **kw: entry.OrderedSegmentStream(*a, **kw)
            with patch.object(entry, 'main', side_effect=lambda argv:
                              original_main(argv+['--latest-frame'])):
                case = gate_tests.MainFrameGateTest()
                self.assertEqual(case.check_main(), ['0.000 0.000\n','0.100 0.200\n','0 0\n'])
                self.assertEqual(case.check_main(True), ['0 0\n'])

    def test_capture_order_keeps_expiry_and_action_mutex(self):
        import road_follow.__main__ as entry
        original_main = entry.main
        with patch.object(entry, 'LatestSegmentStream') as factory:
            factory.side_effect = lambda *a, **kw: entry.OrderedSegmentStream(*a, **kw)
            with patch.object(entry, 'main', side_effect=lambda argv: original_main(
                    argv+['--latest-frame','--latest-result-order','capture'])):
                case = gate_tests.MainFrameGateTest()
                self.assertEqual(case.check_main(), ['0.000 0.000\n','0.100 0.200\n','0 0\n'])
                self.assertEqual(case.check_main(True), ['0 0\n'])


if __name__ == '__main__':
    unittest.main()
