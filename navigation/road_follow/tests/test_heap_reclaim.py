"""回收自由页不能改变调用者仍持有的 ndarray；异常退出要停止后台线程。"""
import sys
import unittest
import numpy as np
from road_follow.heap_reclaim import HeapReclaimer


@unittest.skipUnless(sys.platform == 'linux', 'malloc_trim 是板端可选功能')
class HeapReclaimTests(unittest.TestCase):
    def test_live_buffer_preserved(self):
        retained = np.arange(1024*1024, dtype=np.int64)
        expected = retained.copy()
        temporary = np.ones((1024,1024), dtype=np.float64)
        del temporary
        reclaimer = HeapReclaimer(60)
        reclaimer.reclaim_once()
        np.testing.assert_array_equal(retained, expected)

    def test_thread_closed_on_error(self):
        reclaimer = HeapReclaimer(60)
        with self.assertRaisesRegex(RuntimeError, 'test failure'):
            with reclaimer:
                raise RuntimeError('test failure')
        self.assertFalse(reclaimer.thread.is_alive())
