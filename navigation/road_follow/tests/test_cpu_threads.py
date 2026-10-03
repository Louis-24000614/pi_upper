"""线程预算是进程全局状态，正常退出和异常退出都必须恢复。"""
import unittest
import cv2
from road_follow.cpu_threads import cpu_thread_budget


class ThreadBudgetTests(unittest.TestCase):
    def test_opencv_restored_on_exception(self):
        original = cv2.getNumThreads()
        with self.assertRaisesRegex(RuntimeError, 'test failure'):
            with cpu_thread_budget(opencv_threads=1):
                self.assertEqual(cv2.getNumThreads(), 1)
                raise RuntimeError('test failure')
        self.assertEqual(cv2.getNumThreads(), original)

    def test_blas_restored(self):
        try:
            from threadpoolctl import threadpool_info
        except ImportError:
            self.skipTest('仅可选 BLAS 调优需要 threadpoolctl')
        original = threadpool_info()
        with cpu_thread_budget(blas_threads=1):
            self.assertTrue(all(row['num_threads'] == 1 for row in threadpool_info()
                                if row['user_api'] == 'blas'))
        self.assertEqual(threadpool_info(), original)


if __name__ == '__main__':
    unittest.main()
