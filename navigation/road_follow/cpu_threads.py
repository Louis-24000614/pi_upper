"""进程级 CPU 线程预算：外层已有多 context，避免每个小矩阵再争抢八核。"""
from contextlib import contextmanager
import cv2


@contextmanager
def cpu_thread_budget(opencv_threads=None, blas_threads=None):
    """只在 worker 启动前设置、退出后恢复；禁止并发线程各自反复改全局预算。"""
    previous_cv = cv2.getNumThreads()
    limiter = None
    try:
        if opencv_threads is not None:
            if opencv_threads < 1:
                raise ValueError('OpenCV 线程数必须大于零')
            cv2.setNumThreads(opencv_threads)
        if blas_threads is not None:
            if blas_threads < 1:
                raise ValueError('BLAS 线程数必须大于零')
            # 可选参数才依赖 threadpoolctl；默认入口不增加依赖或更改线程配置。
            from threadpoolctl import threadpool_limits
            limiter = threadpool_limits(limits=blas_threads, user_api='blas')
        yield
    finally:
        if limiter is not None:
            limiter.restore_original_limits()
        if opencv_threads is not None:
            cv2.setNumThreads(previous_cv)
