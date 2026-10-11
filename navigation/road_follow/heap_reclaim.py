"""低频归还 glibc 已释放的堆页；不释放仍由 ndarray/RKNN 持有的内存。"""
from collections import deque
import ctypes as C
import gc
import math
from pathlib import Path
import threading
import time


def rss_kib():
    try:
        for line in Path('/proc/self/status').read_text().splitlines():
            if line.startswith('VmRSS:'):
                return int(line.split()[1])
    except OSError:
        pass
    return None


class HeapReclaimer:
    """后台低频 trim；每帧 trim 会重复缺页并降吞吐，因此默认关闭。"""
    def __init__(self, interval_s=0):
        if not math.isfinite(interval_s) or interval_s < 0 or 0 < interval_s < 1:
            raise ValueError('堆回收间隔必须为 0（关闭）或至少 1 秒')
        self.interval_s, self.thread = interval_s, None
        self.stop = threading.Event()
        self.samples, self.errors = deque(maxlen=32), []
        self.began = time.monotonic()
        self.trim = None
        if interval_s:
            try:
                self.trim = C.CDLL(None).malloc_trim
            except AttributeError as exc:
                raise RuntimeError('可选堆回收需要支持 malloc_trim 的 libc') from exc
            self.trim.argtypes, self.trim.restype = [C.c_size_t], C.c_int

    def reclaim_once(self):
        before, began = rss_kib(), time.monotonic()
        # ctypes/原生适配对象可能形成引用环。先回收不可达对象，再归还空闲页；
        # 单做 trim 无法回收仍被这些对象持有的块。两项耗时分开记录以验证来源。
        collected = gc.collect()
        gc_ms = (time.monotonic()-began)*1000
        trim_began = time.monotonic()
        # glibc 按 arena 加锁，仅归还空闲页；不能用主动释放 tensor 代替此操作。
        released = self.trim(0)
        self.samples.append({'elapsed_s': time.monotonic()-self.began,
                             'gc_ms': gc_ms, 'collected_objects': collected,
                             'trim_ms': (time.monotonic()-trim_began)*1000,
                             'before_rss_kib': before, 'after_rss_kib': rss_kib(),
                             'released': released})

    def _loop(self):
        while not self.stop.wait(self.interval_s):
            try:
                self.reclaim_once()
            except Exception as exc:
                self.errors.append(repr(exc))
                return

    def __enter__(self):
        if self.interval_s:
            self.thread = threading.Thread(target=self._loop, name='road-heap-reclaim', daemon=True)
            self.thread.start()
        return self

    def close(self):
        """先停止定时器再收集报告，避免报告写完后出现未记录的回收或异常。"""
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                raise RuntimeError('堆回收线程未结束')
        if self.errors:
            raise RuntimeError('堆回收异常: '+ '; '.join(self.errors))

    def __exit__(self, *_exc):
        self.close()
