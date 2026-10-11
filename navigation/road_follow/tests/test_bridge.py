"""串口进程已经退出时，写速度要返回失败，不能抛 BrokenPipe。"""

from __future__ import annotations

import subprocess
import sys
import unittest

from road_follow.__main__ import write_velocity


class BridgeWriteTest(unittest.TestCase):
    def test_dead_bridge_returns_false(self) -> None:
        # 用当前 Python 创建已退出的进程，使 Windows 和板端都能验证断管处理。
        proc = subprocess.Popen(
            [sys.executable, "-c", "pass"], stdin=subprocess.PIPE, text=True
        )
        try:
            proc.wait(timeout=2)
            self.assertFalse(write_velocity(proc, "0.000 0.000"))
        finally:
            proc.stdin.close()


if __name__ == "__main__":
    unittest.main()
