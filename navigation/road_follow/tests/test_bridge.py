"""串口进程已经退出时，写速度要返回失败，不能抛 BrokenPipe。"""

from __future__ import annotations

import subprocess
import unittest

from road_follow.__main__ import write_velocity


class BridgeWriteTest(unittest.TestCase):
    def test_dead_bridge_returns_false(self) -> None:
        proc = subprocess.Popen(["/bin/true"], stdin=subprocess.PIPE, text=True)
        proc.wait(timeout=2)
        self.assertFalse(write_velocity(proc, "0.000 0.000"))


if __name__ == "__main__":
    unittest.main()
