"""UART 文本事件在纯视觉拓扑和 RFID 单测模式中的分发边界。"""

from __future__ import annotations

import io
import queue
import unittest
from contextlib import redirect_stderr
from types import SimpleNamespace

from road_follow.__main__ import _watch_uart_notes


class UartNoteDispatchTest(unittest.TestCase):
    def setUp(self) -> None:
        self.actions: queue.Queue[str] = queue.Queue()
        self.rfid: queue.Queue[tuple[int, int]] = queue.Queue()
        self.odom: queue.Queue[tuple[float, float, float]] = queue.Queue()

    def run_notes(self, lines: list[str], *, rfid_enabled: bool) -> None:
        proc = SimpleNamespace(stdout=iter(lines))
        with redirect_stderr(io.StringIO()):
            _watch_uart_notes(
                proc,
                self.actions,
                self.rfid,
                self.odom,
                rfid_enabled,
            )

    def test_topology_mode_discards_rfid_but_keeps_motion_and_odom(self) -> None:
        self.run_notes(
            [
                "RFID_EVENT 7 3\n",
                "RFID_REMOVED\n",
                "FORWARD_DONE\n",
                "ODOM 1.0 2.0 -0.5 1\n",
            ],
            rfid_enabled=False,
        )

        self.assertTrue(self.rfid.empty())
        self.assertEqual(self.actions.get_nowait(), "FORWARD_DONE")
        self.assertEqual(self.odom.get_nowait(), (1.0, 2.0, -0.5))

    def test_explicit_rfid_mode_dispatches_card_event(self) -> None:
        self.run_notes(["RFID_EVENT 7 3\n"], rfid_enabled=True)

        self.assertEqual(self.rfid.get_nowait(), (7, 3))


if __name__ == "__main__":
    unittest.main()
