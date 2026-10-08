"""UART 文本事件在纯视觉拓扑和 RFID 单测模式中的分发边界。"""

from __future__ import annotations

import io
import queue
import unittest
from unittest.mock import patch
from contextlib import redirect_stderr
from types import SimpleNamespace

from road_follow.__main__ import _watch_uart_notes


class UartNoteDispatchTest(unittest.TestCase):
    def setUp(self) -> None:
        self.actions: queue.Queue[str] = queue.Queue()
        self.rfid: queue.Queue[tuple[int, int]] = queue.Queue()
        self.odom: queue.Queue[tuple[float, float, float, float]] = queue.Queue()

    def run_notes(self, lines: list[str], *, rfid_enabled: bool) -> str:
        proc = SimpleNamespace(stdout=iter(lines))
        err = io.StringIO()
        with redirect_stderr(err), patch("road_follow.__main__.time.monotonic", return_value=10.0):
            _watch_uart_notes(
                proc,
                self.actions,
                self.rfid,
                self.odom,
                rfid_enabled,
            )
        return err.getvalue()

    def test_topology_mode_logs_rfid_without_dispatching_it(self) -> None:
        log = self.run_notes(
            [
                "RFID_EVENT 7 3\n",
                "RFID_REMOVED\n",
                "FORWARD_DONE\n",
                "ODOM 1.0 2.0 -0.5 1\n",
            ],
            rfid_enabled=False,
        )

        self.assertIn("[RFID] 7 号", log)
        self.assertTrue(self.rfid.empty())
        self.assertEqual(self.actions.get_nowait(), "FORWARD_DONE")
        self.assertEqual(self.odom.get_nowait(), (1.0, 2.0, -0.5, 10.0))

    def test_explicit_rfid_mode_dispatches_card_event(self) -> None:
        self.run_notes(["RFID_EVENT 7 3\n"], rfid_enabled=True)

        self.assertEqual(self.rfid.get_nowait(), (7, 3))


if __name__ == "__main__":
    unittest.main()
