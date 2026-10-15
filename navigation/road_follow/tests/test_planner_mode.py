"""全任务命令的 -d 开关仅改变全局规划；全部硬件入口均被替换。"""

from contextlib import redirect_stderr
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from road_follow import __main__ as entry


FULL_TASK = [
    "--config", "config/nav_camera.yaml",
    "--model", "models/road_yolo11n_seg.rknn", "--drive",
    "--uart-bin", "build-turn/uart/uart_vel", "--turn-at-junction", "right",
    "--culvert-stop", "--culvert-estimated-camera", "--culvert-inspect",
    "--inspection-web", "--inspection-config", "config/culvert_inspection.json",
    "--record-video",
]


class PlannerModeTests(unittest.TestCase):
    def parse_full_task(self, extra):
        # 板端标定和巡检文件在这里使用替身，测试实际入口参数组合及透传。
        with (
            redirect_stderr(io.StringIO()),
            patch("vision.ipm_proto.manual_ground.runtime_activation", return_value=None),
            patch("road_follow.culvert_runtime.validate_culvert_config",
                  return_value=({}, None, SimpleNamespace(metadata={"camera": "offline-test"}))),
            patch("road_follow.inspection_config.Settings", return_value="inspection-settings"),
            patch("road_follow.inspection_config.preflight"),
            patch.object(entry, "needs_frequency_guard", return_value=False),
            patch.object(entry, "_run", return_value=0) as run,
        ):
            self.assertEqual(entry.main([*FULL_TASK, *extra]), 0)
        return vars(run.call_args.args[0]).copy()

    def test_full_task_arguments_only_differ_in_planner_selection(self):
        original = self.parse_full_task([])
        forward = self.parse_full_task(["-d"])
        self.assertFalse(original.pop("no_planned_reverse"))
        self.assertTrue(forward.pop("no_planned_reverse"))
        self.assertEqual(original, forward)
        for name in ("drive", "culvert_stop", "culvert_estimated_camera", "culvert_inspect", "inspection_web"):
            self.assertTrue(forward[name])
        self.assertEqual(forward["record_video"], "")
        self.assertEqual(forward["inspection_config"], Path("config/culvert_inspection.json"))

    def test_long_option_selects_same_mode(self):
        self.assertEqual(self.parse_full_task(["-d"]), self.parse_full_task(["--no-planned-reverse"]))

    def test_frequency_guard_preserves_d_option(self):
        argv = ["--drive", "--turn-at-junction", "right", "-d"]
        with (
            patch("vision.ipm_proto.manual_ground.runtime_activation", return_value=None),
            patch.object(entry, "needs_frequency_guard", return_value=True),
            patch.object(entry, "run_guarded", return_value=17) as guard,
        ):
            self.assertEqual(entry.main(argv), 17)
        guard.assert_called_once_with([entry.sys.executable, "-m", "road_follow", *argv])

    def test_d_without_topology_is_rejected_before_devices_or_frequency_guard(self):
        for argv in (["-d"], ["-d", "--drive"], ["-d", "--turn-at-junction", "right"]):
            with (
                self.subTest(argv=argv), redirect_stderr(io.StringIO()),
                patch.object(entry, "needs_frequency_guard") as guard,
                patch.object(entry, "_run") as run,
                self.assertRaises(SystemExit) as error,
            ):
                entry.main(argv)
            self.assertEqual(error.exception.code, 2)
            guard.assert_not_called()
            run.assert_not_called()

    def test_route_factory_selects_planner_and_preserves_initial_exit(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                agent, _xy, first = entry._start_route(no_planned_reverse=enabled)
                self.assertEqual(agent.no_planned_reverse, enabled)
                self.assertEqual((first.from_node, first.to_node), ("0_0", "0_J"))
                self.assertEqual(agent.node_reached("0_J").to_node, "1_2")
                self.assertEqual(len(agent.state.route_edges), 48 if enabled else 33)


if __name__ == "__main__":
    unittest.main()
