"""主循环终止路线后，仍看到道路也必须持续发送零速。"""

import io
import unittest
from contextlib import ExitStack, redirect_stderr
from unittest.mock import Mock, patch

import numpy as np

from agent.route_agent import RouteAgent
from ipm_proto.junction import KIND_STRAIGHT, JunctionRead
from navigation.topo_proto.graph import load_topology
from road_follow.__main__ import main
from road_follow.control import VelocityCommand
from road_follow.entrance import EntranceDeparture
from road_follow.pipeline import FollowDiagnostics


class _Segment:
    """分割 worker 在别的线程调用 mask，不能用 Mock。"""

    def __init__(self, *_args, **_kwargs):
        self.closed = False

    def mask(self, image):
        return np.zeros(image.shape[:2], dtype=np.uint8)

    def close(self):
        self.closed = True


class RouteLoopTest(unittest.TestCase):
    def test_invalid_map_stops_bridge_before_opening_camera(self):
        bridge = Mock()
        bridge.poll.return_value = None
        bridge.stdout = iter(())
        with (
            redirect_stderr(io.StringIO()),
            patch("road_follow.__main__.needs_frequency_guard", return_value=False),
            patch("road_follow.__main__.RoadSegmenter", _Segment),
            patch("road_follow.__main__.subprocess.Popen", return_value=bridge),
            patch("road_follow.__main__.Path.is_file", return_value=True),
            patch("road_follow.__main__.signal.signal"),
            patch("road_follow.__main__._start_route", side_effect=ValueError("bad arrival mode")),
            patch("road_follow.__main__._open_camera") as camera,
        ):
            result = main(["--drive", "--turn-at-junction", "right"])
        self.assertEqual(result, 1)
        camera.assert_not_called()
        bridge.stdin.write.assert_called_once_with("0 0\n")
        bridge.stdin.close.assert_called_once()

    def test_done_and_fault_with_no_target_never_resume_visual_motion(self):
        for terminal in ("done", "fault"):
            with self.subTest(terminal=terminal):
                graph = load_topology()
                agent = RouteAgent(graph)
                first = agent.start()
                xy = {key: (node.x, node.y) for key, node in graph.nodes.items()}
                bridge = Mock()
                bridge.poll.return_value = None
                bridge.stdout = iter(())
                camera = Mock()
                camera.read.return_value = (True, np.zeros((2, 2, 3), dtype=np.uint8))
                diag = FollowDiagnostics(2000, 2000, 80, 80, 80, False, 0.2, 1.0, True, 0)
                reading = JunctionRead(KIND_STRAIGHT, True, False, False, 0, 0.2)

                def finish_route(policy, **kwargs):
                    if terminal == "done":
                        while agent.state.current_edge is not None:
                            agent.node_reached(agent.state.to_node)
                    else:
                        agent.state.current_edge = None
                        agent.state.phase = "fault"
                    kwargs["state"].phase = terminal
                    return kwargs["state"], kwargs["patrol_state"], VelocityCommand(0, 0, terminal), ""

                with ExitStack() as stack, redirect_stderr(io.StringIO()):
                    def mock(name, **kwargs):
                        return stack.enter_context(patch("road_follow.__main__." + name, **kwargs))

                    mock("needs_frequency_guard", return_value=False)
                    mock("RoadSegmenter", new=_Segment)
                    mock("BevProjector.project", return_value=(Mock(), np.zeros((2, 2), dtype=np.uint8)))
                    mock("_open_camera", return_value=camera)
                    mock("_load_config", return_value={"capture": {"width": 2, "height": 2}})
                    mock("_start_route", return_value=(agent, xy, first))
                    mock("subprocess.Popen", return_value=bridge)
                    mock("signal.signal")
                    mock("Path.is_file", return_value=True)
                    mock("EntranceDeparture", return_value=EntranceDeparture(phase="done"))
                    mock("command_from_mask_with_diagnostics", return_value=(VelocityCommand(0.08, 0.1, "follow"), diag))
                    mock("_junction_read", return_value=(KIND_STRAIGHT, reading))
                    step = mock("step_route_arrival", side_effect=finish_route)
                    result = main(["--drive", "--turn-at-junction", "right", "--frames", "2"])
                self.assertEqual(result, 0)
                self.assertEqual(step.call_count, 1)
                writes = [args[0] for args, _ in bridge.stdin.write.call_args_list]
                self.assertEqual(writes[:2], ["0.000 0.000\n", "0.000 0.000\n"])
                camera.release.assert_called_once()


if __name__ == "__main__":
    unittest.main()
