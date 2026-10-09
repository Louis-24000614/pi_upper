"""执行真实主循环控制仲裁；相机/NPU/UART均为测试替身。"""
from contextlib import ExitStack, redirect_stdout
import io
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np

from agent.route_agent import RouteAgent
from topo_proto.graph import load_topology
from road_follow import __main__ as entry
from road_follow.control import VelocityCommand
from road_follow.culvert import CulvertOutcome
from road_follow.entrance import EntranceDeparture
from road_follow.pipeline import FollowDiagnostics
from ipm_proto.junction import KIND_STRAIGHT, JunctionRead


class Segment:
    def __init__(self,*args,**kwargs): pass
    def mask(self,image): return np.zeros(image.shape[:2],np.uint8)
    def close(self): pass


class CulvertLoopTest(unittest.TestCase):
    def run_loop(self,hard=False,slow=False,trial=False,stream_age=None,complete=False):
        graph=load_topology(); agent=RouteAgent(graph); first=agent.start()
        initial_index=agent.state.route_index
        xy={key:(node.x,node.y) for key,node in graph.nodes.items()}
        bridge=Mock(); bridge.poll.return_value=None; bridge.stdout=iter(())
        camera=Mock(); camera.read.return_value=(True,np.zeros((2,2,3),np.uint8))
        runtime=Mock(); runtime.first_capture_s=None
        runtime.controller.owns=True
        runtime.controller.phase="task"
        runtime.navigation_time_guard_enabled=not trial
        runtime.update.return_value=CulvertOutcome(VelocityCommand(0,0,"stop_culvert_task"),True,False)
        runtime.split.return_value=([],[],[])
        if complete:
            def finish(**kwargs):
                runtime.controller.phase="reacquire"
                runtime.navigation_time_guard_enabled=True
                return CulvertOutcome(VelocityCommand(0,0,"stop_culvert_reacquire"),True,False)
            runtime.update.side_effect=finish
        stream=Mock()
        stream.read.side_effect=lambda *args:NS(image=camera.read.return_value[1],
            mask=np.zeros((2,2),np.uint8),inference_s=stream_age,
            captured_s=time.monotonic()-stream_age,sequence=1,source=0)
        self.stream=stream
        diag=FollowDiagnostics(2000,2000,80,80,80,False,.2,1,True,0)
        reading=JunctionRead(KIND_STRAIGHT,True,False,False,0,.2)
        judge=Mock()
        judge.update.return_value=NS(just_confirmed=hard,score=.9,stable_frames=3)
        def detect(_):
            if slow: time.sleep(.21)
            return [],0
        with ExitStack() as stack, redirect_stdout(io.StringIO()):
            def mock(name,**kwargs): return stack.enter_context(patch("road_follow.__main__."+name,**kwargs))
            mock("needs_frequency_guard",return_value=False)
            mock("RoadSegmenter",new=Segment)
            mock("OrderedSegmentStream",return_value=stream)
            mock("BevProjector.project",return_value=(Mock(),np.zeros((2,2),np.uint8)))
            mock("_open_camera",return_value=camera)
            mock("_load_config",return_value={"capture":{"width":2,"height":2}})
            mock("_start_route",return_value=(agent,xy,first))
            mock("subprocess.Popen",return_value=bridge)
            mock("signal.signal")
            mock("Path.is_file",return_value=True)
            mock("EntranceDeparture",return_value=EntranceDeparture(phase="done"))
            mock("command_from_mask_with_diagnostics",return_value=(VelocityCommand(.08,.1,"follow"),diag))
            mock("_junction_read",return_value=(KIND_STRAIGHT,reading))
            arrival=mock("step_route_arrival")
            recovery=mock("step_obstacle_route",side_effect=lambda state,backup,**kwargs:
                (state,backup,NS(command=VelocityCommand(0,0,"backup_hold"),reset_progress=False,reset_smoother=False)))
            stack.enter_context(patch("road_follow.culvert_runtime.validate_culvert_config",return_value=({},None,None)))
            stack.enter_context(patch("road_follow.culvert_runtime.CulvertRuntime",return_value=runtime))
            stack.enter_context(patch("vision.obstacle.blockage.HardBlockageJudge",return_value=judge))
            detector=stack.enter_context(patch("vision.obstacle.detect.ObstacleDetector.detect",side_effect=detect))
            stack.enter_context(patch("road_follow.inspection_config.Settings"))
            stack.enter_context(patch("road_follow.inspection_config.preflight"))
            frames=1 if complete else 2
            options=["--drive","--turn-at-junction","right","--npu-contexts","1" if stream_age is None else "2",
                     "--culvert-stop","--frames",str(frames)]
            if trial: options.extend(["--culvert-inspect","--inspection-ignore-nav-timeout"])
            result=entry.main(options)
        self.assertEqual(result,0)
        expected=0 if stream_age is not None and stream_age>.2 and not trial else frames
        self.assertEqual(detector.call_count,expected)
        self.assertEqual(agent.state.route_index,initial_index)
        camera.release.assert_called_once()
        runtime.close.assert_called_once()
        return runtime,judge,arrival,recovery,[args[0] for args,_ in bridge.stdin.write.call_args_list]

    def test_culvert_owner_blocks_arrival_and_velocity_commands(self):
        runtime,judge,arrival,recovery,writes=self.run_loop()
        self.assertEqual(runtime.update.call_count,2)
        arrival.assert_not_called(); recovery.assert_not_called()
        self.assertEqual(writes,["0 0\n"])

    def test_hard_obstacle_preempts_culvert_before_arrival(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(hard=True)
        self.assertEqual(runtime.cancel_for_obstacle.call_count,2)
        self.assertEqual(recovery.call_count,2)
        runtime.update.assert_not_called(); arrival.assert_not_called()
        self.assertTrue(all(line.startswith("0") for line in writes))

    def test_yolo_latency_cannot_commit_obstacle_or_arrival(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(hard=True,slow=True)
        judge.update.assert_not_called(); arrival.assert_not_called(); recovery.assert_not_called()
        self.assertTrue(any(args[1]=="stale_after_yolo" for args,_ in runtime.unsafe_frame.call_args_list))
        self.assertEqual(writes,["0 0\n"])

    def test_timeout_trial_allows_slow_yolo_but_keeps_zero_velocity(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(slow=True,trial=True)
        runtime.unsafe_frame.assert_not_called()
        self.assertEqual(runtime.update.call_count,2)
        judge.update.assert_called()
        arrival.assert_not_called(); recovery.assert_not_called()
        self.assertEqual(writes,["0 0\n"])

    def test_timeout_trial_still_allows_hard_obstacle_preemption(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(hard=True,slow=True,trial=True)
        self.assertEqual(runtime.cancel_for_obstacle.call_count,2)
        self.assertEqual(recovery.call_count,2)
        self.assertTrue(all(line.startswith("0") for line in writes))

    def test_timeout_trial_does_not_discard_slow_segmentation_or_hide_road_command(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(trial=True,stream_age=.4)
        runtime.unsafe_frame.assert_not_called()
        self.assertEqual(runtime.update.call_count,2)
        self.assertEqual(runtime.update.call_args.kwargs["command"].reason,"follow")
        self.assertEqual(writes,["0 0\n"])

    def test_default_discards_expired_segmentation(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(stream_age=.4)
        runtime.update.assert_not_called()
        self.assertTrue(any(args[1]=="stale_frame" for args,_ in runtime.unsafe_frame.call_args_list))
        self.assertEqual(writes,["0 0\n"])

    def test_trial_completion_invalidates_stream_without_faulting_on_finishing_frame(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(trial=True,stream_age=.4,complete=True)
        self.stream.invalidate.assert_called_once()
        runtime.unsafe_frame.assert_not_called()
        self.assertTrue(runtime.navigation_time_guard_enabled)
        self.assertEqual(writes,["0 0\n"])


if __name__=="__main__": unittest.main()
