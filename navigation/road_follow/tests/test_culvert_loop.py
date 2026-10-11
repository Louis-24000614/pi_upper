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
    def run_loop(self,hard=False,slow=False,inspect=False,stream_age=None,complete=False,change_edge=False):
        graph=load_topology(); agent=RouteAgent(graph); first=agent.start()
        initial_index=agent.state.route_index
        xy={key:(node.x,node.y) for key,node in graph.nodes.items()}
        bridge=Mock(); bridge.poll.return_value=None; bridge.stdout=iter(())
        camera=Mock(); camera.read.return_value=(True,np.zeros((2,2,3),np.uint8))
        runtime=Mock(); runtime.first_capture_s=None
        runtime.controller.owns=True
        runtime.controller.phase="task"
        runtime.update.return_value=CulvertOutcome(VelocityCommand(0,0,"stop_culvert_task"),True,False)
        runtime.split.return_value=([],[],[])
        if complete:
            def finish(**kwargs):
                runtime.controller.phase="reacquire"
                return CulvertOutcome(VelocityCommand(0,0,"stop_culvert_reacquire"),True,False)
            runtime.update.side_effect=finish
        stream=Mock()
        stream.read.side_effect=lambda *args:NS(image=camera.read.return_value[1],
            mask=np.zeros((2,2),np.uint8),inference_s=stream_age,
            captured_s=time.monotonic()-stream_age,sequence=1,source=0)
        self.stream=stream
        diag=FollowDiagnostics(2000,2000,80,80,80,False,.2,1,True,0)
        reading=JunctionRead(KIND_STRAIGHT,True,False,False,0,.2)
        if change_edge:
            from vision.obstacle.blockage import HardBlockageJudge, HardBlockConfig
            from vision.obstacle.detect import Detection
            camera.read.return_value=(True,np.zeros((480,640,3),np.uint8))
            boxes=[Detection(1,"施工警示牌",.9,280,80,360,300)]
            judge=HardBlockageJudge(HardBlockConfig(confirm_frames=2))
            runtime.split.return_value=(boxes,[],[])
            def change_direction(**kwargs):
                agent.state.from_node,agent.state.to_node=agent.state.to_node,agent.state.from_node
                return CulvertOutcome(VelocityCommand(0,0,"stop_culvert_task"),True,False)
            runtime.update.side_effect=change_direction
        else:
            judge=Mock()
        if not change_edge: judge.update.return_value=NS(just_confirmed=hard,score=.9,stable_frames=3)
        def detect(_):
            if slow: time.sleep(.21)
            return (boxes if change_edge else []),0
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
            stack.enter_context(patch("road_follow.obstacle_edge.filter_current_edge",side_effect=lambda items,**kwargs:(items,[])))
            stack.enter_context(patch("road_follow.inspection_config.Settings"))
            stack.enter_context(patch("road_follow.inspection_config.preflight"))
            frames=1 if complete else 2
            options=["--drive","--turn-at-junction","right","--npu-contexts","1" if stream_age is None else "2",
                     "--culvert-stop","--frames",str(frames)]
            if inspect: options.append("--culvert-inspect")
            result=entry.main(options)
        self.assertEqual(result,0)
        self.assertEqual(detector.call_count,frames)
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

    def test_slow_yolo_still_confirms_obstacle_without_navigation_timeout(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(hard=True,slow=True)
        judge.update.assert_called()
        self.assertEqual(recovery.call_count,2)
        runtime.unsafe_frame.assert_not_called()
        self.assertTrue(all(line.startswith("0") for line in writes))

    def test_inspection_allows_slow_yolo_but_keeps_zero_velocity(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(slow=True,inspect=True)
        runtime.unsafe_frame.assert_not_called()
        self.assertEqual(runtime.update.call_count,2)
        judge.update.assert_called()
        arrival.assert_not_called(); recovery.assert_not_called()
        self.assertEqual(writes,["0 0\n"])

    def test_inspection_still_allows_hard_obstacle_preemption(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(hard=True,slow=True,inspect=True)
        self.assertEqual(runtime.cancel_for_obstacle.call_count,2)
        self.assertEqual(recovery.call_count,2)
        self.assertTrue(all(line.startswith("0") for line in writes))

    def test_inspection_does_not_discard_slow_segmentation_or_hide_road_command(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(inspect=True,stream_age=.4)
        runtime.unsafe_frame.assert_not_called()
        self.assertEqual(runtime.update.call_count,2)
        self.assertEqual(runtime.update.call_args.kwargs["command"].reason,"follow")
        self.assertEqual(writes,["0 0\n"])

    def test_opposite_direction_does_not_inherit_previous_edge_confirmation(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(change_edge=True)
        recovery.assert_not_called()
        self.assertEqual(judge._stable_frames,1)
        self.assertFalse(judge._hard_blocked)

    def test_default_accepts_slow_segmentation_without_extra_flags(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(stream_age=.4)
        self.assertEqual(runtime.update.call_count,2)
        runtime.unsafe_frame.assert_not_called()
        self.assertEqual(runtime.update.call_args.kwargs["command"].reason,"follow")
        self.assertEqual(writes,["0 0\n"])

    def test_inspect_completion_invalidates_stream_without_faulting_on_finishing_frame(self):
        runtime,judge,arrival,recovery,writes=self.run_loop(inspect=True,stream_age=.4,complete=True)
        self.stream.invalidate.assert_called_once()
        runtime.unsafe_frame.assert_not_called()
        self.assertEqual(writes,["0 0\n"])


if __name__=="__main__": unittest.main()
