"""同一条边涵洞/障碍的真实感知、任务完成和倒车交接回归。"""
from dataclasses import replace
from contextlib import ExitStack, redirect_stderr
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

from agent.route_agent import RouteAgent
from road_follow.backup import Backup
from road_follow.control import VelocityCommand
from road_follow.culvert import CulvertOutcome, CulvertTarget
from road_follow.obstacle_edge import filter_current_edge
from road_follow.obstacle_route import ObstacleRecovery, step_obstacle_route
import test_culvert_runtime as runtime_fixture
from vision.obstacle.blockage import HardBlockageJudge, hard_block_config_from_mapping
from vision.obstacle.detect import Detection


class CulvertObstaclePriorityTest(unittest.TestCase):
    def setUp(self):
        self.fixture = runtime_fixture.CulvertRuntimeTest()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.runtime = self.fixture.runtime
        self.agent = RouteAgent(self.fixture.graph, home="4_3")
        self.agent.start()
        self.runtime.agent = self.agent
        self.judge = HardBlockageJudge(hard_block_config_from_mapping(self.fixture.mapping))
        self.recovery, self.backup = ObstacleRecovery(), Backup()
        self.far = Detection(2, "fence", .9, 280, 150, 360, 295)
        self.command = self.fixture.command

    def tick(self, t, obstacles=(), *, culvert=True, position=None):
        f = self.fixture
        position = .2+t*.05 if position is None else position
        for stamp in (t-.02, t+.03):
            sample = (position, 0, 0, stamp)
            f.progress.update(*sample)
            f.progress.s_m = position
            self.runtime.record_odom(sample, f.progress)
        frame = np.zeros((480,640,3), np.uint8)
        frame[0,0,0] = int(t*100)%256
        detections = ([f.box] if culvert else [])+list(obstacles)
        hard, _, _ = self.runtime.split(detections)
        hard, details = filter_current_edge(hard, graph=self.agent.graph,
            edge_key=self.runtime.edge_key(), progress_m=self.runtime.history.progress_at(t),
            ipm=self.runtime.calibration.ipm, image_shape=frame.shape,
            settings=self.runtime.obstacle_settings(self.judge), lane_x_m=0)
        observation = self.judge.update(hard, frame.shape)
        self.runtime.observe_obstacles(details, observation)
        prepared = None
        preempts = observation.just_confirmed
        if observation.hard_blocked or self.runtime.pending_obstacle is not None:
            prepared = self.runtime.observe_culvert(detections=detections, frame=frame,
                mask=f.mask, captured_s=t, sequence=int(t*100), source=0,
                now=t+.03, progress=f.progress)
            preempts = self.runtime.obstacle_preempts(observation, prepared, now=t+.03,
                hard_detections=hard, image_shape=frame.shape, hard_config=self.judge.config)
        if preempts:
            self.runtime.cancel_for_obstacle(t+.03, f.notes)
            self.recovery, self.backup, outcome = step_obstacle_route(self.recovery, self.backup,
                armed=True, confirmed=True, agent=self.agent,
                node_xy={k:(v.x,v.y) for k,v in self.agent.graph.nodes.items()},
                progress_s_m=f.progress.s_m, near_x_m=0, command=self.command,
                now_s=t+.03, notes=f.notes, send=lambda line:f.sent.append(line) or True,
                yaw_rad=0, odom_valid=True, frame_captured_s=t)
        else:
            outcome = self.runtime.update(detections=detections, frame=frame, mask=f.mask,
                captured_s=t, sequence=int(t*100), source=0, now=t+.03,
                progress=f.progress, command=self.command, eligible=True, notes=f.notes,
                culvert_observation=prepared)
        return preempts, outcome

    def defer(self):
        for t in (.7,.8,.9): preempts, outcome = self.tick(t,[self.far])
        self.assertFalse(preempts)
        self.assertEqual(self.runtime.controller.phase,"entering")
        self.assertIsNotNone(self.runtime.pending_obstacle)
        self.assertEqual(outcome.command.reason,"culvert_enter")
        self.assertEqual(self.agent.state.blocked_edges,set())

    def test_nearer_culvert_finishes_then_backs_up_even_if_obstacle_disappears(self):
        self.defer()
        target = self.runtime.controller.target
        self.tick(1.0,position=target.target_s_m,culvert=False)
        self.fixture.notes.put("STOP_DONE")
        for i in range(11,90):
            t=i/10
            preempts, outcome = self.tick(t,position=target.target_s_m,culvert=False)
            self.assertFalse(preempts)
            self.assertEqual(self.recovery.phase,"idle")
            self.assertEqual(outcome.command.v_mps,0)
            if self.runtime.records.done(target.edge_id):break
        else:self.fail("涵洞任务没有成功完成")
        self.assertEqual(self.runtime.controller.phase,"reacquire")
        preempts,outcome = self.tick(t+.1,position=target.target_s_m,culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")
        self.assertEqual(outcome.command.reason,"backup_hold")
        self.assertEqual(self.agent.state.phase,"backing")
        self.assertEqual(self.agent.state.blocked_edges,{target.edge_id})
        self.assertTrue(self.runtime.records.done(target.edge_id))
        self.assertEqual(self.runtime.records.entries[target.edge_id]["status"],"done")
        self.assertAlmostEqual(self.fixture.progress.s_m,target.target_s_m)

    def assert_obstacle_first(self, bottom):
        obstacle=replace(self.far,y2=bottom)
        for t in (.7,.8,.9):preempts,_=self.tick(t,[obstacle])
        self.assertTrue(preempts)
        self.assertEqual(self.runtime.records.entries,{})
        self.assertEqual(self.recovery.phase,"holding")

    def test_closer_obstacle_preempts_before_culvert_starts(self):
        self.assert_obstacle_first(350)

    def test_equal_bottoms_keep_obstacle_priority(self):
        self.assert_obstacle_first(325)

    def test_obstacle_only_keeps_original_backup(self):
        for t in (.7,.8,.9):preempts,_=self.tick(t,[self.far],culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")

    def test_culvert_confirmation_lag_holds_until_three_distinct_frames(self):
        self.tick(.6,[self.far],culvert=False)
        self.tick(.7,[self.far])
        preempts,outcome=self.tick(.8,[self.far])
        self.assertFalse(preempts)
        self.assertEqual(outcome.command.reason,"wait_culvert_confirmation")
        self.assertEqual(outcome.command.v_mps,0)
        preempts,outcome=self.tick(.9,[self.far])
        self.assertFalse(preempts)
        self.assertEqual(outcome.command.reason,"culvert_enter")

    def test_lost_culvert_during_confirmation_releases_saved_obstacle(self):
        self.tick(.6,[self.far],culvert=False)
        self.tick(.7,[self.far]);self.tick(.8,[self.far])
        preempts,_=self.tick(.9,culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")

    def test_confirmation_wait_is_bounded(self):
        self.tick(.6,[self.far],culvert=False)
        self.tick(.7,[self.far]);self.tick(.8,[self.far])
        preempts,_=self.tick(1.4,[self.far])
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")

    def test_nearest_of_multiple_valid_obstacles_wins_even_with_lower_score(self):
        near=replace(self.far,score=.55,y2=350)
        for t in (.7,.8,.9):preempts,_=self.tick(t,[self.far,near])
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")

    def test_low_score_closer_box_does_not_override_valid_culvert(self):
        rejected=replace(self.far,score=.2,y2=350)
        for t in (.7,.8,.9):preempts,_=self.tick(t,[self.far,rejected])
        self.assertFalse(preempts)
        self.assertIsNotNone(self.runtime.pending_obstacle)

    def test_other_lane_obstacle_cannot_defer_or_preempt(self):
        other=replace(self.far,x1=530,x2=620,y2=350)
        for t in (.7,.8,.9):preempts,_=self.tick(t,[other])
        self.assertFalse(preempts)
        self.assertIsNone(self.runtime.pending_obstacle)
        self.assertEqual(self.runtime.controller.phase,"entering")

    def test_obstacle_beyond_current_edge_does_not_participate(self):
        beyond=replace(self.far,y1=100,y2=180)
        for t in (.7,.8,.9):preempts,_=self.tick(t,[beyond],position=.6)
        self.assertFalse(preempts)
        self.assertIsNone(self.runtime.pending_obstacle)
        self.assertEqual(self.runtime.last_obstacle_details[0]["reason"],"beyond_current_edge")

    def test_partial_inspection_is_finished_and_preserved_before_backup(self):
        self.defer()
        target=self.runtime.controller.target
        self.runtime.controller.phase="task"
        result={"status":"partial","sides":[{"direction":"A"},{"direction":"B"}]}
        self.runtime.controller.executor=NS(step=lambda now:"done",result=result)
        self.tick(1.0,position=target.target_s_m,culvert=False)
        self.assertEqual(self.runtime.records.entries[target.edge_id]["status"],"partial")
        preempts,_=self.tick(1.1,position=target.target_s_m,culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")
        self.assertEqual(self.runtime.records.entries[target.edge_id]["status"],"partial")

    def test_new_closer_obstacle_during_entry_keeps_original_preemption(self):
        self.defer()
        closer=replace(self.far,y2=350)
        preempts,_=self.tick(1.0,[self.far,closer])
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase,"holding")
        self.assertEqual(self.runtime.records.entries[self.runtime.edge_key()[0]]["failure_reason"],"hard_obstacle")

    def test_culvert_fault_with_pending_obstacle_keeps_stop(self):
        self.defer()
        self.command=VelocityCommand(0,0,"stop_road")
        self.tick(1.0,culvert=False)
        self.assertEqual(self.runtime.controller.fault_reason,"vision_unsafe")
        preempts,outcome=self.tick(1.1,culvert=False)
        self.assertFalse(preempts)
        self.assertEqual(outcome.command.v_mps,0)
        self.assertEqual(self.recovery.phase,"idle")

    def test_changed_directed_edge_discards_pending_obstacle(self):
        self.defer()
        self.agent.state.from_node,self.agent.state.to_node="5_3","4_3"
        self.assertFalse(self.runtime.obstacle_preempts(None,None,now=1.1))
        self.assertIsNone(self.runtime.pending_obstacle)

    def lock_without_obstacle(self, *, reverse=False):
        if reverse:
            self.agent = RouteAgent(self.fixture.graph, home="5_3")
            self.agent.start()
            self.runtime.agent = self.agent
            self.assertEqual(self.runtime.edge_key()[1:], ("5_3", "4_3"))
        for t in (.7, .8, .9):
            preempts, _ = self.tick(t)
            self.assertFalse(preempts)
        self.assertEqual(self.runtime.controller.phase, "entering")
        self.assertIsNone(self.runtime.pending_obstacle)
        return self.runtime.controller.target

    def late_defer(self, *, reverse=False):
        target = self.lock_without_obstacle(reverse=reverse)
        for t in (1.0, 1.1, 1.2):
            preempts, outcome = self.tick(t, [self.far], culvert=False,
                                         position=target.target_s_m-.015)
            self.assertFalse(preempts)
        self.assertEqual(self.runtime.controller.phase, "entering")
        self.assertEqual(outcome.command.reason, "culvert_enter")
        self.assertIsNotNone(self.runtime.pending_obstacle)
        events = [json.loads(line) for line in
                  Path(self.runtime.log.name).read_text(encoding="utf-8").splitlines()]
        deferred = [e for e in events if e["event"] == "culvert_obstacle_deferred"]
        self.assertEqual(deferred[-1]["reason"], "behind_locked_culvert")
        return target

    def test_late_obstacle_after_culvert_box_disappears_finishes_then_backs_up(self):
        target = self.late_defer()
        self.tick(1.3, position=target.target_s_m, culvert=False)
        self.fixture.notes.put("STOP_DONE")
        for i in range(14, 95):
            t = i/10
            preempts, outcome = self.tick(t, position=target.target_s_m, culvert=False)
            self.assertFalse(preempts)
            self.assertEqual(outcome.command.v_mps, 0)
            if self.runtime.records.done(target.edge_id):
                break
        else:
            self.fail("后方障碍不应中断已锁定涵洞的任务")
        preempts, outcome = self.tick(t+.1, position=target.target_s_m, culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(outcome.command.reason, "backup_hold")
        self.assertEqual(self.recovery.phase, "holding")
        self.assertEqual(self.agent.state.blocked_edges, {target.edge_id})
        self.assertEqual(self.runtime.records.entries[target.edge_id]["status"], "done")

    def test_late_obstacle_uses_directed_position_on_reverse_edge(self):
        target = self.late_defer(reverse=True)
        detail = self.runtime.last_obstacle_details[0]
        edge = self.agent.graph.edges[target.edge_id]
        self.assertGreater(edge.length_m-detail["target"]["canonical_offset_m"], target.exit_s_m)
        self.assertLess(detail["target"]["canonical_offset_m"], target.exit_s_m)

    def test_late_obstacle_preserves_partial_inspection_before_backup(self):
        target = self.late_defer()
        self.runtime.controller.phase = "task"
        result = {"status":"partial", "sides":[{"direction":"A"}, {"direction":"B"}]}
        self.runtime.controller.executor = NS(step=lambda now:"done", result=result)
        preempts, _ = self.tick(1.3, position=target.target_s_m, culvert=False)
        self.assertFalse(preempts)
        preempts, _ = self.tick(1.4, position=target.target_s_m, culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase, "holding")
        self.assertEqual(self.runtime.records.entries[target.edge_id]["status"], "partial")

    def test_late_closer_obstacle_without_culvert_box_still_preempts(self):
        target = self.lock_without_obstacle()
        closer = replace(self.far, y2=450)
        for t in (1.0, 1.1, 1.2):
            preempts, _ = self.tick(t, [closer], position=target.target_s_m-.015, culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase, "holding")
        self.assertIsNone(self.runtime.pending_obstacle)

    def test_new_closer_obstacle_without_culvert_box_overrides_saved_obstacle(self):
        target = self.late_defer()
        closer = replace(self.far, y2=450)
        preempts, _ = self.tick(1.3, [self.far, closer], position=target.target_s_m-.015, culvert=False)
        self.assertTrue(preempts)
        self.assertEqual(self.recovery.phase, "holding")

    def test_locked_target_requires_matching_valid_current_edge_projection(self):
        from copy import deepcopy
        target = self.lock_without_obstacle()
        box = (self.far.x1, self.far.y1, self.far.x2, self.far.y2)
        observation = NS(just_confirmed=True, bbox=box)
        detail = {"class_id":self.far.class_id, "bbox":list(box), "reason":"current_edge",
                  "target":{"edge_id":target.edge_id, "from_node":target.from_node,
                            "to_node":target.to_node, "canonical_offset_m":target.exit_s_m+.1}}
        invalid = [[], [{**detail, "reason":"outside_current_lane"}],
                   [{**detail, "class_id":self.runtime.culvert_id}],
                   [{**detail, "bbox":[1, 2, 3, 4]}]]
        for field, value in (("edge_id", "other"), ("from_node", target.to_node),
                             ("canonical_offset_m", None), ("canonical_offset_m", float("nan")),
                             ("canonical_offset_m", 2.0), ("canonical_offset_m", -1.0),
                             ("canonical_offset_m", target.exit_s_m)):
            altered = deepcopy(detail)
            altered["target"][field] = value
            invalid.append([altered])
        for details in invalid:
            with self.subTest(details=details):
                self.runtime.last_obstacle_details = details
                self.assertTrue(self.runtime.obstacle_preempts(observation, None, now=1.2))
                self.assertIsNone(self.runtime.pending_obstacle)

    def test_fault_without_pending_obstacle_is_not_cancelled_by_late_confirmation(self):
        self.lock_without_obstacle()
        self.runtime.controller.fault("vision_unsafe", 1.0)
        box = (self.far.x1, self.far.y1, self.far.x2, self.far.y2)
        self.assertFalse(self.runtime.obstacle_preempts(NS(just_confirmed=True, bbox=box), None, now=1.2))
        self.assertEqual(self.runtime.controller.phase, "fault")
        self.assertEqual(self.recovery.phase, "idle")

    def test_recorded_222112_late_confirmation_keeps_locked_culvert(self):
        # 实车第134.192秒：涵洞框已消失，施工牌首次确认，车距停车点约1.46厘米。
        key = ("2_1__3_1", "2_1", "3_1")
        edge = runtime_fixture.Edge(key[0], key[1], key[2], .97)
        self.runtime.agent = NS(graph=NS(edges={edge.id:edge}),
            state=NS(current_edge=key[0], from_node=key[1], to_node=key[2]))
        self.runtime.controller.target = CulvertTarget(*key, .21142737331385972,
            .34642737331385975, .48142737331385976, .34642737331385975, 6464.780426557)
        self.runtime.controller.phase = "entering"
        box = (589.1590118408203, 46.03144836425781, 725.6164665222168, 258.10322189331055)
        self.runtime.last_obstacle_details = [{"class_id":1, "reason":"current_edge", "bbox":box,
            "target":{"edge_id":key[0], "from_node":key[1], "to_node":key[2],
                      "canonical_offset_m":.6913069027347489}}]
        observation = NS(just_confirmed=True, bbox=box)
        no_culvert = NS(reason="no_culvert", bbox=None)
        self.assertFalse(self.runtime.obstacle_preempts(observation, no_culvert, now=6472.446225614))
        self.assertEqual(self.runtime.pending_obstacle["edge"], key)
        self.assertEqual(self.runtime.controller.phase, "entering")

    def test_main_loop_arbitrates_before_backup_and_skips_arrival_when_deferred(self):
        from road_follow import __main__ as entry
        from road_follow.entrance import EntranceDeparture
        from road_follow.pipeline import FollowDiagnostics
        from ipm_proto.junction import KIND_STRAIGHT, JunctionRead
        import test_route_loop as route_fixture
        first=NS(edge_id="4_3__5_3",from_node="4_3",to_node="5_3")
        from agent.route_agent import FollowEdge
        first=FollowEdge(first.edge_id,first.from_node,first.to_node)
        bridge=Mock()
        bridge.poll.return_value=None
        bridge.stdout=iter(())
        camera=Mock()
        camera.read.return_value=(True,np.zeros((480,640,3),np.uint8))
        detector=Mock(config=self.fixture.mapping)
        detector.detect.return_value=([self.fixture.box,self.far],0)
        observation=NS(reason="confirmed",bbox=(243,150,397,325))
        runtime=Mock()
        runtime.controller=NS(owns=True,phase="entering")
        runtime.pending_obstacle=None
        runtime.edge_key.return_value=self.runtime.edge_key()
        runtime.history.progress_at.return_value=.245
        runtime.calibration=self.runtime.calibration
        runtime.split=self.runtime.split
        runtime.obstacle_settings.return_value=self.runtime.obstacle_settings()
        runtime.observe_culvert.return_value=observation
        runtime.obstacle_preempts.return_value=False
        runtime.update.return_value=CulvertOutcome(VelocityCommand(.05,0,"culvert_enter"),True)
        diag=FollowDiagnostics(2000,2000,80,80,80,False,.2,1,True,0)
        cfg={"capture":{"width":640,"height":480}}
        detector.config={**self.fixture.mapping,"hard_block":{"confirm_frames":1}}
        with ExitStack() as stack,redirect_stderr(io.StringIO()):
            def mock(name, **kwargs):
                return stack.enter_context(patch("road_follow.__main__."+name,**kwargs))
            stack.enter_context(patch("road_follow.culvert_runtime.validate_culvert_config",
                                      return_value=(self.fixture.mapping,self.runtime.config,self.runtime.calibration)))
            stack.enter_context(patch("road_follow.culvert_runtime.CulvertRuntime",return_value=runtime))
            stack.enter_context(patch("vision.obstacle.detect.ObstacleDetector",return_value=detector))
            mock("needs_frequency_guard",return_value=False)
            mock("RoadSegmenter",new=route_fixture._Segment)
            mock("_load_config",return_value=cfg)
            mock("_open_camera",return_value=camera)
            mock("_start_route",return_value=(self.agent,{k:(n.x,n.y) for k,n in self.agent.graph.nodes.items()},first))
            mock("subprocess.Popen",return_value=bridge)
            mock("signal.signal")
            mock("Path.is_file",return_value=True)
            mock("EntranceDeparture",return_value=EntranceDeparture(phase="done"))
            mock("BevProjector.project",return_value=(self.runtime.calibration.ipm,self.fixture.mask))
            mock("command_from_mask_with_diagnostics",return_value=(self.command,diag))
            mock("_junction_read",return_value=(KIND_STRAIGHT,JunctionRead(KIND_STRAIGHT,True,False,False,0,.2)))
            mock("resolve_arrival_policy")
            backup=mock("step_obstacle_route")
            arrival=mock("step_route_arrival")
            result=entry.main(["--drive","--turn-at-junction","right","--culvert-stop","--frames","1"])
        self.assertEqual(result,0)
        runtime.obstacle_preempts.assert_called_once()
        runtime.observe_culvert.assert_called_once()
        self.assertIs(runtime.update.call_args.kwargs["culvert_observation"],observation)
        backup.assert_not_called()
        arrival.assert_not_called()
        self.assertIn("0.050 0.000\n",[args[0] for args,_ in bridge.stdin.write.call_args_list])


if __name__=="__main__":unittest.main()
