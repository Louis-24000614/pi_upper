import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from road_follow.backup import EdgeProgress
from road_follow.control import VelocityCommand
from road_follow.culvert import CulvertConfig, CulvertController, CulvertTarget, OdomHistory
from road_follow.culvert_map import CulvertRecords
from topo_proto.graph import Edge, Node, TopologyGraph


KEY = ("a__b", "a", "b")
GRAPH = lambda: TopologyGraph({}, {"a":Node("a",0,0),"b":Node("b",1.2,0)},
                              {KEY[0]:Edge(KEY[0],"a","b",1.2)})


class CulvertControlTest(unittest.TestCase):
    def setUp(self):
        self.config = CulvertConfig()
        self.history = OdomHistory(self.config)
        self.records = CulvertRecords(GRAPH())
        self.sent, self.events = [], []
        self.control = CulvertController(self.config,self.history,
            send=lambda line:self.sent.append(line) or True,
            event=lambda name,**detail:self.events.append((name,detail)),records=self.records)
        self.target = CulvertTarget(*KEY,.5,.635,.77,.635,0)
        self.command = VelocityCommand(.08,.16,"follow")

    def sample(self, t, s=.635, x=None, yaw=0):
        self.history.add(KEY,t,s,s if x is None else x,0,yaw)

    def step(self, t, s=.635, notes=(), visual=True, key=KEY):
        return self.control.step(now=t,current_s=s,edge_key=key,visual_ok=visual,
            frame_s=t,command=self.command,near_speed=.05,notes=notes)

    def start(self):
        self.sample(0,.5)
        self.assertTrue(self.control.begin(self.target,0,.5,.05))

    def parked(self):
        self.start()
        self.sample(.1)
        self.step(.1)
        self.sample(.2)
        self.step(.2,notes=["STOP_DONE"])
        for i in range(3,24):
            self.sample(i/10)
            self.step(i/10)
        self.assertEqual(self.control.phase,"task")

    def test_speed_and_curvature_are_capped_without_progress_reset(self):
        self.start()
        outcome = self.step(.01,.5)
        self.assertAlmostEqual(outcome.command.v_mps,.05)
        self.assertAlmostEqual(outcome.command.omega_radps,.10)
        self.assertEqual(self.history.samples[-1][1],.5)

    def test_stop_ack_is_not_physical_stop_and_five_seconds_start_after_stability(self):
        self.parked()
        self.assertEqual(self.sent,["stop"])
        self.assertFalse(self.records.done(KEY[0]))
        for i in range(24,72):
            self.sample(i/10); self.step(i/10)
        self.assertFalse(self.records.done(KEY[0]))
        self.sample(7.4); self.step(7.4)
        self.assertTrue(self.records.done(KEY[0]))
        self.assertEqual(self.control.phase,"reacquire")
        for t in (7.5,7.6):
            self.sample(t); self.assertFalse(self.step(t).resumed)
        self.sample(7.7)
        self.assertTrue(self.step(7.7).resumed)
        self.assertEqual(self.control.phase,"idle")

    def test_stop_failure_does_not_complete(self):
        self.start(); self.sample(.1); self.step(.1)
        self.sample(.2); result=self.step(.2,notes=["STOP_FAIL"])
        self.assertEqual(self.control.fault_reason,"stop_failed")
        self.assertFalse(result.send_velocity)
        self.assertFalse(self.records.done(KEY[0]))

    def test_stop_submission_failure_does_not_complete(self):
        self.start(); self.control.send=lambda _:False
        self.sample(.1); self.step(.1)
        self.assertEqual(self.control.fault_reason,"stop_submit_failed")

    def test_stop_ack_timeout(self):
        self.start(); self.sample(.1); self.step(.1)
        self.sample(2.11); self.step(2.11)
        self.assertEqual(self.control.fault_reason,"stop_ack_timeout")

    def test_odom_stale_and_road_lost_stop(self):
        self.start(); self.step(.6,.5)
        self.assertEqual(self.control.fault_reason,"odom_stale")
        self.setUp(); self.start(); self.step(.1,.5,visual=False)
        self.assertEqual(self.control.fault_reason,"vision_unsafe")

    def test_edge_change_stops_without_completing(self):
        self.start(); self.step(.1,.5,key=("c__d","c","d"))
        self.assertEqual(self.control.fault_reason,"edge_changed")

    def test_motion_during_task_fails_and_logs_pose_delta_and_limits(self):
        self.parked(); self.sample(2.4,x=.66); self.step(2.4)
        self.assertEqual(self.control.fault_reason,"moved_during_task")
        event = next(details for name,details in reversed(self.events)
                     if name == "culvert_phase" and details.get("phase") == "fault")
        self.assertEqual(event["reason"],"moved_during_task")
        self.assertEqual(event["odom_sample_s"],2.4)
        self.assertAlmostEqual(event["position_delta_x_m"],.025)
        self.assertAlmostEqual(event["position_delta_y_m"],0)
        self.assertAlmostEqual(event["position_delta_m"],.025)
        self.assertAlmostEqual(event["yaw_delta_rad"],0)
        self.assertEqual(event["position_limit_m"],self.config.stable_position_m)
        self.assertEqual(event["yaw_limit_rad"],self.config.stable_yaw_rad)
        self.assertAlmostEqual(event["reference_pose"]["x_m"],.635)
        self.assertAlmostEqual(event["observed_pose"]["x_m"],.66)
        self.assertFalse(self.records.done(KEY[0]))

    def test_executor_can_be_replaced_and_failure_is_not_completion(self):
        self.parked()
        self.control.executor=SimpleNamespace(step=lambda _:"failed")
        self.sample(2.4); self.step(2.4)
        self.assertEqual(self.control.fault_reason,"task_failed")
        self.assertFalse(self.records.done(KEY[0]))

    def test_obstacle_preemption_does_not_complete(self):
        self.start(); self.control.cancel_for_obstacle(.1)
        self.assertFalse(self.control.owns)
        self.assertFalse(self.records.done(KEY[0]))

    def test_stop_outside_center_tolerance_fails(self):
        self.start(); self.sample(.1,.70); self.step(.1,.70)
        self.sample(.2,.70); self.step(.2,.70,notes=["STOP_DONE"])
        for i in range(3,24):
            self.sample(i/10,.70); self.step(i/10,.70)
        self.assertEqual(self.control.fault_reason,"stop_position_error")

    def test_completed_edge_is_shared_by_both_directions_and_new_task_clears(self):
        self.records.discover(self.target); self.records.complete(KEY[0],.001)
        reversed_target=CulvertTarget(KEY[0],"b","a",.5,.635,.77,.565,1)
        self.assertFalse(self.control.begin(reversed_target,1,.5,.05))
        self.assertFalse(CulvertRecords(GRAPH()).done(KEY[0]))

    def test_history_interpolates_capture_clock_and_never_extrapolates(self):
        self.sample(1,.2); self.sample(1.1,.205)
        self.assertAlmostEqual(self.history.progress_at(1.04),.202)
        self.assertIsNone(self.history.progress_at(.9))
        self.assertIsNone(self.history.progress_at(1.2))
        self.history.reset(("x","b","c"))
        self.assertIsNone(self.history.progress_at(1.04))

    def test_round_trip_motion_cannot_be_mistaken_for_stability(self):
        for i in range(23):
            self.sample(i/10,x=.02 if i==10 else 0)
        self.assertFalse(self.history.stable(2.2,0))

    def test_wrapped_yaw_is_unwrapped_for_stability(self):
        for i in range(23):
            self.sample(i/10,yaw=math.pi-.001 if i%2 else -math.pi+.001)
        self.assertTrue(self.history.stable(2.2,0))

    def test_telemetry_gap_cannot_be_mistaken_for_stability(self):
        for t in (0,.1,.2,1.3,1.4,1.5,2.1,2.2): self.sample(t)
        self.assertFalse(self.history.stable(2.2,0))

    def test_invalid_odom_invalidates_history(self):
        self.sample(0)
        self.history.add(KEY,.1,.6,float("nan"),0,0)
        self.assertFalse(self.history.fresh(.1))

    def test_configuration_rejects_invalid_limits(self):
        for mapping in ({"near_mps":.05},{"length_m":0},{"confirm_frames":2.5},
                        {"stop_error_m":.2},{"min_distance_m":.7},{"settle_timeout_s":1}):
            with self.subTest(mapping=mapping),self.assertRaises(ValueError):
                CulvertConfig.from_mapping(mapping)

    def test_entry_distance_bias_defaults_to_zero_and_accepts_signed_finite_values(self):
        self.assertEqual(CulvertConfig.from_mapping({}).entry_distance_bias_m,0)
        for bias in (-.10,0,.10):
            with self.subTest(bias=bias):
                config=CulvertConfig.from_mapping({"entry_distance_bias_m":bias})
                self.assertEqual(config.entry_distance_bias_m,bias)

    def test_entry_distance_bias_rejects_non_numeric_and_non_finite_values(self):
        for bias in (True,False,None,"-0.10",float("nan"),float("inf"),float("-inf")):
            with self.subTest(bias=bias),self.assertRaises(ValueError):
                CulvertConfig.from_mapping({"entry_distance_bias_m":bias})

    def test_map_exports_discovery_and_completion_without_editing_graph(self):
        import json,xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)/"run"
            graph=GRAPH(); records=CulvertRecords(graph,prefix)
            records.discover(self.target); records.complete(KEY[0],.002)
            payload=json.loads(Path(str(prefix)+".culverts.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["culverts"][KEY[0]]["status"],"done")
            ET.parse(Path(str(prefix)+".culverts.svg"))
            self.assertEqual(graph.blocked,set())
            self.assertFalse(graph.edges[KEY[0]].tunnel)
            with self.assertRaises(FileExistsError): CulvertRecords(graph,prefix)


if __name__ == "__main__":
    unittest.main()
