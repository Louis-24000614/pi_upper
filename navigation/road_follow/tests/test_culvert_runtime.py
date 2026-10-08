import copy
import hashlib
import json
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from road_follow.backup import EdgeProgress
from road_follow.control import VelocityCommand
from road_follow.culvert import CulvertConfig, CulvertOutcome
from road_follow.culvert_perception import CulvertCalibration
from road_follow.culvert_runtime import CulvertRuntime, culvert_eligible, validate_culvert_config
from road_follow.junction_turn import JunctionTurn
from road_follow.rfid_arrival import RfidArrival
from vision.obstacle.detect import Detection
from topo_proto.graph import Edge, Node, TopologyGraph


CAL = {"verified":True,"ground_contact_verified":True,
    "origin":"navigation_camera_ground_projection","image_size":[640,480],
    "image_points":[[100,100],[540,100],[100,450],[540,450]],
    "ground_points_m":[[-.4,.8],[.4,.8],[-.4,.1],[.4,.1]]}
ROOT=Path(__file__).resolve().parents[3]


class CulvertRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.mapping=yaml.safe_load((ROOT/"config/culvert.yaml").read_text(encoding="utf-8"))
        self.mapping["calibration"]=CAL
        self.graph=TopologyGraph({}, {"a":Node("a",0,0),"b":Node("b",1.2,0)},
            {"a__b":Edge("a__b","a","b",1.2)})
        self.agent=NS(graph=self.graph,state=NS(current_edge="a__b",from_node="a",to_node="b",
            phase="following",current_node="a",route_index=3,covered_edges=set(),blocked_edges=set()))
        self.sent=[]
        self.runtime=CulvertRuntime(self.mapping,CulvertConfig.from_mapping(self.mapping["culvert"]),CulvertCalibration(CAL),
            nav_config={"follow":{"near_mps":.05},"road_prior":{"expected_width_m":.20,"min_width_m":.14,"max_width_m":.32}},agent=self.agent,
            send=lambda s:self.sent.append(s) or True,event=lambda *_:None,root=ROOT,
            prefix=Path(self.folder.name)/"task")
        self.addCleanup(self.runtime.close)
        self.progress=EdgeProgress()
        self.notes=queue.Queue()
        self.command=VelocityCommand(.08,0,"follow")
        bev=np.zeros((88,100),np.uint8)
        bev[:,40:61]=255
        self.mask=self.runtime.calibration.ipm.warp_to_image(bev,(640,480),flags=cv2.INTER_NEAREST)
        self.box=Detection(3,"culvert",.9,243,150,397,325)

    def update(self,t,detections=None,age=.04):
        for timestamp in (t-.05,t+age):
            s=.2+timestamp*.05
            sample=(s,0,0,timestamp)
            self.progress.update(*sample)
            # Establish the same along-edge origin as a car already following.
            self.progress.s_m=s
            self.runtime.record_odom(sample,self.progress)
        frame=np.zeros((480,640,3),np.uint8)
        frame[0,0,0]=int(t*100)%256
        return self.runtime.update(detections=[self.box] if detections is None else detections,
            frame=frame,mask=self.mask,captured_s=t,sequence=int(t*100),source=0,now=t+age,
            progress=self.progress,command=self.command,eligible=True,notes=self.notes)

    def test_real_runtime_confirms_and_owns_before_arrival_without_changing_route(self):
        self.assertFalse(self.update(.7).owns)
        self.assertFalse(self.update(.8).owns)
        result=self.update(.9)
        self.assertTrue(result.owns)
        self.assertEqual(result.command.reason,"culvert_enter")
        self.assertEqual(self.agent.state.route_index,3)
        self.assertEqual(self.agent.state.current_node,"a")
        self.assertEqual(self.graph.blocked,set())

    def test_configured_bias_logs_raw_and_corrected_distance_and_completes_at_corrected_center(self):
        for t in (.7,.8,.9): self.update(t)
        target=self.runtime.controller.target
        self.assertAlmostEqual(self.runtime.config.entry_distance_bias_m,-.10)
        self.assertAlmostEqual(target.target_s_m,.24+.35-.10+.135)
        events=[json.loads(line) for line in
                Path(self.runtime.log.name).read_text(encoding="utf-8").splitlines()]
        observation=next(e for e in events if e.get("reason")=="confirmed")
        self.assertAlmostEqual(observation["entry_distance_m"],.35)
        self.assertAlmostEqual(observation["corrected_entry_distance_m"],.25)
        self.assertAlmostEqual(observation["entry_distance_bias_m"],-.10)
        def parked_step(t):
            sample=(target.target_s_m,0,0,t)
            self.progress.s_m=target.target_s_m
            self.runtime.record_odom(sample,self.progress)
            frame=np.zeros((480,640,3),np.uint8)
            return self.runtime.update(detections=[],frame=frame,mask=self.mask,captured_s=t,
                sequence=int(t*100),source=0,now=t,progress=self.progress,
                command=self.command,eligible=True,notes=self.notes)
        parked_step(1.0)
        self.notes.put("STOP_DONE")
        parked_step(1.1)
        for i in range(12,85): parked_step(i/10)
        self.assertIsNone(self.runtime.controller.fault_reason)
        entry=self.runtime.records.entries[target.edge_id]
        self.assertEqual(entry["status"],"done")
        self.assertAlmostEqual(entry["stop_error_m"],0)
        self.assertAlmostEqual(entry["canonical_offset_m"],target.target_s_m)

    def test_hard_classes_are_split_and_do_not_enter_culvert_branch(self):
        items=[self.box,Detection(2,"fence",.9,243,150,397,325),Detection(9,"unknown",.9,243,150,397,325)]
        hard,culvert,unknown=self.runtime.split(items)
        self.assertEqual([d.class_id for d in hard],[2])
        self.assertEqual([d.class_id for d in culvert],[3])
        self.assertEqual([d.class_id for d in unknown],[9])

    def test_unknown_near_target_stops_without_sealing_edge(self):
        unknown=Detection(9,"unknown",.9,243,150,397,325)
        for t in (.7,.8,.9): outcome=self.update(t,[unknown])
        self.assertTrue(outcome.owns)
        self.assertEqual(self.runtime.controller.fault_reason,"unknown_near_target")
        self.assertEqual(self.graph.blocked,set())
        self.assertEqual(self.runtime.records.entries,{})

    def test_expired_yolo_result_never_confirms(self):
        self.update(.7); self.update(.8)
        result=self.update(.9,age=.21)
        self.assertFalse(result.owns)
        self.assertEqual(len(self.runtime.perception.positions),0)

    def test_only_owner_consumes_stop_notes(self):
        self.notes.put("STOP_DONE")
        self.update(.7)
        self.assertEqual(self.notes.qsize(),1)
        self.update(.8); self.update(.9)
        self.assertTrue(self.notes.empty())

    def test_obstacle_preemption_discards_old_owner_notes(self):
        self.update(.7); self.update(.8); self.update(.9)
        self.notes.put("STOP_DONE")
        self.runtime.cancel_for_obstacle(1,self.notes)
        self.assertFalse(self.runtime.controller.owns)
        self.assertTrue(self.notes.empty())
        self.assertFalse(self.runtime.records.done("a__b"))

    def test_resume_clears_old_vision_but_keeps_edge_progress_and_route(self):
        junction=JunctionTurn(phase="approach",side="left",branch_latched=True,departure="left")
        arrival=RfidArrival(edge_latched=True)
        self.progress.s_m=.635
        smoother=NS(reset=lambda:None); stream=NS(invalidate=lambda:None)
        self.runtime.resume(junction,arrival,None,smoother,stream)
        self.assertEqual(self.progress.s_m,.635)
        self.assertEqual(self.agent.state.route_index,3)
        self.assertEqual(junction.phase,"follow")
        self.assertEqual(junction.departure,"left")
        self.assertFalse(arrival.edge_latched)

    def test_eligible_allows_approach_but_rejects_finite_actions(self):
        entrance=NS(phase="done"); recovery=NS(phase="idle")
        for phase,expected in (("follow",True),("approach",True),("heading_hold",False),("turning",False),("align",False)):
            self.assertEqual(culvert_eligible(entrance,self.agent,NS(phase=phase),recovery,NS(phase="follow")),expected)

    def test_progress_origin_reset_invalidates_same_edge_interpolation(self):
        self.update(.7)
        self.assertIsNotNone(self.runtime.history.progress_at(.7))
        self.runtime.reset_progress_origin()
        self.assertIsNone(self.runtime.history.progress_at(.7))
        self.assertEqual(len(self.runtime.perception.positions),0)

    def test_config_validation_binds_model_mapping_and_calibration(self):
        model=Path(self.folder.name)/"model.rknn"; model.write_bytes(b"offline fixture")
        mapping=copy.deepcopy(self.mapping)
        fingerprint=hashlib.sha256(model.read_bytes()).hexdigest()
        mapping["model"].update(path=model.name,sha256=fingerprint)
        mapping["class_mapping"]["model_sha256"]=fingerprint
        path=Path(self.folder.name)/"config.yaml"
        for mutation,error in (({},False),({"calibration":{"verified":False}},True),
                               ({"class_mapping":{"verified":False}},True)):
            candidate=copy.deepcopy(mapping)
            for section,fields in mutation.items(): candidate[section].update(fields)
            path.write_text(yaml.safe_dump(candidate),encoding="utf-8")
            if error:
                with self.assertRaises(ValueError): validate_culvert_config(path,model.parent,drive=True,image_size=(640,480),near_speed=.05)
            else:
                validate_culvert_config(path,model.parent,drive=True,image_size=(640,480),near_speed=.05)
        model.write_bytes(b"different")
        with self.assertRaises(ValueError): validate_culvert_config(path,model.parent,drive=True,image_size=(640,480),near_speed=.05)

    def test_invalid_startup_never_opens_camera_uart_or_frequency_guard(self):
        from road_follow import __main__ as entry
        with patch.object(entry,"_open_camera") as camera,patch.object(entry.subprocess,"Popen") as uart, \
             patch.object(entry,"needs_frequency_guard") as guard,patch.object(entry,"RoadSegmenter") as segment:
            with self.assertRaises(SystemExit) as error:
                entry.main(["--drive","--turn-at-junction","left","--culvert-stop"])
            self.assertEqual(error.exception.code,2)
            camera.assert_not_called(); uart.assert_not_called(); guard.assert_not_called(); segment.assert_not_called()


if __name__=="__main__": unittest.main()
