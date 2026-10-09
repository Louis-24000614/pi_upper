"""合成地面投影与拓扑，不使用相机、NPU、UART 或 PWM。"""
from copy import deepcopy
import json
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from road_follow.obstacle_edge import OBSTACLE_DEFAULTS, filter_current_edge, locate_ahead
from road_follow.culvert_map import CulvertRecords
from road_follow.inspection_config import Settings
from topo_proto.graph import Edge, Node, TopologyGraph
from vision.obstacle.detect import Detection
from vision.obstacle.blockage import HardBlockageJudge


KEY = ("a__b", "a", "b")


def graph():
    return TopologyGraph({}, {"a":Node("a",0,0),"b":Node("b",0,1),"c":Node("c",0,2)},
        {"a__b":Edge("a__b","a","b",1),"b__c":Edge("b__c","b","c",1)})


class EdgeObstacleTest(unittest.TestCase):
    def setUp(self):
        self.graph = graph()
        self.box = Detection(1,"施工警示牌",.9,280,80,360,300)
        self.settings = deepcopy(OBSTACLE_DEFAULTS)
        self.ipm = NS(image_points_to_ground=lambda points:[[0,.35]])

    def filter(self, progress=.2, box=None):
        return filter_current_edge([box or self.box], graph=self.graph, edge_key=KEY,
            progress_m=progress, ipm=self.ipm, image_shape=(480,640), settings=self.settings)

    def test_current_edge_candidate_can_confirm(self):
        items, details = self.filter()
        self.assertEqual(items,[self.box])
        self.assertEqual(details[0]["reason"],"current_edge")
        self.assertAlmostEqual(details[0]["target"]["canonical_offset_m"],.55)
        judge=HardBlockageJudge()
        for _ in range(3): observation=judge.update(items,(480,640))
        self.assertTrue(observation.just_confirmed)

    def test_next_edge_obstacle_is_located_but_cannot_confirm_current_edge(self):
        items, details = self.filter(progress=.8)
        self.assertEqual(items,[])
        self.assertEqual(details[0]["reason"],"beyond_current_edge")
        self.assertEqual(details[0]["target"]["edge_id"],"b__c")
        self.assertAlmostEqual(details[0]["target"]["canonical_offset_m"],.15)
        judge=HardBlockageJudge()
        for _ in range(5): observation=judge.update(items,(480,640))
        self.assertFalse(observation.hard_blocked)

    def test_real_reported_direction_does_not_seal_wrong_edge(self):
        nodes={"5_4":Node("5_4",2.8,.4),"4_4":Node("4_4",2.8,1.2),"3_4":Node("3_4",2.8,2)}
        g=TopologyGraph({},nodes,{"4_4__5_4":Edge("4_4__5_4","4_4","5_4",.97),
                                  "3_4__4_4":Edge("3_4__4_4","3_4","4_4",.97)})
        items, details=filter_current_edge([self.box],graph=g,edge_key=("4_4__5_4","5_4","4_4"),
            progress_m=.82,ipm=self.ipm,image_shape=(480,640),settings=self.settings)
        self.assertEqual(items,[])
        target=details[0]["target"]
        self.assertEqual(target["edge_id"],"3_4__4_4")
        self.assertAlmostEqual(target["canonical_offset_m"],.77)
        self.assertEqual(g.blocked,set())

    def test_end_margin_and_distance_bias_change_subsequent_decisions(self):
        self.assertEqual(self.filter(progress=.62)[1][0]["reason"],"near_edge_boundary")
        self.settings["edge_end_margin_m"]=.01
        self.assertEqual(self.filter(progress=.62)[1][0]["reason"],"current_edge")
        self.settings["distance_bias_m"]=-.1
        self.assertEqual(self.filter(progress=.7)[1][0]["reason"],"current_edge")

    def test_missing_alignment_clipped_projection_and_other_lane_do_not_assign(self):
        self.assertEqual(self.filter(progress=None)[1][0]["reason"],"unaligned_odom")
        clipped=Detection(1,"牌",.9,280,80,360,479)
        self.assertEqual(self.filter(box=clipped)[1][0]["reason"],"clipped_ground_contact")
        self.ipm.image_points_to_ground=lambda points:[[.3,.35]]
        self.assertEqual(self.filter()[1][0]["reason"],"outside_current_lane")
        self.ipm.image_points_to_ground=lambda points:[[0,-1]]
        self.assertEqual(self.filter()[1][0]["reason"],"outside_distance_window")

    def test_turn_or_ambiguous_continuation_does_not_invent_next_edge(self):
        self.graph.nodes["c"]=Node("c",1,1)
        self.assertIsNone(locate_ahead(self.graph,KEY,.8,.35))
        self.graph.nodes["c"]=Node("c",0,2)
        self.graph.nodes["d"]=Node("d",0,3)
        self.graph.edges["b__d"]=Edge("b__d","b","d",2)
        self.assertIsNone(locate_ahead(self.graph,KEY,.8,.35))

    def test_invalid_detection_and_odometry_are_safe_for_json_status(self):
        box=Detection(1,"牌",float("nan"),float("inf"),80,360,300)
        items,details=self.filter(box=box)
        self.assertEqual(items,[])
        json.dumps(details,allow_nan=False)
        self.assertEqual(details[0]["reason"],"invalid_box")
        items,details=self.filter(progress=float("nan"))
        self.assertEqual(items,[])
        json.dumps(details,allow_nan=False)
        self.assertEqual(details[0]["reason"],"unaligned_odom")

    def test_reverse_map_offset_uses_canonical_edge_direction(self):
        target=locate_ahead(self.graph,("a__b","b","a"),.2,.35)
        self.assertAlmostEqual(target["canonical_offset_m"],.45)


class TaskMapTest(unittest.TestCase):
    def test_observed_next_edge_does_not_block_and_confirmed_current_edge_is_distinct(self):
        g=graph();records=CulvertRecords(g)
        detail={"class_id":1,"label":"施工警示牌","score":.9,"bbox":[1,2,3,4],
            "target":locate_ahead(g,KEY,.8,.35),"reason":"beyond_current_edge"}
        records.observe_obstacles([detail],confirmed_bbox=detail["bbox"])
        self.assertEqual(records.topology_snapshot()["obstacles"]["b__c:1"]["status"],"observed")
        self.assertEqual(g.blocked,set())
        detail.update(target=locate_ahead(g,KEY,.2,.35),reason="current_edge")
        records.observe_obstacles([detail],confirmed_bbox=detail["bbox"])
        self.assertEqual(records.topology_snapshot()["obstacles"]["a__b:1"]["status"],"confirmed")
        # 记录层不直接替代现有规划器的封边操作。
        self.assertEqual(g.blocked,set())

    def test_map_saves_new_and_confirmed_events_immediately_but_throttles_updates(self):
        with tempfile.TemporaryDirectory() as folder:
            records=CulvertRecords(graph(),Path(folder)/"task")
            detail={"class_id":1,"label":"施工警示牌","score":.9,"bbox":[1,2,3,4],
                "target":locate_ahead(records.graph,KEY,.2,.35),"reason":"current_edge"}
            with patch("road_follow.culvert_map.time.monotonic",return_value=100),patch.object(records,"_save",wraps=records._save) as save:
                records.observe_obstacles([detail])
                self.assertEqual(save.call_count,1)
                detail["score"]=.91
                records.observe_obstacles([detail])
                self.assertEqual(save.call_count,1)
                records.observe_obstacles([detail],confirmed_bbox=detail["bbox"])
                self.assertEqual(save.call_count,2)
                detail["score"]=.92
                records.observe_obstacles([detail])
                self.assertEqual(save.call_count,2)
                records.flush()
                self.assertEqual(save.call_count,3)
            payload=json.loads((Path(folder)/"task.culverts.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["obstacles"]["a__b:1"]["score"],.92)

    def test_new_task_resets_markers_and_retains_old_task_file(self):
        with tempfile.TemporaryDirectory() as folder:
            first=CulvertRecords(graph(),Path(folder)/"first")
            first.set_vehicle(KEY,.4,"idle");first.mark_rfid(6,1,1)
            saved=Path(folder)/"first.culverts.json"
            before=saved.read_bytes()
            second=CulvertRecords(graph(),Path(folder)/"second")
            payload=second.topology_snapshot()
            self.assertEqual(payload["rfids"],[])
            self.assertEqual(payload["obstacles"],{})
            self.assertEqual(payload["culverts"],{})
            self.assertEqual(payload["blocked_edges"],[])
            self.assertNotEqual(first.session_id,second.session_id)
            self.assertEqual(saved.read_bytes(),before)

    def test_rfid_snapshot_is_not_mutated_by_vehicle_updates_and_svg_is_chinese(self):
        records=CulvertRecords(graph())
        records.set_vehicle(KEY,.4,"idle");records.mark_rfid(6,1,1)
        records.set_vehicle(KEY,.6,"idle")
        payload=records.topology_snapshot()
        self.assertEqual(payload["rfids"][0]["location"]["progress_m"],.4)
        self.assertNotIn("uid",payload["rfids"][0])
        ET.fromstring(records.svg())
        self.assertIn("标签 6",records.svg().decode())

    def test_uart_map_events_do_not_enter_navigation_turn_queue(self):
        from road_follow.__main__ import _watch_uart_notes
        turns,maps,actions=queue.Queue(),queue.Queue(),queue.Queue()
        _watch_uart_notes(NS(stdout=iter(["RFID_EVENT 6 3\n","RFID_EVENT 0 4\n"])),
            actions,turns,rfid_enabled=False,map_rfid_events=maps)
        self.assertTrue(turns.empty())
        self.assertEqual(maps.qsize(),1)
        self.assertEqual(maps.get()[:2],(6,3))


class ObstacleSettingsTest(unittest.TestCase):
    def test_atomic_save_restart_preserves_recognition_roi_and_separate_revision(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"settings.json"
            path.write_bytes((Path(__file__).with_name("fixtures")/"culvert_inspection.json").read_bytes())
            settings=Settings(path);old,revision=settings.snapshot()
            values,obstacle_revision=settings.update_obstacle({"edge_end_margin_m":.12})
            self.assertEqual(obstacle_revision,1)
            self.assertEqual(settings.snapshot()[1],revision)
            self.assertEqual(settings.snapshot()[0]["recognition"],old["recognition"])
            self.assertEqual(Settings(path).obstacle_snapshot()[0]["edge_end_margin_m"],.12)
            self.assertEqual(list(Path(folder).glob("*.tmp")),[])
            before=path.read_bytes()
            for change in ({"min_score":float("nan")},{"confirm_frames":2.5},{"distance_bias_m":True},{"pwm":1}):
                with self.subTest(change=change),self.assertRaises(ValueError):settings.update_obstacle(change)
            self.assertEqual(path.read_bytes(),before)


if __name__ == "__main__":
    unittest.main()
