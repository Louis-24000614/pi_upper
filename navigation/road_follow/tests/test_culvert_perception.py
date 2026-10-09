import unittest
import numpy as np

from road_follow.culvert import CulvertConfig, OdomHistory
from road_follow.culvert_perception import CulvertCalibration, CulvertPerception
from topo_proto.graph import Edge
from vision.obstacle.detect import Detection, ObstacleDetector


def calibration_mapping():
    return {"verified":True,"ground_contact_verified":True,
        "origin":"navigation_camera_ground_projection","image_size":[640,480],
        "image_points":[[100,100],[540,100],[100,450],[540,450]],
        "ground_points_m":[[-.4,.8],[.4,.8],[-.4,.1],[.4,.1]]}


class CulvertPerceptionTest(unittest.TestCase):
    def setUp(self):
        self.config=CulvertConfig()
        self.calibration=CulvertCalibration(calibration_mapping())
        self.perception=CulvertPerception(self.config,self.calibration,3)
        self.history=OdomHistory(self.config)
        self.edge=Edge("a__b","a","b",1.2)
        self.points=[(0,y) for y in np.arange(.12,.81,.01)]
        self.box=Detection(3,"culvert",.9,243,150,397,325)
        for i in range(8): self.history.add((self.edge.id,"a","b"),i/10,.2+i*.005,i*.005,0,0)

    def observe(self,t,*,box=None,now=None,signature=None,points=None,done=False,from_node="a",to_node="b",shape=(480,640)):
        received = t+.04 if now is None else now
        self.history.add((self.edge.id,from_node,to_node),received,.2+received*.05,received*.05,0,0)
        return self.perception.observe([box or self.box],image_shape=shape,captured_s=t,
            now=t+.04 if now is None else now,frame_signature=int(t*100) if signature is None else signature,
            edge=self.edge,from_node=from_node,to_node=to_node,points=self.points if points is None else points,
            history=self.history,already_done=done)

    def test_three_frames_lock_camera_center_with_capture_clock_compensation(self):
        self.assertIsNone(self.observe(.7).target)
        self.assertIsNone(self.observe(.8).target)
        target=self.observe(.9,now=1.04).target
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target.target_s_m,.24+.35+.135,places=4)
        self.assertAlmostEqual(target.canonical_offset_m,target.target_s_m)

    def test_adjacent_lane_entry_does_not_confirm(self):
        box=Detection(3,"culvert",.9,408,150,562,325)
        self.assertEqual(self.observe(.7,box=box).reason,"outside_current_lane")

    def test_road_center_is_used_instead_of_image_center(self):
        shifted=[(.30,y) for _,y in self.points]
        box=Detection(3,"culvert",.9,408,150,562,325)
        self.assertEqual(self.observe(.7,box=box,points=shifted).reason,"tracking")

    def test_far_near_clipped_and_wrong_class_are_rejected(self):
        for expected,box in (
            ("outside_distance_window",Detection(3,"culvert",.9,243,20,397,150)),
            ("outside_distance_window",Detection(3,"culvert",.9,243,100,397,450)),
            ("clipped_entry",Detection(3,"culvert",.9,0,150,397,325)),
            ("no_culvert",Detection(2,"fence",.9,243,150,397,325))):
            with self.subTest(expected=expected):
                self.perception.reset()
                self.assertEqual(self.observe(.7,box=box).reason,expected)

    def test_next_edge_entry_is_not_assigned_to_current_edge(self):
        self.edge=Edge("a__b","a","b",.7)
        self.assertEqual(self.observe(.7).reason,"outside_current_edge")

    def test_missing_or_bent_road_rejects(self):
        self.assertEqual(self.observe(.7,points=[]).reason,"lane_not_straight")
        self.assertEqual(self.observe(.8,points=[(.5*y,y) for _,y in self.points]).reason,"lane_not_straight")

    def test_duplicate_and_reordered_capture_never_add_votes(self):
        self.observe(.7)
        self.assertEqual(self.observe(.7).reason,"duplicate_or_reordered_frame")
        self.assertEqual(self.observe(.6).reason,"duplicate_or_reordered_frame")
        self.assertEqual(len(self.perception.positions),1)

    def test_repeated_image_does_not_add_votes(self):
        self.observe(.7,signature=123)
        self.assertEqual(self.observe(.8,signature=123).reason,"duplicate_image")
        self.assertEqual(len(self.perception.positions),1)

    def test_slow_result_after_yolo_adds_vote_with_aligned_odom(self):
        self.observe(.7)
        self.assertEqual(self.observe(.8,now=1.01).reason,"tracking")
        self.assertEqual(len(self.perception.positions),2)

    def test_no_timestamp_bracket_is_not_extrapolated(self):
        self.history.reset((self.edge.id,"a","b"))
        self.assertEqual(self.observe(1.15).reason,"unaligned_odom")

    def test_completed_edge_does_not_confirm(self):
        self.assertEqual(self.observe(.7,done=True).reason,"already_done")

    def test_shape_and_calibration_are_required(self):
        self.assertEqual(self.observe(.7,shape=(720,1280)).reason,"calibration_missing_or_shape_mismatch")
        self.perception.calibration=CulvertCalibration({"verified":False})
        self.assertEqual(self.observe(.8).reason,"calibration_missing_or_shape_mismatch")

    def test_calibration_cannot_assert_unverified_ground_contact(self):
        for changes in ({"ground_contact_verified":False},{"origin":"front_bumper"},
                        {"image_points":[[1,1]]*4},{"ground_points_m":[[0,0]]*4}):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                CulvertCalibration({**calibration_mapping(),**changes})

    def test_reverse_direction_uses_same_canonical_map_offset(self):
        self.history.reset((self.edge.id,"b","a"))
        for i in range(8): self.history.add((self.edge.id,"b","a"),i/10,.2+i*.005,0,0,0)
        self.observe(.7,from_node="b",to_node="a")
        self.observe(.8,from_node="b",to_node="a")
        target=self.observe(.9,from_node="b",to_node="a").target
        self.assertAlmostEqual(target.canonical_offset_m,self.edge.length_m-target.target_s_m)

    def test_minus_ten_cm_bias_moves_entry_center_exit_and_map_in_both_directions(self):
        for from_node,to_node in (("a","b"),("b","a")):
            with self.subTest(from_node=from_node):
                targets=[]
                for bias in (0,-.10):
                    self.perception=CulvertPerception(
                        CulvertConfig.from_mapping({"entry_distance_bias_m":bias}),self.calibration,3)
                    self.history.reset((self.edge.id,from_node,to_node))
                    for i in range(8):
                        self.history.add((self.edge.id,from_node,to_node),i/10,.2+i*.005,0,0,0)
                    tracking=self.observe(.7,from_node=from_node,to_node=to_node)
                    self.assertAlmostEqual(tracking.distance_m,.35)
                    self.assertAlmostEqual(tracking.corrected_distance_m,.35+bias)
                    self.observe(.8,from_node=from_node,to_node=to_node)
                    result=self.observe(.9,now=1.04,from_node=from_node,to_node=to_node)
                    self.assertAlmostEqual(result.distance_m,.35)
                    self.assertAlmostEqual(result.corrected_distance_m,.35+bias)
                    self.assertIsNotNone(result.target)
                    targets.append(result.target)
                raw,corrected=targets
                for name in ("entrance_s_m","target_s_m","exit_s_m"):
                    self.assertAlmostEqual(getattr(corrected,name)-getattr(raw,name),-.10)
                self.assertAlmostEqual(corrected.exit_s_m-corrected.entrance_s_m,.27)
                self.assertAlmostEqual(corrected.canonical_offset_m-raw.canonical_offset_m,
                                       -.10 if from_node=="a" else .10)

    def test_bias_preserves_raw_distance_window_and_current_lane_checks(self):
        config=CulvertConfig.from_mapping({"entry_distance_bias_m":-.10})
        self.perception=CulvertPerception(config,self.calibration,3)
        # 原始 25 cm 仍在观察窗内，校正后 15 cm 不改变既有候选窗。
        box=Detection(3,"culvert",.9,243,150,397,375)
        result=self.observe(.7,box=box)
        self.assertEqual(result.reason,"tracking")
        self.assertAlmostEqual(result.distance_m,.25)
        self.assertAlmostEqual(result.corrected_distance_m,.15)
        far=Detection(3,"culvert",.9,243,20,397,150)
        self.assertEqual(self.observe(.8,box=far).reason,"outside_distance_window")
        adjacent=Detection(3,"culvert",.9,408,150,562,325)
        self.assertEqual(self.observe(.9,box=adjacent).reason,"outside_current_lane")

    def test_bias_cannot_put_entry_behind_camera_or_exit_outside_current_edge(self):
        for bias in (-.40,.40):
            with self.subTest(bias=bias):
                config=CulvertConfig.from_mapping({"entry_distance_bias_m":bias})
                self.perception=CulvertPerception(config,self.calibration,3)
                self.assertEqual(self.observe(.7).reason,
                    "corrected_entry_not_ahead" if bias<0 else "outside_current_edge")


class CulvertNmsTest(unittest.TestCase):
    def setUp(self):
        self.detector=ObstacleDetector.__new__(ObstacleDetector)
        self.detector.class_aware_nms=True
        self.detector.conf_thres=.45; self.detector.iou_thres=.5
        self.detector.class_count=4; self.detector.input_size=640

    def test_overlapping_fence_and_culvert_are_both_kept(self):
        boxes=np.array([[100,100,300,300],[100,100,300,300],[101,101,301,301]],np.float32)
        keep=self.detector._nms(boxes,np.array([.9,.8,.7]),np.array([3,2,3]))
        self.assertEqual(set(keep),{0,1})

    def test_xyxy_is_converted_to_width_height_for_nms(self):
        boxes=np.array([[200,200,220,220],[230,200,250,220]],np.float32)
        keep=self.detector._nms(boxes,np.array([.9,.8]),np.array([3,3]))
        self.assertEqual(set(keep),{0,1})

    def test_wrong_class_head_count_fails_instead_of_silent_decode(self):
        outputs=[]
        for side in (80,40,20):
            outputs.extend([np.zeros((1,64,side,side)),np.zeros((1,3,side,side)),np.zeros((1,1,side,side))])
        with self.assertRaises(ValueError): self.detector._decode(outputs)


if __name__ == "__main__": unittest.main()
