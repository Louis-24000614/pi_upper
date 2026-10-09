"""侧视任务离线验证：合成帧、假 HTTP/PWM，不接任何设备。"""
from concurrent.futures import Future
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

from road_follow.inspection import SideInspection
from road_follow.inspection_config import Settings, preflight, pwm14m0_selected
from road_follow.inspection_io import FrameHub, Recognizer, SideFrame, SideCamera, crop, offset_box
from road_follow.culvert import CulvertConfig, CulvertController, CulvertTarget, OdomHistory
from road_follow.culvert_map import CulvertRecords
from road_follow.control import VelocityCommand
from topo_proto.graph import Edge, Node, TopologyGraph

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).with_name("fixtures")/"culvert_inspection.json"
KEY = ("a__b", "a", "b")
TARGET = CulvertTarget(*KEY, .5, .635, .77, .635, 0)


def graph():
    return TopologyGraph({}, {"a": Node("a", 0, 0), "b": Node("b", 1.2, 0)},
                         {KEY[0]: Edge(KEY[0], "a", "b", 1.2)})


def accepted(label="knife_01"):
    return {"accepted": True, "category": "knife", "identity": label, "score": .5,
            "bbox": [0, 0, 10, 10], "margin": .01, "reason": "accepted"}


class Pool:
    def __init__(self):
        self.jobs, self.manual = [], False
    def submit(self, fn, *args):
        future = Future()
        self.jobs.append((future, fn, args))
        if not self.manual:
            self.finish()
        return future
    def finish(self):
        future, fn, args = self.jobs[-1]
        try:
            future.set_result(fn(*args))
        except Exception as exc:
            future.set_exception(exc)
    def shutdown(self, **kwargs):
        pass


class Servo:
    def __init__(self):
        self.angles, self.closed, self.state = [], False, "done"
    def request(self, angle):
        self.angles.append(angle)
        return str(len(self.angles))
    def poll(self, token):
        return self.state
    def close(self):
        self.closed = True


class InspectionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/"settings.json"
        self.path.write_bytes(FIXTURE.read_bytes())
        self.settings = Settings(self.path)
        self.hub, self.servo, self.pool = FrameHub(), Servo(), Pool()
        self.recognizer = Mock()
        self.recognizer.recognize.return_value = accepted()
        self.notifications = []
        self.task = SideInspection(self.settings, self.hub, self.servo, self.recognizer,
                                   notify=self.notifications.append, pool=self.pool)
        self.addCleanup(self.task.close)
        self.task.start(0, TARGET)

    def vote(self, t, result=None):
        if result is not None:
            self.recognizer.recognize.return_value = result
        self.hub.publish(np.zeros((20, 30, 3), np.uint8), t)
        self.task.step(t)
        return self.task.step(t+.01)

    def confirm(self, start=0):
        for t in (start, start+1.3, start+2.6):
            result = self.vote(t)
        return result

    def second_side(self):
        self.task.step(2.62)
        self.assertEqual(self.task.phase, "settling")
        self.task.step(5.63)
        self.assertEqual(self.task.phase, "recognizing")

    def test_two_sides_one_flip_then_next_task_reverses(self):
        self.confirm()
        self.assertEqual(self.servo.angles, [180])
        self.second_side()
        self.assertEqual(self.confirm(6), "done")
        self.assertEqual(self.task.result["status"], "done")
        self.assertEqual([s["direction"] for s in self.task.result["sides"]], ["A", "B"])
        self.assertEqual(len(self.notifications), 2)
        self.assertFalse(self.task.result["speech_enabled"])
        self.task.start(10, TARGET)
        self.assertEqual(self.task.side, "B")
        self.confirm(10)
        self.assertEqual(self.servo.angles, [180, 0])

    def test_repeated_capture_frame_never_votes_twice(self):
        self.vote(0)
        for t in (.1, .2, .3, .9):
            self.task.step(t)
        self.assertEqual(self.task.count, 1)
        self.assertEqual(len(self.pool.jobs), 1)

    def test_identity_change_and_low_score_reset_streak(self):
        self.vote(0)
        self.vote(1.3, accepted("knife_02"))
        self.assertEqual(self.task.count, 1)
        self.vote(2.6, {"accepted": False, "reason": "low_score"})
        self.assertEqual(self.task.count, 0)
        self.vote(3.9, accepted("knife_02"))
        self.assertEqual(self.task.count, 1)

    def test_parameters_reset_count_without_extending_deadline(self):
        self.vote(0)
        deadline = self.task.deadline_s
        self.settings.update({"knife_threshold": .6, "side_timeout_s": 60})
        self.task.step(.2)
        self.assertEqual(self.task.count, 0)
        self.assertEqual(self.task.deadline_s, deadline)
        self.assertEqual(self.task.revision, 1)

    def test_stale_settings_response_is_discarded(self):
        self.pool.manual = True
        self.hub.publish(np.zeros((20, 30, 3), np.uint8), 0)
        self.task.step(0)
        self.settings.update({"roi": [0, 0, 1, .8]})
        self.task.step(.1)
        self.pool.finish()
        self.task.step(.2)
        self.assertEqual(self.task.count, 0)

    def test_old_side_response_after_timeout_is_discarded(self):
        self.pool.manual = True
        self.hub.publish(np.zeros((20, 30, 3), np.uint8), 0)
        self.task.step(0)
        self.task.step(45)
        self.task.step(45.1)
        self.task.step(48.2)
        self.pool.finish()
        self.task.step(48.3)
        self.assertEqual(self.task.count, 0)
        self.assertEqual(self.task.side, "B")

    def test_timeout_on_each_side_produces_partial_and_continues(self):
        self.task.step(45)
        self.task.step(45.1)
        self.task.step(48.2)
        self.assertEqual(self.task.step(93.3), "done")
        self.assertEqual(self.task.result["status"], "partial")
        self.assertEqual(self.servo.angles, [180])
        self.assertTrue(all(s["status"] == "unconfirmed" for s in self.task.result["sides"]))

    def test_failed_servo_is_task_failure_not_unconfirmed(self):
        self.confirm()
        self.servo.state = "failed"
        self.assertEqual(self.task.step(2.62), "failed")
        self.assertTrue(self.servo.closed)
        self.assertIsNone(self.task.result)

    def test_servo_timeout_is_failure(self):
        self.confirm()
        self.servo.state = "running"
        self.assertEqual(self.task.step(5), "failed")

    def test_servo_process_exit_stops_before_accepting_identity(self):
        self.servo.alive = lambda: False
        self.assertEqual(self.task.step(.1), "failed")
        self.assertTrue(self.servo.closed)
        self.assertIsNone(self.task.result)

    def test_camera_loss_and_http_error_reset_streak(self):
        self.vote(0)
        self.hub.fail("lost")
        self.task.step(.2)
        self.assertEqual(self.task.count, 0)
        self.recognizer.recognize.side_effect = TimeoutError("test")
        self.vote(1.3)
        self.assertEqual(self.task.count, 0)
        self.assertTrue(self.task.last_reason.startswith("request_failed"))

    def test_close_and_cancel_disable_without_extra_turn(self):
        self.task.cancel("navigation_fault")
        self.assertEqual(self.task.step(.1), "failed")
        self.task.close()
        self.assertTrue(self.servo.closed)
        self.assertEqual(self.servo.angles, [])

    def test_settings_persist_and_invalid_save_does_not_change_file(self):
        self.settings.update({"roi": [.1, 0, .9, .75], "face_threshold": .6})
        restarted, _ = Settings(self.path).snapshot()
        self.assertEqual(restarted["recognition"]["roi"], [.1, 0, .9, .75])
        before = self.path.read_bytes()
        for values in [{"face_threshold": .44}, {"roi": [0, 0, 0, 1]}, {"confirm_frames": True}, {"knife_threshold": float("nan")}, {"servo": {}}]:
            with self.assertRaises(ValueError):
                self.settings.update(values)
            self.assertEqual(before, self.path.read_bytes())

    def test_full_frame_initial_roi_and_default_scores(self):
        config, _ = self.settings.snapshot()
        self.assertEqual(config["recognition"]["roi"], [0, 0, 1, 1])
        self.assertEqual(config["recognition"]["face_threshold"], .5)
        self.assertEqual(config["recognition"]["knife_threshold"], .5)
        self.assertEqual(config["servo"]["pwm_route"], "PWM14_M0")


class RecognizerTest(unittest.TestCase):
    def setUp(self):
        self.rec = json.loads(FIXTURE.read_text())["recognition"]
        self.rec["roi"] = [.1, .2, .9, .8]
        self.frame = SideFrame(np.zeros((100, 200, 3), np.uint8), 1, 1, 0, "A")
        self.request = Mock()
        self.service = Recognizer({"face": "face", "knife": "knife"}, self.request)

    def test_outer_crop_affects_both_models_and_maps_boxes_to_original(self):
        self.request.side_effect = [{"results": []}, {"top1_class": "knife_03", "top1_score": .5,
                    "quality_ok": True, "roi_source": "light_surface", "roi_xyxy": [2, 3, 70, 40], "margin": .001}]
        result = self.service.recognize(self.frame, self.rec)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["bbox"], [22, 23, 90, 60])
        self.assertEqual(self.request.call_args_list[0].args[1].shape, (60, 160, 3))
        fields = self.request.call_args_list[1].args[2]
        self.assertEqual(fields["roi_selected"], "false")
        self.assertEqual(fields["frame_id"], 1)

    def test_registered_face_wins_and_knife_not_requested(self):
        self.request.return_value = {"results": [{"name": "suspect_02", "score": .5, "bbox": [1, 2, 20, 30]}]}
        result = self.service.recognize(self.frame, self.rec)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["bbox"], [21, 22, 40, 50])
        self.assertEqual(self.request.call_count, 1)

    def test_numeric_registered_ids_match_task_ids_without_changing_response(self):
        for number in range(1, 11):
            with self.subTest(number=number):
                self.request.reset_mock()
                response = {"results": [{"name": str(number), "score": .5, "bbox": [1, 2, 20, 30]}]}
                self.request.return_value = response
                result = self.service.recognize(self.frame, self.rec)
                self.assertTrue(result["accepted"])
                self.assertEqual(result["identity"], f"suspect_{number:02d}")
                self.assertEqual(response["results"][0]["name"], str(number))
                self.assertEqual(self.request.call_count, 1)

    def test_numeric_aliases_keep_low_and_unknown_faces_rejected(self):
        for label, score in [("1", .49), ("0", .9), ("11", .9), ("01", .9), (1, .9)]:
            with self.subTest(label=label):
                self.request.reset_mock()
                self.request.return_value = {"results": [{"name": label, "score": score}]}
                self.assertFalse(self.service.recognize(self.frame, self.rec)["accepted"])
                self.assertEqual(self.request.call_count, 1)

    def test_unknown_or_low_face_never_falls_back_to_knife(self):
        for label, score in [("Unknown", .7), ("suspect_01", .49)]:
            self.request.reset_mock()
            self.request.return_value = {"results": [{"name": label, "score": score}]}
            self.assertFalse(self.service.recognize(self.frame, self.rec)["accepted"])
            self.assertEqual(self.request.call_count, 1)

    def test_face_failure_is_not_no_face(self):
        self.request.side_effect = TimeoutError()
        with self.assertRaises(TimeoutError):
            self.service.recognize(self.frame, self.rec)
        self.assertEqual(self.request.call_count, 1)

    def test_knife_needs_automatic_roi_and_score(self):
        for override in [{"roi_source": None}, {"top1_score": .49}, {"quality_ok": False}, {"roi_xyxy": None}]:
            knife = {"top1_class": "knife_01", "top1_score": .9, "quality_ok": True,
                     "roi_source": "foreground", "roi_xyxy": [0, 0, 40, 40], **override}
            self.request.side_effect = [{"results": []}, knife]
            self.assertFalse(self.service.recognize(self.frame, self.rec)["accepted"])

    def test_rotation_remedy_not_rejected_by_foreground_ratio(self):
        self.request.side_effect = [{"results": []}, {"top1_class": "knife_01", "top1_score": .9,
            "quality_ok": True, "roi_source": "foreground", "roi_xyxy": [0, 0, 40, 40],
            "recognition_mode": "raw_roi_rotation", "preprocess": {"foreground_ratio": .01}}]
        self.assertTrue(self.service.recognize(self.frame, self.rec)["accepted"])

    def test_generation_rejects_frame_captured_before_turn(self):
        hub = FrameHub()
        before = hub.generation()
        hub.invalidate("B")
        self.assertFalse(hub.publish(self.frame.image, 1, before))
        self.assertIsNone(hub.latest(1))

    def test_nonfinite_scores_cannot_be_confirmed_or_break_status_json(self):
        self.request.side_effect = [{"results": []}, {"top1_class": "knife_01", "top1_score": float("nan"),
            "margin": float("nan"), "quality_ok": True, "roi_source": "foreground", "roi_xyxy": [0, 0, 40, 40]}]
        result = self.service.recognize(self.frame, self.rec)
        self.assertFalse(result["accepted"])
        json.dumps(result, allow_nan=False)

    def test_invalid_and_tiny_roi_rejected(self):
        with self.assertRaises(ValueError):
            crop(self.frame.image, [0, 0, .001, .001])
        self.assertIsNone(offset_box([0, 0, float("nan"), 1], (1, 2)))


class HardwareBoundaryTest(unittest.TestCase):
    def test_route_uses_live_pinctrl_not_chip_number(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            node = root/"chip/device/of_node/pinctrl-0"
            expected = root/"dt/pinctrl/pwm14/pwm14m0-pins/phandle"
            node.parent.mkdir(parents=True)
            expected.parent.mkdir(parents=True)
            expected.write_bytes(bytes.fromhex("0000017c"))
            node.write_bytes(bytes.fromhex("00000174"))
            self.assertFalse(pwm14m0_selected(root/"chip", root/"dt"))
            node.write_bytes(expected.read_bytes())
            self.assertTrue(pwm14m0_selected(root/"chip", root/"dt"))

    def test_missing_camera_or_unverified_pwm_never_starts_hardware(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"settings.json"
            path.write_bytes(FIXTURE.read_bytes())
            with patch("road_follow.inspection_io.cv2.VideoCapture") as camera, patch("road_follow.inspection_io.subprocess.Popen") as servo:
                with self.assertRaises(ValueError):
                    preflight(Settings(path), ROOT, "/dev/navigation")
                config = json.loads(path.read_text())
                device = Path(folder)/"fake_camera"
                device.touch()
                config["camera"]["device"] = str(device)
                path.write_text(json.dumps(config))
                with self.assertRaisesRegex(ValueError, "hardware_verified"):
                    preflight(Settings(path), ROOT, "/dev/navigation")
                camera.assert_not_called()
                servo.assert_not_called()

    def test_camera_single_capture_shared_with_preview_and_releases(self):
        import time
        hub = FrameHub()
        capture = Mock()
        capture.isOpened.return_value = True
        def read():
            time.sleep(.005)
            return True, np.zeros((20, 30, 3), np.uint8)
        capture.read.side_effect = read
        factory = Mock(return_value=capture)
        camera = SideCamera({"device": "fake", "width": 30, "height": 20}, hub, factory)
        try:
            limit = time.monotonic()+1
            while hub.latest() is None and time.monotonic() < limit:
                time.sleep(.01)
            self.assertIsNotNone(hub.latest())
            self.assertIs(hub.latest(), hub.latest())
            factory.assert_called_once_with("fake")
        finally:
            camera.close()
        capture.release.assert_called_once()

    def test_cli_invalid_combinations_reject_before_models_or_devices(self):
        import contextlib
        import io
        from road_follow import __main__ as entry
        for args in [["--inspection-web"], ["--culvert-inspect"]]:
            with patch.object(entry, "_open_camera") as camera, patch.object(entry, "RoadSegmenter") as model, patch.object(entry, "needs_frequency_guard") as guard, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    entry.main(args)
                camera.assert_not_called()
                model.assert_not_called()
                guard.assert_not_called()


class ControllerIntegrationTest(unittest.TestCase):
    def control(self, executor):
        history = OdomHistory(CulvertConfig())
        records = CulvertRecords(graph())
        sent = []
        control = CulvertController(CulvertConfig(), history, send=lambda x: sent.append(x) or True,
                                    event=lambda *a, **k: None, records=records, executor=executor)
        control.begin(TARGET, 0, .5, .05)
        records.discover(TARGET)
        control.phase, control.task_pose = "task", (.635, 0, 0)
        history.add(KEY, 1, .635, .635, 0, 0)
        return control, records, history, sent

    def step(self, control, now=1, visual=True):
        return control.step(now=now, current_s=.635, edge_key=KEY, visual_ok=visual,
                            frame_s=now, command=VelocityCommand(.08, 0, "follow"), near_speed=.05)

    def test_partial_map_is_handled_but_not_successful_and_reacquires(self):
        result = {"status": "partial", "sides": [{"direction": "A", "identity": "suspect_01"}, {"direction": "B", "identity": None}]}
        executor = NS(step=lambda now: "done", result=result)
        control, records, history, _ = self.control(executor)
        self.step(control)
        self.assertEqual(control.phase, "reacquire")
        self.assertTrue(records.handled(KEY[0]))
        self.assertFalse(records.done(KEY[0]))
        self.assertEqual(records.entries[KEY[0]]["inspection"], result)
        for t in (1.1, 1.2, 1.3):
            history.add(KEY, t, .635, .635, 0, 0)
            outcome = self.step(control, t)
        self.assertTrue(outcome.resumed)
        self.assertFalse(control.begin(TARGET, 2, .5, .05))

    def test_safety_fault_still_stops_and_cancels_recognition(self):
        executor = Mock()
        control, records, _, sent = self.control(executor)
        self.step(control, visual=False)
        self.assertEqual(control.phase, "fault")
        executor.cancel.assert_called_once_with("vision_unsafe")
        executor.step.assert_not_called()
        self.assertEqual(sent, ["stop"])
        self.assertFalse(records.handled(KEY[0]))

    def test_task_failure_never_marks_partial_completion(self):
        executor = NS(step=lambda now: "failed")
        control, records, _, _ = self.control(executor)
        self.step(control)
        self.assertEqual(control.phase, "fault")
        self.assertFalse(records.handled(KEY[0]))

    def test_partial_json_svg_persist_without_modifying_graph(self):
        with tempfile.TemporaryDirectory() as folder:
            prefix = Path(folder)/"task"
            records = CulvertRecords(graph(), prefix)
            records.discover(TARGET)
            result = {"status": "partial", "sides": [{"direction": "A", "identity": "knife_01"}, {"direction": "B", "identity": None}]}
            records.complete_inspection(KEY[0], .001, result)
            saved = json.loads(Path(str(prefix)+".culverts.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["culverts"][KEY[0]]["status"], "partial")
            svg = Path(str(prefix)+".culverts.svg").read_text(encoding="utf-8")
            self.assertIn("knife_01", svg)
            self.assertIn("unconfirmed", svg)
            self.assertFalse(records.graph.edges[KEY[0]].tunnel)


if __name__ == "__main__":
    unittest.main()
