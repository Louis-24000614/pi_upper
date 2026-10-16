"""Face-pair timing and announcement tests with synthetic frames and no hardware."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from road_follow.inspection import SideInspection
from road_follow.inspection_config import Settings
from road_follow.inspection_io import FrameHub
from road_follow.inspection_pairs import FACE_KNIFE_PAIRS, paired_knife_identity
from road_follow.speech import InspectionSpeech, SpeechConfig, inspection_audio_id
from road_follow.tests.test_inspection import Pool, Servo, TARGET, accepted


def candidate(identity):
    return {**accepted(identity), "category": "suspect" if identity.startswith("suspect_") else "knife"}


class PairTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        path = Path(temp.name)/"settings.json"
        path.write_bytes((Path(__file__).with_name("fixtures")/"culvert_inspection.json").read_bytes())
        self.settings = Settings(path)
        self.settings.update({"side_timeout_s": 10})
        self.make_task()

    def make_task(self, paired=True):
        if hasattr(self, "task"):
            self.task.close()
        self.hub, self.servo, self.pool, self.recognizer = FrameHub(), Servo(), Pool(), Mock()
        self.notes, self.commands = [], []
        self.speech = InspectionSpeech(SpeechConfig(), lambda line: self.commands.append(line) or True,
                                       lambda *args, **kwargs: None, "test")
        def notify(record):
            self.notes.append(record)
            self.speech.request(record)
        self.task = SideInspection(self.settings, self.hub, self.servo, self.recognizer,
                                   notify=notify, pool=self.pool, pair_fallback=paired)
        self.addCleanup(self.task.close)
        self.task.start(0, TARGET)

    def vote(self, t, identity):
        self.recognizer.recognize.return_value = candidate(identity)
        self.hub.publish(np.zeros((20, 30, 3), np.uint8), t)
        self.task.step(t)
        return self.task.step(t+.001)

    def confirm(self, identity, start=0):
        for delta in (0, .05, .10):
            outcome = self.vote(start+delta, identity)
        return outcome

    def flip(self, now):
        self.task.step(now)
        self.assertEqual(self.task.phase, "settling")
        self.task.step(now+3.01)
        self.assertEqual(self.task.phase, "recognizing")

    def audio_ids(self):
        return [int(line.split()[2]) for line in self.commands]

    def test_pair_timeout_uses_saved_web_value_on_both_sides(self):
        self.settings.update({"side_timeout_s": 7})
        self.make_task()
        self.assertEqual(self.task.deadline_s, 7)
        self.assertEqual(self.task.status()["effective_side_timeout_s"], 7)
        self.confirm("suspect_08")
        self.flip(.12)
        self.assertAlmostEqual(self.task.deadline_s, 10.13)
        self.task.step(self.task.deadline_s)
        self.assertEqual(self.audio_ids(), [20, 29])

    def test_web_timeout_update_applies_next_side_and_survives_reload(self):
        self.assertEqual(self.task.deadline_s, 10)
        self.settings.update({"side_timeout_s": 6})
        self.task.step(.2)
        self.assertEqual(self.task.deadline_s, 10)
        self.assertEqual(self.task.status()["effective_side_timeout_s"], 6)
        config, _ = Settings(self.settings.path).snapshot()
        self.assertEqual(config["recognition"]["side_timeout_s"], 6)
        self.confirm("suspect_08", .3)
        self.flip(.42)
        self.assertAlmostEqual(self.task.deadline_s, 9.43)

    def test_page_keeps_pair_timeout_editable_and_does_not_force_a_literal(self):
        from road_follow.inspection_web import PAGE
        self.assertNotIn("$('side_timeout_s').disabled", PAGE)
        self.assertNotIn("计时15秒", PAGE)
        self.assertNotIn("15秒未确认", PAGE)

    def test_all_ten_supplied_pairs(self):
        expected = {1: 8, 2: 6, 3: 5, 4: 3, 5: 2, 6: 1, 7: 4, 8: 7, 9: 9, 10: 10}
        self.assertEqual(FACE_KNIFE_PAIRS, expected)
        for person, knife in expected.items():
            self.assertEqual(paired_knife_identity(f"suspect_{person:02d}"), f"knife_{knife:02d}")
        for identity in (None, "Unknown", "suspect_00", "suspect_11"):
            self.assertIsNone(paired_knife_identity(identity))

    def test_face_first_all_pairs_wait_configured_ten_seconds(self):
        for person, knife in FACE_KNIFE_PAIRS.items():
            with self.subTest(person=person):
                self.make_task()
                self.confirm(f"suspect_{person:02d}")
                self.assertEqual(self.audio_ids(), [12+person])
                self.flip(.12)
                deadline = self.task.deadline_s
                self.assertAlmostEqual(deadline, 13.13)
                self.task.step(deadline-.001)
                self.assertEqual(self.audio_ids(), [12+person])
                self.assertEqual(self.task.step(deadline), "done")
                self.assertEqual(self.audio_ids(), [12+person, 22+knife])
                self.assertEqual(self.task.result["status"], "partial")
                self.assertEqual(self.task.sides[1]["status"], "unconfirmed")
                inferred = self.task.result["inferred_announcements"][0]
                self.assertEqual(inferred["status"], "inferred")
                self.assertEqual(inferred["based_on"], f"suspect_{person:02d}")
                self.assertEqual(inferred["valid_frames"], 0)
                self.task.step(deadline+1)
                self.assertEqual(len(self.commands), 2)

    def test_knife_timeout_first_all_pairs_wait_three_seconds_after_face(self):
        for person, knife in FACE_KNIFE_PAIRS.items():
            with self.subTest(person=person):
                self.make_task()
                self.task.step(10)
                self.flip(10.01)
                self.confirm(f"suspect_{person:02d}", 13.03)
                self.assertEqual(self.task.phase, "announcing")
                self.assertEqual(self.audio_ids(), [12+person])
                due = self.task.pending_announcement[0]
                self.assertAlmostEqual(due, 16.131)
                self.task.step(due-.001)
                self.assertEqual(self.audio_ids(), [12+person])
                self.assertEqual(self.task.step(due), "done")
                self.assertEqual(self.audio_ids(), [12+person, 22+knife])

    def test_face_first_wrong_knife_never_announced_and_does_not_finish_early(self):
        self.confirm("suspect_08")
        self.flip(.12)
        self.confirm("knife_05", 3.2)
        self.assertEqual(self.task.phase, "recognizing")
        self.assertEqual(self.task.count, 0)
        self.assertEqual(self.task.last_reason, "pair_mismatch")
        self.assertEqual(self.audio_ids(), [20])
        self.task.step(self.task.deadline_s)
        self.assertEqual(self.audio_ids(), [20, 29])

    def test_face_first_correct_knife_finishes_normally_without_duplicate(self):
        self.confirm("suspect_08")
        self.flip(.12)
        self.assertEqual(self.confirm("knife_07", 3.2), "done")
        self.assertEqual(self.audio_ids(), [20, 29])
        self.assertEqual(self.task.result["status"], "done")
        self.assertEqual(self.task.inferred_announcements, [])
        self.task.step(100)
        self.assertEqual(self.audio_ids(), [20, 29])

    def test_knife_first_wrong_knife_is_held_until_person_then_corrected(self):
        self.confirm("knife_05")
        self.assertEqual(self.task.phase, "recognizing")
        self.assertEqual(self.commands, [])
        self.task.step(9.99)
        self.assertEqual(self.task.phase, "recognizing")
        self.task.step(10)
        self.flip(10.01)
        self.confirm("suspect_08", 13.03)
        self.assertEqual(self.audio_ids(), [20])
        self.task.step(self.task.pending_announcement[0])
        self.assertEqual(self.audio_ids(), [20, 29])
        self.assertFalse(self.task.sides[0]["pair_match"])
        self.assertEqual(self.task.result["status"], "partial")

    def test_knife_first_remembers_matching_confirmed_candidate_among_wrong_ones(self):
        self.confirm("knife_07")
        self.confirm("knife_05", 1)
        self.task.step(10)
        self.flip(10.01)
        self.confirm("suspect_08", 13.03)
        self.task.step(self.task.pending_announcement[0])
        self.assertEqual(self.audio_ids(), [20, 29])
        self.assertEqual(self.task.sides[0]["identity"], "knife_07")
        self.assertEqual(self.task.result["status"], "done")
        self.assertEqual(self.task.inferred_announcements, [])

    def test_unmapped_face_does_not_infer_a_knife(self):
        # All registered faces now have pairs; simulate an incomplete mapping.
        with patch.dict(FACE_KNIFE_PAIRS, {8: 7}, clear=True):
            self.confirm("suspect_01")
            self.flip(.12)
            self.task.step(self.task.deadline_s)
            self.assertEqual(self.audio_ids(), [13])
            self.assertEqual(self.task.inferred_announcements, [])

    def test_no_face_never_creates_a_pair_inference(self):
        self.task.step(10)
        self.flip(10.01)
        self.task.step(self.task.deadline_s)
        self.assertEqual(self.commands, [])
        self.assertEqual(self.task.inferred_announcements, [])

    def test_second_face_cannot_replace_expected_knife(self):
        self.confirm("suspect_08")
        self.flip(.12)
        self.confirm("suspect_03", 3.2)
        self.assertEqual(self.audio_ids(), [20])
        self.task.step(self.task.deadline_s)
        self.assertEqual(self.audio_ids(), [20, 29])

    def test_disabled_flag_keeps_configured_deadline_and_real_results(self):
        self.settings.update({"side_timeout_s": 45})
        self.make_task(paired=False)
        self.assertEqual(self.task.deadline_s, 45)
        self.confirm("suspect_08")
        self.flip(.12)
        self.assertEqual(self.confirm("knife_05", 3.2), "done")
        self.assertEqual(self.audio_ids(), [20, 27])
        self.assertEqual(self.task.inferred_announcements, [])

    def test_cancel_clears_delayed_announcement(self):
        self.task.step(10)
        self.flip(10.01)
        self.confirm("suspect_08", 13.03)
        due = self.task.pending_announcement[0]
        self.task.cancel()
        self.assertIsNone(self.task.pending_announcement)
        self.task.step(due+1)
        self.assertEqual(self.audio_ids(), [20])

    def test_restart_clears_delayed_announcement_and_all_evidence(self):
        self.confirm("knife_05")
        self.task.step(10)
        self.flip(10.01)
        self.confirm("suspect_08", 13.03)
        due = self.task.pending_announcement[0]
        self.task.start(14, TARGET, initial_turn=True)
        self.assertIsNone(self.task.pending_announcement)
        self.assertEqual(self.task.knife_observations, {})
        self.assertEqual(self.task.inferred_announcements, [])
        self.assertEqual(self.task.sides, [])
        self.task.step(due+1)
        self.assertEqual(self.audio_ids(), [20])

    def test_old_inflight_request_after_restart_is_discarded(self):
        self.pool.manual = True
        self.hub.publish(np.zeros((20, 30, 3), np.uint8), 0)
        self.recognizer.recognize.return_value = candidate("suspect_08")
        self.task.step(0)
        future = self.task.future
        future.set_running_or_notify_cancel()
        self.task.start(.1, TARGET)
        self.pool.finish()
        self.task.step(.2)
        self.assertEqual(self.task.count, 0)
        self.assertEqual(self.commands, [])

    def test_settings_change_resets_streak_and_held_knife_evidence(self):
        self.confirm("knife_07")
        self.settings.update({"knife_threshold": .6})
        self.task.step(.2)
        self.assertEqual(self.task.knife_observations, {})
        self.assertEqual(self.task.deadline_s, 10)

    def test_only_explicit_correct_face_pair_is_allowed_for_inferred_speech(self):
        record = {"status": "inferred", "source": "face_pair", "category": "knife",
                  "identity": "knife_07", "based_on": "suspect_08",
                  "edge_id": TARGET.edge_id, "direction": "B"}
        self.assertEqual(inspection_audio_id(record), 29)
        for bad in ({**record, "source": "vision"}, {**record, "based_on": "suspect_03"},
                    {**record, "status": "unconfirmed"}, {**record, "category": "suspect"}):
            self.assertIsNone(inspection_audio_id(bad))
        self.assertTrue(self.speech.request(record))
        self.assertFalse(self.speech.request(record))
        self.assertEqual(self.audio_ids(), [29])


    def test_navigation_rejects_pair_flag_without_inspection_before_hardware(self):
        import contextlib
        import io
        from unittest.mock import patch
        from road_follow.__main__ import main
        with patch("road_follow.inspection_io.SideCamera") as camera, patch(
                "road_follow.inspection_io.ServoBridge") as servo, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main(["-b"])
        self.assertEqual(error.exception.code, 2)
        camera.assert_not_called()
        servo.assert_not_called()

if __name__ == "__main__":
    unittest.main()
