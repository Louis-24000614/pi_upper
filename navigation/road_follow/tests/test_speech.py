"""播报接口离线测试：假发送函数与桥消息，不打开串口或识别设备。"""
import json
from pathlib import Path
import tempfile
import unittest

from road_follow.speech import InspectionSpeech, SpeechConfig, inspection_audio_id, parse_speech_note


ROOT = Path(__file__).resolve().parents[3]


def record(kind="suspect", number=1, edge="a__b", side="A", status="confirmed"):
    prefix = "suspect" if kind == "suspect" else "knife"
    return {"status": status, "category": kind, "identity": f"{prefix}_{number:02d}",
            "edge_id": edge, "direction": side}


class SpeechConfigTest(unittest.TestCase):
    def config_file(self, values):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name) / "speech.json"
        path.write_text(json.dumps({"version": 1, "durations_ms": values}), encoding="utf-8")
        return path

    def test_delivery_config_has_no_guessed_durations_and_keeps_legacy_uid_args(self):
        config = SpeechConfig.load(ROOT / "config/speech.json")
        self.assertFalse(config.ready)
        self.assertEqual(config.missing_ids, tuple(range(1, 33)))
        self.assertEqual(config.bridge_args(), [])

    def test_complete_measured_table_generates_each_bridge_argument(self):
        values = {str(n): 1000 + n for n in range(1, 33)}
        config = SpeechConfig.load(self.config_file(values))
        self.assertTrue(config.ready)
        self.assertEqual(config.bridge_args()[:4], ["--speech-hold-ms", "1:1001", "--speech-hold-ms", "2:1002"])
        self.assertEqual(config.bridge_args()[-2:], ["--speech-hold-ms", "32:1032"])
        self.assertEqual(len(config.bridge_args()), 64)

    def test_partial_table_cannot_enable_recognition_or_change_uid_mode(self):
        values = {str(n): 1000 for n in range(1, 33)}
        values["8"] = None
        config = SpeechConfig.load(self.config_file(values))
        self.assertEqual(config.missing_ids, (8,))
        self.assertFalse(config.ready)
        self.assertEqual(config.bridge_args(), [])

    def test_bad_table_is_rejected(self):
        for value in (False, True, 0, -1, 1.5, "1000", 0x100000000):
            values = {str(n): 1000 for n in range(1, 33)}
            values["13"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                SpeechConfig.load(self.config_file(values))
        with self.assertRaises(ValueError):
            SpeechConfig.load(self.config_file({"1": None}))


class InspectionSpeechTest(unittest.TestCase):
    def setUp(self):
        self.sent, self.events = [], []
        self.speech = self.make_speech("task-one")

    def make_speech(self, session, config=None, sender=None):
        return InspectionSpeech(config or SpeechConfig((1000,) * 32),
                                sender or (lambda line: self.sent.append(line) or True),
                                lambda kind, **values: self.events.append({"event": kind, **values}),
                                session)

    def event_id(self):
        return self.sent[-1].split()[1]

    def test_all_twenty_identity_mappings_and_boundaries(self):
        for n in range(1, 11):
            self.assertEqual(inspection_audio_id(record("suspect", n)), 12+n)
            self.assertEqual(inspection_audio_id(record("knife", n)), 22+n)
        for item in (record(number=0), record(number=11), record(kind="other"),
                     {**record(), "identity": "suspect_1"}, {**record(), "identity": "knife_01"}):
            self.assertIsNone(inspection_audio_id(item))

    def test_unconfirmed_or_invalid_results_never_send(self):
        self.assertFalse(self.speech.request(record(status="unconfirmed")))
        self.assertFalse(self.speech.request(record(number=11)))
        self.assertFalse(self.speech.request({**record(), "direction": "C"}))
        self.assertEqual(self.sent, [])

    def test_dedup_scope_is_task_edge_and_side_not_global_identity(self):
        self.assertTrue(self.speech.request(record()))
        first_id = self.event_id()
        self.assertFalse(self.speech.request(record()))
        self.assertFalse(self.speech.request(record(number=2)))
        self.assertTrue(self.speech.request(record(side="B")))
        self.assertTrue(self.speech.request(record(edge="b__c")))
        self.assertTrue(self.make_speech("task-two").request(record()))
        self.assertEqual(len(self.sent), 4)
        self.assertNotEqual(self.event_id(), first_id)
        self.assertTrue(all(line.startswith("speech inspect-") for line in self.sent))

    def test_missing_durations_are_explicit_and_do_not_submit_or_retry(self):
        blocked = self.make_speech("task-missing", SpeechConfig())
        self.assertFalse(blocked.request(record()))
        self.assertFalse(blocked.request(record()))
        self.assertEqual(self.sent, [])
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.events[0]["reason"], "missing_durations")
        self.assertEqual(self.events[0]["missing_ids"], list(range(1, 33)))

    def test_bridge_acceptance_is_not_playback_completion(self):
        self.speech.request(record())
        token = self.event_id()
        for text in (f"SPEECH_QUEUED {token} 13", f"SPEECH_SENT {token} 13",
                     f"SPEECH_RESULT {token} 13 ACK_OK"):
            self.assertTrue(self.speech.handle_note(text))
        self.assertEqual(self.speech.requests[token]["state"], "accepted_by_lower")
        self.assertFalse(self.events[-1]["playback_verified"])
        self.speech.handle_note(f"SPEECH_QUEUED {token} 13")
        self.assertEqual(self.speech.requests[token]["state"], "accepted_by_lower")

    def test_reject_timeout_and_link_loss_are_recorded_without_replay(self):
        for number, result in ((1, "ACK_TIMEOUT"), (2, "LINK_LOST")):
            item = record(edge=f"edge_{number}")
            self.speech.request(item)
            token = self.event_id()
            self.speech.handle_note(f"SPEECH_RESULT {token} 13 {result}")
            self.assertEqual(self.speech.requests[token]["state"], "unknown")
            self.assertFalse(self.speech.request(item))
        self.speech.request(record(edge="full"))
        token = self.event_id()
        self.speech.handle_note(f"SPEECH_REJECT {token} 13 queue_full")
        self.assertEqual(self.speech.requests[token]["state"], "rejected")
        self.assertFalse(self.speech.request(record(edge="full")))
        self.assertEqual(len(self.sent), 3)

    def test_wrong_audio_id_or_unknown_uid_note_does_not_complete_inspection(self):
        self.speech.request(record())
        token = self.event_id()
        self.speech.handle_note(f"SPEECH_RESULT {token} 23 ACK_OK")
        self.assertEqual(self.speech.requests[token]["state"], "submitted")
        self.assertTrue(self.speech.handle_note("SPEECH_RESULT uid-1 8 ACK_OK"))
        self.assertEqual(self.events[-1]["event"], "speech_bridge")

    def test_bridge_unavailable_or_exception_is_nonfatal_and_not_retried(self):
        failed = self.make_speech("write-failed", sender=lambda line: False)
        self.assertFalse(failed.request(record()))
        self.assertFalse(failed.request(record()))
        def broken(line):
            raise BrokenPipeError("closed")
        raised = self.make_speech("write-error", sender=broken)
        self.assertFalse(raised.request(record()))
        self.assertEqual(self.events[-1]["state"], "submit_failed")

    def test_only_well_formed_bridge_messages_are_parsed(self):
        self.assertIsNotNone(parse_speech_note("SPEECH_REJECT uid-3 12 queue_full"))
        for text in ("STOP_DONE", "SPEECH_DONE x 13", "SPEECH_SENT x 0", "SPEECH_SENT x 33",
                     "SPEECH_RESULT x 13", "SPEECH_QUEUED x nope", "SPEECH_SENT x 13 extra"):
            self.assertIsNone(parse_speech_note(text))


if __name__ == "__main__":
    unittest.main()
