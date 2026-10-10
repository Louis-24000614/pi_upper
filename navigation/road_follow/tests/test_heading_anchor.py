"""航向参考离线验证：合成视觉/ODOM/回执，不打开任何设备。"""
import io
import queue
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from road_follow.control import HeadingAnchorGate, VelocityCommand, step_heading_anchor
from road_follow.__main__ import _watch_uart_notes
from road_follow.arrival_policy import resolve_arrival_policy, step_route_arrival
from road_follow.junction_turn import JunctionTurn, JunctionTurnConfig
from road_follow.rfid_arrival import RfidArrival, RfidArrivalConfig
from ipm_proto.junction import KIND_CROSS, JunctionRead
from navigation.topo_proto.graph import load_topology


class HeadingAnchorTest(unittest.TestCase):
    def setUp(self):
        self.gate = HeadingAnchorGate(started_s=1.0)
        self.notes = queue.Queue()
        self.sent = []
        self.phase = "align"

    def step(self, now, **overrides):
        args = dict(lane_heading_rad=0.01, frame_captured_s=now, visual_safe=True,
                    now_s=now, odom_valid=True, odom_received_s=now, yaw_rad=0.6,
                    notes=self.notes, send=lambda line: self.sent.append(line) or True,
                    max_abs_heading_rad=0.05, stable_frames=3, align_timeout_s=1.5,
                    align_gain=4.0, max_abs_omega=0.5)
        args.update(overrides)
        self.phase, command, ready = step_heading_anchor(self.gate, self.phase, **args)
        return command, ready

    def submit(self):
        for t in (1.0, 1.05, 1.1):
            command, ready = self.step(t)
            self.assertEqual((command.v_mps, command.omega_radps), (0, 0))
            self.assertFalse(ready)
        self.assertEqual(self.phase, "anchor_wait")
        self.assertEqual(self.sent, ["0 0", "anchor_heading"])

    def test_already_aligned_needs_distinct_stable_frames_and_only_one_request(self):
        self.step(1)
        self.step(1.01, frame_captured_s=1)
        self.assertEqual(self.gate.stable_frames, 1)
        self.step(1.05)
        self.step(1.1)
        self.step(1.15)
        self.assertEqual(self.sent, ["0 0", "anchor_heading"])
        self.assertEqual(self.phase, "anchor_wait")

    def test_missing_stale_future_or_unsafe_evidence_resets_count(self):
        for bad in (dict(lane_heading_rad=None), dict(lane_heading_rad=float("nan")),
                    dict(frame_captured_s=0.5), dict(frame_captured_s=2),
                    dict(visual_safe=False), dict(odom_valid=False)):
            with self.subTest(bad=bad):
                self.gate = HeadingAnchorGate(started_s=1)
                self.phase = "align"
                self.step(1)
                command, ready = self.step(1.05, **bad)
                self.assertEqual(self.gate.stable_frames, 0)
                self.assertFalse(ready)
                self.assertEqual((command.v_mps, command.omega_radps), (0, 0))
                self.assertEqual(self.sent, [])

    def test_out_of_band_steers_but_timeout_never_submits(self):
        command, _ = self.step(1, lane_heading_rad=0.2)
        self.assertNotEqual(command.omega_radps, 0)
        command, ready = self.step(2.5)
        self.assertEqual(self.phase, "fault")
        self.assertEqual(self.sent, ["stop"])
        self.assertFalse(ready)
        self.assertEqual(command.v_mps, 0)

    def test_old_ack_is_discarded_and_success_requires_odom_after_new_ack(self):
        self.notes.put(("ANCHOR_DONE", 0.9))
        self.submit()
        self.notes.put(("ANCHOR_DONE", 1.09))
        self.step(1.15)
        self.assertIsNone(self.gate.acknowledged_s)
        self.notes.put(("ANCHOR_DONE", 1.16))
        self.step(1.2, odom_received_s=1.15)
        self.assertEqual(self.phase, "anchor_wait")
        _, ready = self.step(1.25, odom_received_s=1.21)
        self.assertTrue(ready)
        self.assertEqual(self.phase, "heading_hold")

    def test_ack_without_valid_yaw_or_fresh_odom_keeps_stopped(self):
        self.submit()
        self.notes.put(("ANCHOR_DONE", 1.12))
        for t, args in ((1.2, dict(yaw_rad=None)), (1.25, dict(yaw_rad=float("nan"))),
                        (1.3, dict(odom_received_s=1.1)), (1.8, dict(odom_received_s=1.2))):
            command, ready = self.step(t, **args)
            self.assertFalse(ready)
            self.assertEqual((command.v_mps, command.omega_radps), (0, 0))

    def test_rejection_and_evidence_loss_stop_without_replaying(self):
        for lost in (False, True):
            with self.subTest(lost=lost):
                self.setUp()
                self.submit()
                if not lost:
                    self.notes.put(("ANCHOR_FAIL ACK_TIMEOUT", 1.12))
                command, ready = self.step(1.2, visual_safe=not lost)
                self.assertEqual(self.phase, "fault")
                self.assertEqual(self.sent, ["0 0", "anchor_heading", "stop"])
                self.assertFalse(ready)
                self.assertEqual(command.v_mps, 0)

    def test_submission_failure_and_no_ack_timeout_stop(self):
        self.submit()
        _, ready = self.step(4.11)
        self.assertEqual(self.phase, "fault")
        self.assertFalse(ready)
        self.setUp()
        self.step(1)
        self.step(1.05)
        self.step(1.1, send=lambda line: self.sent.append(line) or line == "0 0")
        self.assertEqual(self.phase, "fault")
        self.assertEqual(self.gate.failure, "submit_failed")

    def test_watcher_separates_anchor_speech_motion_and_odom(self):
        action, rfid, odom, anchor, speech = [queue.Queue() for _ in range(5)]
        proc = SimpleNamespace(stdout=io.StringIO(
            "SPEECH_RESULT inspect-one 13 ACK_TIMEOUT\nANCHOR_DONE\n"
            "ODOM 1 2 .5 1\nTURN_DONE\n"))
        with patch("road_follow.__main__.time.monotonic", side_effect=[10, 11]):
            _watch_uart_notes(proc, action, rfid, odom,
                              anchor_notes=anchor, speech_notes=speech)
        self.assertEqual(anchor.get_nowait(), ("ANCHOR_DONE", 10))
        self.assertEqual(odom.get_nowait(), (1, 2, .5, 11))
        self.assertEqual(action.get_nowait(), "TURN_DONE")
        self.assertEqual(speech.get_nowait(), "SPEECH_RESULT inspect-one 13 ACK_TIMEOUT")
        self.assertTrue(rfid.empty())


class RouteAnchorTest(unittest.TestCase):
    def test_normal_and_patrol_routes_preserve_17_and_20_cm_after_anchor(self):
        for source, target, length, patrol in (
            ("1_2", "2_2", .97, False), ("0_J", "1_2", .4, False),
            ("2_1", "3_1", .97, True), ("0_J", "1_2", .4, True)):
            with self.subTest(source=source, patrol=patrol):
                jcfg, pcfg = JunctionTurnConfig(), RfidArrivalConfig()
                policy = resolve_arrival_policy(load_topology(), source, target, length, jcfg, pcfg)
                state = JunctionTurn(phase="follow" if patrol else "align")
                pstate = RfidArrival(phase="align")
                active = pstate if patrol else state
                active.anchor = HeadingAnchorGate(started_s=1)
                if patrol:
                    active.edge_latched = True
                notes, anchor_notes, sent = queue.Queue(), queue.Queue(), []
                reading = JunctionRead(KIND_CROSS, True, True, True, 0, .20, .50, 1, .85)

                def step(now, sample, progress=.8):
                    nonlocal state, pstate
                    state, pstate, command, _ = step_route_arrival(
                        policy, patrol=patrol, state=state, patrol_state=pstate,
                        reading=reading, opening=reading.kind, command=VelocityCommand(.08, 0, "follow"),
                        progress_m=progress, odom_valid=True, notes=notes,
                        send=lambda line: sent.append(line) or True, junction_cfg=jcfg, patrol_cfg=pcfg,
                        lane_heading_rad=0, yaw_rad=.7, visual_safe=True,
                        frame_captured_s=now, odom_received_s=sample, anchor_notes=anchor_notes,
                        now_s=now)
                    return command

                for i in range(policy.align_stable_frames):
                    command = step(1 + i*.03, 1 + i*.03)
                    self.assertEqual(command.v_mps, 0)
                self.assertEqual(sent, ["0 0", "anchor_heading"])
                anchor_notes.put(("ANCHOR_DONE", 1.4))
                self.assertEqual(step(1.42, 1.39).v_mps, 0)
                command = step(1.45, 1.44)
                active = pstate if patrol else state
                self.assertEqual(active.phase, "heading_hold")
                self.assertAlmostEqual(active.hold_start_m, .8)
                self.assertAlmostEqual(active.hold_yaw_rad, .7)
                self.assertGreater(command.v_mps, 0)
                step(1.5, 1.49, .8 + policy.final_forward_m)
                self.assertEqual((pstate if patrol else state).phase, "arrived")


if __name__ == "__main__":
    unittest.main()
