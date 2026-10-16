"""涵洞遇障中断后的恢复回归：合成帧和假 PWM/识别器，不连接设备。"""
from concurrent.futures import Future
from copy import deepcopy
import unittest
from unittest.mock import Mock

import numpy as np

from road_follow.control import VelocityCommand
from road_follow.culvert import CulvertConfig, CulvertController, CulvertTarget, OdomHistory
from road_follow.culvert_map import CulvertRecords
from topo_proto.graph import Edge, Node, TopologyGraph
from road_follow.inspection import SideInspection
from road_follow.inspection_io import FrameHub


TARGET = CulvertTarget("4_3__5_3", "4_3", "5_3", .5, .635, .77, .635, 0)
CANDIDATE = {"accepted": True, "category": "knife", "identity": "knife_01",
             "score": .5, "reason": "accepted"}


class FakeServo:
    def __init__(self):
        self.closed, self.state = False, "done"
        self.angles, self.close_calls = [], 0

    def request(self, angle):
        if self.closed:
            raise RuntimeError("PWM bridge closed")
        self.angles.append(angle)
        return str(len(self.angles))

    def poll(self, token):
        return self.state

    def close(self):
        self.closed = True
        self.close_calls += 1


class FakePool:
    def __init__(self):
        self.jobs, self.manual = [], False

    def submit(self, fn, *args):
        future = Future()
        self.jobs.append((future, fn, args))
        if not self.manual:
            future.set_result(fn(*args))
        return future

    def finish(self):
        future, fn, args = self.jobs[-1]
        future.set_result(fn(*args))

    def shutdown(self, **kwargs):
        pass


class ObstacleRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.config = {
            "recognition": {"confirm_frames": 3, "side_timeout_s": 45.0,
                            "sample_interval_s": 0},
            "servo": {"initial_side": "A", "angles": {"A": 0, "B": 180},
                      "ack_timeout_s": 2.0, "settle_s": 3.0},
        }
        self.settings = Mock()
        self.settings.snapshot.side_effect = lambda: (deepcopy(self.config), 1)
        self.hub, self.servo, self.pool = FrameHub(), FakeServo(), FakePool()
        self.recognizer = Mock()
        self.recognizer.recognize.return_value = CANDIDATE
        self.notifications = []
        self.task = SideInspection(self.settings, self.hub, self.servo, self.recognizer,
                                   notify=self.notifications.append, pool=self.pool)
        self.addCleanup(self.task.close)

    def publish(self, now):
        self.hub.publish(np.zeros((20, 30, 3), np.uint8), now)

    def vote(self, now):
        self.publish(now)
        self.task.step(now)
        return self.task.step(now + .01)

    def confirm(self, now):
        for offset in (0, .1, .2):
            result = self.vote(now + offset)
        return result

    def settle(self, now):
        self.task.step(now)
        self.assertEqual(self.task.phase, "settling")
        self.task.step(now + 3.01)
        self.assertEqual(self.task.phase, "recognizing")

    def test_cancel_before_inspection_allows_next_task_and_normal_reuse(self):
        # 遇障发生在驶入阶段，侧视执行器尚未 start。
        self.task.cancel("hard_obstacle")
        self.task.cancel("hard_obstacle")
        self.assertFalse(self.servo.closed)
        self.assertEqual(self.servo.angles, [])
        self.task.start(1, TARGET)
        self.settle(1.1)
        self.confirm(4.2)
        self.settle(4.5)
        self.assertEqual(self.confirm(7.6), "done")
        self.assertEqual(self.task.result["status"], "done")
        self.assertEqual([s["direction"] for s in self.task.result["sides"]], ["A", "B"])
        self.assertEqual(len(self.notifications), 2)
        self.assertEqual(self.servo.angles, [0, 180])
        self.assertFalse(self.servo.closed)
        self.task.start(8, TARGET)
        self.assertEqual(self.task.phase, "recognizing")
        self.assertEqual(self.task.side, "B")
        self.assertEqual(self.servo.angles, [0, 180])

    def test_interrupted_turn_waits_for_new_ack_settle_and_fresh_frame(self):
        self.task.start(0, TARGET)
        self.confirm(0)
        self.assertEqual(self.servo.angles, [180])
        self.assertEqual(self.task.side, "A")
        self.task.cancel("hard_obstacle")
        self.assertFalse(self.servo.closed)
        self.servo.state = "pending"
        self.task.start(1, TARGET)
        self.assertEqual(self.servo.angles, [180, 0])
        self.publish(1.1)
        self.task.step(1.1)
        self.assertEqual(self.task.phase, "turning")
        self.servo.state = "done"
        self.task.step(1.2)
        self.publish(4.1)
        self.task.step(4.1)
        self.assertEqual(self.task.phase, "settling")
        self.assertEqual(self.recognizer.recognize.call_count, 3)
        self.task.step(4.21)
        self.task.step(4.22)
        self.assertEqual(self.task.phase, "recognizing")
        self.assertEqual(self.task.count, 0)
        self.assertEqual(self.recognizer.recognize.call_count, 3)
        self.vote(4.3)
        self.assertEqual(self.task.count, 1)

    def test_interrupted_settling_keeps_last_confirmed_side(self):
        self.task.start(0, TARGET)
        self.confirm(0)
        self.task.step(.3)
        self.assertEqual(self.task.side, "B")
        self.assertEqual(self.task.phase, "settling")
        self.task.cancel("hard_obstacle")
        self.task.start(1, TARGET)
        self.assertEqual(self.servo.angles, [180, 180])
        self.settle(1.1)
        self.assertEqual(self.task.side, "B")
        self.assertEqual(self.task.sides, [])

    def test_running_old_recognition_does_not_count_after_restart(self):
        self.task.start(0, TARGET)
        self.pool.manual = True
        self.publish(0)
        self.task.step(0)
        old_future = self.task.future
        self.assertTrue(old_future.set_running_or_notify_cancel())
        self.task.pending_announcement = (1, CANDIDATE)
        self.task.cancel("hard_obstacle")
        self.assertIsNone(self.task.pending_announcement)
        self.task.start(1, TARGET)
        self.settle(1.1)
        self.pool.finish()
        self.task.step(4.2)
        self.assertIsNone(self.task.future)
        self.assertEqual(self.task.count, 0)
        self.assertIsNone(self.task.candidate)
        self.assertEqual(self.notifications, [])
        self.pool.manual = False
        self.vote(4.3)
        self.assertEqual(self.task.count, 1)

    def test_initial_turn_still_selects_configured_side_after_obstacle(self):
        self.task.start(0, TARGET)
        self.confirm(0)
        self.task.step(.3)
        self.task.cancel("hard_obstacle")
        self.task.start(1, TARGET, initial_turn=True)
        self.assertEqual(self.servo.angles, [180, 0])
        self.settle(1.1)
        self.assertEqual(self.task.side, "A")

    def test_navigation_fault_and_default_cancel_still_close(self):
        for reason in ("navigation_fault", "cancelled"):
            with self.subTest(reason=reason):
                servo = FakeServo()
                task = SideInspection(self.settings, FrameHub(), servo, self.recognizer,
                                      pool=FakePool())
                self.addCleanup(task.close)
                task.start(0, TARGET)
                if reason == "cancelled":
                    task.cancel()
                else:
                    task.cancel(reason)
                self.assertTrue(servo.closed)
                with self.assertRaises(RuntimeError):
                    task.start(1, TARGET)

    def test_final_close_after_obstacle_closes_once_and_rejects_restart(self):
        self.task.cancel("hard_obstacle")
        self.assertEqual(self.servo.close_calls, 0)
        self.task.close()
        self.task.close()
        self.assertEqual(self.servo.close_calls, 1)
        with self.assertRaises(RuntimeError):
            self.task.start(1, TARGET)

    def test_recovery_servo_failure_still_closes_bridge(self):
        self.task.cancel("hard_obstacle")
        self.task.start(1, TARGET)
        self.servo.state = "failed"
        self.assertEqual(self.task.step(1.1), "failed")
        self.assertEqual(self.task.failure_reason, "servo_write_failed_or_timeout")
        self.assertTrue(self.servo.closed)

    def test_recovery_servo_ack_timeout_still_closes_bridge(self):
        self.task.cancel("hard_obstacle")
        self.task.start(1, TARGET)
        self.servo.state = "pending"
        self.assertEqual(self.task.step(3), "failed")
        self.assertEqual(self.task.failure_reason, "servo_write_failed_or_timeout")
        self.assertTrue(self.servo.closed)

    def test_controller_completes_next_culvert_after_entry_obstacle(self):
        # 日志中的顺序：完成一处涵洞 -> 下一处驶入遇障 -> 再下一处检查。
        keys = (("1_2__2_2", "1_2", "2_2"),
                ("2_2__2_1", "2_2", "2_1"),
                ("2_3__1_3", "2_3", "1_3"))
        nodes = {name: Node(name, row, column)
                 for name, row, column in (("1_2", 1, 2), ("2_2", 2, 2),
                                           ("2_1", 2, 1), ("2_3", 2, 3),
                                           ("1_3", 1, 3))}
        graph = TopologyGraph({}, nodes,
                              {key[0]: Edge(*key, .98) for key in keys})
        records = CulvertRecords(graph)
        config = CulvertConfig()
        history = OdomHistory(config)
        sent, events = [], []
        control = CulvertController(
            config, history, send=lambda line: sent.append(line) or True,
            event=lambda name, **detail: events.append((name, detail)),
            records=records, executor=self.task)
        targets = [CulvertTarget(*key, .5, .635, .77, .635, 0) for key in keys]

        def step(target, now, notes=()):
            key = (target.edge_id, target.from_node, target.to_node)
            history.add(key, now, target.target_s_m, target.target_s_m, 0, 0)
            if self.task.phase == "recognizing":
                self.publish(now)
            return control.step(
                now=now, current_s=target.target_s_m, edge_key=key,
                visual_ok=True, frame_s=now, command=VelocityCommand(.08, 0, "follow"),
                near_speed=.05, notes=notes)

        def complete(target, start):
            self.assertTrue(control.begin(target, start, .5, .05))
            for index in range(1, 151):
                now = start + index / 10
                step(target, now, ("STOP_DONE",) if index == 2 else ())
                self.assertNotEqual(control.phase, "fault", events)
                if control.phase == "reacquire":
                    break
            self.assertEqual(control.phase, "reacquire")
            self.assertTrue(records.done(target.edge_id))
            self.assertEqual(len(records.entries[target.edge_id]["inspection"]["sides"]), 2)
            for index in range(1, config.reacquire_frames + 1):
                step(target, now + index / 10)
            self.assertEqual(control.phase, "idle")
            return now + 1

        now = complete(targets[0], 0)
        self.assertTrue(control.begin(targets[1], now, .5, .05))
        self.assertEqual(control.phase, "entering")
        control.cancel_for_obstacle(now + .1)
        self.assertEqual(control.phase, "idle")
        self.assertFalse(self.servo.closed)
        self.assertEqual(records.entries[keys[1][0]]["failure_reason"], "hard_obstacle")
        complete(targets[2], now + 1)
        self.assertEqual(records.entries[keys[1][0]]["status"], "failed")
        self.assertIsNone(control.fault_reason)
        self.assertEqual(sent, ["stop", "stop"])
        self.assertEqual(len(self.notifications), 4)
        self.assertFalse(any(name == "culvert_task_error" for name, _ in events))


if __name__ == "__main__":
    unittest.main()
