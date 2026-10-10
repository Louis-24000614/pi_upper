"""把涵洞状态机接入现有拓扑主循环，唯一控制方消费动作队列。"""
from __future__ import annotations

from dataclasses import asdict, replace
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import queue
import zlib

import yaml

from road_follow.control import VelocityCommand
from road_follow.culvert import CulvertConfig, CulvertController, CulvertOutcome, OdomHistory
from road_follow.culvert_map import CulvertRecords
from road_follow.culvert_perception import CulvertCalibration, CulvertPerception
from road_follow.speech import InspectionSpeech, SpeechConfig
from vision.obstacle.blockage import HardBlockageJudge, hard_block_config_from_mapping


def validate_culvert_config(path, root, *, drive, image_size, near_speed,
                            estimated_camera=False, nav_config=None):
    mapping = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    config = CulvertConfig.from_mapping(mapping.get("culvert", {}))
    classes = mapping.get("class_mapping", {})
    model = mapping["model"]
    fingerprint = str(model.get("sha256", "")).lower()
    if not classes.get("verified") or classes.get("model_sha256") != fingerprint:
        raise ValueError("涵洞类别映射尚未核实或不属于当前模型")
    count = model["classes"]["count"]
    culvert_id = classes.get("culvert_class_id")
    hard_ids = classes.get("hard_block_class_ids", [])
    if not isinstance(culvert_id, int) or not 0 <= culvert_id < count:
        raise ValueError("涵洞类别编号不合法")
    if any(not isinstance(i,int) or i < 0 or i >= count or i == culvert_id for i in hard_ids):
        raise ValueError("硬障碍类别编号不合法或与涵洞重叠")
    if len(hard_ids) != len(set(hard_ids)):
        raise ValueError("硬障碍类别编号重复")
    if not mapping.get("detect", {}).get("class_aware_nms") or not model.get("verify_sha256"):
        raise ValueError("涵洞模式必须启用按类NMS和模型指纹检查")
    model_path = Path(root)/model["path"]
    if not model_path.is_file() or hashlib.sha256(model_path.read_bytes()).hexdigest() != fingerprint:
        raise ValueError("涵洞模型缺失或SHA256不匹配")
    calibration = (CulvertCalibration.from_camera_parameters(nav_config or {}, image_size)
                   if estimated_camera else CulvertCalibration(mapping.get("calibration", {})))
    if drive and (not calibration.valid or tuple(image_size) != calibration.size):
        raise ValueError("自动涵洞停车需要已核实的四点地面标定，且原始图像尺寸必须一致")
    if not math.isfinite(near_speed) or near_speed <= 0:
        raise ValueError("涵洞进入使用的 near_mps 必须大于零")
    return mapping, config, calibration


def culvert_eligible(entrance, agent, junction, recovery, rfid_arrival):
    return (entrance.phase == "done" and agent is not None and agent.state.phase == "following"
            and agent.state.current_edge is not None and junction.phase in ("follow", "approach")
            and recovery.phase == "idle" and rfid_arrival.phase == "follow")


def drain_notes(notes):
    received = []
    while True:
        try:
            received.append(notes.get_nowait())
        except queue.Empty:
            return received


class CulvertRuntime:
    def __init__(self, mapping, config, calibration, *, nav_config, agent, send, event, root, prefix=None,
                 inspection_settings=None, inspection_web=False, speech_config=None):
        self.inspection_settings = inspection_settings
        self.obstacle_revision = None
        self.obstacle_judge_revisions = {}
        self.last_obstacle_details = []
        self.mapping, self.config, self.calibration = mapping, config, calibration
        self.nav_config, self.agent, self.console_event = nav_config, agent, event
        self.history = OdomHistory(config)
        self.perception = CulvertPerception(config, calibration, mapping["class_mapping"]["culvert_class_id"])
        self.hard_ids = set(mapping["class_mapping"]["hard_block_class_ids"])
        self.culvert_id = mapping["class_mapping"]["culvert_class_id"]
        self.unknown_judge = HardBlockageJudge(hard_block_config_from_mapping(mapping))
        prefix = Path(prefix) if prefix else Path(root)/"data"/"road"/("culvert_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self.records = CulvertRecords(agent.graph, prefix)
        self.log = Path(str(prefix)+".culvert.jsonl").open("x", encoding="utf-8")
        self.speech = InspectionSpeech(speech_config or SpeechConfig(), send, self.event,
                                       self.records.session_id)
        executor = None
        try:
            if inspection_settings is not None:
                from road_follow.inspection import create_inspection
                executor = create_inspection(inspection_settings, root, web_enabled=inspection_web, event=self.event,
                                             notify=self.speech.request, speech_enabled=self.speech.enabled)
                if executor.web is not None:
                    executor.web.map_status = self.records.topology_snapshot
                    executor.web.map_svg = self.records.svg
                    executor.web.obstacle_status = lambda: self.last_obstacle_details
            self.controller = CulvertController(config, self.history, send=send, event=self.event,
                                                records=self.records, executor=executor)
        except BaseException:
            if executor is not None:
                executor.close()
            self.log.close()
            raise
        self.last_key = None
        self.first_capture_s = None
        self.paused_s = None
        self.event("culvert_projection", **calibration.metadata)

    def close(self):
        try:
            if hasattr(self.controller.executor, "close"):
                self.controller.executor.close()
        finally:
            try:
                self.records.flush()
            finally:
                self.log.close()

    def edge_key(self):
        state = self.agent.state
        return (state.current_edge, state.from_node, state.to_node)

    def handle_speech_note(self, text):
        return self.speech.handle_note(text)

    def record_odom(self, sample, progress):
        if len(sample) != 4:
            raise ValueError("涵洞模式需要带接收时钟的ODOM；不能使用旧桥接接口")
        x, y, yaw, received_s = sample
        self.history.add(self.edge_key(), received_s, progress.s_m, x, y, yaw)

    def event(self, kind, **details):
        payload = {"event":kind, "edge":self.edge_key(), **details}
        self.log.write(json.dumps(payload, ensure_ascii=False)+"\n")
        self.log.flush()
        if kind == "culvert_phase":
            self.console_event("涵洞", f"{details['before']} → {details['phase']}；{details.get('reason','')}")

    def split(self, detections):
        return ([d for d in detections if d.class_id in self.hard_ids],
                [d for d in detections if d.class_id == self.culvert_id],
                 [d for d in detections if d.class_id not in self.hard_ids and d.class_id != self.culvert_id])

    def obstacle_settings(self, judge=None):
        from road_follow.obstacle_edge import OBSTACLE_DEFAULTS
        values, revision = (self.inspection_settings.obstacle_snapshot() if self.inspection_settings is not None
                            else (dict(OBSTACLE_DEFAULTS), 0))
        for target in (judge, self.unknown_judge):
            if target is not None and self.obstacle_judge_revisions.get(id(target)) != revision:
                target.config = replace(target.config, **{key: values[key] for key in
                    ("min_score", "min_bottom_ratio", "confirm_frames")})
                target.reset()
                self.obstacle_judge_revisions[id(target)] = revision
        if revision != self.obstacle_revision:
            self.obstacle_revision = revision
            self.event("obstacle_settings", revision=revision, values=values)
        return values

    def observe_obstacles(self, details, observation):
        self.last_obstacle_details = details
        self.records.observe_obstacles(details, observation.bbox if observation.just_confirmed else None)
        if details:
            self.event("obstacle_observation", details=details, confirmed=observation.just_confirmed,
                       stable_frames=observation.stable_frames, reason=observation.reason)

    def cancel_for_obstacle(self, now, notes):
        if self.controller.owns:
            self.controller.cancel_for_obstacle(now)
            drain_notes(notes)
            self.perception.reset()
            self.paused_s = None

    def update(self, *, detections, frame, mask, captured_s, sequence, source, now, progress,
               command, eligible, notes, navigation_diagnostics=None, inference_s=None):
        key = self.edge_key()
        if key != self.last_key:
            self.unknown_judge.reset()
            self.perception.reset()
            self.last_key = key
            if self.history.key != key:
                self.history.reset(key)
        hard, culverts, unknown = self.split(detections)
        if unknown:
            from road_follow.obstacle_edge import filter_current_edge
            unknown, _ = filter_current_edge(unknown, graph=self.agent.graph, edge_key=key,
                progress_m=self.history.progress_at(captured_s), ipm=self.calibration.ipm,
                image_shape=frame.shape, settings=self.obstacle_settings(),
                lane_x_m=getattr(navigation_diagnostics, "near_x_m", 0))
        if (eligible or self.controller.owns) and self.unknown_judge.update(unknown, frame.shape).hard_blocked:
            self.controller.fault("unknown_near_target", now)
        was_inspecting = self.controller.phase == "task"
        frame_age_s = now-captured_s
        frame_fresh = math.isfinite(frame_age_s) and frame_age_s >= 0
        road_safe = frame_fresh and command.v_mps > 0 and command.reason in ("follow", "follow_near")
        if eligible and not self.controller.owns:
            points = self.calibration.road_points(mask, self.nav_config) if self.calibration.check_shape(frame.shape) else []
            edge = self.agent.graph.edges[key[0]]
            observation = self.perception.observe(culverts, image_shape=frame.shape, captured_s=captured_s,
                now=now, frame_signature=zlib.crc32(frame), edge=edge, from_node=key[1], to_node=key[2],
                points=points, history=self.history, already_done=self.records.handled(edge.id))
            if self.first_capture_s is None:
                self.first_capture_s = captured_s
            self.event("culvert_observation", captured_s=captured_s, sequence=sequence, source=source,
                       video_relative_s=captured_s-self.first_capture_s,
                       detections=[asdict(d) for d in detections],
                       reason=observation.reason, stable_frames=observation.stable_frames,
                       bbox=observation.bbox, entry_distance_m=observation.distance_m,
                       corrected_entry_distance_m=observation.corrected_distance_m,
                       entry_distance_bias_m=self.config.entry_distance_bias_m,
                       lateral_m=observation.lateral_m, progress_m=progress.s_m)
            if observation.target and road_safe:
                drain_notes(notes)  # 接管前清理已经结束的旧动作消息。
                near_speed = float((self.nav_config.get("follow") or {}).get("near_mps", .05))
                if self.controller.begin(observation.target, now, progress.s_m, near_speed):
                    self.paused_s = now
        if not self.controller.owns:
            return CulvertOutcome(command, False)
        near_speed = float((self.nav_config.get("follow") or {}).get("near_mps", .05))
        received = drain_notes(notes)
        outcome = self.controller.step(now=now, current_s=progress.s_m, edge_key=key,
            visual_ok=road_safe, frame_s=captured_s, command=command, near_speed=near_speed, notes=received)
        if was_inspecting and self.controller.phase == "reacquire":
            # 完成识别的这一帧保持停车，下一轮只接收新的导航帧。
            outcome = CulvertOutcome(outcome.command, outcome.owns, False, outcome.resumed)
        self.event("culvert_control", now_s=now, phase=self.controller.phase,
                   progress_m=progress.s_m, target_s_m=(self.controller.target.target_s_m if self.controller.target else None),
                   notes=received, command=asdict(outcome.command), input_command=asdict(command),
                   captured_s=captured_s, frame_age_s=frame_age_s, inference_s=inference_s,
                   navigation_diagnostics=asdict(navigation_diagnostics) if navigation_diagnostics is not None else None)
        return outcome

    def unsafe_frame(self, now, reason):
        self.perception.reset()
        if self.controller.owns:
            self.controller.fault(reason, now)

    def reset_progress_origin(self):
        self.history.reset(self.edge_key())
        self.perception.reset()
        self.event("culvert_progress_origin_reset")

    def resume(self, junction, rfid_arrival, tracker, smoother, stream):
        # 只清除视觉证据。当前边、路线索引、已走里程和预定转向保持原值。
        junction.phase = "follow"
        junction.arm = junction.clear = junction.approach_age = 0
        junction.branch_latched = False
        junction.branch_votes.clear()
        junction.align_stable = 0
        junction.hold_start_m = junction.hold_yaw_rad = None
        rfid_arrival.phase = "follow"
        rfid_arrival.edge_seen_frames = rfid_arrival.road_end_missing_frames = 0
        rfid_arrival.odom_handoff_frames = rfid_arrival.align_stable = 0
        rfid_arrival.edge_latched = rfid_arrival.edge_left_seen = rfid_arrival.edge_right_seen = False
        rfid_arrival.hold_start_m = rfid_arrival.hold_yaw_rad = None
        smoother.reset()
        if stream is not None:
            stream.invalidate()
        self.paused_s = None
        self.perception.reset()
