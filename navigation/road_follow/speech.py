"""识别结果到已有 UART 桥的播报请求；不操作设备、不把 ACK 当播放完成。"""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re


_EVENT_ID = re.compile(r"[A-Za-z0-9_.:-]{1,96}\Z")
_KINDS = {"SPEECH_QUEUED", "SPEECH_REJECT", "SPEECH_SENT", "SPEECH_RESULT"}


@dataclass(frozen=True)
class SpeechConfig:
    durations_ms: tuple = (None,) * 32

    def __post_init__(self):
        if len(self.durations_ms) != 32:
            raise ValueError("语音时长表必须包含 1～32 号音轨")
        for value in self.durations_ms:
            if value is not None and (type(value) is not int or not 1 <= value <= 0xFFFFFFFF):
                raise ValueError("音轨时长须为已测的正整数毫秒，未知时填 null")

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError("不支持的语音配置版本")
        values = data.get("durations_ms")
        keys = {str(n) for n in range(1, 33)}
        if not isinstance(values, dict) or set(values) != keys:
            raise ValueError("语音时长表须明确列出 1～32，未知音轨填 null")
        return cls(tuple(values[str(n)] for n in range(1, 33)))

    @property
    def missing_ids(self):
        return tuple(n for n, value in enumerate(self.durations_ms, 1) if value is None)

    @property
    def ready(self):
        return not self.missing_ids

    def bridge_args(self):
        # 部分表不改变原 UID 流程；完整表才切入统一、按实际时长排程的模式。
        if not self.ready:
            return []
        return [arg for n, ms in enumerate(self.durations_ms, 1)
                for arg in ("--speech-hold-ms", f"{n}:{ms}")]


def inspection_audio_id(record):
    if not isinstance(record, dict) or record.get("status") != "confirmed":
        return None
    category, identity = record.get("category"), record.get("identity")
    if not isinstance(identity, str):
        return None
    for kind, prefix, offset in (("suspect", "suspect_", 12), ("knife", "knife_", 22)):
        if category == kind and identity in {f"{prefix}{n:02d}" for n in range(1, 11)}:
            return offset + int(identity[len(prefix):])
    return None


def parse_speech_note(text):
    fields = text.split()
    if not fields or fields[0] not in _KINDS:
        return None
    expected = 4 if fields[0] in ("SPEECH_REJECT", "SPEECH_RESULT") else 3
    if len(fields) != expected or not _EVENT_ID.fullmatch(fields[1]):
        return None
    try:
        audio_id = int(fields[2])
    except ValueError:
        return None
    if not 1 <= audio_id <= 32:
        return None
    return {"kind": fields[0], "event_id": fields[1], "audio_id": audio_id,
            "result": fields[3] if expected == 4 else None}


class InspectionSpeech:
    """一次任务内按涵洞边与 A/B 侧去重；失败只记录，不自动重新提交。"""
    def __init__(self, config, send, event, session_id):
        self.config, self.send, self.event = config, send, event
        self.session_id = session_id
        self.seen = set()
        self.requests = {}

    @property
    def enabled(self):
        return self.config.ready

    def request(self, record):
        if not isinstance(record, dict) or record.get("status") != "confirmed":
            return False
        audio_id = inspection_audio_id(record)
        edge_id, side = record.get("edge_id"), record.get("direction")
        if audio_id is None or not isinstance(edge_id, str) or not edge_id or side not in ("A", "B"):
            self.event("inspection_speech", state="rejected", reason="invalid_confirmed_identity_or_scope")
            return False
        key = (self.session_id, edge_id, side)
        if key in self.seen:
            return False
        self.seen.add(key)
        token = json.dumps(key, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        event_id = "inspect-" + hashlib.sha256(token).hexdigest()
        details = {"event_id": event_id, "audio_id": audio_id, "session_id": self.session_id,
                   "edge_id": edge_id, "side": side, "identity": record["identity"]}
        if not self.enabled:
            self.event("inspection_speech", **details, state="blocked", reason="missing_durations",
                       missing_ids=list(self.config.missing_ids))
            return False
        self.requests[event_id] = {**details, "state": "submitted"}
        try:
            submitted = self.send(f"speech {event_id} {audio_id}")
        except Exception as exc:
            self.requests[event_id].update(state="submit_failed", reason="bridge_write_failed")
            self.event("inspection_speech", **details, state="submit_failed", reason="bridge_write_failed",
                       error=str(exc)[:160])
            return False
        state = "submitted" if submitted else "submit_failed"
        self.requests[event_id]["state"] = state
        self.event("inspection_speech", **details, state=state,
                   reason="bridge_submission" if submitted else "bridge_unavailable")
        return bool(submitted)

    def handle_note(self, text):
        note = parse_speech_note(text)
        if note is None:
            return False
        # 同时记录桥内产生的 UID 请求，保持 UID 与识别共享队列的可观察性。
        self.event("speech_bridge", message_type=note["kind"], event_id=note["event_id"],
                   audio_id=note["audio_id"], result=note["result"], playback_verified=False)
        request = self.requests.get(note["event_id"])
        if request is None or request["audio_id"] != note["audio_id"]:
            return True
        if request["state"] in ("rejected", "accepted_by_lower", "unknown", "failed", "submit_failed"):
            return True
        kind, result = note["kind"], note["result"]
        if kind == "SPEECH_QUEUED":
            state = "queued"
        elif kind == "SPEECH_SENT":
            state = "sent"
        elif kind == "SPEECH_REJECT":
            state = "rejected"
        elif result == "ACK_OK":
            state = "accepted_by_lower"
        elif result in ("ACK_TIMEOUT", "LINK_LOST"):
            state = "unknown"
        else:
            state = "failed"
        request.update(state=state, reason=result)
        self.event("inspection_speech", **{key: value for key, value in request.items()
                                          if key not in ("state", "reason")},
                   state=state, reason=result, playback_verified=False)
        return True
