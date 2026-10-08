"""单相机侧视调试入口：真实识别与网页，不连接 PWM、UART 或导航。"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import signal
import threading
import time

from road_follow.inspection import SideInspection
from road_follow.inspection_config import Settings
from road_follow.inspection_io import FrameHub, Recognizer, SideCamera
from road_follow.inspection_web import InspectionWeb


ROOT = Path(__file__).resolve().parents[2]


class DisabledServo:
    """拒绝转向请求，避免将未接硬件的调试误报为真实两侧检查。"""
    def __init__(self):
        self.closed = False

    def alive(self):
        return not self.closed

    def request(self, angle):
        raise RuntimeError("单侧调试已禁用 PWM，不能转向")

    def close(self):
        self.closed = True


class SingleSideDebug(SideInspection):
    def _finish_side(self, now, confirmed):
        record = {"direction": self.side, "status": "confirmed" if confirmed else "unconfirmed",
                  "category": self.candidate.get("category") if confirmed else None,
                  "identity": self.candidate.get("identity") if confirmed else None,
                  "score": self.candidate.get("score") if confirmed else None,
                  "valid_frames": self.count if confirmed else 0,
                  "settings_revision": self.revision,
                  "parameters": deepcopy(self.config["recognition"]),
                  "reason": "confirmed" if confirmed else "side_timeout:"+self.last_reason,
                  "completed_s": now, "debug_only": True}
        self.sides = [record]
        self.result = {"status": "debug_confirmed" if confirmed else "debug_unconfirmed",
                       "sides": deepcopy(self.sides), "debug_only": True, "speech_enabled": False}
        self.phase = "done"
        self.generation += 1
        self._emit("inspection_debug_result", result=self.result)

    def status(self):
        state = super().status()
        state["phase"] = {"recognizing": "单侧调试：识别中（PWM 禁用）",
                          "done": "单侧调试：本轮结束（PWM 禁用）"}.get(self.phase, self.phase)
        state.update(mode="single_side_debug", pwm_enabled=False, drive_enabled=False)
        return state


def create_debug(settings, device, *, event=None):
    config, _ = settings.snapshot()
    if not Path(device).exists():
        raise ValueError("指定的调试相机不存在")
    camera_config = {**config["camera"], "device": device}
    hub = FrameHub(config["servo"]["initial_side"])
    task = SingleSideDebug(settings, hub, DisabledServo(), Recognizer(config["services"]), event=event)
    try:
        task.camera = SideCamera(camera_config, hub)
        limit = time.monotonic()+5
        while hub.latest() is None:
            if task.camera.stop.wait(.05) or time.monotonic() >= limit:
                raise OSError("调试相机没有新鲜画面："+hub.info()["error"])
        task.start(time.monotonic(), None)
        task.web = InspectionWeb(settings, hub, task.status)
        return task
    except BaseException:
        task.close()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"config/culvert_inspection.json")
    parser.add_argument("--camera", required=True, help="明确指定用于调试的单台摄像头")
    args = parser.parse_args(argv)
    settings = Settings(args.config)
    stop = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    task = None
    try:
        def event(kind, **values):
            print(json.dumps({"event": kind, **values}, ensure_ascii=False), flush=True)
        task = create_debug(settings, args.camera, event=event)
        config, _ = settings.snapshot()
        print(f"单侧识别调试已启动：0.0.0.0:{config['web']['port']}；PWM/UART/导航禁用。", flush=True)
        restart_s = None
        while not stop.wait(.02):
            now = time.monotonic()
            outcome = task.step(now)
            if outcome == "failed":
                raise RuntimeError(task.failure_reason or "调试识别失败")
            if outcome == "done":
                if restart_s is None:
                    restart_s = now+5
                elif now >= restart_s:
                    task.start(now, None)
                    restart_s = None
        return 0
    finally:
        try:
            if task is not None:
                task.close()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
