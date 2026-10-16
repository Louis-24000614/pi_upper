"""独立完整涵洞任务：网页按钮启动/重启，两侧识别、舵机与语音。"""
import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import yaml

from road_follow.inspection import SideInspection
from road_follow.inspection_config import Settings, preflight
from road_follow.inspection_io import FrameHub, Recognizer, ServoBridge, SideCamera
from road_follow.inspection_web import InspectionWeb
from road_follow.speech import InspectionSpeech, SpeechConfig


ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class DebugTarget:
    edge_id: str


class DebugSpeechTransport:
    """Serial, asynchronous uart_speak invocations; this binary never arms the drive."""
    def __init__(self, binary, uart, *, event=None):
        self.binary, self.uart = Path(binary), dict(uart)
        self.event = event or (lambda *args, **kwargs: None)
        self.queue = deque()
        self.process = self.current = None
        self.closed = False

    def send(self, line):
        fields = line.split()
        if self.closed or len(fields) != 3 or fields[0] != "speech":
            return False
        try:
            audio_id = int(fields[2])
        except ValueError:
            return False
        if not 1 <= audio_id <= 32:
            return False
        self.queue.append((fields[1], audio_id))
        return True

    def poll(self, now, speech):
        if self.process is not None:
            if self.process.poll() is None:
                if now-self.started_s < 7:
                    return
                self._stop_process()
                speech.handle_note(f"SPEECH_RESULT {self.current[0]} {self.current[1]} ACK_TIMEOUT")
                self.current = None
            else:
                output, _ = self.process.communicate()
                ok = self.process.returncode == 0 and "ACK_OK" in output.splitlines()
                speech.handle_note(f"SPEECH_RESULT {self.current[0]} {self.current[1]} "
                                   + ("ACK_OK" if ok else "FAILED"))
                self.process = self.current = None
        if not self.closed and self.queue:
            self.current = self.queue.popleft()
            event_id, audio_id = self.current
            try:
                self.process = subprocess.Popen(
                    [str(self.binary), str(audio_id), "--device", self.uart["device"],
                     "--baud", str(self.uart["baud"])],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                self.started_s = now
                self.event("inspection_debug_speech", event_id=event_id, audio_id=audio_id,
                           state="process_started", playback_verified=False)
            except OSError as exc:
                speech.handle_note(f"SPEECH_RESULT {event_id} {audio_id} FAILED")
                self.event("inspection_debug_speech", event_id=event_id, state="failed", error=str(exc))
                self.current = None

    def _stop_process(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1)
        self.process.communicate()
        self.process = None

    def reset(self):
        self.queue.clear()
        self._stop_process()
        self.current = None

    def close(self):
        self.closed = True
        self.reset()


class FullInspectionDebug(SideInspection):
    def __init__(self, settings, hub, recognizer, *, servo_factory, transport,
                 event=None, pool=None, pair_fallback=False):
        super().__init__(settings, hub, None, recognizer, event=event, pool=pool,
                         speech_enabled=True, pair_fallback=pair_fallback)
        self.servo_factory, self.transport = servo_factory, transport
        self.restart_requested = False
        self.run_number = 0
        self.session_id = "debug-"+uuid.uuid4().hex
        self.speech = InspectionSpeech(SpeechConfig(), transport.send, self.event, self.session_id)
        self.notify = self.speech.request

    def request_start(self):
        # HTTP threads only queue the intent; the task loop owns PWM and UART.
        with self.lock:
            if self.closed:
                raise ValueError("涵洞任务已关闭")
            self.restart_requested = True
            return {"state": "restart_requested", "next_run": self.run_number+1}

    def _step(self, now):
        if self.restart_requested and not self.closed:
            self.restart_requested = False
            self.transport.reset()
            self.run_number += 1
            self.speech = InspectionSpeech(SpeechConfig(), self.transport.send,
                                           self.event, self.session_id)
            self.notify = self.speech.request
            if self.servo is None or getattr(self.servo, "closed", False) or (
                    hasattr(self.servo, "alive") and not self.servo.alive()):
                if self.servo is not None:
                    self.servo.close()
                config, _ = self.settings.snapshot()
                self.servo = self.servo_factory(config["servo"])
            self.start(now, DebugTarget(f"debug-run-{self.run_number}"), initial_turn=True)
            self._emit("inspection_debug_started", run_number=self.run_number)
        outcome = super()._step(now)
        self.transport.poll(now, self.speech)
        return outcome

    def _fail(self, reason):
        self.transport.reset()
        return super()._fail(reason)

    def status(self):
        with self.lock:
            state = super().status()
            state.update(mode="full_inspection_debug", pwm_enabled=True, drive_enabled=False,
                         run_number=self.run_number, restart_requested=self.restart_requested,
                         speech_requests=list(self.speech.requests.values()))
            return state

    def close(self):
        try:
            super().close()
        finally:
            self.transport.close()


def create_debug(settings, device, *, root=ROOT, nav_config=None, speech_bin=None,
                 event=None, pair_fallback=False):
    root = Path(root)
    nav = yaml.safe_load(Path(nav_config or root/"config/nav_camera.yaml").read_text(encoding="utf-8"))
    uart = nav["uart"]
    if (not isinstance(uart.get("device"), str) or not uart["device"].strip()
            or type(uart.get("baud")) is not int or uart["baud"] <= 0):
        raise ValueError("导航配置中的 UART device/baud 无效")
    config = preflight(settings, root, nav.get("capture", {}).get("device"), camera_device=device)
    binary = Path(speech_bin or root/"build-turn/uart/uart_speak").resolve()
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise ValueError("缺少可执行的 uart_speak："+str(binary))
    hub = FrameHub(config["servo"]["initial_side"])
    transport = DebugSpeechTransport(binary, uart, event=event)
    task = FullInspectionDebug(settings, hub, Recognizer(config["services"]),
                               servo_factory=lambda servo: ServoBridge(root, servo),
                               transport=transport, event=event, pair_fallback=pair_fallback)
    try:
        task.camera = SideCamera({**config["camera"], "device": device}, hub)
        limit = time.monotonic()+5
        while hub.latest() is None:
            if task.camera.stop.wait(.05) or time.monotonic() >= limit:
                raise OSError("侧视相机没有新鲜画面："+hub.info()["error"])
        task.web = InspectionWeb(settings, hub, task.status, start_task=task.request_start)
        return task
    except BaseException:
        task.close()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"config/culvert_inspection.json")
    parser.add_argument("--camera", required=True, help="用于涵洞任务的侧视相机")
    parser.add_argument("--nav-config", type=Path, default=ROOT/"config/nav_camera.yaml",
                        help="读取已有 UART 设备与波特率")
    parser.add_argument("--speech-bin", type=Path, default=ROOT/"build-turn/uart/uart_speak")
    parser.add_argument("-b", "--inspection-pair-fallback", action="store_true",
                        help="刀具侧15秒未确认时，按已确认人脸补播对应刀号")
    args = parser.parse_args(argv)
    settings = Settings(args.config)
    stop = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stop.set()) for sig in (signal.SIGINT, signal.SIGTERM)}
    task = None
    try:
        def event(kind, **values):
            print(json.dumps({"event": kind, **values}, ensure_ascii=False), flush=True)
        task = create_debug(settings, args.camera, event=event, pair_fallback=args.inspection_pair_fallback,
                            nav_config=args.nav_config, speech_bin=args.speech_bin)
        config, _ = settings.snapshot()
        print(f"涵洞调试网页端口 {config['web']['port']}：点击“开始涵洞任务”运行一轮；"
              "运行中再按则从初始方向重新开始。包含舵机与语音。", flush=True)
        while not stop.wait(.02):
            task.step(time.monotonic())
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
