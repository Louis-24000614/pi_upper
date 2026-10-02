"""连续验证主程序使用的分割流水线，AVI 替代相机，不打开 UART。"""
import argparse
from collections import deque
import json
from pathlib import Path
import resource
import time
import cv2
import numpy as np
import yaml
from road_follow.parallel_segment import OrderedSegmentStream
from road_follow.__main__ import SLOW_S
from experiments.seg_fps.navigation import Navigation
from experiments.seg_fps.frames import Frame, ResultGate
from experiments.seg_fps.telemetry import snapshot, summarize


class LoopCapture:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise RuntimeError("无法读取长稳录像")
        self.expected = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.decoded = 0
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.expected <= 0 or self.fps <= 0:
            self.cap.release()
            raise RuntimeError("长稳录像没有有效帧率或帧数")

    def read(self):
        ok, image = self.cap.read()
        if not ok:
            if self.decoded != self.expected:
                raise RuntimeError("长稳录像中途读取失败")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.decoded = 0
            ok, image = self.cap.read()
        if ok:
            self.decoded += 1
        return ok, image


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--video", type=Path, required=True)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--cores", default="1,2,4")
    p.add_argument("--seconds", type=float, default=600)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.seconds <= 0:
        p.error("时长必须大于零")
    cores = [int(x) for x in args.cores.split(",")]
    capture = LoopCapture(args.video)
    stream = OrderedSegmentStream(args.model, cores, correct_nms=True, reuse_buffers=True)
    navigation = Navigation(yaml.safe_load(args.config.read_text()), True)
    gate = ResultGate(max_age_s=SLOW_S)
    rows, resources, errors = deque(maxlen=4096), [], []
    completed = accepted = last = progress_completed = progress_accepted = 0
    began = cpu_began = None
    before = snapshot()
    try:
        stream.warmup(np.zeros((720,1280,3), np.uint8))
        began, cpu_began = time.monotonic(), time.process_time()
        next_sample, next_progress = began+10, began+60
        while time.monotonic()-began < args.seconds:
            result = stream.read(capture)
            if time.monotonic()-began > args.seconds:
                break
            if result.sequence != last:
                raise AssertionError("结果重复或乱序")
            last = result.sequence+1
            completed += 1
            packet = Frame(result.sequence, result.sequence/capture.fps,
                           result.captured_s, result.captured_s, result.source, result.image)
            fresh = gate.accept(packet, time.monotonic())
            if fresh:
                navigation.process(result.mask)
                accepted += 1
            rows.append({"sequence": result.sequence, "source": result.source,
                         "worker_ms": result.inference_s*1000,
                         "age_ms": (time.monotonic()-result.captured_s)*1000,
                         "accepted": fresh})
            now = time.monotonic()
            if now >= next_sample:
                resources.append({"elapsed_s": now-began, **snapshot()})
                next_sample += 10
            if now >= next_progress:
                print("STABILITY", round(now-began), "seconds", completed-progress_completed,
                      "processed/min", accepted-progress_accepted, "accepted/min",
                      snapshot().get("/proc/self/status"), flush=True)
                progress_completed, progress_accepted = completed, accepted
                next_progress += 60
    except BaseException as exc:
        errors.append(repr(exc))
        raise
    finally:
        elapsed = min(args.seconds, time.monotonic()-began) if began is not None else 0
        cpu_s = time.process_time()-cpu_began if cpu_began is not None else 0
        try:
            stream.close()
        except BaseException as exc:
            errors.append("close: " + repr(exc))
            raise
        finally:
            capture.cap.release()
            args.output.write_text(json.dumps({"passed": not errors, "errors": errors,
                "seconds": elapsed, "completed": completed, "consumed": accepted,
                "fps": completed/max(elapsed,1e-6), "consumed_fps": accepted/max(elapsed,1e-6),
                "cores": cores, "backend": "lite", "variant": "buffers", "mode": "full",
                "cpu_percent": cpu_s/max(elapsed,1e-6)*100,
                "rss_peak_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                "metrics": {name: summarize([r[name] for r in rows])
                            for name in ["worker_ms", "age_ms"]},
                "rejections": gate.counts, "slow_stop_count": 0,
                "metrics_scope": f"last {len(rows)} completed frames; FPS uses all completions",
                "before": before, "after": snapshot(), "resources": resources, "raw": list(rows),
                "scope": "actual OrderedSegmentStream with AVI adapter; visual only, no ODOM/IMU/UART"}, indent=2))
    print("stability passed", completed, "processed", accepted, "accepted", flush=True)


if __name__ == "__main__":
    main()
