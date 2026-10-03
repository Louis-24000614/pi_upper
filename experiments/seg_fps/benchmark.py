"""可复现纯 RKNN / seg / 视觉导航吞吐与实时延迟，不连接硬件控制。"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor, wait
from collections import deque
from dataclasses import asdict
import json
import os
import queue
from pathlib import Path
import resource
import time
import cv2
import numpy as np
import yaml
from road_follow.segment import RoadSegmenter, letterbox
from road_follow.recording import VideoRecorder
from road_follow.async_recording import AsyncVideoRecorder
from road_follow.control import VelocityCommand
from road_follow.__main__ import SLOW_S
from road_follow.cpu_threads import cpu_thread_budget
from experiments.seg_fps.frames import VideoFrames, ResultGate
from experiments.seg_fps.navigation import Navigation
from experiments.seg_fps.telemetry import snapshot, summarize


VARIANTS = {
    "a0": (False, False, False, False, False),
    "a1": (True, False, False, False, False),
    "bev": (True, True, False, False, False),
    "lazy": (True, True, False, False, False),
    "buffers": (True, True, True, False, False),
    "fifo": (True, True, True, True, False),
    "latest": (True, True, True, True, True),
}


def run(args) -> dict:
    correct, reuse_bev, buffers, threaded, latest = VARIANTS[args.variant]
    cfg = yaml.safe_load(args.config.read_text())
    navigation = Navigation(cfg, reuse_bev, lazy_raw=args.variant == "lazy")
    masks = [int(x) for x in args.cores.split(",")]
    segments = [RoadSegmenter(args.model, correct_nms=correct, reuse_buffers=buffers,
                              core_mask=x, measure=True, backend=args.backend) for x in masks]
    frames = VideoFrames(args.video, threaded=threaded, latest=latest,
                         realtime=args.realtime, speed=args.speed, loop=True)
    executors = [ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"seg-{i}")
                 for i in range(len(segments))]
    pure_pool = []
    # 长稳运行只保留有限诊断样本；总完成计数独立累加，不能把样本数当 FPS。
    rows = deque(maxlen=args.retained_rows or None)
    events = deque(maxlen=args.retained_rows or None)
    if args.retained_rows:
        frames.decode_ms = deque(maxlen=args.retained_rows)
    counts = {"completed": 0, "consumed": 0, "slow_stop": 0, "worker_slow": 0}
    last_identity = -1

    def save_row(row):
        counts["completed"] += 1
        counts["consumed"] += not row.get("rejected", False)
        counts["slow_stop"] += row.get("slow_stop", False)
        counts["worker_slow"] += row.get("worker_slow", False)
        rows.append(row)
    gate = ResultGate(max_age_s=args.max_age if args.realtime else None)
    before = snapshot()
    env = {"opencv_threads": cv2.getNumThreads(),
           "openblas_threads": os.getenv("OPENBLAS_NUM_THREADS"),
           "omp_threads": os.getenv("OMP_NUM_THREADS")}
    try:
        from threadpoolctl import threadpool_info
        env["threadpools"] = threadpool_info()
    except ImportError:
        env["threadpools"] = "unavailable"
    recorder = None
    mixed = None
    mixed_report = None
    if args.record != "none":
        args.output.parent.mkdir(parents=True, exist_ok=True)
        record_path = args.output.with_suffix(".avi")
        recorder = (VideoRecorder(record_path) if args.record == "sync"
                    else AsyncVideoRecorder(record_path))
    try:
        # 双方使用相同预热；保留逐次时间，以便判断是否稳态，不能盲套 200 次。
        capture = cv2.VideoCapture(str(args.video))
        for _ in range(8):
            ok, image = capture.read()
            if not ok:
                break
            pure_pool.append(np.ascontiguousarray(letterbox(image)[0])[None])
        capture.release()
        if not pure_pool:
            raise RuntimeError("录像没有可解码帧")
        # 即使标准 IO 只读输入，也给每个 context 独立 RAM 池，避免隐式别名。
        pure_pools = [[tensor.copy() for tensor in pure_pool] for _ in segments]
        warmup_rows = []
        for index, segment in enumerate(segments):
            def warmup(segment=segment, index=index):
                session = segment._load()
                samples = []
                for step in range(args.warmup):
                    tensor = pure_pools[index][step % len(pure_pool)]
                    began = time.monotonic()
                    session.inference(inputs=[tensor], data_format=["nhwc"])
                    samples.append((time.monotonic()-began)*1000)
                return samples
            warmup_rows.append(executors[index].submit(warmup).result())
        if args.mixed_load:
            from experiments.seg_fps.mixed_load import MixedLoad
            # 受控负载使用既有服务和障碍模型；单独报告，不能混入单道路收益。
            capture = cv2.VideoCapture(str(args.video))
            try:
                ok, load_image = capture.read()
            finally:
                capture.release()
            if not ok:
                raise RuntimeError("没有混合负载输入")
            mixed = MixedLoad(load_image)
            mixed.start()

        def work(index, frame, sequence):
            segment = segments[index]
            started = time.monotonic()
            if args.mode == "pure":
                outputs = segment._load().inference(
                    inputs=[pure_pools[index][sequence % len(pure_pool)]], data_format=["nhwc"])
                if outputs is None or len(outputs) < 2:
                    raise RuntimeError("RKNN 返回空输出")
                timings = {"rknn_ms": (time.monotonic()-started)*1000}
                mask = None
            else:
                mask = segment.mask(frame.image)
                timings = dict(segment.last_timings)
            timings.update(getattr(segment._session, "last_timings", {}))
            return sequence, frame, mask, started, time.monotonic(), timings

        iterator = iter(frames) if args.mode != "pure" else None
        pending = deque()
        sequence = 0
        start_cpu, start = time.process_time(), time.monotonic()
        deadline = start + args.seconds
        previous = None
        resource_samples = []
        next_sample = start + args.sample_interval
        next_progress, progress_completed, progress_consumed = start+60, 0, 0
        # 每个 worker 最多一个在途任务，控制队列年龄，不用长队列换吞吐。
        while time.monotonic() < deadline or pending:
            while len(pending) < len(segments) and time.monotonic() < deadline:
                index = sequence % len(segments)
                if iterator is not None and pending and frames.threaded:
                    if pending[0].done():
                        break
                    try:
                        frame = frames.read_nowait()
                    except queue.Empty:
                        break
                    if frame is None:
                        break
                else:
                    frame = next(iterator) if iterator is not None else None
                if recorder is not None and frame is not None:
                    recorder.write(frame.image, frame.decoded_s)
                pending.append(executors[index].submit(work, index, frame, sequence))
                sequence += 1
            if not pending:
                break
            if (iterator is not None and frames.threaded and len(pending) < len(segments)
                    and not pending[0].done() and time.monotonic() < deadline):
                # 实时源可能尚未发布下一帧。短暂等候后重新取输入，让空闲 context
                # 在画面到达时立即工作；直接 result() 会把多 worker 退化成串行。
                wait([pending[0]], timeout=.002)
                continue
            result = pending.popleft().result()
            identity, frame, mask, began, finished, timings = result
            # 统一窗口按完成时刻计数；末尾排空结果单独排除，避免多 worker 统计虚增。
            if finished > deadline:
                continue
            if identity <= last_identity:
                raise AssertionError("重复或乱序完成编号")
            last_identity = identity
            consumed = time.monotonic()
            row = {"id": identity, "start_s": began-start, "end_s": finished-start,
                   **timings, "worker_ms": (finished-began)*1000}
            row["worker_slow"] = row["worker_ms"] > SLOW_S*1000
            if frame is not None:
                row.update(frame_id=frame.sequence, video_s=frame.video_s,
                           source=frame.source, publish_s=frame.published_s-start,
                           decode_s=frame.decoded_s-start,
                           queue_ms=(began-frame.published_s)*1000)
                row["result_age_ms"] = (consumed-frame.published_s)*1000
                if not gate.accept(frame, consumed):
                    row["rejected"] = True
                    save_row(row)
                    continue
                if args.mode == "full":
                    nav_start = time.monotonic()
                    command, diag, opening, junction = navigation.process(mask)
                    # 与正式入口相同：分割超过原有预算仍生成停车意图，不改安全阈值。
                    if row["worker_ms"] > SLOW_S*1000:
                        command = VelocityCommand(0.0, 0.0, "stop_slow")
                        row["slow_stop"] = True
                    row["nav_ms"] = (time.monotonic()-nav_start)*1000
                    signature = (command.reason, opening)
                    if signature != previous:
                        events.append({"frame_id": frame.sequence, "video_s": frame.video_s,
                                       "command": asdict(command), "opening": opening})
                        previous = signature
                row["consume_s"] = time.monotonic()-start
                row["age_ms"] = (time.monotonic()-frame.published_s)*1000
                if time.monotonic() > deadline:
                    continue
            save_row(row)
            if time.monotonic() >= next_sample:
                resource_samples.append({"elapsed_s": time.monotonic()-start, **snapshot()})
                next_sample += args.sample_interval
            if time.monotonic() >= next_progress:
                print("PROGRESS", json.dumps({"elapsed_s": time.monotonic()-start,
                      "completed_last_minute": counts["completed"]-progress_completed,
                      "consumed_last_minute": counts["consumed"]-progress_consumed,
                      "rejections": gate.counts,
                      "rss": snapshot().get("/proc/self/status")}), flush=True)
                progress_completed, progress_consumed = counts["completed"], counts["consumed"]
                next_progress += 60
        elapsed = min(time.monotonic()-start, args.seconds)
        cpu_s = time.process_time()-start_cpu
        metrics = {key: summarize([r[key] for r in rows if key in r])
                   for key in ["pre_ms", "rknn_ms", "post_ms", "nav_ms", "worker_ms",
                               "input_io_ms", "run_ms", "output_io_ms",
                               "python_output_copy_ms",
                               "post_filter_ms", "post_matrix_sigmoid_ms",
                               "post_resize_merge_ms", "post_restore_ms",
                               "queue_ms", "age_ms", "result_age_ms"]}
        accepted = counts["consumed"]
        if recorder is not None:
            recorder.close()
        if mixed is not None:
            mixed_report = mixed.close()
        return {"variant": args.variant, "mode": args.mode, "cores": masks,
                "seconds": elapsed, "completed": counts["completed"], "consumed": accepted,
                "fps": (accepted if args.mode == "full" else counts["completed"])/elapsed,
                "seg_completed_fps": counts["completed"]/elapsed, "consumed_fps": accepted/elapsed,
                "dropped_input": frames.dropped, "rejections": gate.counts,
                "cpu_percent": cpu_s/elapsed*100,
                "rss_peak_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                "metrics": metrics, "decode_ms": summarize(frames.decode_ms),
                "environment": env, "before": before, "after": snapshot(),
                "record": args.record,
                "backend": args.backend,
                "mixed_load": mixed_report,
                "inputs": {"video": str(args.video), "model": str(args.model),
                           "config": str(args.config), "realtime": args.realtime,
                           "speed": args.speed, "max_age_s": args.max_age},
                "record_dropped": getattr(recorder, "dropped", 0),
                "slow_stop_count": counts["slow_stop"],
                "worker_slow_count": counts["worker_slow"],
                "warmup_iterations_per_context": args.warmup,
                "warmup_call_ms": warmup_rows,
                "resource_sample_interval_s": args.sample_interval,
                "metrics_scope": "all completed frames" if not args.retained_rows
                                 else f"last {len(rows)} completed frames; FPS uses all completions",
                "retained_rows": args.retained_rows,
                "resources": resource_samples, "raw": list(rows), "events": list(events),
                "scope": "visual navigation intention; no ODOM/IMU/UART/physical actions"}
    finally:
        frames.close()
        for executor in executors:
            executor.shutdown(wait=True, cancel_futures=True)
        for segment in segments:
            segment.close()
        if recorder is not None:
            recorder.close()
        if mixed is not None:
            mixed.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--variant", choices=VARIANTS, default="a1")
    parser.add_argument("--mode", choices=["pure", "seg", "full"], default="full")
    parser.add_argument("--cores", default="1", help="RKNN 位掩码：1/2/4/7；逗号分隔独立 context")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--speed", type=float, default=1)
    parser.add_argument("--max-age", type=float, default=0.2)
    parser.add_argument("--record", choices=["none", "sync", "async"], default="none")
    parser.add_argument("--backend", choices=["lite", "c-standard", "c-input-zero"], default="lite")
    parser.add_argument("--retained-rows", type=int, default=0,
                        help="长稳诊断最多保留 N 帧样本；0 保留全部，完成数仍覆盖完整窗口")
    parser.add_argument("--mixed-load", action="store_true", help="受控障碍/刀具/人脸检测混合负载")
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--sample-interval", type=float, default=10)
    parser.add_argument("--opencv-threads", type=int)
    parser.add_argument("--blas-threads", type=int)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("统计窗口必须大于零")
    if args.retained_rows < 0:
        parser.error("诊断保留数量不能为负数")
    if args.warmup < 1 or args.sample_interval <= 0:
        parser.error("预热次数和资源采样间隔必须为正数")
    with cpu_thread_budget(args.opencv_threads, args.blas_threads):
        report = run(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps({k: report[k] for k in ["variant", "mode", "cores", "fps", "consumed_fps",
                                            "metrics", "dropped_input", "rejections"]}), flush=True)


if __name__ == "__main__":
    main()
