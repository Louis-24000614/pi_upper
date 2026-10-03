"""对照生产 Ordered/Latest 流水线，统一发布钟/解码/频率/线程/导航及时效门。"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import resource
import time
import cv2
import numpy as np
import yaml
from road_follow.parallel_segment import OrderedSegmentStream
from road_follow.latest_segment import LatestSegmentStream
from road_follow.__main__ import SLOW_S
from road_follow.cpu_threads import cpu_thread_budget
from road_follow.heap_reclaim import HeapReclaimer
from experiments.seg_fps.camera_replay import ClockedCapture
from experiments.seg_fps.frames import Frame, ResultGate
from experiments.seg_fps.navigation import Navigation
from experiments.seg_fps.telemetry import snapshot, summarize


def run(args):
    capture = ClockedCapture(args.video, args.source_fps, args.driver_capacity)
    stream_type = LatestSegmentStream if args.strategy == 'latest' else OrderedSegmentStream
    stream = stream_type(args.model, (1,2,4), correct_nms=True, reuse_buffers=True,
                         capture_timestamp=lambda: capture.captured_s)
    gate = ResultGate(max_age_s=SLOW_S)
    navigation = Navigation(yaml.safe_load(args.config.read_text()), True)
    rows = deque(maxlen=args.retained_rows or None)
    resources, errors = [], []
    completed = accepted = worker_slow = 0
    last = -1
    began = cpu_began = None
    reclaimer = HeapReclaimer(args.heap_trim_interval)
    try:
        warmup_capture = cv2.VideoCapture(str(args.video))
        pool = []
        try:
            for _ in range(8):
                ok, image = warmup_capture.read()
                if not ok:
                    raise RuntimeError('预热素材不足')
                pool.append(image)
        finally:
            warmup_capture.release()
        # 双方都走真实 mask 入口并使用相同预热，不从早期纯 RKNN 吞吐推断本策略。
        for index in range(args.warmup):
            stream.warmup(pool[index % len(pool)])
        reclaimer.__enter__()
        before = snapshot()
        began, cpu_began = time.monotonic(), time.process_time()
        next_sample, next_progress = began+args.sample_interval, began+60
        progress_completed = progress_accepted = 0
        while time.monotonic()-began < args.seconds:
            result = stream.read(capture)
            now = time.monotonic()
            if now-began >= args.seconds:
                break
            if result.sequence <= last:
                raise AssertionError('返回结果重复或乱序')
            last = result.sequence
            fresh = gate.accept(Frame(result.sequence, 0, result.captured_s,
                                     result.captured_s, result.source, result.image), now)
            row = dict(sequence=result.sequence, source=result.source,
                       worker_ms=result.inference_s*1000,
                       queue_ms=(result.started_s-result.captured_s)*1000,
                       result_age_ms=(now-result.captured_s)*1000, accepted=fresh)
            if fresh:
                navigation.process(result.mask)
            ended = time.monotonic()
            if ended-began >= args.seconds:
                break
            completed += 1
            accepted += fresh
            worker_slow += result.inference_s > SLOW_S
            row['age_ms'] = (ended-result.captured_s)*1000
            rows.append(row)
            if ended >= next_sample:
                resources.append(dict(elapsed_s=ended-began, **snapshot()))
                next_sample += args.sample_interval
            if ended >= next_progress:
                print('LATEST_STABILITY', round(ended-began), completed-progress_completed,
                      'processed/min', accepted-progress_accepted, 'fresh/min',
                      snapshot().get('/proc/self/status'), flush=True)
                progress_completed, progress_accepted = completed, accepted
                next_progress += 60
        # 截止窗口先保存统计，再排空在途任务，不能把退出期间完成数计入 FPS。
        stats = dict(getattr(stream, 'stats', {}))
        elapsed, cpu_s = args.seconds, time.process_time()-cpu_began
    except BaseException as exc:
        errors.append(repr(exc))
        raise
    finally:
        try:
            stream.close()
        except BaseException as exc:
            errors.append('close: '+repr(exc))
            raise
        finally:
            if not getattr(stream, 'owns_capture', False):
                capture.release()
            reclaimer.close()
            if began is not None:
                report = dict(strategy=args.strategy, seconds=args.seconds,
                    completed=completed, consumed=accepted, fps=completed/args.seconds,
                    consumed_fps=accepted/args.seconds, worker_slow_count=worker_slow,
                    passed=not errors and not reclaimer.errors, errors=errors,
                    stream_stats=locals().get('stats', dict(getattr(stream, 'stats', {}))),
                    source_fps=args.source_fps, simulated_driver_capacity=args.driver_capacity,
                    capture_reads=capture.reads, camera_dropped=capture.camera_dropped,
                    rejections=gate.counts, cpu_percent=locals().get('cpu_s',0)/args.seconds*100,
                    rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                    metrics={key: summarize([row[key] for row in rows]) for key in
                        ['worker_ms','queue_ms','result_age_ms','age_ms']},
                    accepted_age_ms=summarize([row['age_ms'] for row in rows if row['accepted']]),
                    raw=list(rows), metrics_scope=f'last {len(rows)} results; FPS all window',
                    resources=resources, decode_ms=summarize(capture.decode_ms),
                    heap_reclaim=dict(interval_s=args.heap_trim_interval,
                                      samples=list(reclaimer.samples), errors=reclaimer.errors),
                    warmup=args.warmup, opencv_threads=cv2.getNumThreads(), blas_threads=1,
                    cores=[1,2,4], model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                    config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                    before=locals().get('before'), after=snapshot(),
                    scope='AVI independent publish clock; simulated driver FIFO; no camera/UART/vehicle')
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print('RESULT', args.strategy, args.source_fps, completed, accepted,
          round(report['consumed_fps'],3), report['metrics']['age_ms'], flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--strategy', choices=('ordered','latest'), required=True)
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--source-fps', type=float, default=30, help='模拟发布率；0 表示不限制读取')
    p.add_argument('--driver-capacity', type=int, default=4)
    p.add_argument('--warmup', type=int, default=20)
    p.add_argument('--sample-interval', type=float, default=1)
    p.add_argument('--heap-trim-interval', type=float, default=60)
    p.add_argument('--retained-rows', type=int, default=0)
    args = p.parse_args()
    if args.seconds <= 0 or args.warmup < 1 or args.sample_interval <= 0 or args.retained_rows < 0:
        p.error('时长/预热/采样必须为正，保留数量不能为负')
    with cpu_thread_budget(8,1):
        run(args)


if __name__ == '__main__':
    main()
