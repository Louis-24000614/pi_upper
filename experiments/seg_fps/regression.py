"""完整片段或均匀抽检回归，NMS 修正与等价计算复用分别统计。"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from road_follow.segment import RoadSegmenter, letterbox, decode_road_mask, _orient_heads
from road_follow.segment_buffers import SegmentBuffers
from road_follow.pipeline import make_ipm
from experiments.seg_fps.navigation import Navigation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--videos", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--sample-per-video", type=int, default=0,
                        help="每段均匀抽取 N 帧（含首尾）；只验证抽样序列，不声称全量时序通过")
    parser.add_argument("--video-sample-count", type=int, default=0,
                        help="在排序后的录像中均匀选取 N 段；0 表示全部录像")
    parser.add_argument("--core", type=int, default=1)
    parser.add_argument("--reference-core", type=int)
    parser.add_argument("--backend", default="lite")
    parser.add_argument("--reference-backend", default="lite")
    parser.add_argument("--reuse-adjacent-heads", action="store_true",
                        help="仅正确性回归：完全相同的相邻图像复用模型输出，仍逐帧计算状态")
    args = parser.parse_args()
    if args.sample_per_video < 0 or args.video_sample_count < 0 or (args.sample_per_video and args.limit):
        parser.error("抽样帧数必须非负，且不能同时指定 --limit")
    segment = RoadSegmenter(args.model, core_mask=args.core, backend=args.backend)
    reference = (RoadSegmenter(args.model, core_mask=args.reference_core, backend=args.reference_backend)
                 if args.reference_core is not None else None)
    buffers = SegmentBuffers()
    cfg = yaml.safe_load(args.config.read_text())
    all_videos = sorted(args.videos.glob("*.avi"))
    if not all_videos:
        parser.error("录像目录没有 AVI")
    selected_videos = all_videos
    if args.video_sample_count:
        # 录像总数也按用户指定缩减。选点可复现，避免人工只挑容易的片段。
        indices = np.unique(np.linspace(0, len(all_videos)-1,
                            min(len(all_videos), args.video_sample_count), dtype=int))
        selected_videos = [all_videos[index] for index in indices]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {"videos": [], "errors": [], "reuse_adjacent_heads": args.reuse_adjacent_heads,
              "sample_per_video": args.sample_per_video,
              "available_videos": len(all_videos),
              "selected_videos": [path.name for path in selected_videos],
              "scope": ("sampled visual sequence only; not complete event timing; no ODOM/IMU"
                        if args.sample_per_video else
                        "visual only; no synchronized ODOM/IMU; not a throughput benchmark")}
    total = 0
    try:
        session = segment._load()
        for path in selected_videos:
            base, corrected, optimized = Navigation(cfg), Navigation(cfg), Navigation(cfg, True)
            lazy = Navigation(cfg, True, True)
            cap = cv2.VideoCapture(str(path))
            count, repeated, nms_changes, command_changes, junction_changes = 0, 0, 0, 0, 0
            previous_hash, retained, cached_heads = None, None, None
            inference_frames = 0
            events = []
            started = time.monotonic()
            try:
                if not cap.isOpened():
                    raise RuntimeError(f"无法读取: {path}")
                expected = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                # 按用户要求改为均匀抽检，避免只取开头而漏掉中后段画面。
                # 状态仍按抽样编号顺序推进，但不能把稀疏序列当成真实连续时序。
                sampled = (np.unique(np.linspace(0, expected-1,
                            min(expected, args.sample_per_video), dtype=int)).tolist()
                           if args.sample_per_video else None)
                while True:
                    if sampled is not None:
                        if count >= len(sampled):
                            break
                        source_frame = sampled[count]
                        cap.set(cv2.CAP_PROP_POS_FRAMES, source_frame)
                    else:
                        source_frame = count
                    ok, image = cap.read()
                    if not ok:
                        if sampled is not None:
                            raise RuntimeError(f"抽样帧读取失败: {path}:{source_frame}")
                        break
                    if args.limit and count >= args.limit:
                        break
                    digest = hashlib.sha256(image.tobytes()).hexdigest()
                    duplicate = digest == previous_hash
                    repeated += duplicate
                    previous_hash = digest
                    rgb, ratio, left, top = letterbox(image)
                    reused = buffers.letterbox(image)
                    if not np.array_equal(rgb, reused[0]) or (ratio,left,top) != reused[1:]:
                        raise AssertionError("预处理张量不一致")
                    tensor = np.ascontiguousarray(rgb)[None]
                    if args.reuse_adjacent_heads and duplicate and cached_heads is not None:
                        outputs, ref = cached_heads
                    else:
                        outputs = session.inference(inputs=[tensor], data_format=["nhwc"])
                        if outputs is None or len(outputs) < 2:
                            raise RuntimeError("RKNN 无有效输出")
                        ref = None
                        if reference is not None:
                            ref = reference._load().inference(inputs=[tensor], data_format=["nhwc"])
                            if ref is None or len(ref) != len(outputs):
                                raise AssertionError("参考后端的输出数量不一致")
                            for a, b in zip(outputs, ref):
                                np.testing.assert_allclose(a, b, atol=1e-5, rtol=1e-5)
                        inference_frames += 1
                        # 录像按固定时间轴可能补写相同画面。仅回归允许复用确定性模型
                        # 的相邻重复输出；下方 mask、EMA、路口仍逐帧执行，不能漏掉帧数判据。
                        # 复制输出所有权，避免下一次原生推理覆盖缓存；性能测试从不走此路径。
                        if args.reuse_adjacent_heads:
                            cached_heads = ([x.copy() for x in outputs],
                                            None if ref is None else [x.copy() for x in ref])
                    pred, proto = _orient_heads(*outputs[:2])
                    raw = decode_road_mask(pred, proto, ratio, left, top, image.shape[:2])
                    a1 = decode_road_mask(pred, proto, ratio, left, top, image.shape[:2], correct_nms=True)
                    candidate = decode_road_mask(pred, proto, ratio, left, top, image.shape[:2],
                                                 correct_nms=True, buffers=buffers)
                    if reference is not None:
                        ref_mask = decode_road_mask(*_orient_heads(*ref[:2]), ratio, left, top,
                                                   image.shape[:2], correct_nms=True)
                        np.testing.assert_array_equal(ref_mask, candidate)
                    if not np.array_equal(a1, candidate):
                        raise AssertionError("缓冲复用 mask 不一致")
                    original_ipm = make_ipm(cfg, a1.shape)
                    original_bev = original_ipm.warp_to_bev(a1, flags=cv2.INTER_NEAREST)
                    _, reused_bev = optimized.projector.project(candidate, cfg)
                    np.testing.assert_array_equal(original_bev, reused_bev)
                    # 每次保留上一张 mask 到下一轮，检查复用没有覆盖调用者持有的结果。
                    if retained is not None:
                        old, old_hash = retained
                        if hashlib.sha256(old.tobytes()).hexdigest() != old_hash:
                            raise AssertionError("上一帧 mask 被覆盖")
                    retained = candidate, hashlib.sha256(candidate.tobytes()).hexdigest()
                    old_nav = base.process(raw)
                    good_nav = corrected.process(a1)
                    fast_nav = optimized.process(candidate)
                    lazy_nav = lazy.process(candidate)
                    if good_nav != fast_nav:
                        raise AssertionError("中心线诊断、控制或路口时序不一致")
                    if corrected.smoother._prev != optimized.smoother._prev:
                        raise AssertionError("平滑中心线逐点不一致")
                    if good_nav != lazy_nav or corrected.smoother._prev != lazy.smoother._prev:
                        raise AssertionError("按需普通中心线的控制、诊断或中心线不一致")
                    changed = not np.array_equal(raw, a1)
                    nms_changes += changed
                    command_changes += old_nav[0] != good_nav[0]
                    junction_changes += old_nav[2:] != good_nav[2:]
                    if changed:
                        events.append({"frame": source_frame, "changed_pixels": int(np.count_nonzero(raw != a1)),
                                       "a0_command": asdict(old_nav[0]), "a1_command": asdict(good_nav[0]),
                                       "a0_junction": asdict(old_nav[3]), "a1_junction": asdict(good_nav[3])})
                        if len(events) <= 2:
                            cv2.imwrite(str(args.output.parent / f"{path.stem}_{source_frame}_nms_diff.png"),
                                        np.where(raw != a1, 255, 0).astype(np.uint8))
                    count += 1
                    total += 1
                    if count % 100 == 0:
                        print(path.name, count, "frames checked", flush=True)
                if not args.limit and sampled is None and count != expected:
                    raise AssertionError(f"读取帧数 {count} != 声明帧数 {expected}")
            finally:
                cap.release()
            item = {"video": path.name, "frames": count, "duplicates_adjacent": repeated,
                    "declared_frames": expected, "sampled_indices": sampled,
                    "inference_frames": inference_frames,
                    "nms_changed_masks": nms_changes, "nms_changed_commands": command_changes,
                    "nms_changed_junctions": junction_changes, "events": events,
                    "equivalent_differences": 0, "seconds": time.monotonic()-started}
            report["videos"].append(item)
            report["total_frames"] = total
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print("REGRESSION", path.name, {k:v for k,v in item.items() if k != "events"}, flush=True)
    except BaseException as exc:
        report["errors"].append(repr(exc))
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        raise
    finally:
        segment.close()
        if reference is not None:
            reference.close()


if __name__ == "__main__":
    main()
