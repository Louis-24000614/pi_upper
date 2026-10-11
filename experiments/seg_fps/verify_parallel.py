"""实际多 context 并发回归：张量、mask、帧身份、导航与软件发送序列。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from road_follow.segment import RoadSegmenter, letterbox, _orient_heads, decode_road_mask
from road_follow.segment_buffers import SegmentBuffers
from experiments.seg_fps.navigation import Navigation


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--videos", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--cores", default="1,2,4")
    p.add_argument("--backend", default="lite")
    args = p.parse_args()
    masks = [int(x) for x in args.cores.split(",")]
    segments = [RoadSegmenter(args.model, core_mask=c, backend=args.backend) for c in masks]
    reference = RoadSegmenter(args.model)
    pools = [ThreadPoolExecutor(max_workers=1) for _ in masks]
    buffers = [SegmentBuffers() for _ in masks]
    retained = [None for _ in masks]
    cfg = yaml.safe_load(args.config.read_text())
    good, fast = Navigation(cfg), Navigation(cfg, True)
    rows, sent_good, sent_fast = [], [], []
    inputs = []
    try:
        for path in sorted(args.videos.glob("*.avi"))[::3]:
            cap = cv2.VideoCapture(str(path))
            try:
                for index in [0, int(cap.get(cv2.CAP_PROP_FRAME_COUNT))//2]:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                    ok, image = cap.read()
                    if not ok:
                        raise RuntimeError(f"无法读取 {path}:{index}")
                    inputs.append((path.name, index, image))
            finally:
                cap.release()

        def work(worker, sequence, image):
            # 每个 worker 私有预处理内存；交错输入不同录像，检查下一轮污染。
            rgb, ratio, left, top = buffers[worker].letterbox(image)
            outputs = segments[worker]._load().inference(
                inputs=[np.ascontiguousarray(rgb)[None]], data_format=["nhwc"])
            mask = decode_road_mask(*_orient_heads(*outputs[:2]), ratio, left, top,
                                    image.shape[:2], correct_nms=True, buffers=buffers[worker])
            old = retained[worker]
            if old is not None:
                old_mask, digest = old
                assert hashlib.sha256(old_mask.tobytes()).hexdigest() == digest
            retained[worker] = mask, hashlib.sha256(mask.tobytes()).hexdigest()
            # 主动制造不同完成顺序；消费者仍按提交编号推进状态，不能按完成先后发指令。
            if sequence % len(masks) == 0:
                time.sleep(.025)
            return sequence, rgb.copy(), [x.copy() for x in outputs], mask

        sequence = 0
        for offset in range(0, len(inputs), len(masks)):
            batch = inputs[offset:offset+len(masks)]
            expected = []
            for name, index, image in batch:
                rgb, ratio, left, top = letterbox(image)
                outputs = reference._load().inference(inputs=[rgb[None]], data_format=["nhwc"])
                expected.append((rgb, [x.copy() for x in outputs],
                    decode_road_mask(*_orient_heads(*outputs[:2]), ratio, left, top,
                                     image.shape[:2], correct_nms=True)))
            futures = [pools[i].submit(work, i, sequence+i, item[2])
                       for i, item in enumerate(batch)]
            for i, future in enumerate(futures):
                identity, tensor, outputs, mask = future.result()
                assert identity == sequence+i
                np.testing.assert_array_equal(tensor, expected[i][0])
                for a, b in zip(outputs, expected[i][1]):
                    np.testing.assert_allclose(a, b, atol=1e-5, rtol=1e-5)
                np.testing.assert_array_equal(mask, expected[i][2])
                baseline, optimized = good.process(expected[i][2]), fast.process(mask)
                assert baseline == optimized
                assert good.smoother._prev == fast.smoother._prev
                for result, lines in [(baseline, sent_good), (optimized, sent_fast)]:
                    command = result[0]
                    lines.append(f"{command.v_mps:.3f} {command.omega_radps:.3f}")
                rows.append({"sequence": identity, "core": masks[i], "video": batch[i][0],
                             "frame": batch[i][1], "command": asdict(optimized[0])})
            sequence += len(batch)
        assert sent_good == sent_fast
        args.output.write_text(json.dumps({"passed": True, "cores": masks,
            "frames": len(rows), "records": rows, "software_velocity_lines": sent_fast,
            "scope": "visual and software serialization; no real UART or ODOM/IMU"}, indent=2))
        print("parallel equivalence passed", len(rows), flush=True)
    finally:
        for pool in pools:
            pool.shutdown(wait=True)
        for segment in segments + [reference]:
            segment.close()


if __name__ == "__main__":
    main()
