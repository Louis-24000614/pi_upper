"""Compare real road masks, BEV, temporal navigation, and retained ownership."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from road_follow.cpu_threads import cpu_thread_budget
from road_follow.segment import RoadSegmenter
from road_follow.pipeline import BevProjector
from experiments.seg_fps.navigation import Navigation
from experiments.seg_fps.hardware_candidates import OpenClProjector
from experiments.seg_fps.regression_threads import InspectSession


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--videos", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()
    paths = sorted(args.videos.glob("*.avi"))
    chosen = [paths[i] for i in np.unique(np.linspace(0, len(paths) - 1, min(9, len(paths)), dtype=int))]
    cfg = yaml.safe_load(args.config.read_text())
    segment = RoadSegmenter(args.model, core_mask=1)
    cpu, gpu = BevProjector(), OpenClProjector()
    report = {"passed": False, "videos": [], "frames": 0, "errors": [],
              "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
              "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
              "scope": "nine selected full videos; unchanged RKNN shared; GPU BEV and temporal navigation compared"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    retained = None
    try:
        with cpu_thread_budget(8, 1):
            segment._session = InspectSession(segment._load())
            for path in chosen:
                cap = cv2.VideoCapture(str(path))
                declared = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                count = 0
                navs = [Navigation(cfg, True), Navigation(cfg, True)]
                navs[1].projector = gpu
                try:
                    while not args.limit or count < args.limit:
                        ok, image = cap.read()
                        if not ok:
                            break
                        mask = segment.mask(image)
                        a = cpu.project(mask, cfg)[1]
                        b = gpu.project(mask, cfg)[1]
                        np.testing.assert_array_equal(a, b)
                        if navs[0].process(mask) != navs[1].process(mask):
                            raise AssertionError(f"navigation mismatch {path.name}:{count}")
                        if navs[0].smoother._prev != navs[1].smoother._prev:
                            raise AssertionError("EMA mismatch")
                        if retained is not None:
                            old, digest = retained
                            if hashlib.sha256(old.tobytes()).hexdigest() != digest:
                                raise AssertionError("retained GPU BEV was overwritten")
                        retained = b, hashlib.sha256(b.tobytes()).hexdigest()
                        count += 1
                        if count % 200 == 0:
                            print("GPU_REGRESSION", path.name, count, flush=True)
                    if count <= 0 or (not args.limit and count != declared):
                        raise AssertionError(f"incomplete decode {count}/{declared}")
                finally:
                    cap.release()
                report["videos"].append({"video": path.name, "frames": count, "declared": declared,
                                         "complete": count == declared, "differences": 0})
                report["frames"] += count
                args.output.write_text(json.dumps(report, indent=2))
            report["passed"] = True
    except BaseException as exc:
        report["errors"].append(repr(exc))
        raise
    finally:
        gpu.close()
        segment.close()
        args.output.write_text(json.dumps(report, indent=2))
    print("GPU_REGRESSION_DONE", report["frames"], flush=True)


if __name__ == "__main__":
    main()
