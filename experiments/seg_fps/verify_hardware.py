"""Quantify hardware preprocessing differences; strictly gate equivalent IO."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from road_follow.cpu_threads import cpu_thread_budget
from road_follow.segment import RoadSegmenter
from experiments.seg_fps.hardware_candidates import RgaBuffers, SharedRgaBuffers, OutputSession
from experiments.seg_fps.navigation import Navigation
from experiments.seg_fps.regression_threads import InspectSession


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", choices=("rga", "native-output", "mpp-decode", "rga-shared"), required=True)
    parser.add_argument("--videos", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--core", type=int, default=1)
    args = parser.parse_args()
    paths = sorted(args.videos.glob("*.avi"))
    if not paths:
        parser.error("no videos")
    chosen = [paths[i] for i in np.unique(np.linspace(0, len(paths) - 1, min(9, len(paths)), dtype=int))]
    cfg = yaml.safe_load(args.config.read_text())
    segments = [RoadSegmenter(args.model, core_mask=args.core) for _ in range(2)]
    resource = None
    resources = []
    report = {"execution_passed": False, "equivalent": True, "candidate": args.candidate,
              "frames": 0, "errors": [], "videos": [], "core": args.core,
              "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
              "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
              "scope": "selected nine videos; complete when limit=0; adjacent-identical inputs cached only for correctness"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with cpu_thread_budget(8, 1):
            if args.candidate == "rga":
                resource = RgaBuffers()
                segments[1]._buffers = resource
            elif args.candidate == "native-output":
                for i, segment in enumerate(segments):
                    segment._session = OutputSession(args.model, args.core, 3 if i else 1)
            elif args.candidate == "rga-shared":
                for i, segment in enumerate(segments):
                    segment._session = OutputSession(args.model, args.core, 1)
                    segment._buffers = SharedRgaBuffers(segment._session) if i else RgaBuffers()
                    resources.append(segment._buffers)
            for segment in segments:
                segment._session = InspectSession(segment._load())
            for path in chosen:
                cap = cv2.VideoCapture(str(path))
                hardware_cap = None
                if args.candidate == "mpp-decode":
                    from experiments.seg_fps.mpp_capture import MppCapture
                    hardware_cap = MppCapture(path)
                declared, count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), 0
                navs = [Navigation(cfg, True) for _ in range(2)]
                row = {"video": path.name, "mask_changed_frames": 0, "command_changed_frames": 0,
                       "navigation_changed_frames": 0, "min_mask_iou": 1.0, "mean_mask_iou": 0.0,
                       "tensor_max_abs": 0.0, "declared_frames": declared}
                row.update(bgr_max_abs=0, bgr_mean_abs=0.0)
                retained = None
                try:
                    while not args.limit or count < args.limit:
                        ok, image = cap.read()
                        if not ok:
                            break
                        images = [image, image]
                        if hardware_cap is not None:
                            ok_hardware, images[1] = hardware_cap.read()
                            if not ok_hardware or images[1].shape != image.shape:
                                raise AssertionError("MPP frame sequence/size mismatch")
                            diff = np.abs(image.astype(np.int16) - images[1].astype(np.int16))
                            row["bgr_max_abs"] = max(row["bgr_max_abs"], int(diff.max()))
                            row["bgr_mean_abs"] += float(diff.mean())
                        masks = [s.mask(frame) for s, frame in zip(segments, images)]
                        states = [nav.process(mask) for nav, mask in zip(navs, masks)]
                        changed = not np.array_equal(*masks)
                        row["mask_changed_frames"] += changed
                        row["command_changed_frames"] += states[0][0] != states[1][0]
                        state_changed = states[0] != states[1] or navs[0].smoother._prev != navs[1].smoother._prev
                        row["navigation_changed_frames"] += state_changed
                        intersection = np.count_nonzero(masks[0] & masks[1])
                        union = np.count_nonzero(masks[0] | masks[1])
                        iou = intersection / union if union else 1.0
                        row["min_mask_iou"] = min(row["min_mask_iou"], iou)
                        row["mean_mask_iou"] += iou
                        for a, b in zip(segments[0]._session.outputs, segments[1]._session.outputs):
                            row["tensor_max_abs"] = max(row["tensor_max_abs"], float(np.max(np.abs(a - b))))
                            if args.candidate in ("native-output", "rga-shared"):
                                np.testing.assert_allclose(a, b, atol=1e-5, rtol=1e-5)
                        if args.candidate in ("native-output", "rga-shared") and (changed or state_changed):
                            raise AssertionError(f"non-equivalent native output {path.name}:{count}")
                        if retained:
                            old, digest = retained
                            if hashlib.sha256(old.tobytes()).hexdigest() != digest:
                                raise AssertionError("retained mask overwritten")
                        retained = masks[1], hashlib.sha256(masks[1].tobytes()).hexdigest()
                        count += 1
                        if count % 200 == 0:
                            print("VERIFY", args.candidate, path.name, count, flush=True)
                    if count <= 0 or (not args.limit and count != declared):
                        raise AssertionError(f"incomplete decode {count}/{declared}")
                finally:
                    cap.release()
                    if hardware_cap is not None:
                        hardware_cap.release()
                row.update(frames=count, complete=count == declared)
                row["mean_mask_iou"] /= count
                row["bgr_mean_abs"] /= count
                report["equivalent"] &= not row["mask_changed_frames"] and not row["navigation_changed_frames"]
                report["frames"] += count
                report["videos"].append(row)
                args.output.write_text(json.dumps(report, indent=2))
            report["execution_passed"] = True
    except BaseException as exc:
        report["errors"].append(repr(exc))
        raise
    finally:
        for item in resources:
            item.close()
        for segment in segments:
            segment.close()
        if resource:
            resource.close()
        args.output.write_text(json.dumps(report, indent=2))
    print("VERIFY_DONE", args.candidate, report["frames"], report["equivalent"], flush=True)


if __name__ == "__main__":
    main()
