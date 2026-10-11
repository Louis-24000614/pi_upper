"""Run one isolated candidate through the existing full navigation benchmark."""
import argparse
import json
import sys
from pathlib import Path
from dataclasses import replace
import cv2
from experiments.seg_fps.hardware_candidates import install


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--candidate", required=True)
    options, rest = parser.parse_known_args()
    resources = install(options.candidate)
    sys.argv = [sys.argv[0], *rest]
    from experiments.seg_fps import benchmark
    video = Path(rest[rest.index("--video") + 1])
    capture = cv2.VideoCapture(str(video))
    ok, warm_image = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError(f"no warmup frame: {video}")
    warmup = int(rest[rest.index("--warmup") + 1]) if "--warmup" in rest else 20
    original_segment = benchmark.RoadSegmenter
    original_navigation = benchmark.Navigation
    original_frames = benchmark.VideoFrames
    original_gate = benchmark.ResultGate

    class WarmSegmenter(original_segment):
        def _load(self):
            session = super()._load()
            if not getattr(self, "_complete_warmup", False):
                self._complete_warmup = True
                for _ in range(warmup):
                    self.mask(warm_image)
            return session

    class WarmNavigation(original_navigation):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            if self.projector is not None:
                import numpy as np
                self.projector.project(np.zeros(warm_image.shape[:2], np.uint8), self.cfg)
                if hasattr(self.projector, "samples"):
                    self.projector.samples.clear()

    class ReadEndFrames(original_frames):
        def _iterate(self):
            for frame in super()._iterate():
                yield frame if self.realtime else replace(frame, published_s=frame.decoded_s)

    class HardwareGate(original_gate):
        def __init__(self, source=0, max_age_s=None):
            super().__init__(source, 0.2 if max_age_s is None else max_age_s)

    benchmark.RoadSegmenter = WarmSegmenter
    benchmark.Navigation = WarmNavigation
    benchmark.VideoFrames = ReadEndFrames
    benchmark.ResultGate = HardwareGate
    try:
        benchmark.main()
        output = Path(rest[rest.index("--output") + 1])
        report = json.loads(output.read_text())
        report["hardware_candidate"] = options.candidate
        report["hardware_path_used"] = options.candidate != "cpu"
        report["hardware_path_used"] = options.candidate not in ("cpu", "native-standard")
        report["frame_age_origin"] = "read end to navigation end; max-age gate at original pre-navigation location"
        report["offline_max_age_s"] = 0.2
        report["complete_mask_warmup_per_context"] = warmup
        report["hardware_last_timings"] = [getattr(r, "last_timings", {}) for r in resources]
        from experiments.seg_fps.telemetry import summarize
        report["hardware_timing_metrics"] = [
            {key: summarize([row[key] for row in r.samples])
             for key in r.samples[0]} if getattr(r, "samples", []) else {}
            for r in resources]
        output.write_text(json.dumps(report, indent=2))
    finally:
        for resource in resources:
            resource.close()


if __name__ == "__main__":
    main()
