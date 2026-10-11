"""Serialized board-only experiment stages with durable process/result state."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--videos", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    options = parser.parse_args()
    options.output_dir.mkdir(parents=True, exist_ok=False)
    report = {"pid": os.getpid(), "started_unix": time.time(), "stages": [], "finished": False}
    status = options.output_dir / "status.json"
    shared = ["--video", str(options.video), "--model", str(options.model), "--config", str(options.config),
              "--variant", "buffers", "--cores", "1,2,4", "--warmup", "20",
              "--opencv-threads", "8", "--blas-threads", "1", "--sample-interval", "10"]
    verification = ["--videos", str(options.videos), "--model", str(options.model),
                    "--config", str(options.config)]
    stages = []
    for candidate in ("native-output", "rga"):
        stages.append((f"verify_{candidate}", ["-m", "experiments.seg_fps.verify_hardware",
                       "--candidate", candidate, "--output", str(options.output_dir / f"verify_{candidate}.json"),
                       *verification]))
    for a, b in (("cpu", "gpu-bev"), ("cpu", "rga"), ("native-standard", "native-output")):
        for count, seconds, suffix in ((1, 30, "screen"), (5, 60, "formal")):
            name = f"{b}_{suffix}"
            stages.append((name, ["-m", "experiments.seg_fps.hardware_pairs", "--a", a, "--b", b,
                           "--pairs", str(count), "--seconds", str(seconds),
                           "--output-dir", str(options.output_dir / name), *shared]))
    stages.append(("record_formal", ["-m", "experiments.seg_fps.hardware_pairs", "--a", "cpu", "--b", "cpu",
                   "--record-a", "sync", "--record-b", "async", "--pairs", "5", "--seconds", "60",
                   "--output-dir", str(options.output_dir / "record_formal"), *shared]))
    report["planned_stages"] = [name for name, _ in stages]
    for name, command in stages:
        # A failed stage stays visible, but cannot silently prevent other candidates being evaluated.
        row = {"name": name, "start_unix": time.time(), "command": [sys.executable, *command]}
        report["stages"].append(row)
        with (options.output_dir / f"{name}.log").open("w") as log:
            process = subprocess.Popen(row["command"], stdout=log, stderr=subprocess.STDOUT)
            row["child_pid"] = process.pid
            status.write_text(json.dumps(report, indent=2))
            print("START", name, process.pid, flush=True)
            row["exit_code"] = process.wait()
            row["end_unix"] = time.time()
        status.write_text(json.dumps(report, indent=2))
        print("END", name, row["exit_code"], flush=True)
    report["finished"] = True
    report["end_unix"] = time.time()
    status.write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
