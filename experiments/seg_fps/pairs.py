"""交替 A→B / B→A，统一轮长与预热，逐轮保留原始 JSON。"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--a", default="a1")
    parser.add_argument("--b", default="bev")
    parser.add_argument("--cores-a", default="1")
    parser.add_argument("--cores-b", default="1")
    parser.add_argument("--pairs", type=int, default=3)
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--label", required=True)
    parser.add_argument("--mode", default="full")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--speed", type=float, default=1)
    parser.add_argument("--record-a", default="none")
    parser.add_argument("--record-b", default="none")
    parser.add_argument("--backend-a", default="lite")
    parser.add_argument("--backend-b", default="lite")
    parser.add_argument("--mixed-a", action="store_true")
    parser.add_argument("--mixed-b", action="store_true")
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    directory = args.output_dir / args.label
    directory.mkdir(parents=True, exist_ok=True)
    results = []
    for index in range(args.pairs):
        pair = {}
        for name in (["a", "b"] if index % 2 == 0 else ["b", "a"]):
            output = directory / f"{index+1}_{name}.json"
            command = [sys.executable, "-m", "experiments.seg_fps.benchmark",
                       "--video", str(args.video), "--model", str(args.model),
                       "--config", str(args.config), "--output", str(output),
                       "--variant", getattr(args, name), "--mode", args.mode,
                       "--cores", getattr(args, "cores_"+name),
                       "--record", getattr(args, "record_"+name),
                       "--backend", getattr(args, "backend_"+name),
                       "--seconds", str(args.seconds), "--speed", str(args.speed)]
            if args.realtime:
                command.append("--realtime")
            if getattr(args, "mixed_"+name):
                command.append("--mixed-load")
            with output.with_suffix(".log").open("w") as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            data = json.loads(output.read_text())
            pair[name] = {k: data[k] for k in ["fps", "consumed_fps", "metrics", "cpu_percent",
                                               "rss_peak_kib", "dropped_input", "rejections"]}
            print(args.label, index+1, name, "fps", round(data["fps"], 3),
                  "age_p95", data["metrics"].get("age_ms", {}).get("p95"), flush=True)
        results.append(pair)
    ratios = [p["b"]["fps"]/p["a"]["fps"]-1 for p in results if p["a"]["fps"] > 0]
    summary = {"label": args.label, "pairs": results,
               "mean_gain_pct": float(np.mean(ratios)*100) if ratios else None,
               "min_gain_pct": float(np.min(ratios)*100) if ratios else None,
               "max_gain_pct": float(np.max(ratios)*100) if ratios else None,
               "std_gain_pct": float(np.std(ratios)*100) if ratios else None}
    (directory / "summary.json").write_text(json.dumps(summary, indent=2))
    print("SUMMARY", args.label, {k:v for k,v in summary.items() if k != "pairs"}, flush=True)


if __name__ == "__main__":
    main()
