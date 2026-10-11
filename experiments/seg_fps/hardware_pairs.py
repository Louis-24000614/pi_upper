"""Alternating single-factor hardware experiments, with no physical actions."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--a", default="cpu")
    parser.add_argument("--b", required=True)
    parser.add_argument("--pairs", type=int, default=5)
    parser.add_argument("--seconds", type=int, default=60)
    parser.add_argument("--record-a", default="none")
    parser.add_argument("--record-b", default="none")
    parser.add_argument("--output-dir", type=Path, required=True)
    options, rest = parser.parse_known_args()
    options.output_dir.mkdir(parents=True, exist_ok=False)
    results = []
    for index in range(options.pairs):
        pair = {}
        for side in ("ab" if index % 2 == 0 else "ba"):
            output = options.output_dir / f"{index + 1}_{side}.json"
            command = [sys.executable, "-m", "experiments.seg_fps.hardware_benchmark",
                       "--candidate", getattr(options, side), "--seconds", str(options.seconds),
                       "--record", getattr(options, "record_" + side),
                       "--output", str(output), *rest]
            with output.with_suffix(".log").open("w") as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            data = json.loads(output.read_text())
            pair[side] = {k: data[k] for k in ("fps", "metrics", "cpu_percent", "rss_peak_kib",
                                             "rejections", "dropped_input", "hardware_candidate")}
            print("PAIR", index + 1, side, round(data["fps"], 4), flush=True)
        results.append(pair)
    gains = np.array([p["b"]["fps"] / p["a"]["fps"] - 1 for p in results]) * 100
    summary = {"a": options.a, "b": options.b, "pairs": results,
               "mean_gain_pct": float(np.mean(gains)), "std_gain_pct": float(np.std(gains)),
               "min_gain_pct": float(np.min(gains)), "max_gain_pct": float(np.max(gains)),
               "seconds_per_side": options.seconds, "argv": sys.argv}
    (options.output_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print("SUMMARY", {k: v for k, v in summary.items() if k not in ("pairs", "argv")}, flush=True)


if __name__ == "__main__":
    main()
