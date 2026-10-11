"""完整解码实验录像，核对尺寸、10 FPS 时间轴与原始丢弃统计。"""
import argparse
import json
from pathlib import Path
import cv2


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--results", type=Path, required=True)
    args = p.parse_args()
    rows = []
    for path in sorted(args.results.glob("record_*/*.avi")):
        metrics = json.loads(path.with_suffix(".json").read_text())
        cap = cv2.VideoCapture(str(path))
        try:
            if not cap.isOpened():
                raise AssertionError(f"无法打开录像 {path}")
            declared = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            count = 0
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                if frame.shape != (720,1280,3):
                    raise AssertionError("录像尺寸改变")
                count += 1
            if count != declared or count == 0 or fps != 10:
                raise AssertionError("录像帧数或帧率异常")
            if abs(count/fps-metrics["seconds"]) > 1:
                raise AssertionError("录像时间轴偏离测量窗口超过 1 秒")
            rows.append({"path": str(path), "record": metrics["record"],
                         "frames": count, "fps": fps, "duration_s": count/fps,
                         "dropped_input": metrics["record_dropped"]})
        finally:
            cap.release()
    if len(rows) != 9:
        raise AssertionError(f"预期 9 段同步/异步录像，实际 {len(rows)}")
    (args.results/"recording_validation.json").write_text(json.dumps(
        {"passed": True, "videos": rows}, indent=2))
    print("recording validation passed", len(rows), flush=True)


if __name__ == "__main__":
    main()
