"""RealSense D435i -> ArcFace server test client.

Captures color frames from the D435i via pyrealsense2, JPEG-encodes them,
and streams them to the face server's WebSocket endpoint for recognition.

Usage:
    python realsense_client.py [--uri ws://127.0.0.1:20004/ws/recognize]
                               [--width 640] [--height 480] [--fps 30]
                               [--show]
"""

from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    raise SystemExit(
        "pyrealsense2 not installed. Run:\n"
        "  .venv/bin/pip install pyrealsense2"
    )

from websockets.sync.client import connect


def draw_results(frame: np.ndarray, results: list[dict]) -> np.ndarray:
    vis = frame.copy()
    for r in results:
        x1, y1, x2, y2 = [int(v) for v in r.get("bbox", [0, 0, 0, 0])]
        name = r.get("name", "Unknown")
        score = r.get("score", 0.0)
        color = (0, 255, 0) if name != "Unknown" else (0, 0, 255)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            vis, f"{name} {score:.2f}", (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
        )
    return vis


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uri", default="ws://127.0.0.1:20004/ws/recognize")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--every", type=int, default=3, help="recognize every N frames")
    ap.add_argument("--show", action="store_true", help="show annotated preview window")
    args = ap.parse_args()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        raise SystemExit(f"Failed to start RealSense pipeline: {e}\nIs the D435i plugged in?")

    dev = profile.get_device()
    print(f"[client] Device: {dev.get_info(rs.camera_info.name)} "
          f"(SN: {dev.get_info(rs.camera_info.serial_number)})")
    print(f"[client] Streaming {args.width}x{args.height}@{args.fps} -> {args.uri}")

    frame_idx = 0
    last_results: list[dict] = []
    t_last = time.time()

    try:
        with connect(args.uri, open_timeout=10) as ws:
            print("[client] WebSocket connected. Press Ctrl+C to quit.")
            while True:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                img = np.asanyarray(color_frame.get_data())
                frame_idx += 1

                if frame_idx % args.every == 0:
                    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok:
                        ws.send(buf.tobytes())
                        try:
                            resp = json.loads(ws.recv(timeout=5))
                        except TimeoutError:
                            print("[client] WARN: server response timeout")
                            continue

                        if resp.get("status") == "success":
                            last_results = resp["data"]["results"]
                            now = time.time()
                            fps = args.every / max(now - t_last, 1e-6)
                            t_last = now
                            if last_results:
                                summary = ", ".join(
                                    f"{r['name']}({r['score']:.2f})" for r in last_results
                                )
                                print(f"[client] {fps:5.1f} rec-fps | {summary}")
                            else:
                                print(f"[client] {fps:5.1f} rec-fps | no face")
                        else:
                            print(f"[client] server error: {resp.get('message')}")

                if args.show:
                    vis = draw_results(img, last_results)
                    cv2.imshow("D435i face recognition", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    except KeyboardInterrupt:
        print("\n[client] Stopped by user.")
    finally:
        pipeline.stop()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
