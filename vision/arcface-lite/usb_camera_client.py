"""USB monocular camera (UVC, e.g. SIMI319-HD720P) -> ArcFace server test client.

Captures frames from a standard USB webcam via OpenCV, JPEG-encodes them,
and streams them to the face server's WebSocket endpoint for recognition.

Usage:
    python usb_camera_client.py [--uri ws://127.0.0.1:20004/ws/recognize]
                                [--device 0] [--width 1280] [--height 720]
                                [--every 3] [--show]
"""

from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np
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
    ap.add_argument("--device", type=int, default=0, help="/dev/videoN index")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--every", type=int, default=3, help="recognize every N frames")
    ap.add_argument("--show", action="store_true", help="show annotated preview window")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(
            f"Cannot open /dev/video{args.device}. "
            "Is the USB camera plugged in? Try --device 1, 2, ..."
        )
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[client] /dev/video{args.device} opened: {w}x{h} -> {args.uri}")

    frame_idx = 0
    last_results: list[dict] = []
    t_last = time.time()

    try:
        with connect(args.uri, open_timeout=10) as ws:
            print("[client] WebSocket connected. Press Ctrl+C to quit.")
            while True:
                ok, img = cap.read()
                if not ok or img is None:
                    print("[client] WARN: failed to grab frame")
                    time.sleep(0.05)
                    continue

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
                    cv2.imshow("USB camera face recognition", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    except KeyboardInterrupt:
        print("\n[client] Stopped by user.")
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
