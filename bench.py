"""Quick benchmark: model load time and per-frame inference time on this machine.

Usage:
  python bench.py <image.jpg> [face_db.npz] [runs]

If a database file is given, the full recognize pipeline is timed;
otherwise only detection + embedding (app.get) is timed.
"""

import os
import sys
import time

import cv2
import numpy as np

from engine import FacialRecognition


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    img_path = sys.argv[1]
    db_path = sys.argv[2] if len(sys.argv) > 2 else None
    runs = int(sys.argv[3]) if len(sys.argv) > 3 else 5

    img = cv2.imread(img_path)
    if img is None:
        print(f"Cannot read image: {img_path}")
        sys.exit(1)
    print(f"Image: {img.shape[1]}x{img.shape[0]}")

    fr = FacialRecognition()
    t0 = time.time()
    fr.init_app()
    print(f"Model init: {time.time() - t0:.2f}s")

    # warmup (first inference is slower)
    fr.app.get(img)

    use_db = db_path and os.path.exists(db_path)
    print(f"Mode: {'recognize (db=' + db_path + ')' if use_db else 'detect + embed only'}")

    ts = []
    for _ in range(runs):
        t0 = time.time()
        if use_db:
            _, results = fr.recognize_from_frame(img, db_path)
        else:
            results = fr.app.get(img)
        ts.append(time.time() - t0)

    n = len(results)
    print(f"Faces found: {n}")
    print(
        f"Per-frame over {runs} runs: "
        f"avg {np.mean(ts) * 1000:.0f}ms  "
        f"min {min(ts) * 1000:.0f}ms  "
        f"max {max(ts) * 1000:.0f}ms"
    )


if __name__ == "__main__":
    main()
