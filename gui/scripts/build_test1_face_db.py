"""只用十张登记照建立测试1 ArcFace 库，并核对独立的 DINOv3 人脸模板。"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np


ENROLL = Path("/home/orangepi/vision_compare_data_v2/face_enroll")
RESULTS = Path("/home/orangepi/vision_compare_results")
DB = RESULTS / "face_db.npz"
DINO_GALLERY = RESULTS / "face_dino_gallery.json"
LABELS = [f"suspect_{index:02d}" for index in range(1, 11)]


def enrollment_paths() -> list[Path]:
    if not ENROLL.is_dir():
        raise FileNotFoundError(f"登记照目录不存在：{ENROLL}")
    children = sorted(path.name for path in ENROLL.iterdir())
    if children != LABELS:
        raise ValueError(f"登记照身份目录不符合预期：{children}")
    paths = []
    for label in LABELS:
        folder = ENROLL / label
        files = sorted(path.name for path in folder.iterdir())
        if files != ["reference.jpg"]:
            raise ValueError(f"{folder} 必须只含 reference.jpg，实际：{files}")
        paths.append(folder / "reference.jpg")
    return paths


def check_dino_gallery(paths: list[Path]) -> str:
    if not DINO_GALLERY.is_file():
        return f"DINOv3 人脸模板不存在：{DINO_GALLERY}"
    gallery = json.loads(DINO_GALLERY.read_text(encoding="utf-8"))
    audit = gallery.get("enrollment", [])
    if gallery.get("labels") != LABELS or len(audit) != len(paths):
        raise ValueError("DINOv3 人脸模板身份集合与登记照不一致")
    embeddings = np.asarray(gallery.get("embeddings"), dtype=np.float32)
    if embeddings.shape != (10, 384) or not np.isfinite(embeddings).all():
        raise ValueError(f"DINOv3 人脸模板向量无效：{embeddings.shape}")
    for path, record in zip(paths, audit):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if record.get("path") != str(path) or record.get("sha256") != digest:
            raise ValueError(f"DINOv3 模板来源与登记照不一致：{path}")
    return f"DINOv3 人脸模板已核对：{DINO_GALLERY}（10 人，独立于刀具模板）"


def main() -> int:
    paths = enrollment_paths()
    print(f"登记照核对通过：{len(paths)} 张，来源仅 {ENROLL}")
    try:
        print(check_dino_gallery(paths))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"DINOv3 人脸模板核对失败：{error}", file=sys.stderr)

    repo = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo / "vision" / "arcface-lite"))
    from engine import FacialRecognition

    RESULTS.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(prefix="face_db_", suffix=".npz", dir=RESULTS,
                                     delete=False) as handle:
        temporary = Path(handle.name)
    try:
        recognizer = FacialRecognition(model_name="buffalo_sc")
        recognizer.init_app(det_size=(640, 640))
        recognizer.build_registry(str(ENROLL), str(temporary), det_size=640)
        with np.load(temporary, allow_pickle=False) as database:
            names = list(database["names"].astype(str))
            vectors = database["embs"]
        if names != LABELS or vectors.shape[0] != 10 or not np.isfinite(vectors).all():
            raise RuntimeError(f"ArcFace 库身份或向量异常：{names}, {vectors.shape}")
        for path in paths:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"登记照读取失败：{path}")
            _, results = recognizer.recognize_from_frame(image, str(temporary))
            expected = path.parent.name
            if not any(result.get("name") == expected for result in results):
                raise RuntimeError(f"登记照冒烟测试失败：{expected} -> {results}")
            print(f"冒烟测试通过：{expected}")
        os.replace(temporary, DB)
        print(f"ArcFace 库已保存：{DB}（10 人）")
        return 0
    finally:
        temporary.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
