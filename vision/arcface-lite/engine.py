"""Standalone lightweight face recognition engine, optimized for ARM CPU (e.g. Orange Pi RK3588).

Extracted from the Radish project (service/arcface/face/engine.py) with these changes:
  - CPU-only inference (ONNX Runtime CPUExecutionProvider, no CUDA)
  - Lightweight InsightFace model pack "buffalo_sc":
      detection   SCRFD-500M   (instead of RetinaFace-ResNet50)
      recognition MobileFaceNet (instead of ResNet100)
    -> roughly 5-10x faster than the default buffalo_l pack on ARM
  - Lower default detection input size (320x320 instead of 640x640)

IMPORTANT: embeddings are NOT compatible with a database built with the
buffalo_l pack. Re-register all faces after switching models.

CLI:
  python engine.py photo <img> [--out out.jpg]
  python engine.py register <root_dir> <out_db>     # root_dir/PersonName/*.jpg
  python engine.py recognize <img> <db> [--th 0.45] [--out out.jpg]
"""

import argparse
import os

import cv2
import numpy as np

# Compatibility for dependencies expecting np.int (removed in NumPy 1.24+).
if not hasattr(np, "int"):
    np.int = int

from insightface.app import FaceAnalysis


def l2_normalize(v, eps=1e-12):
    return v / (np.linalg.norm(v) + eps)


class FacialRecognition:
    """Face detection, embedding, registry build, and recognition against an .npz DB."""

    def __init__(self, model_name="buffalo_sc"):
        self.model_name = model_name
        self.app = None

    def init_app(self, det_size=(320, 320), det_thresh=None):
        """Prepare FaceAnalysis on CPU."""
        app = FaceAnalysis(name=self.model_name, providers=["CPUExecutionProvider"])
        try:
            if det_thresh is None:
                app.prepare(ctx_id=-1, det_size=det_size)
            else:
                app.prepare(ctx_id=-1, det_size=det_size, det_thresh=det_thresh)
        except TypeError:
            app.prepare(ctx_id=-1, det_size=det_size)
        self.app = app
        return app

    def pick_largest_face(self, faces):
        if not faces:
            return None
        areas = []
        for f in faces:
            x1, y1, x2, y2 = f.bbox
            areas.append((x2 - x1) * (y2 - y1))
        return faces[int(np.argmax(areas))]

    def recognize_from_frame(self, color_img, db_path, threshold=0.45):
        """
        Run recognition on a BGR numpy frame (H, W, 3).

        Returns:
            (vis_img, results) where results is a list of dicts with name, score, bbox.
        """
        if color_img is None:
            return None, []

        if self.app is None:
            self.init_app()

        names, db_embs = self.load_db(db_path)

        img = color_img.copy()

        faces = self.app.get(img)
        faces = self.sort_faces_left_to_right(faces)

        img_cx = color_img.shape[1] / 2.0
        results = []

        for f in faces:
            emb = f.normed_embedding.astype(np.float32)

            who, score = self.recognize_one(emb, names, db_embs, threshold)

            x1, y1, x2, y2 = map(int, f.bbox)
            bbox_cx = (x1 + x2) / 2.0
            direction_px = round(img_cx - bbox_cx, 1)

            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                img,
                f"{who} {score:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            results.append({"name": who, "score": float(score), "bbox": (x1, y1, x2, y2), "direction_px": direction_px})

        return img, results

    def register_face_from_frame(
        self, color_img, person_name, db_path, min_det_score=0.30, min_face_px=25
    ):
        """
        Register the largest face from a BGR frame into db_path (.npz).

        Returns:
            (success: bool, message: str)
        """
        if color_img is None:
            return False, "Image is empty"

        if self.app is None:
            self.init_app()

        faces, used_scale = self.detect_faces_pyramid(color_img)
        face = self.pick_largest_face(faces)

        if face is None:
            return False, "No face detected"

        score = float(getattr(face, "det_score", 1.0))
        x1, y1, x2, y2 = face.bbox
        fw, fh = float(x2 - x1), float(y2 - y1)

        if score < min_det_score:
            return False, f"Detection score too low: {score:.3f}"

        if fw < min_face_px or fh < min_face_px:
            return False, f"Face too small: {fw:.1f}x{fh:.1f}px"

        emb = face.normed_embedding.astype(np.float32)

        try:
            if os.path.exists(db_path):
                data = np.load(db_path, allow_pickle=True)
                names = list(data["names"])
                embs = list(data["embs"])
            else:
                names = []
                embs = []
        except Exception as e:
            return False, f"Failed to load database: {e}"

        if person_name in names:
            idx = names.index(person_name)
            old_emb = embs[idx].astype(np.float32)
            new_emb = l2_normalize((old_emb + emb) / 2).astype(np.float32)
            embs[idx] = new_emb
            print(f"[INFO] Updated embedding for {person_name}")
        else:
            names.append(person_name)
            embs.append(l2_normalize(emb).astype(np.float32))
            print(f"[INFO] Added new identity {person_name}")

        try:
            names_array = np.array(names)
            embs_array = np.stack(embs, axis=0)
            np.savez(db_path, names=names_array, embs=embs_array)
            return True, f"Registered {person_name} successfully"
        except Exception as e:
            return False, f"Failed to save database: {e}"

    def detect_faces_pyramid(self, img, scales=(1.0, 1.5, 2.0, 3.0, 4.0), max_side=4200):
        """Multi-scale detection for small / distant faces. Returns (faces, scale_used)."""
        H, W = img.shape[:2]
        for s in scales:
            newW, newH = int(W * s), int(H * s)
            if max(newW, newH) > max_side:
                continue
            if s == 1.0:
                r = img
            else:
                r = cv2.resize(img, (newW, newH), interpolation=cv2.INTER_CUBIC)

            faces = self.app.get(r)
            if len(faces) > 0:
                return faces, s

        return [], None

    def build_registry(
        self,
        root_dir,
        out_db_path,
        det_size=1920,
        det_thresh=0.35,
        min_det_score=0.30,
        min_face_px=25,
    ):
        """Build face_db.npz from root_dir/PersonName/*.jpg layout."""
        if self.app is None:
            self.init_app(det_size=(det_size, det_size), det_thresh=det_thresh)

        names = []
        embs = []

        for person_name in sorted(os.listdir(root_dir)):
            person_dir = os.path.join(root_dir, person_name)
            if not os.path.isdir(person_dir):
                continue

            person_embs = []

            for fn in sorted(os.listdir(person_dir)):
                if not fn.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".webp")):
                    continue
                path = os.path.join(person_dir, fn)
                img = cv2.imread(path)
                if img is None:
                    print(f"[WARN] Cannot read image: {path}")
                    continue

                faces, used_scale = self.detect_faces_pyramid(img)
                face = self.pick_largest_face(faces)
                if face is None:
                    print(f"[WARN] {person_name}/{fn}: no face (pyramid tried)")
                    continue

                score = float(getattr(face, "det_score", 1.0))
                x1, y1, x2, y2 = face.bbox
                fw, fh = float(x2 - x1), float(y2 - y1)

                if score < min_det_score:
                    print(
                        f"[WARN] {person_name}/{fn} low det_score={score:.3f} (scale={used_scale})"
                    )
                    continue

                if fw < min_face_px or fh < min_face_px:
                    print(
                        f"[WARN] {person_name}/{fn} face too small: {fw:.1f}x{fh:.1f}px (scale={used_scale})"
                    )
                    continue

                emb = face.normed_embedding.astype(np.float32)
                person_embs.append(emb)
                print(
                    f"[OK] {person_name}/{fn} det_score={score:.3f} face={fw:.0f}x{fh:.0f}px scale={used_scale}"
                )

            if len(person_embs) == 0:
                print(f"[WARN] {person_name}: no usable faces, skip")
                continue

            mean_emb = l2_normalize(np.mean(person_embs, axis=0)).astype(np.float32)
            names.append(person_name)
            embs.append(mean_emb)
            print(f"[OK] {person_name}: registered, {len(person_embs)} samples")

        if len(names) == 0:
            raise RuntimeError(
                "No identities registered. Use closer, higher-resolution face photos."
            )

        names = np.array(names)
        embs = np.stack(embs, axis=0)
        np.savez(out_db_path, names=names, embs=embs)

        print("\n==============================")
        print("Saved database:", out_db_path)
        print("Count:", len(names))
        print("Identities:", list(names))

    def analyze_photo(self, img_path, out_path=None):
        """Detect faces, draw boxes, save image."""
        if not os.path.exists(img_path):
            raise FileNotFoundError(f"Image not found: {img_path}")
        if self.app is None:
            self.init_app()

        img = cv2.imread(img_path)
        faces = self.app.get(img)
        rimg = self.app.draw_on(img.copy(), faces)
        out_path = out_path or os.path.splitext(img_path)[0] + "_out.jpg"
        cv2.imwrite(out_path, rimg)
        print("Faces:", len(faces))
        print("Output:", out_path)

    def sort_faces_left_to_right(self, faces):
        def cx(f):
            x1, y1, x2, y2 = f.bbox
            return (x1 + x2) / 2

        return sorted(faces, key=cx)

    def load_db(self, db_path):
        data = np.load(db_path, allow_pickle=True)
        names = data["names"]
        embs = data["embs"].astype(np.float32)
        embs = np.stack([l2_normalize(e) for e in embs], axis=0).astype(np.float32)
        return names, embs

    def list_faces(self, db_path):
        """Return list of registered names from the database."""
        if not os.path.exists(db_path):
            return []
        data = np.load(db_path, allow_pickle=True)
        return list(data["names"])

    def delete_face(self, db_path, person_name):
        """Remove a face identity from the database. Returns (success, message)."""
        if not os.path.exists(db_path):
            return False, "Database not found"

        data = np.load(db_path, allow_pickle=True)
        names = list(data["names"])
        embs = list(data["embs"])

        if person_name not in names:
            return False, f"{person_name!r} not found in database"

        idx = names.index(person_name)
        names.pop(idx)
        embs.pop(idx)

        if len(names) == 0:
            os.remove(db_path)
            return True, f"Deleted {person_name!r} (database is now empty)"

        names_array = np.array(names)
        embs_array = np.stack(embs, axis=0)
        np.savez(db_path, names=names_array, embs=embs_array)
        return True, f"Deleted {person_name!r} ({len(names)} identities remaining)"

    def recognize_one(self, emb, names, db_embs, threshold=0.45):
        sims = db_embs @ emb
        idx = int(np.argmax(sims))
        best = float(sims[idx])
        if best >= threshold:
            return str(names[idx]), best
        return "Unknown", best

    def recognize_group(self, img_path, db_path, out_path=None, threshold=0.45):
        """Recognize all faces in an image and save annotated output."""
        if not os.path.exists(db_path):
            raise FileNotFoundError("Database not found; build one first (e.g. build_registry)")
        if not os.path.exists(img_path):
            raise FileNotFoundError(f"Image not found: {img_path}")

        names, db_embs = self.load_db(db_path)
        if self.app is None:
            self.init_app()

        img = cv2.imread(img_path)
        faces = self.app.get(img)
        faces = self.sort_faces_left_to_right(faces)

        vis = img.copy()
        print("Face count:", len(faces))
        for i, f in enumerate(faces, 1):
            emb = f.normed_embedding.astype(np.float32)
            who, score = self.recognize_one(emb, names, db_embs, threshold=threshold)

            x1, y1, x2, y2 = map(int, f.bbox)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"{who} {score:.2f}",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            print(f"Face#{i}: {who} score={score:.3f} bbox=({x1},{y1},{x2},{y2})")

        out_path = out_path or os.path.splitext(img_path)[0] + "_out.jpg"
        cv2.imwrite(out_path, vis)
        print("Output:", out_path)


def main():
    p = argparse.ArgumentParser(description="Lightweight face recognition CLI (CPU / ARM)")
    sub = p.add_subparsers(dest="cmd")

    pa = sub.add_parser("photo")
    pa.add_argument("img")
    pa.add_argument("--out", default=None)

    pr = sub.add_parser("register")
    pr.add_argument("root_dir")
    pr.add_argument("out_db")
    pr.add_argument("--det_size", type=int, default=1920)
    pr.add_argument("--det_thresh", type=float, default=0.35)

    rc = sub.add_parser("recognize")
    rc.add_argument("img")
    rc.add_argument("db")
    rc.add_argument("--out", default=None)
    rc.add_argument("--th", type=float, default=0.45)

    args = p.parse_args()
    if args.cmd is None:
        p.print_help()
        return

    fr = FacialRecognition()

    if args.cmd == "photo":
        fr.analyze_photo(args.img, out_path=args.out)
    elif args.cmd == "register":
        fr.init_app(
            det_size=(args.det_size, args.det_size),
            det_thresh=args.det_thresh,
        )
        fr.build_registry(args.root_dir, args.out_db)
    elif args.cmd == "recognize":
        fr.recognize_group(args.img, args.db, out_path=args.out, threshold=args.th)


if __name__ == "__main__":
    main()
