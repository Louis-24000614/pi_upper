"""离线合成身份库与假人脸模型；不加载模型、不连接摄像头。"""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


def load_engine():
    app = ModuleType("insightface.app")
    app.FaceAnalysis = Mock(side_effect=AssertionError("offline tests must not load models"))
    spec = importlib.util.spec_from_file_location("face_engine_offline", Path(__file__).parents[1]/"engine.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"insightface": ModuleType("insightface"), "insightface.app": app}):
        spec.loader.exec_module(module)
    return module


engine = load_engine()


class EngineFastPathTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"faces.npz"
        np.savez(self.path, names=np.array(["1", "2"]), embs=np.eye(2, dtype=np.float32))
        self.service = engine.FacialRecognition()

    def test_repeated_frames_reuse_database_and_replacement_reloads(self):
        with patch.object(engine.np, "load", wraps=np.load) as read:
            first = self.service.load_db(self.path)
            second = self.service.load_db(self.path)
            self.assertEqual(read.call_count, 1)
            np.testing.assert_array_equal(first[0], second[0])
            previous = self.path.stat()
            replacement = self.path.with_name("new.npz")
            np.savez(replacement, names=np.array(["3", "4"]), embs=np.eye(2, dtype=np.float32))
            os.utime(replacement, ns=(previous.st_atime_ns, previous.st_mtime_ns))
            os.replace(replacement, self.path)
            self.assertEqual(list(self.service.load_db(self.path)[0]), ["3", "4"])
            self.assertEqual(read.call_count, 2)

    def test_registration_and_deletion_invalidate_cached_identities(self):
        self.service.load_db(self.path)
        face = SimpleNamespace(bbox=np.array([0, 0, 40, 40]),
                               normed_embedding=np.array([1., 0.], np.float32), det_score=.9)
        self.service.app = Mock(get=Mock(return_value=[face]))
        ok, _ = self.service.register_face_from_frame(np.zeros((50, 50, 3), np.uint8), "3", self.path)
        self.assertTrue(ok)
        self.assertEqual(list(self.service.load_db(self.path)[0]), ["1", "2", "3"])
        ok, _ = self.service.delete_face(self.path, "1")
        self.assertTrue(ok)
        self.assertEqual(list(self.service.load_db(self.path)[0]), ["2", "3"])
        self.service.delete_face(self.path, "2")
        self.assertEqual(list(self.service.load_db(self.path)[0]), ["3"])
        self.path.unlink()
        with self.assertRaises(FileNotFoundError):
            self.service.load_db(self.path)

    def test_data_only_results_match_drawing_without_modifying_input(self):
        face = SimpleNamespace(bbox=np.array([4, 5, 20, 30]),
                               normed_embedding=np.array([1., 0.], np.float32))
        self.service.app = Mock(get=Mock(return_value=[face]))
        image = np.zeros((50, 60, 3), np.uint8)
        with patch.object(engine.cv2, "rectangle") as rectangle, patch.object(engine.cv2, "putText") as text:
            output, fast = self.service.recognize_from_frame(image, self.path, draw=False)
            self.assertIs(output, image)
            rectangle.assert_not_called()
            text.assert_not_called()
            annotated, original = self.service.recognize_from_frame(image, self.path)
            self.assertIsNot(annotated, image)
            self.assertEqual(fast, original)
            rectangle.assert_called_once()
            text.assert_called_once()
        self.assertFalse(image.any())


if __name__ == "__main__":
    unittest.main()
