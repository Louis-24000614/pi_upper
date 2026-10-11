import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from ipm_proto.ipm import CameraExtrinsics
from road_follow.culvert_perception import CulvertCalibration
from road_follow.culvert_runtime import validate_culvert_config
from road_follow.pipeline import make_ipm


ROOT = Path(__file__).resolve().parents[3]


class EstimatedCameraTest(unittest.TestCase):
    def setUp(self):
        self.nav = yaml.safe_load((ROOT / "config/nav_camera.yaml").read_text(encoding="utf-8"))
        self.path = ROOT / "config/culvert.yaml"

    def validate(self, **kwargs):
        return validate_culvert_config(self.path, ROOT, drive=True, image_size=(1280, 720),
                                       near_speed=.05, nav_config=self.nav, **kwargs)

    def test_estimated_mode_requires_explicit_selection_and_keeps_config_unverified(self):
        with self.assertRaises(ValueError):
            self.validate()
        mapping, _, projection = self.validate(estimated_camera=True)
        self.assertTrue(projection.valid)
        self.assertFalse(mapping["calibration"]["verified"])
        self.assertFalse(projection.metadata["verified"])
        self.assertFalse(projection.metadata["ground_contact_verified"])
        self.assertEqual(projection.mode, "estimated_camera")
        self.assertEqual(projection.metadata["camera"]["height_m"], self.nav["camera"]["height_m"])
        self.assertEqual(projection.metadata["camera"]["pitch_deg"], 28)

    def test_projection_matches_original_camera_and_physical_ground_points(self):
        _, _, projection = self.validate(estimated_camera=True)
        original = make_ipm(self.nav, (720, 1280))
        pixels = [(640, 400), (550, 500), (750, 450)]
        np.testing.assert_allclose(projection.ipm.image_points_to_ground(pixels),
                                   original.image_points_to_ground(pixels), atol=1e-6)
        camera = self.nav["camera"]
        ground = np.array([[-.10, .20], [.10, .60], [0, .4]])
        uv = CameraExtrinsics(camera["height_m"], np.deg2rad(camera["pitch_deg"]),
            camera["fx"], camera["fy"], camera["cx"], camera["cy"]).project_ground(ground)
        np.testing.assert_allclose(projection.ipm.image_points_to_ground(uv), ground, atol=1e-6)
        self.assertTrue(projection.check_shape((720, 1280, 3)))
        self.assertFalse(projection.check_shape((480, 640, 3)))

    def test_invalid_camera_parameters_cannot_enable_estimated_mode(self):
        cases = [("height_m", 0), ("height_m", -.16), ("fx", 0), ("fy", -1),
                 ("pitch_deg", 0), ("pitch_deg", 90), ("cx", 1280), ("cy", -1),
                 ("fx", float("nan")), ("fy", float("inf")), ("height_m", True)]
        for key, value in cases:
            with self.subTest(key=key, value=value):
                nav = copy.deepcopy(self.nav)
                nav["camera"][key] = value
                with self.assertRaises(ValueError):
                    CulvertCalibration.from_camera_parameters(nav, (1280, 720))
        nav = copy.deepcopy(self.nav)
        del nav["camera"]["fy"]
        with self.assertRaises(ValueError):
            CulvertCalibration.from_camera_parameters(nav, (1280, 720))

    def test_estimated_mode_still_checks_model_fingerprint(self):
        with tempfile.TemporaryDirectory() as folder:
            mapping = yaml.safe_load(self.path.read_text(encoding="utf-8"))
            mapping["model"]["sha256"] = "0" * 64
            mapping["class_mapping"]["model_sha256"] = "0" * 64
            path = Path(folder) / "culvert.yaml"
            path.write_text(yaml.safe_dump(mapping), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "SHA256"):
                validate_culvert_config(path, ROOT, drive=True, image_size=(1280, 720),
                    near_speed=.05, nav_config=self.nav, estimated_camera=True)

    def test_cli_requires_culvert_stop_before_hardware_or_frequency_guard(self):
        from road_follow import __main__ as entry
        with patch.object(entry, "_open_camera") as camera, \
             patch.object(entry.subprocess, "Popen") as uart, \
             patch.object(entry, "needs_frequency_guard") as guard:
            with self.assertRaises(SystemExit) as result:
                entry.main(["--culvert-estimated-camera"])
            self.assertEqual(result.exception.code, 2)
            camera.assert_not_called()
            uart.assert_not_called()
            guard.assert_not_called()

    def test_cli_passes_estimated_parameters_without_opening_hardware_in_test(self):
        from road_follow import __main__ as entry
        captured = []
        with patch.object(entry, "needs_frequency_guard", return_value=False), \
             patch.object(entry, "_run", side_effect=lambda args, parser: captured.append(args) or 0), \
             patch.object(entry, "_open_camera") as camera, \
             patch.object(entry.subprocess, "Popen") as uart:
            self.assertEqual(entry.main(["--drive", "--turn-at-junction", "left", "--culvert-stop",
                "--culvert-estimated-camera", "--heap-trim-interval", "0"]), 0)
        self.assertEqual(captured[0].culvert_setup[2].mode, "estimated_camera")
        camera.assert_not_called()
        uart.assert_not_called()


if __name__ == "__main__":
    unittest.main()
