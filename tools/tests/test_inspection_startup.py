"""临时目录中的假 sysfs/DT；不接触相机、PWM、UART 或模型。"""
import errno
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch


TOOLS = Path(__file__).resolve().parents[1]


def module(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS/(name+".py"))
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


pwm = module("prepare_pwm14m0")
health = module("wait_inspection_health")


class PwmPreparationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.sysfs, self.dt, self.link = self.root/"pwm", self.root/"dt", self.root/"run/link"
        route = self.dt/"pinctrl/pwm14/pwm14m0-pins/phandle"
        route.parent.mkdir(parents=True)
        route.write_bytes(b"\x00\x00\x01\x7e")
        self.chip = self.make_chip("pwmchip9", route.read_bytes())
        self.account = Mock(pw_uid=os.getuid(), pw_gid=os.getgid())

    def make_chip(self, name, phandle):
        chip = self.sysfs/name
        (chip/"device/of_node").mkdir(parents=True)
        (chip/"device/of_node/pinctrl-0").write_bytes(phandle)
        (chip/"npwm").write_text("1")
        (chip/"export").write_text("")
        return chip

    def make_channel(self, chip):
        channel = chip/"pwm0"
        channel.mkdir()
        values = {"period":"20000000", "duty_cycle":"2500000", "polarity":"normal", "enable":"1"}
        for key, value in values.items():
            (channel/key).write_text(value)
            os.chmod(channel/key, 0o600)
        return values

    def prepare(self, **kwargs):
        with patch.object(pwm.pwd, "getpwnam", return_value=self.account):
            return pwm.prepare(self.sysfs, self.dt, self.link, **kwargs)

    def test_route_matching_ignores_chip_number_and_grants_only_four_nodes(self):
        self.make_chip("pwmchip2", b"\x00\x00\x00\x7e")
        values = self.make_channel(self.chip)
        with patch.object(pwm.os, "chown") as chown:
            chip, exported = self.prepare()
        self.assertEqual(chip, self.chip)
        self.assertFalse(exported)
        self.assertEqual(self.link.resolve(), self.chip.resolve())
        self.assertEqual(len(chown.call_args_list), 4)
        self.assertEqual({c.args[0].name for c in chown.call_args_list}, set(pwm.CONTROL_NODES))
        for key, value in values.items():
            self.assertEqual((self.chip/"pwm0"/key).read_text(), value)
            self.assertEqual((self.chip/"pwm0"/key).stat().st_mode & 0o777, 0o644)

    def test_missing_channel_is_exported_once_then_repeated_setup_preserves_enable(self):
        exporter = Mock(side_effect=self.make_channel)
        self.assertTrue(self.prepare(exporter=exporter)[1])
        self.assertFalse(self.prepare(exporter=exporter)[1])
        exporter.assert_called_once_with(self.chip)
        self.assertEqual((self.chip/"pwm0/enable").read_text(), "1")
        self.assertEqual((self.chip/"export").read_text(), "")

    def test_wrong_or_ambiguous_route_never_exports_or_grants_permissions(self):
        (self.chip/"device/of_node/pinctrl-0").write_bytes(b"bad!")
        exporter = Mock()
        with self.assertRaises(ValueError):
            self.prepare(exporter=exporter)
        exporter.assert_not_called()
        (self.chip/"device/of_node/pinctrl-0").write_bytes(b"\x00\x00\x01\x7e")
        self.make_chip("pwmchip2", b"\x00\x00\x01\x7e")
        with self.assertRaises(ValueError):
            self.prepare(exporter=exporter)
        exporter.assert_not_called()

    def test_incomplete_export_times_out_without_changing_permissions_or_link(self):
        with patch.object(pwm.os, "chown") as chown:
            with self.assertRaises(TimeoutError):
                self.prepare(timeout_s=0, exporter=Mock())
        chown.assert_not_called()
        self.assertFalse(self.link.exists())

    def test_busy_export_waits_for_existing_channel(self):
        def exporter(chip):
            self.make_channel(chip)
            raise OSError(errno.EBUSY, "already exported")
        self.assertFalse(self.prepare(exporter=exporter)[1])
        self.assertEqual((self.chip/"pwm0/enable").read_text(), "1")

    def test_regular_file_at_link_is_not_overwritten(self):
        self.make_channel(self.chip)
        self.link.parent.mkdir()
        self.link.write_text("keep")
        with self.assertRaises(ValueError):
            self.prepare()
        self.assertEqual(self.link.read_text(), "keep")


class HealthTest(unittest.TestCase):
    def test_face_requires_existing_database_and_knife_requires_ready_engine(self):
        self.assertTrue(health.health_ready("face", {"status":"ok","db_exists":True}))
        self.assertFalse(health.health_ready("face", {"status":"ok","db_exists":False}))
        self.assertTrue(health.health_ready("knife", {"status":"ok","ready":True}))
        self.assertFalse(health.health_ready("knife", {"status":"not_ready","ready":False}))
        self.assertFalse(health.health_ready("knife", []))

    def test_health_wait_retries_without_posting_images(self):
        response = Mock()
        response.__enter__ = Mock(return_value=Mock(read=Mock(return_value=b'{"status":"ok","ready":true}')))
        response.__exit__ = Mock(return_value=False)
        with patch.object(health, "urlopen", side_effect=[OSError("offline"),response]) as request, patch.object(health.time, "sleep"):
            health.wait_ready("knife", 2)
        self.assertEqual(request.call_count, 2)
        self.assertEqual(request.call_args.args[0], "http://127.0.0.1:20005/health")

    def test_zero_timeout_fails_without_accessing_service(self):
        with patch.object(health, "urlopen") as request:
            with self.assertRaises(RuntimeError):
                health.wait_ready("face", 0)
        request.assert_not_called()


if __name__ == "__main__":
    unittest.main()
