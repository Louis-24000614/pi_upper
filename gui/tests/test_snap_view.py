"""全屏拍照：设备选择与完整画面缩放（无硬件）。"""

from snap_view import fitted_size, ordered_paths, resolve_device, role_for_path


def test_fitted_size_keeps_full_frame_inside_a_smaller_box() -> None:
    width, height = fitted_size(1280, 720, 400, 300)
    assert (width, height) == (400, 225)
    assert width <= 400 and height <= 300


def test_fitted_size_is_identity_when_the_screen_matches() -> None:
    assert fitted_size(1280, 720, 1280, 720) == (1280, 720)


def test_fitted_size_scales_up_without_cropping() -> None:
    assert fitted_size(1280, 720, 1920, 1080) == (1920, 1080)


def test_resolve_device_defaults_to_first_camera() -> None:
    assert resolve_device(None, ["/dev/video0", "/dev/video2"]) == "/dev/video0"
    assert resolve_device(None, []) is None


def test_resolve_device_uses_requested_path() -> None:
    assert resolve_device("/dev/video2", ["/dev/video0"]) == "/dev/video2"


def test_ordered_paths_puts_requested_device_first() -> None:
    assert ordered_paths("/dev/video2", ["/dev/video0", "/dev/video2"]) == [
        "/dev/video2",
        "/dev/video0",
    ]


def test_role_follows_gui_mapping_unless_overridden() -> None:
    discovered = ["/dev/video0", "/dev/video2"]
    assert role_for_path("/dev/video0", discovered, None) == "recognition_camera"
    assert role_for_path("/dev/video2", discovered, None) == "navigation_camera"
    assert role_for_path("/dev/video0", discovered, "navigation_camera") == "navigation_camera"
