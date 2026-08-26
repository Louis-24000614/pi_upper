"""轻量 V4L2 摄像头发现与 control 读取。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
import shutil
import subprocess
from typing import Optional


@dataclass
class V4L2Control:
    name: str
    kind: str
    minimum: Optional[int] = None
    maximum: Optional[int] = None
    step: Optional[int] = None
    default: Optional[int] = None
    value: Optional[int] = None
    menu_items: dict[int, str] = field(default_factory=dict)
    flags: str = ""


@dataclass
class CameraDevice:
    path: str
    name: str = "--"
    driver: str = "--"
    format_text: str = "--"
    resolution: str = "--"
    fps: str = "--"
    controls: list[V4L2Control] = field(default_factory=list)


CONTROL_LINE = re.compile(
    r"^\s*(?P<name>[A-Za-z0-9_]+)\s+0x[0-9a-fA-F]+\s+"
    r"\((?P<kind>[A-Za-z0-9_]+)\)\s*:\s*(?P<attributes>.*)$"
)
MENU_LINE = re.compile(r"^\s*(?P<value>-?\d+):\s*(?P<label>.+)$")


def v4l2_available() -> bool:
    return shutil.which("v4l2-ctl") is not None


def _run(*args: str) -> Optional[str]:
    try:
        result = subprocess.run(
            ["v4l2-ctl", *args],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout if result.returncode == 0 else None


def _field(text: str, label: str) -> str:
    match = re.search(rf"^\s*{re.escape(label)}\s*:\s*(.+)$", text, re.MULTILINE)
    return match.group(1).strip() if match else "--"


def _is_usb_video_capture(description: str) -> bool:
    """仅保留 UVC 的实际视频采集节点，排除 metadata/ISP/codec 节点。"""
    driver = _field(description, "Driver name")
    device_caps = description.partition("Device Caps")[2].partition("Media Driver Info")[0]
    return driver == "uvcvideo" and "Video Capture" in device_caps


def list_cameras() -> list[CameraDevice]:
    """返回可采集视频的真实 V4L2 节点，过滤编解码和 ISP 节点。"""
    if not v4l2_available():
        return []

    cameras: list[CameraDevice] = []
    for node in sorted(Path("/dev").glob("video*"), key=lambda item: item.name):
        description = _run("-d", str(node), "--all")
        if not description or not _is_usb_video_capture(description):
            continue
        cameras.append(read_camera(str(node), description))
    return cameras


def read_camera(path: str, description: Optional[str] = None) -> CameraDevice:
    description = description or _run("-d", path, "--all") or ""
    format_info = _run("-d", path, "--get-fmt-video") or ""
    parm_info = _run("-d", path, "--get-parm") or ""
    controls_info = _run("-d", path, "--list-ctrls-menus") or ""
    width_height = re.search(r"Width/Height\s*:\s*(\d+)\s*/\s*(\d+)", format_info)
    pixel_format = _field(format_info, "Pixel Format")
    fps_match = re.search(r"Frames per second:\s*([\d.]+)", parm_info)

    return CameraDevice(
        path=path,
        name=_field(description, "Card type"),
        driver=_field(description, "Driver name"),
        format_text=pixel_format,
        resolution=(f"{width_height.group(1)} × {width_height.group(2)}" if width_height else "--"),
        fps=fps_match.group(1) if fps_match else "--",
        controls=parse_controls(controls_info),
    )


def parse_controls(output: str) -> list[V4L2Control]:
    controls: list[V4L2Control] = []
    current: Optional[V4L2Control] = None
    for line in output.splitlines():
        match = CONTROL_LINE.match(line)
        if match:
            attributes = match.group("attributes")

            def integer(name: str) -> Optional[int]:
                value_match = re.search(rf"\b{name}=(-?\d+)", attributes)
                return int(value_match.group(1)) if value_match else None

            current = V4L2Control(
                name=match.group("name"),
                kind=match.group("kind").lower(),
                minimum=integer("min"),
                maximum=integer("max"),
                step=integer("step"),
                default=integer("default"),
                value=integer("value"),
                flags=attributes.split("flags=")[-1] if "flags=" in attributes else "",
            )
            controls.append(current)
            continue
        menu_match = MENU_LINE.match(line)
        if current and menu_match and current.kind in {"menu", "intmenu"}:
            current.menu_items[int(menu_match.group("value"))] = menu_match.group("label").strip()
    return controls


def set_control(device_path: str, name: str, value: int) -> tuple[bool, str]:
    if not v4l2_available():
        return False, "系统未安装 v4l2-ctl"
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device_path, "--set-ctrl", f"{name}={value}"],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    if result.returncode == 0:
        return True, ""
    return False, (result.stderr or result.stdout or "v4l2-ctl 设置失败").strip()
