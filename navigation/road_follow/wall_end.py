"""用前墙横缝判断车头是否已经到了侧墙尽头。

寻线仍走道路分割。这里只看原图上的墙缝：横缝下面，某一侧的墙
只剩靠近车头的一截、接不到横缝，就算这一侧到头了。
"""

from __future__ import annotations

import queue
from dataclasses import dataclass

import cv2
import numpy as np

from road_follow.control import VelocityCommand

CREEP_MPS = 0.08
CREEP_M = 0.15
ARM_FRAMES = 3


@dataclass(frozen=True)
class WallEnd:
    """arrived 为真时，side 是开口方向：left、right 或 both。"""

    arrived: bool
    side: str


@dataclass
class WallTurn:
    """跟线、前爬、转弯、离开路口。side 在开始前爬时锁定。"""

    phase: str = "follow"
    side: str = "none"
    creep_m: float = 0.0
    arm: int = 0
    clear: int = 0


def read_wall_end(bgr: np.ndarray) -> WallEnd:
    """前墙横缝以下，只有贴近车头的墙且接不上横缝，才算到头。"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 40, 120)
    height, width = gray.shape
    front_y = _front_seam_y(gray, edges)
    if front_y is None:
        return WallEnd(False, "none")

    opened = [name for name, _ in _sides(width) if _stub_at_bumper(edges, front_y, height, width, name)]
    if not opened:
        return WallEnd(False, "none")
    if len(opened) == 2:
        return WallEnd(True, "both")
    return WallEnd(True, opened[0])


def step_wall_turn(
    state: WallTurn,
    wall: WallEnd,
    command: VelocityCommand,
    dt_s: float,
    notes,
    send,
) -> tuple[WallTurn, VelocityCommand]:
    """到头后再往前爬 0.15 m，然后朝开口原地转 90°。转弯期间不发寻线速度。"""
    while True:
        try:
            note = notes.get_nowait()
        except queue.Empty:
            break
        if state.phase == "turning":
            reason = "turn_done" if note == "TURN_DONE" else "turn_fail"
            state.phase = "cooldown"
            state.clear = 0
            command = VelocityCommand(0.0, 0.0, reason)

    if state.phase == "follow":
        if wall.arrived and wall.side in ("left", "right", "both"):
            state.arm += 1
        else:
            state.arm = 0
        if state.arm < ARM_FRAMES:
            return state, command
        state.phase = "creep"
        state.side = "left" if wall.side == "both" else wall.side
        state.creep_m = 0.0
        state.arm = 0

    if state.phase == "creep":
        state.creep_m += CREEP_MPS * max(0.0, dt_s)
        if state.creep_m < CREEP_M:
            return state, VelocityCommand(CREEP_MPS, 0.0, "creep")
        if send("0.000 0.000") and send(f"turn {state.side}"):
            state.phase = "turning"
            return state, VelocityCommand(0.0, 0.0, f"turn_{state.side}")
        return state, VelocityCommand(0.0, 0.0, "turn_wait")

    if state.phase == "turning":
        return state, VelocityCommand(0.0, 0.0, "turning")

    if wall.arrived:
        state.clear = 0
    else:
        state.clear += 1
    if state.clear >= ARM_FRAMES:
        state.phase = "follow"
        state.side = "none"
        state.creep_m = 0.0
        state.clear = 0
    return state, command if state.phase == "follow" else VelocityCommand(0.0, 0.0, "cooldown")


def _front_seam_y(gray: np.ndarray, edges: np.ndarray) -> int | None:
    """横跨画面、下方是亮路面的那条横缝。"""
    height, width = gray.shape
    best_n = 0
    best_y: int | None = None
    for y in range(int(height * 0.18), int(height * 0.65)):
        xs = np.flatnonzero(edges[y] > 0)
        if xs.size < max(80, width // 16):
            continue
        if int(xs[-1] - xs[0]) < int(width * 0.65):
            continue
        below = gray[y + 8 : y + 28, width // 4 : 3 * width // 4]
        if below.size == 0 or float(below.mean()) < 140:
            continue
        if int(xs.size) > best_n:
            best_n = int(xs.size)
            best_y = y
    return best_y


def _sides(width: int):
    return (("left", (0, width // 3)), ("right", (2 * width // 3, width)))


def _stub_at_bumper(
    edges: np.ndarray,
    front_y: int,
    height: int,
    width: int,
    side: str,
) -> bool:
    """这一侧的墙从车头附近才开始，中间接不到前墙。"""
    x0, x1 = dict(_sides(width))[side]
    ys = [y for y in range(front_y + 12, height) if edges[y, x0:x1].any()]
    if not ys or ys[0] < int(height * 0.70):
        return False
    return int(edges[ys[0] :, x0:x1].sum() / 255) >= 40
