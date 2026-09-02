"""IPM / 中心线原型命令行。

示例：
  PYTHONPATH=vision python3 -m ipm_proto.cli synth --out /tmp/ipm_demo
  PYTHONPATH=vision python3 -m ipm_proto.cli run --image a.jpg --mask a_mask.png --config config_example.yaml --out /tmp/out
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from .centerline import centerline_lateral_error, extract_centerline
from .ipm import BevConfig, CameraExtrinsics, Ipm
from .synth import measure_bev_road_width_m, synthesize_straight_road


def _load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _bev_from_cfg(cfg: dict) -> BevConfig:
    b = cfg.get("bev", {})
    return BevConfig(
        y_min=float(b.get("y_min", 0.3)),
        y_max=float(b.get("y_max", 1.5)),
        x_min=float(b.get("x_min", -0.5)),
        x_max=float(b.get("x_max", 0.5)),
        m_per_px=float(b.get("m_per_px", 0.01)),
    )


def _ipm_from_cfg(cfg: dict, image_shape: tuple[int, ...]) -> Ipm:
    bev = _bev_from_cfg(cfg)
    if "image_points" in cfg and "ground_points" in cfg:
        return Ipm.from_image_ground_points(
            cfg["image_points"], cfg["ground_points"], bev
        )
    cam_cfg = cfg.get("camera", {})
    h_img, w_img = image_shape[:2]
    fx = float(cam_cfg.get("fx", 0.5 * w_img / np.tan(np.deg2rad(63.3) / 2.0)))
    fy = float(cam_cfg.get("fy", fx))
    cam = CameraExtrinsics(
        height_m=float(cam_cfg.get("height_m", 0.25)),
        pitch_rad=float(np.deg2rad(cam_cfg.get("pitch_deg", 28.0))),
        fx=fx,
        fy=fy,
        cx=float(cam_cfg.get("cx", w_img / 2.0)),
        cy=float(cam_cfg.get("cy", h_img / 2.0)),
    )
    return Ipm.from_extrinsics(cam, bev)


def _draw_centerline_on_bev(bev_bgr: np.ndarray, points, bev: BevConfig) -> np.ndarray:
    out = bev_bgr.copy()
    poly = []
    for x_m, y_m in points:
        u, v = bev.ground_to_bev_px(x_m, y_m)
        poly.append((int(round(u)), int(round(v))))
    if len(poly) >= 2:
        cv2.polylines(out, [np.array(poly, dtype=np.int32)], False, (0, 0, 255), 2)
    for p in poly:
        cv2.circle(out, p, 2, (0, 255, 255), -1)
    # 车体中心线
    u0, _ = bev.ground_to_bev_px(0.0, bev.y_min)
    cv2.line(out, (int(u0), 0), (int(u0), bev.height_px - 1), (255, 128, 0), 1)
    return out


def cmd_synth(args: argparse.Namespace) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    scene = synthesize_straight_road()
    bev_mask = scene.ipm.warp_to_bev(scene.mask, flags=cv2.INTER_NEAREST)
    points = extract_centerline(bev_mask, scene.ipm.bev)
    bev_vis = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    bev_vis = _draw_centerline_on_bev(bev_vis, points, scene.ipm.bev)

    cv2.imwrite(str(out / "front_bgr.png"), scene.image_bgr)
    cv2.imwrite(str(out / "front_mask.png"), scene.mask)
    cv2.imwrite(str(out / "bev_mask.png"), bev_mask)
    cv2.imwrite(str(out / "bev_centerline.png"), bev_vis)

    width = measure_bev_road_width_m(bev_mask, scene.ipm.bev, at_y_m=1.0)
    mid = [p for p in points if 0.8 <= p[1] <= 1.3]
    err = centerline_lateral_error(mid if mid else points, near_y_max=1.3)
    print(f"wrote {out}")
    print(f"bev road width @Y=1.0m ≈ {width:.3f} m (expect {scene.road_width_m})")
    print(f"mid centerline |X| mean ≈ {err:.4f} m ({len(points)} points)")
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    cfg = _load_config(Path(args.config)) if args.config else {}
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        print(f"cannot read image: {args.image}", file=sys.stderr)
        return 1
    if args.mask:
        mask = cv2.imread(args.mask, cv2.IMREAD_GRAYSCALE)
        if mask is None:
            print(f"cannot read mask: {args.mask}", file=sys.stderr)
            return 1
    else:
        # 无模型时：用绿色通道启发式，仅便于调试
        mask = cv2.inRange(image, (0, 80, 0), (100, 255, 100))

    ipm = _ipm_from_cfg(cfg, image.shape)
    bev_mask = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
    bev_img = ipm.warp_to_bev(image, flags=cv2.INTER_LINEAR)
    points = extract_centerline(bev_mask, ipm.bev)
    bev_vis = _draw_centerline_on_bev(bev_img, points, ipm.bev)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "bev_mask.png"), bev_mask)
    cv2.imwrite(str(out / "bev_centerline.png"), bev_vis)
    print(f"wrote {out}, centerline points={len(points)}")
    if points:
        print(f"near |X| mean ≈ {centerline_lateral_error(points):.4f} m")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="IPM / centerline prototype")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_synth = sub.add_parser("synth", help="合成直道并跑通 IPM+中心线")
    p_synth.add_argument("--out", default="/tmp/ipm_proto_synth")
    p_synth.set_defaults(func=cmd_synth)

    p_run = sub.add_parser("run", help="对真实图像/mask 做 IPM+中心线")
    p_run.add_argument("--image", required=True)
    p_run.add_argument("--mask", default=None, help="道路 mask；省略则试绿色阈值")
    p_run.add_argument("--config", default=None)
    p_run.add_argument("--out", default="/tmp/ipm_proto_run")
    p_run.set_defaults(func=cmd_run)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
