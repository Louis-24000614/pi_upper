"""回放复用正式循迹/路口算法，独立保存时序状态，输出软件控制意图。"""
from ipm_proto.junction import JunctionTracker, classify_junction
from ipm_proto.temporal import temporal_from_mapping
from road_follow.pipeline import BevProjector, make_ipm, command_from_mask_with_diagnostics
import cv2


class Navigation:
    def __init__(self, cfg, reuse_bev=False, lazy_raw=False):
        self.cfg = cfg
        self.smoother = temporal_from_mapping(cfg)
        self.tracker = JunctionTracker()
        self.projector = BevProjector() if reuse_bev else None
        self.lazy_raw = lazy_raw

    def process(self, mask):
        projection = self.projector.project(mask, self.cfg) if self.projector else None
        command, diag = command_from_mask_with_diagnostics(
            mask, self.cfg, self.smoother, projection=projection, lazy_raw=self.lazy_raw)
        if projection is None:
            ipm = make_ipm(self.cfg, mask.shape)
            bev = ipm.warp_to_bev(mask, flags=cv2.INTER_NEAREST)
        else:
            ipm, bev = projection
        raw = self.cfg.get("junction_turn") or {}
        junction = classify_junction(
            bev, ipm.bev,
            forward_band_min_y_m=float(raw.get("road_end_band_min_y_m", 0.34)),
            forward_band_max_y_m=float(raw.get("road_end_band_max_y_m", 0.48)))
        return command, diag, self.tracker.update(junction), junction
