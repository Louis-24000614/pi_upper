"""按采集时刻里程和地面投影区分当前边与前方相邻边的障碍。"""
from copy import deepcopy
import math

import numpy as np


OBSTACLE_DEFAULTS = {
    "edge_end_margin_m": .05,
    "distance_bias_m": 0.0,
    "max_distance_m": .80,
    "max_lateral_m": .18,
    "min_score": .45,
    "min_bottom_ratio": .35,
    "confirm_frames": 3,
}


def locate_ahead(graph, edge_key, progress_m, distance_m):
    """只沿唯一的直行延续边定位；坐标用于方向，距离使用拓扑边长。"""
    edge_id, source, destination = edge_key
    edge = graph.edges[edge_id]
    offset = progress_m + distance_m
    visited = set()
    while offset > edge.length_m:
        if edge.id in visited:
            return None
        visited.add(edge.id)
        offset -= edge.length_m
        a, b = graph.nodes[source], graph.nodes[destination]
        hx, hy = b.x-a.x, b.y-a.y
        candidates = []
        for other in graph.edges.values():
            if other.id in visited:
                continue
            next_node = (other.v if other.u == destination else
                         other.u if other.bidirectional and other.v == destination else None)
            if next_node is None:
                continue
            c = graph.nodes[next_node]
            dx, dy = c.x-b.x, c.y-b.y
            norm = math.hypot(hx, hy)*math.hypot(dx, dy)
            if norm > 0 and (hx*dx+hy*dy)/norm >= math.cos(math.radians(20)):
                candidates.append((other, next_node))
        if len(candidates) != 1:
            return None
        source, (edge, destination) = destination, candidates[0]
    if offset < 0:
        return None
    return {"edge_id": edge.id, "from_node": source, "to_node": destination,
            "canonical_offset_m": offset if source == edge.u else edge.length_m-offset}


def filter_current_edge(detections, *, graph, edge_key, progress_m, ipm,
                        image_shape, settings, lane_x_m=0.0):
    """返回可参与当前边确认的框，以及可供日志和地图使用的逐框判定。"""
    accepted, details = [], []
    edge = graph.edges[edge_key[0]]
    progress_valid = progress_m is not None and math.isfinite(progress_m)
    remaining = max(0.0, edge.length_m-progress_m) if progress_valid else None
    height, width = image_shape[:2]
    for item in detections:
        detail = {"class_id": item.class_id, "label": item.label, "score": float(item.score),
                  "bbox": [item.x1, item.y1, item.x2, item.y2],
                  "remaining_m": remaining, "distance_m": None, "target": None}
        values = (*detail["bbox"], item.score)
        if not all(math.isfinite(v) for v in values):
            detail["bbox"] = [v if math.isfinite(v) else None for v in detail["bbox"]]
            detail["score"] = float(item.score) if math.isfinite(item.score) else None
            reason = "invalid_box"
        elif progress_m is None or not math.isfinite(progress_m):
            reason = "unaligned_odom"
        elif not (0 <= item.x1 < item.x2 < width and 0 <= item.y1 < item.y2 < height-1):
            reason = "clipped_ground_contact"
        else:
            feet = np.asarray(ipm.image_points_to_ground([
                ((item.x1+item.x2)/2, item.y2)]), dtype=float)
            if feet.shape != (1, 2) or not np.isfinite(feet).all():
                reason = "invalid_ground_projection"
            else:
                x, raw_distance = map(float, feet[0])
                distance = raw_distance + settings["distance_bias_m"]
                detail.update(distance_m=distance, raw_distance_m=raw_distance, lateral_m=x)
                if distance <= 0 or distance > settings["max_distance_m"]:
                    reason = "outside_distance_window"
                elif abs(x-(lane_x_m or 0.0)) > settings["max_lateral_m"]:
                    reason = "outside_current_lane"
                else:
                    detail["target"] = locate_ahead(graph, edge_key, progress_m, distance)
                    if distance > remaining:
                        reason = "beyond_current_edge"
                    elif distance > remaining-settings["edge_end_margin_m"]:
                        reason = "near_edge_boundary"
                    else:
                        reason = "current_edge"
                        accepted.append(item)
        detail["reason"] = reason
        details.append(deepcopy(detail))
    return accepted, details
