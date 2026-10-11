"""本次任务内的涵洞覆盖层；不改静态拓扑 YAML 和规划器。"""
from dataclasses import asdict
import json
from pathlib import Path
from copy import deepcopy
import threading
import time
import xml.etree.ElementTree as ET
import uuid


def identity_label(value):
    if value and value.startswith("suspect_") and value[8:].isdigit():
        return "嫌疑人"+str(int(value[8:]))+"号"
    if value and value.startswith("knife_") and value[6:].isdigit():
        return "刀具"+str(int(value[6:]))+"号"
    return "未确认"


class CulvertRecords:
    def __init__(self, graph, prefix=None):
        self.graph = graph
        self.prefix = Path(prefix) if prefix is not None else None
        self.entries = {}
        self.obstacles = {}
        self.rfids = []
        self.vehicle = None
        self.session_id = self.prefix.name if self.prefix else uuid.uuid4().hex
        self.lock = threading.RLock()
        self.last_save_s = float("-inf")
        if self.prefix:
            self.prefix.parent.mkdir(parents=True, exist_ok=True)
            for suffix in (".culverts.json", ".culverts.svg"):
                path = Path(str(self.prefix) + suffix)
                if path.exists():
                    raise FileExistsError(f"不能覆盖旧任务地图: {path}")
        self._save()

    def done(self, edge_id):
        return self.entries.get(edge_id, {}).get("status") == "done"

    def handled(self, edge_id):
        return self.entries.get(edge_id, {}).get("status") in ("done", "partial")

    def snapshot(self):
        with self.lock:
            return deepcopy(self.entries)

    def topology_snapshot(self):
        with self.lock:
            return deepcopy(self._payload())

    def set_vehicle(self, edge_key, progress_m, phase):
        with self.lock:
            edge = self.graph.edges.get(edge_key[0])
            if edge:
                offset = min(edge.length_m, max(0, progress_m))
                self.vehicle = {"edge_id": edge.id, "from_node": edge_key[1], "to_node": edge_key[2],
                                "progress_m": progress_m, "phase": phase,
                                "canonical_offset_m": offset if edge_key[1] == edge.u else edge.length_m-offset}

    def observe_obstacles(self, details, confirmed_bbox=None):
        changed = False
        significant = False
        with self.lock:
            for detail in details:
                target = detail.get("target")
                if not target:
                    continue
                key = target["edge_id"]+":"+str(detail["class_id"])
                confirmed = confirmed_bbox is not None and list(confirmed_bbox) == detail["bbox"] and detail["reason"] == "current_edge"
                old = self.obstacles.get(key, {})
                record = {**target, "class_id": detail["class_id"], "label": detail["label"],
                          "score": detail["score"], "status": "confirmed" if confirmed else old.get("status", "observed"),
                          "projection": "estimated_ground_contact"}
                if old != record:
                    significant = significant or not old or record["status"] != old.get("status")
                    self.obstacles[key] = record
                    changed = True
            if changed and (significant or time.monotonic()-self.last_save_s >= 1):
                self._save()

    def mark_rfid(self, card_number, generation, received_s):
        with self.lock:
            if not 1 <= card_number <= 12:
                return
            self.rfids.append({"card_number": card_number, "generation": generation,
                               "received_s": received_s, "location": deepcopy(self.vehicle),
                               "location_source": "topology_odometry_estimate"})
            self._save()

    def _payload(self):
        return {"coordinate_unit": "m", "memory_scope": "current_task", "session_id": self.session_id,
                "culverts": self.entries, "obstacles": self.obstacles, "rfids": self.rfids,
                "vehicle": self.vehicle, "blocked_edges": sorted(self.graph.blocked.copy()),
                "nodes": {key: asdict(node) for key, node in self.graph.nodes.items()},
                "edges": {key: asdict(edge) for key, edge in self.graph.edges.items()}}

    def svg(self):
        with self.lock:
            return self._svg()

    def discover(self, target):
        with self.lock:
            self.entries[target.edge_id] = {**asdict(target), "status":"discovered", "stop_error_m":None}
            self._save()

    def fail(self, edge_id, reason):
        with self.lock:
            if edge_id in self.entries:
                self.entries[edge_id]["failure_reason"] = reason
                if not self.handled(edge_id):
                    self.entries[edge_id]["status"] = "failed"
                self._save()

    def complete(self, edge_id, error):
        with self.lock:
            self.entries[edge_id].update(status="done", stop_error_m=error)
            self._save()

    def complete_inspection(self, edge_id, error, result):
        if result.get("status") not in ("done", "partial") or len(result.get("sides", [])) != 2:
            raise ValueError("两侧检查结果不完整")
        with self.lock:
            self.entries[edge_id].update(status=result["status"], stop_error_m=error,
                                         inspection=deepcopy(result))
            self._save()

    def flush(self):
        with self.lock:
            self._save()

    def _save(self):
        if self.prefix is None:
            return
        payload = self._payload()
        path = Path(str(self.prefix) + ".culverts.json")
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)
        svg = Path(str(self.prefix)+".culverts.svg")
        temp_svg = svg.with_suffix(".svg.tmp")
        temp_svg.write_bytes(self._svg())
        temp_svg.replace(svg)
        self.last_save_s = time.monotonic()

    def _svg(self):
        nodes = self.graph.nodes
        xs, ys = [n.x for n in nodes.values()], [n.y for n in nodes.values()]
        sx, sy = max(xs)-min(xs), max(ys)-min(ys)
        scale = min(600/max(sx,1), 500/max(sy,1))
        def point(node):
            return 50+(node.x-min(xs))*scale, 50+(max(ys)-node.y)*scale
        root = ET.Element("svg", xmlns="http://www.w3.org/2000/svg", width="720", height="640", viewBox="0 0 720 640")
        ET.SubElement(root,"rect",width="720",height="640",fill="white")
        for edge in self.graph.edges.values():
            x1,y1 = point(nodes[edge.u]); x2,y2 = point(nodes[edge.v])
            ET.SubElement(root,"line",x1=str(x1),y1=str(y1),x2=str(x2),y2=str(y2),stroke="#c43838" if edge.id in self.graph.blocked else "#a6adb4",**{"stroke-width":"3"})
            entry = self.entries.get(edge.id)
            if entry:
                ratio = entry["canonical_offset_m"] / edge.length_m
                x,y = x1+(x2-x1)*ratio, y1+(y2-y1)*ratio
                color = {"discovered":"#d99b17","done":"#16834a","partial":"#8753a5","failed":"#c43838"}[entry["status"]]
                ET.SubElement(root,"circle",cx=str(x),cy=str(y),r="9",fill=color)
                label = {"discovered":"涵洞已发现", "done":"识别成功", "partial":"部分未确认", "failed":"任务故障"}[entry["status"]]
                if "inspection" in entry:
                    label += " " + " / ".join(s["direction"]+":"+identity_label(s.get("identity")) for s in entry["inspection"]["sides"])
                ET.SubElement(root,"text",x=str(x+12),y=str(y-10),**{"font-size":"12"}).text=label
        def marker_position(record):
            edge = self.graph.edges[record["edge_id"]]
            a, b = point(nodes[edge.u]), point(nodes[edge.v])
            ratio = max(0, min(1, record["canonical_offset_m"]/edge.length_m))
            return a[0]+(b[0]-a[0])*ratio, a[1]+(b[1]-a[1])*ratio
        for record in self.obstacles.values():
            x, y = marker_position(record)
            color = "#c43838" if record["status"] == "confirmed" else "#e59225"
            ET.SubElement(root, "rect", x=str(x-6), y=str(y-6), width="12", height="12", fill=color)
            ET.SubElement(root, "text", x=str(x+9), y=str(y+8), **{"font-size":"12"}).text = "障碍已确认" if record["status"] == "confirmed" else "看见障碍"
        for record in self.rfids:
            if not record["location"]:
                continue
            x, y = marker_position(record["location"])
            ET.SubElement(root, "circle", cx=str(x), cy=str(y), r="7", fill="none", stroke="#1474b8", **{"stroke-width":"3"})
            ET.SubElement(root, "text", x=str(x+9), y=str(y+22), **{"font-size":"12"}).text = "标签 "+str(record["card_number"])
        if self.vehicle:
            x, y = marker_position(self.vehicle)
            ET.SubElement(root, "circle", cx=str(x), cy=str(y), r="5", fill="#1474b8")
        for key,node in nodes.items():
            x,y = point(node)
            ET.SubElement(root,"circle",cx=str(x),cy=str(y),r="4",fill="#253b50")
            ET.SubElement(root,"text",x=str(x+7),y=str(y+16),**{"font-size":"12"}).text=key
        ET.SubElement(root,"text",x="20",y="600",**{"font-size":"13"}).text="涵洞：黄=发现  绿=成功  紫=部分未确认  红=故障"
        ET.SubElement(root,"text",x="20",y="625",**{"font-size":"13"}).text="方框=障碍（橙=看见，红=确认） 蓝圈=标签；仅本次任务，位置按拓扑里程估计"
        return ET.tostring(root, encoding="utf-8", xml_declaration=True)
