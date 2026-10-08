"""本次任务内的涵洞覆盖层；不改静态拓扑 YAML 和规划器。"""
from dataclasses import asdict
import json
from pathlib import Path
from copy import deepcopy
import threading
import xml.etree.ElementTree as ET


class CulvertRecords:
    def __init__(self, graph, prefix=None):
        self.graph = graph
        self.prefix = Path(prefix) if prefix is not None else None
        self.entries = {}
        self.lock = threading.RLock()
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

    def _save(self):
        if self.prefix is None:
            return
        payload = {"coordinate_unit":"m", "memory_scope":"current_task",
                   "culverts":self.entries,
                   "nodes":{key:asdict(node) for key,node in self.graph.nodes.items()},
                   "edges":{key:asdict(edge) for key,edge in self.graph.edges.items()}}
        path = Path(str(self.prefix) + ".culverts.json")
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)
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
            ET.SubElement(root,"line",x1=str(x1),y1=str(y1),x2=str(x2),y2=str(y2),stroke="#a6adb4",**{"stroke-width":"3"})
            entry = self.entries.get(edge.id)
            if entry:
                ratio = entry["canonical_offset_m"] / edge.length_m
                x,y = x1+(x2-x1)*ratio, y1+(y2-y1)*ratio
                color = {"discovered":"#d99b17","done":"#16834a","partial":"#8753a5","failed":"#c43838"}[entry["status"]]
                ET.SubElement(root,"circle",cx=str(x),cy=str(y),r="9",fill=color)
                label = entry["status"]
                if "inspection" in entry:
                    label += " " + " / ".join(s["direction"]+":"+(s.get("identity") or "unconfirmed") for s in entry["inspection"]["sides"])
                ET.SubElement(root,"text",x=str(x+12),y=str(y-10),**{"font-size":"12"}).text=label
        for key,node in nodes.items():
            x,y = point(node)
            ET.SubElement(root,"circle",cx=str(x),cy=str(y),r="4",fill="#253b50")
            ET.SubElement(root,"text",x=str(x+7),y=str(y+16),**{"font-size":"12"}).text=key
        ET.SubElement(root,"text",x="20",y="620",**{"font-size":"14"}).text="Culvert: yellow=discovered green=done purple=partial red=failed (current task only)"
        svg = Path(str(self.prefix)+".culverts.svg")
        temp_svg = svg.with_suffix(".svg.tmp")
        ET.ElementTree(root).write(temp_svg, encoding="utf-8", xml_declaration=True)
        temp_svg.replace(svg)
