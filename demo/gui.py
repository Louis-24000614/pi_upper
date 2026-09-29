"""Tkinter 可视化：点击道路中点预设障碍，逐步观察在线重规划。"""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import messagebox, ttk

from navigation.topo_proto.graph import TopologyGraph, load_topology
from .mission import PostmanMission


class DemoWindow:
    def __init__(self, root: tk.Tk, topology_path=None) -> None:
        self.root = root
        self.topology_path = topology_path
        self.graph: TopologyGraph = load_topology(topology_path)
        self.selected: set[str] = set()
        self.mission: PostmanMission | None = None
        self.autoplay = False

        root.title("道路覆盖与路障重规划演示")
        root.geometry("1080x780")
        panel = ttk.Frame(root, padding=10)
        panel.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(panel, width=690, height=745, bg="#f8fafc")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.bind("<Button-1>", self._select_edge)

        right = ttk.Frame(panel, width=350)
        right.pack(side="right", fill="y", padx=(12, 0))
        right.pack_propagate(False)
        ttk.Label(right, text="点击道路选择 3 个障碍位置", font=("Microsoft YaHei", 13, "bold")).pack(anchor="w")
        ttk.Label(right, text="红方块仅供观察；车辆驶入前并不知道。", wraplength=330).pack(anchor="w", pady=(5, 8))
        self.status = tk.StringVar(value="已选 0/3 条道路")
        ttk.Label(right, textvariable=self.status, wraplength=330).pack(anchor="w", pady=(0, 8))

        buttons = ttk.Frame(right)
        buttons.pack(fill="x", pady=(0, 8))
        for label, command in (("开始", self.start), ("单步", self.step),
                               ("自动", self.toggle_auto), ("重置", self.reset)):
            ttk.Button(buttons, text=label, command=command).pack(side="left", padx=2)

        ttk.Label(right, text="事件日志").pack(anchor="w")
        log_frame = ttk.Frame(right)
        log_frame.pack(fill="both", expand=True)
        self.log = tk.Text(log_frame, width=44, height=35, wrap="word", state="disabled")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        self.log.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self._draw()

    def _xy(self, x: float, y: float) -> tuple[float, float]:
        # YAML 以左下角为原点；画布 y 向下，故这里翻转。
        return 70 + x * 165, 710 - y * 150

    def _log(self, message: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", message + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        mission = self.mission
        next_edge = None
        if mission and mission.phase == "running" and mission.plan_index < len(mission.plan.edges):
            next_edge = mission.plan.edges[mission.plan_index]
        for edge in self.graph.edges.values():
            x1, y1 = self._xy(self.graph.nodes[edge.u].x, self.graph.nodes[edge.u].y)
            x2, y2 = self._xy(self.graph.nodes[edge.v].x, self.graph.nodes[edge.v].y)
            color, width = "#64748b", 5
            if mission and edge.id in mission.covered:
                color = "#93c5a5"
            if edge.id == next_edge:
                color, width = "#0f766e", 8
            if mission and edge.id in mission.discovered:
                color, width = "#ef4444", 5
            canvas.create_line(x1, y1, x2, y2, fill=color, width=width)
            if edge.id in self.selected:
                # 障碍固定在道路中点，视觉预设不进入车辆的已知地图。
                mx, my = (x1 + x2) / 2, (y1 + y2) / 2
                canvas.create_rectangle(mx - 8, my - 8, mx + 8, my + 8,
                                        fill="#dc2626", outline="white", width=2)

        for node in self.graph.nodes.values():
            x, y = self._xy(node.x, node.y)
            fill = "#fbbf24" if node.role == "patrol_slot" else "white"
            if node.id == "0_0":
                fill = "#60a5fa"
            canvas.create_oval(x - 10, y - 10, x + 10, y + 10,
                               fill=fill, outline="#334155", width=2)
            canvas.create_text(x, y - 19, text=node.id, fill="#0f172a",
                               font=("Consolas", 10, "bold"))
        if mission:
            x, y = self._xy(*mission.robot_position())
            canvas.create_oval(x - 12, y - 12, x + 12, y + 12,
                               fill="#1d4ed8", outline="white", width=3)
        canvas.create_text(14, 16, anchor="nw", text="蓝点：车辆    绿色：已走    深绿：下一段    红方块：预设路障",
                           fill="#334155", font=("Microsoft YaHei", 10))

    def _select_edge(self, event) -> None:
        if self.mission is not None:
            return
        best_id, best_dist = None, float("inf")
        for edge in self.graph.edges.values():
            a = self.graph.nodes[edge.u]
            b = self.graph.nodes[edge.v]
            x1, y1 = self._xy(a.x, a.y)
            x2, y2 = self._xy(b.x, b.y)
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, ((event.x - x1) * dx + (event.y - y1) * dy) / (dx * dx + dy * dy)))
            # 只允许点道路中段，避免在交叉口误选相邻路段。
            if not 0.2 <= t <= 0.8:
                continue
            distance = math.hypot(event.x - (x1 + t * dx), event.y - (y1 + t * dy))
            if distance < best_dist:
                best_id, best_dist = edge.id, distance
        if best_id is None or best_dist > 16:
            return
        if best_id in self.selected:
            self.selected.remove(best_id)
        elif len(self.selected) < 3:
            self.selected.add(best_id)
        else:
            messagebox.showinfo("障碍数量", "只能选择 3 条障碍道路；再次点击红方块可取消。")
        self.status.set(f"已选 {len(self.selected)}/3 条道路：{', '.join(sorted(self.selected))}")
        self._draw()

    def start(self) -> None:
        if self.mission is not None:
            return
        if len(self.selected) != 3:
            messagebox.showinfo("障碍数量", "请先选满 3 条道路。")
            return
        self.mission = PostmanMission(self.graph, self.selected)
        self._log(f"初始规划：{len(self.mission.plan.edges)} 段行程，"
                  f"覆盖 {len(self.graph.edges)} 条道路；障碍对车辆仍未知。")
        self.status.set("运行中：已发现 0/3 个障碍")
        self._draw()

    def step(self) -> None:
        if self.mission is None:
            self.start()
        if self.mission is None:
            return
        if self.mission.phase in {"done", "stalled"}:
            return
        event = self.mission.step()
        self._log(event.message)
        if event.kind == "discover" and self.mission.failure_reason is None:
            self._log(f"  新路线：{len(self.mission.plan.edges)} 段；"
                      f"剩余可达必经道路 {len(set(self.graph.edges) - self.mission.covered - self.mission.discovered - set(self.mission.plan.unreachable_edges))} 条")
        self.status.set(
            f"已走道路 {len(self.mission.covered)}/{len(self.graph.edges) - len(self.selected)}；"
            f"已发现障碍 {len(self.mission.discovered)}/3；"
            f"巡逻点 {len(self.mission.visited_patrol)}/12；当前位置 {self.mission.current}"
        )
        self._draw()

    def toggle_auto(self) -> None:
        self.autoplay = not self.autoplay
        if self.autoplay:
            self._auto_tick()

    def _auto_tick(self) -> None:
        if not self.autoplay:
            return
        self.step()
        if self.mission and self.mission.phase in {"done", "stalled"}:
            self.autoplay = False
        else:
            self.root.after(350, self._auto_tick)

    def reset(self) -> None:
        self.autoplay = False
        self.graph = load_topology(self.topology_path)
        self.mission = None
        self.selected.clear()
        self.status.set("已选 0/3 条道路")
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self._draw()


def launch(topology_path=None) -> None:
    root = tk.Tk()
    DemoWindow(root, topology_path)
    root.mainloop()
