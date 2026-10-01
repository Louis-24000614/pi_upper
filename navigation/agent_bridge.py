"""现有 Navigation 事件到 RouteAgent 的薄适配，不拥有相机或 UART。"""

from __future__ import annotations

from agent.route_agent import AgentCommand, RouteAgent
from navigation.road_follow.backup import Backup, EdgeProgress
from vision.obstacle.blockage import ObstacleObservation


class AgentBridge:
    """复用沿边里程，并只在离散事件发生时推进 Agent。"""

    def __init__(self, agent: RouteAgent) -> None:
        self.agent = agent
        self.edge_progress = EdgeProgress()
        self._backup_phase = "idle"

    def start(self) -> AgentCommand:
        self.edge_progress.reset()
        self._backup_phase = "idle"
        return self.agent.start()

    def update_odometry(self, x_m: float, y_m: float, yaw_rad: float) -> float:
        return self.edge_progress.update(x_m, y_m, yaw_rad)

    def node_reached(self, node_id: str) -> AgentCommand:
        self.edge_progress.reset()
        self._backup_phase = "idle"
        return self.agent.node_reached(node_id)

    def obstacle_observed(
        self, observation: ObstacleObservation
    ) -> AgentCommand | None:
        if not observation.just_confirmed:
            return None
        edge_id = self.agent.state.current_edge
        if edge_id is None:
            return self.agent.obstacle_confirmed("", self.edge_progress.s_m)
        return self.agent.obstacle_confirmed(edge_id, self.edge_progress.s_m)

    def backup_updated(self, backup: Backup) -> AgentCommand | None:
        """只把 done/fault 的首次状态转换成 Agent 事件。"""
        if backup.phase == self._backup_phase:
            return None
        self._backup_phase = backup.phase
        state = self.agent.state
        if backup.phase == "done":
            edge_id = state.current_edge
            node_id = state.from_node
            if edge_id is None or node_id is None:
                return self.agent.backup_done("", "")
            self.edge_progress.reset()
            return self.agent.backup_done(edge_id, node_id)
        if backup.phase == "fault":
            edge_id = state.current_edge or ""
            return self.agent.backup_failed(edge_id, "navigation backup fault")
        return None

