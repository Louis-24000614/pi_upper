# Agent 上层决策

`agent/` 与 `navigation/` 并列：Agent 决定下一步应该走哪条道路，Navigation 负责让车辆安全地完成这条道路。

当前先复制并独立保存 Demo 中已验证的中国邮递员规划器。`demo/` 不修改，继续承担仿真和 GUI；后续实车功能只在 `agent/` 中扩展。

```text
agent/
  coverage/
    planner.py      中国邮递员与封边后的剩余道路规划
  state.py          当前节点、执行边、已覆盖边和封闭边
  route_agent.py    边级事件状态机与重新规划
  tests/
    test_planner.py       Agent 规划器及与 Demo 的一致性测试
    test_route_agent.py   无硬件的边级状态机测试
    test_agent_bridge.py  Navigation 事件适配测试
```

Agent 不读取摄像头、不发送 UART，也不直接控制电机。障碍确认、节点到达和视觉倒车的 `BACKUP_DONE/FAIL` 由 Navigation 以事件形式上报。

`navigation/agent_bridge.py` 复用现有 `EdgeProgress`，把障碍确认、节点到达和倒车终态转换为 Agent 事件；它不重新实现相机或 UART。

运行测试：

```bash
PYTHONPATH=.:navigation:vision python3 -m unittest \
  agent.tests.test_planner \
  agent.tests.test_route_agent \
  agent.tests.test_agent_bridge
```

**Related:** [中国邮递员算法](../docs/reference/navigation/chinese-postman.md) · [实车接入计划](../docs/reference/navigation/postman-integration-plan.md)
