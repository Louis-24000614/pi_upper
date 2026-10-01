# navigation/

导航相关 Python 原型（全局拓扑搜路等）。设计见 `docs/nav.md`。

| 子目录 | 说明 |
| --- | --- |
| `road_follow/` | 实车道路分割循迹、出发区动作、拓扑交接和录像日志；保留巡检点与普通路口原有视觉状态机，只取消拓扑任务的 UID 输入；RFID 另保留独立测试 |
| `topo_proto/` | 加载 `config/nav_topology.yaml` + Dijkstra + 模拟封边；模块文档见 `docs/reference/navigation/topo_proto.md` |
| `road_follow/` | 道路分割循线、路口/RFID 转向和遇障倒车；模块文档见 `docs/reference/navigation/road-follow-actions.md` |

只启动视觉循迹：

```bash
PYTHONPATH=.:navigation:vision python3 -m road_follow \
  --drive \
  --uart-bin build-turn/uart/uart_vel
```

```bash
PYTHONPATH=navigation python3 -m topo_proto --start 0_0 --goal 5_2 --block 2_3__2_4
```
