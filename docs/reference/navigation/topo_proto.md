# topo_proto — 先验拓扑 Dijkstra 原型

`navigation/topo_proto/`：加载 [`config/nav_topology.yaml`](../../../config/nav_topology.yaml)，在边上做 Dijkstra 最短路，并支持模拟 `blocked` 封边后重搜。无实车、无视觉。

## 运行

在仓库根目录：

```bash
PYTHONPATH=navigation python3 -m topo_proto.tests.test_graph
PYTHONPATH=navigation python3 -m topo_proto --start 0_0 --goal 5_2
PYTHONPATH=navigation python3 -m topo_proto --start 0_0 --goal 2_4 --block 2_3__2_4
PYTHONPATH=navigation python3 -m topo_proto --list-tunnels
```

依赖：PyYAML。

## 模块

| 文件 | 职责 |
| --- | --- |
| `graph.py` | `load_topology`、`TopologyGraph`、`shortest_path`（Dijkstra） |
| `cli.py` | 命令行对比「封边前 / 封边后」路径 |

## 相关文档

- 分层导航设计：`docs/nav.md`
- 拓扑数据：`config/nav_topology.yaml`
