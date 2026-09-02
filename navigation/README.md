# navigation/

导航相关 Python 原型（全局拓扑搜路等）。设计见 `docs/nav.md`。

| 子目录 | 说明 |
| --- | --- |
| `topo_proto/` | 加载 `config/nav_topology.yaml` + Dijkstra + 模拟封边；模块文档见 `docs/reference/navigation/topo_proto.md` |

```bash
PYTHONPATH=navigation python3 -m topo_proto --start 0_0 --goal 5_2 --block 2_3__2_4
```
