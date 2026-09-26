# 障碍检测与硬堵塞判定

`ObstacleDetector.detect(frame)` 负责把前视图像转换成检测框；`HardBlockageJudge` 负责判断这些框是否稳定出现在当前道路的行驶区域。

```python
from vision.obstacle.blockage import HardBlockageJudge, hard_block_config_from_mapping
from vision.obstacle.detect import ObstacleDetector

detector = ObstacleDetector(config_path, root=repo_root)
judge = HardBlockageJudge(hard_block_config_from_mapping(detector.config))

detections, elapsed_ms = detector.detect(frame)
observation = judge.update(detections, frame.shape)
if observation.just_confirmed:
    publish_obstacle_confirmed(current_edge)
```

判定依次检查置信度、检测框面积、框底部距离、中央行驶区域重叠和跨帧 IoU。连续达到 `confirm_frames` 后，`hard_blocked` 会保持为真，直到车辆进入另一条道路并调用 `judge.reset()`。

本项目的比赛策略是：当前道路一旦确认障碍，整条边封闭，只允许车辆沿原路倒回入口节点。该模块只产生确认事件，不负责封边、倒车或重规划。

参数位于 `config/obstacle.yaml` 的 `hard_block` 段。默认比例是尚待实车回放标定的安全初值，不代表已经完成赛场验收。

运行纯逻辑测试：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 -m unittest vision.obstacle.tests.test_blockage
```

