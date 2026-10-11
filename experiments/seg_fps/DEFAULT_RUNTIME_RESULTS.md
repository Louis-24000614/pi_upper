# 采纳项成为默认行为（2026-10-03）

正式入口不再需要 `--fps-opt --correct-nms --npu-contexts 3 --blas-threads 1
--heap-trim-interval 60` 或外部调频启动器。默认组合是：

- RKNN 三核各一个私有 context，使用原有 Ordered 有序流水线。
- IPM/同帧 BEV 复用、私有分割工作缓冲、正确 NMS。
- OpenCV 8 线程、BLAS 1 线程，Linux 每 60 秒 GC/空闲堆页回收。
- 请求录像时自动使用有界异步编码；未请求录像时不创建编码器。
- RK3588 入口自动调用保护进程固定合法最高 CPU/DDR/NPU，结束后恢复。

latest、零拷贝/C IO 和按需普通中心线仍未采纳；保留为实验选项。
ONNX 与单核覆盖继续单 context，200ms 门、有限动作互斥和转弯来源隔离保持。
频率实现移入正式道路模块，实验启动器仅转发，不维护第二份实现。

## 验证

板端 108 项测试通过。默认入口不传优化参数，使用真实模型和导航，测试夹具
仅用 AVI 替代 V4L2 相机并观察入口配置；没有启动车辆。短录像验证仅传
模型/配置/帧数/录像路径，自动创建异步编码器，正常保存及释放资源。

完整 300 秒默认入口回放：10001 次有效导航更新，33.337 FPS，过期/乱序/异常
均为 0；实际 BLAS 线程为 1，三 context 的缓冲和 BEV 优化已启用。
最近 4096 次有效更新的平均帧龄 83.39ms、P95 105.54ms，时间起点是相机
`read` 完成，终点是软件状态计算完成，不能与包含解码/驱动发布延迟的实验混算。
60 秒回收任务实际执行，结束后采集、模型、编码和回收线程均正常释放。

正常/异常退出、非零退出 7、TERM 143、INT 130 均确认恢复 governor/min/max。
九段完整 4006 帧分割/导航等价结果沿用此前已验证算法，本轮不重复声称新九段实测。

代码默认改动为 67beb7c；5 分钟测试时短暂存在的 c31f77a 仅改变认证，不改变
分割/导航默认组合。用户要求后以 00a1052 撤回该 SSH/stdin 认证适配，最终
108 项测试和调频恢复专项再次通过。系统 sudo 权限没有替用户修改。

原目录 `/home/orangepi/pi_upper/main` 已 fast-forward 纳入这些改动，原有未跟踪
文件及 `config/nav_camera.yaml` 的 SHA256 均保持。原目录缺少默认道路模型，
补齐此前验证一直使用的冻结 `road_yolo11n_seg.rknn`，SHA256 为
`e65064aafa5c75b373167cc57ba5489df3d4ad9e01ba8d34cbf7c74997feed51`。

## 普通启动与用户自行设置权限

在既有环境下普通启动，无须任何优化参数：

```bash
cd ~/pi_upper
PYTHONPATH=.:navigation:vision /home/orangepi/miniconda3/envs/pi_upper/bin/python -m road_follow
```

OS 权限由用户自行配置。如果希望该用户已有 sudo 权限免密码，可执行：

```bash
sudo visudo -f /etc/sudoers.d/orangepi-nopasswd
```

写入 `Defaults:orangepi !authenticate`。这会取消 orangepi 所有已有 sudo 权限的
密码认证，而不仅是调频。配置由用户稍后修改，本次没有写入 `/etc/sudoers`。
