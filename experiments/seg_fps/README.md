# 道路帧率 A/B 实验

所有入口只读冻结录像，输出视觉控制意图，不打开相机或 UART。模型输入保持
640×640；标定、阈值、速度、EMA、连续帧判据和协议不做调整。

板端独立分支：`codex/seg-fps-ab`，工作目录 `/home/orangepi/pi_upper_seg_fps`。
冻结素材 `/home/orangepi/seg_fps_data`，原始结果 `/home/orangepi/seg_fps_results`。
本次环境使用 `/home/orangepi/vision_compare_pi_upper/bin/python`。

```bash
cd /home/orangepi/pi_upper_seg_fps
export PYTHONPATH=.:navigation:vision
python -m experiments.seg_fps.pairs --a a1 --b bev --label bev \
  --video /home/orangepi/seg_fps_data/pi_upper/data/road/drive_20260926_160230_028280.avi \
  --model /home/orangepi/seg_fps_data/pi_upper/models/road_yolo11n_seg.rknn \
  --config /home/orangepi/seg_fps_data/pi_upper/config/nav_camera.yaml \
  --output-dir /home/orangepi/seg_fps_results
```

版本按单一因素递增：`a0` 原始逻辑，`a1` 仅 NMS 坐标修正，`bev` 复用投影，
`buffers` 继续复用分割临时内存，`fifo` 解码线程背压且不丢帧，`latest` 替换等待帧。
测量模式：`pure` 只测 RKNN 同步调用及 IO；`seg` 包含分割预后处理；`full` 再加
循迹及路口视觉计算。full 的范围不包含真实整车反馈与下位机动作。

`--cores 1` 是 NPU0；`7` 是一个 context 允许三核；`1,2`/`1,2,4` 是各核独立
context；`1,1,2,2,4,4` 是每核两个 context。每个 worker 同一时刻仅一个任务，
输入输出不共享可写缓冲，消费者按提交顺序更新导航状态。

每轮独立进程、相同预热，统一时间窗口按完成总数统计；原始逐帧耗时先存内存，
结束后批量写 JSON。FIFO 的队列年龄包含解码背压，不能将长队列误认为低延迟。
实时模式 `--realtime --speed 1` 用固定录像时间轴，年龄从该帧应发布时刻开始，
不是相机曝光到控制的真实延迟。`--max-age .2` 拒绝过期结果；source/sequence
检查拒绝旧源、重复与乱序，不让它们更新 EMA 或路口状态。

`--record none|sync|async` 比较录像影响；AVI 时间轴由实际提交时间决定。
异步队列替换等待帧并统计 `record_dropped`，正常退出排空队列，编码异常会抛出。
有限队列只限制待编码帧，固定时间轴仍可能补写重复帧。

```bash
python -m experiments.seg_fps.regression \
  --videos /home/orangepi/seg_fps_data/pi_upper/data/road \
  --model /home/orangepi/seg_fps_data/pi_upper/models/road_yolo11n_seg.rknn \
  --config /home/orangepi/seg_fps_data/pi_upper/config/nav_camera.yaml \
  --video-sample-count 9 --reuse-adjacent-heads \
  --output /home/orangepi/seg_fps_results/regression.json
```

逐帧回归共享同一次 RKNN 输出，严格比较原始预处理张量、A1 mask、缓冲 mask、
BEV、诊断、控制和路口序列，单独记录 A0→A1 的行为变化。可指定
`--reference-core` 对比另一核心的原始 tensor（绝对/相对容差各 1e-5）。
不会把共享 tensor 的 CPU 回归冒充跨核心 NPU 回归。
`--video-sample-count 9` 在排序后的录像中均匀选取九段，默认逐帧完整检查所选片段；
`--sample-per-video N` 才会进一步抽取片段内的帧。均匀抽帧的稀疏观测序列不能用于
声明完整路口时序通过。`--reuse-adjacent-heads` 只在正确性回归复用完全相同的
相邻图像输出，CPU mask/BEV、EMA 与路口仍逐帧运行，不用于吞吐计数。

正式相机入口提供默认关闭的独立开关：`--fps-opt` 复用投影与缓冲，
`--correct-nms` 单独修正 NMS，`--npu-core-mask` 选择单 context 核心，
`--npu-contexts 3` 启用三个核各一个 context 的有界有序流水线，
`--record-video ... --async-record-video` 开启异步录像。超过原有 200ms 预算的
画面不更新视觉状态；转弯完成后清空旧代画面与证据，重新计数原有识路帧数。
所有优化默认关闭，实际相机曝光/采集缓冲与车辆动作仍需台架验证。
`--npu-contexts` 与单 context 的 `--npu-core-mask` 互斥；历史 benchmark 的 latest
变体仅用于回放，后文新增的 `--latest-frame` 才接入实际相机入口。

本次按用户反馈接受三个 context 的吞吐/单帧延迟权衡：纯 RKNN 筛选中，
相对两个 context 吞吐约 +68%，单帧 P95 约 +6.8%；完整视觉链路筛选约
10.49→17.67 FPS，实际资源和各轮波动见报告。同步→异步录像约 +45.7%。
原生标准/输入绑定 IO 与按需普通中心线没有明确 FPS 收益，保留为实验选项。

重启后五对 60 秒正式验证：A1 单 context 与三 context 组合平均
6.26→19.53 FPS（配对平均 +212.0%）；单帧工作 P95 168.44→194.80ms。
10 分钟实际 OrderedSegmentStream 连续验证完成 10,729 帧，9,610 帧通过导航前
200ms 时效门，处理/有效更新分别为 17.88/16.02 FPS，无异常或乱序。
按用户最终要求，完整回放均匀选取的九段；结果、录像帧数、NMS 单独变化和资源
曲线见 RESULTS.md，不声称全部 29 段通过。实时/混合负载后续缩短为一对 20 秒
抽检，不能当成统计显著性结论。所有中断记录保留，不混入完整轮次的平均。

原生 IO 后端默认关闭。`bash navigation/road_follow/native/build.sh` 编译后，可用
`--backend c-standard|c-input-zero` 做回放 A/B，正式入口对应 `--rknn-backend`。
输入绑定路径只改变输入 IO，输出仍走相同的标准 float 获取路径；报告不能将它
写成输入输出全 zero-copy。每次查询原生布局/行跨度，显式同步输入缓存，返回
输出数组时复制所有权。UINT8 的 NPU 归一化方式参考
[Rockchip 官方示例](https://github.com/airockchip/rknn_model_zoo/blob/main/examples/yolo11/cpp/rknpu2/yolo11_zero_copy.cc)，
需要用本项目真实模型逐元素及逐帧验证；不能仅依据示例认为结果等价。

本次数据没有确认逐帧同步的 ODOM/IMU/动作完成日志；道路丢失、障碍确认和转弯
流程的既有单元测试属于软件模拟，最终仍需要真实相机及静止台架确认。

2026-10-03 按 07 补充建议完成下一轮，结果见 SUPPLEMENT_RESULTS.md。
五组同一合法最高频率对照中，只将 NumPy/BLAS 从 8 降至 1 线程，完整回放平均
22.55→34.69 FPS（+53.84%）；单帧 P95 151.56→89.93ms。OpenCV 保持原 8 线程，
Lite 后端和三个 context 保留，六 context、C IO 及强制 mmap 回收未纳入最终组合。

原普通调频下实际程序只有 19.78/18.18 处理/有效 FPS，不能用锁频基准替代。
最终合法固频 + BLAS=1 + 每 60 秒 GC/空闲堆页回收，实际流水线五分钟
33.55 FPS，10,066 帧全部通过导航前 200ms 时效门，异常/乱序/过期均为零。
回收间隔默认关闭；启用后记录完整资源曲线，五分钟结论不外推为三十分钟。
最新 stability.py 默认时长已改为用户要求的 300 秒，前文十分钟为已完成历史结果。

推荐试运行（不开串口；实际模型和相机配置按项目默认，性能素材另有冻结路径）：

    export PYTHONPATH=.:navigation:vision
    /home/orangepi/vision_compare_pi_upper/bin/python -m experiments.seg_fps.run_with_frequency \
      --policy all-max -- /home/orangepi/vision_compare_pi_upper/bin/python -m road_follow \
      --fps-opt --correct-nms --npu-contexts 3 --blas-threads 1 --heap-trim-interval 60

启动器先授权 sudo，只有 CPU/DDR/NPU 调频命令用 root，主程序仍以普通用户运行。
原 governor/min/max 写入新快照，正常退出及 INT/TERM/HUP 会在子命令结束后恢复，
并逐域读回比对。使用新的 --state 路径，禁止覆盖原快照。--sudo-stdin 仅供批处理
读取一次密码，密码不进入命令参数或日志，运行期间自动续期本进程的 sudo 票据。
生产入口不自动修改频率，所有新开关默认保持原行为；GPU、OPP、电压和温控未改。

regression_threads.py 对相同九段全部 4006 帧使用真实 mask 入口比较线程预算，
RGB、原始 tensor、mask、EMA、控制和路口均一致；完全相同的相邻模型输入只在
正确性回归缓存输出，不进入性能测试。高频 GC 下另有三 context 并发 20 帧
所有权、tensor、mask、顺序及软件速度序列专项，88 项道路测试和三项启动器恢复
检查通过。最新数据保存在 /home/orangepi/seg_fps_results/supplement_20261003_1420。

### 并行读取与单帧替换

相机入口新增 `--latest-frame`，默认关闭。单/双/三个 context 均可使用；每个
context 仍独占模型及输入工作区。读取线程不断解码并发布独立图像副本；只有一个
尚未开工的输入槽，新帧覆盖该槽，空闲 worker 直接取走最新一帧。已开始推理的
图像不替换、不取消。每个 context 最多保留一份未消费结果，下游慢时保持有界。
完成结果按实际完成顺序提供；较旧编号的晚到结果丢弃，不能倒序推进 EMA/路口。
正式入口继续在导航前拒绝超过原有 200ms 预算的结果，有限动作互斥与 TURN_DONE
后来源代次失效仍保留。丢帧可能改变按帧数累计的确认耗时，阈值没有改，实车需验证。

录像回调仅在读取线程执行，退出先停止生产者再关闭编码器；为免同步编码阻塞
持续读取，录制时建议同时启用既有 `--async-record-video`。采集句柄仅读取线程
释放，禁止与 OpenCV/V4L2 的 read 并发 release。不可中断的硬件 read 需要自行返回；
5 秒未退出会报告错误，不能把它当成已安全释放。生产时间戳为 read 完成时刻，
并不直接代表硬件曝光时刻。

同条件 A/B 使用 `latest_pairs.py`：三核、OpenCV=8、BLAS=1、每边相同 20 次 mask
预热、GC/堆回收间隔 60 秒、相同频率策略与素材。`camera_replay.py` 以独立时钟
模拟 30/60 FPS 发布及明确的驱动 FIFO 容量，过期判断使用发布时刻，不能把 AVI
录制文件标注的 10 FPS 当成真实相机率。各轮原始 JSON 包含驱动丢帧、输入替换、
晚到结果丢弃、有效更新 FPS、排队/工作/消费年龄及资源数据。模拟 FIFO 不是实测
相机缓冲配置。`latest_regression.py` 用无损握手源检查九段全部帧，避免丢帧掩盖
错误；仅此正确性回归缓存完全相同的相邻模型输入，不能用其耗时报告 FPS。
结果见 LATEST_RESULTS.md。
