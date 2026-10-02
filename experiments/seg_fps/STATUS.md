# 停测保存状态

用户要求立即停止测试，关闭香橙派并让电脑睡眠。

已完成三组筛选：三个 context 的完整视觉链路平均 17.67 FPS，对照两个 context 10.49 FPS（+68.5%）；同步→异步录像平均 +45.7%。
三个 context 的纯推理单帧 P95 +6.8% 已按用户意见接受为候选权衡。
主程序新增默认关闭的 --npu-contexts 3；200ms 过期帧不更新导航，转弯完成清空旧源画面。
NMS、BEV、缓冲复用、录像和实验工具代码保存在独立分支 codex/seg-fps-ab。

已完成：软件道路/IPM/调度测试、20 帧真实并发 tensor/mask/导航抽检、24 组原生后端及输出所有权对照、9 段实验录像完整解码验证、29 段录像各前 10 帧（290 帧）初步回归。

未完成：5×60 秒正式 A/B（只完成前三对）、全部 16,730 帧回归、连续 10 分钟稳定性、更新后的实时回放及混合负载。不能声明这些验证已通过。
正式轮次的 JSON 与中断日志原样保存，未完成的阶段不纳入汇总。

原始素材 /home/orangepi/seg_fps_data；结果 /home/orangepi/seg_fps_results。
主目录 /home/orangepi/pi_upper 已更新至 dbb06a9，原有 experiments/ 未删除；优化仅位于独立工作目录 /home/orangepi/pi_upper_seg_fps，未合并或推送。
