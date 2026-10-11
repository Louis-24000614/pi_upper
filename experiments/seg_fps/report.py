"""汇总所有原始 A/B 统计；缺失的正式验证明确标记，不用最好一轮代替平均。"""
import argparse
import json
from pathlib import Path
import statistics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--results",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    rows=[]
    for summary_path in sorted(args.results.glob("*/summary.json")):
        s=json.loads(summary_path.read_text())
        files=list(summary_path.parent.glob("[0-9]*_[ab].json"))
        sides={"a":[],"b":[]}
        for f in files:
            sides[f.stem[-1]].append(json.loads(f.read_text()))
        def avg(side,key):
            return statistics.mean(x[key] for x in sides[side])
        def metric(side,key):
            values=[x["metrics"].get(key,{}).get("p95") for x in sides[side]]
            values=[x for x in values if x is not None]
            return statistics.mean(values) if values else None
        rows.append({"label":s["label"],"pairs":len(s["pairs"]),
                     "seconds_per_round":avg("a","seconds"),
                     "a_variant":sides["a"][0]["variant"],"b_variant":sides["b"][0]["variant"],
                     "a_cores":sides["a"][0]["cores"],"b_cores":sides["b"][0]["cores"],
                     "a_backend":sides["a"][0].get("backend","lite"),
                     "b_backend":sides["b"][0].get("backend","lite"),
                     "mode":sides["a"][0]["mode"],
                     "a_fps":avg("a","fps"),"b_fps":avg("b","fps"),
                     "mean_gain_pct":s["mean_gain_pct"],"std_gain_pct":s["std_gain_pct"],
                     "a_age_p95_ms":metric("a","age_ms"),"b_age_p95_ms":metric("b","age_ms"),
                     "a_worker_p95_ms":metric("a","worker_ms"),"b_worker_p95_ms":metric("b","worker_ms"),
                     "a_slow_stop_pct":statistics.mean(100*x.get("slow_stop_count",0)/max(1,x["consumed"]) for x in sides["a"]),
                     "b_slow_stop_pct":statistics.mean(100*x.get("slow_stop_count",0)/max(1,x["consumed"]) for x in sides["b"]),
                     "a_cpu_percent":avg("a","cpu_percent"),"b_cpu_percent":avg("b","cpu_percent"),
                     "a_rss_peak_mib":avg("a","rss_peak_kib")/1024,
                     "b_rss_peak_mib":avg("b","rss_peak_kib")/1024})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    lines=["# 道路帧率优化实测", "", "板端分支 `codex/seg-fps-ab`，起点 `dbb06a9`。",
           "模型、640 输入、标定和控制参数固定；系统使用原调频策略。", "",
           "表中数据为配对轮次的平均；P95 是各轮 P95 的平均，原始分布见 JSON。",
           "full 只包含解码、分割、循迹和路口视觉意图，不代表整车闭环。", "",
           "|实验|配对×秒|模式|A FPS|B FPS|平均提升|配对波动|A/B 单帧 P95 ms|A/B 帧龄 P95 ms|A/B CPU %|A/B 峰值内存 MiB|",
           "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    def show(x):return "—" if x is None else f"{x:.1f}"
    for r in rows:
        lines.append(f"|{r['label']}|{r['pairs']}×{r['seconds_per_round']:.0f}|{r['mode']}|{r['a_fps']:.2f}|{r['b_fps']:.2f}|"
                     f"{show(r['mean_gain_pct'])}%|{show(r['std_gain_pct'])}%|"
                     f"{show(r['a_worker_p95_ms'])}/{show(r['b_worker_p95_ms'])}|"
                     f"{show(r['a_age_p95_ms'])}/{show(r['b_age_p95_ms'])}|"
                     f"{r['a_cpu_percent']:.0f}/{r['b_cpu_percent']:.0f}|"
                     f"{r['a_rss_peak_mib']:.0f}/{r['b_rss_peak_mib']:.0f}|")
    for filename in ["regression_screen.json","regression_full.json","stability.json"]:
        path=args.results/filename
        lines += ["",f"## {filename}",""]
        if path.exists():
            obj=json.loads(path.read_text())
            if "videos" in obj:
                lines.append(f"录像 {len(obj['videos'])} 段，处理 {obj.get('total_frames',0)} 帧，错误 {len(obj['errors'])}。")
                lines.append(f"NMS 修正影响 mask {sum(v['nms_changed_masks'] for v in obj['videos'])} 帧，"
                             f"控制 {sum(v['nms_changed_commands'] for v in obj['videos'])} 帧。")
                lines.append(f"相邻重复画面 {sum(v['duplicates_adjacent'] for v in obj['videos'])} 帧；"
                             f"实际模型输入 {sum(v.get('inference_frames',v['frames']) for v in obj['videos'])} 帧。"
                             "重复输出复用仅属于正确性验证，所有帧仍更新视觉状态；此时长不计算性能收益。")
            else:
                lines.append(f"时长 {obj['seconds']:.0f} 秒，完成 {obj['completed']} 次，"
                             f"消费 {obj['consumed']} 次，FPS {obj['fps']:.2f}。")
                lines.append(f"慢帧停车 {obj.get('slow_stop_count',0)} 次；拒绝结果 {obj['rejections']}。"
                             f"诊断范围：{obj.get('metrics_scope','all frames')}。")
                resources=obj.get('resources',[])
                rss=[]
                temperatures=[]
                for sample in resources:
                    status=sample.get('/proc/self/status','')
                    for value in status.splitlines():
                        if value.startswith('VmRSS:'):
                            rss.append((sample['elapsed_s'],int(value.split()[1])/1024))
                    for key,value in sample.items():
                        if 'thermal_zone' in key and key.endswith('/temp'):
                            temperatures.append(int(value)/1000)
                if rss:
                    steady=[value for elapsed,value in rss if elapsed>=300]
                    lines.append(f"RSS 全程 {min(v for _,v in rss):.1f}–{max(v for _,v in rss):.1f} MiB；"
                                 f"第 5 分钟后 {min(steady):.1f}–{max(steady):.1f} MiB。" if steady else
                                 f"RSS 全程 {min(v for _,v in rss):.1f}–{max(v for _,v in rss):.1f} MiB。")
                if temperatures:
                    lines.append(f"温度样本 {min(temperatures):.1f}–{max(temperatures):.1f} ℃。")
        else:
            lines.append("尚未完成。")
    args.output.write_text("\n".join(lines)+"\n",encoding="utf-8")
    print("report",args.output,flush=True)


if __name__=="__main__":
    main()
