"""逐级核配置实验；以吞吐和单帧 P95 共同筛选，避免只取最高 FPS。"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--video",required=True)
    parser.add_argument("--model",required=True)
    parser.add_argument("--config",required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--max-p95-rise",type=float,default=3,
                        help="允许的 P95 增幅百分比；按用户帧率/延迟偏好单独设置")
    args=parser.parse_args()
    records=[]
    def pair(label, cores_a, cores_b, mode="pure"):
        command=[sys.executable,"-m","experiments.seg_fps.pairs","--a","buffers","--b","buffers",
                 "--cores-a",cores_a,"--cores-b",cores_b,"--mode",mode,"--label",label,
                 "--video",args.video,"--model",args.model,"--config",args.config,
                 "--output-dir",str(args.output_dir)]
        subprocess.run(command,check=True)
        summary=json.loads((args.output_dir/label/"summary.json").read_text())
        latency=[]
        for p in summary["pairs"]:
            latency.append(p["b"]["metrics"]["worker_ms"]["p95"] /
                           p["a"]["metrics"]["worker_ms"]["p95"]-1)
        summary["worker_p95_change_pct"]=float(np.mean(latency)*100)
        allowed=args.max_p95_rise/100
        summary["candidate"]=(summary["mean_gain_pct"]>=5 and min(latency)<=allowed
                              and summary["worker_p95_change_pct"]<=args.max_p95_rise)
        records.append(summary)
        print("DECISION",label,"candidate" if summary["candidate"] else "not selected",
              "gain",summary["mean_gain_pct"],"latency",summary["worker_p95_change_pct"],flush=True)
        return summary
    pair("n1_multicore","1","7")
    pair("n2_two_contexts","1","1,2")
    n3=pair("n3_three_contexts","1,2","1,2,4")
    if n3["candidate"]:
        pair("n4_six_contexts","1,2,4","1,1,2,2,4,4")
    else:
        print("DECISION n4 skipped: N3 has no verified throughput/latency advantage",flush=True)
    candidates=[r for r in records if r["candidate"]]
    # 各阶段的 A 不同，不能比较“相对提升百分比”选最终配置；比较 B 的实际吞吐。
    best=max(candidates,key=lambda x:np.mean([p["b"]["fps"] for p in x["pairs"]])) if candidates else None
    cores={"n1_multicore":"7","n2_two_contexts":"1,2","n3_three_contexts":"1,2,4",
           "n4_six_contexts":"1,1,2,2,4,4"}
    selected=cores[best["label"]] if best else "1"
    if selected!="1":
        pair("npu_selected_seg","1",selected,"seg")
        pair("npu_selected_full","1",selected,"full")
    (args.output_dir/"npu_campaign.json").write_text(json.dumps({"records":records,
                                                               "screen_candidate":selected},indent=2))


if __name__=="__main__":
    main()
