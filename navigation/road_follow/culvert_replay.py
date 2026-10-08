"""本地AVI离线检测；没有相机、UART、运动或频率配置入口。"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

import cv2

from vision.obstacle.detect import ObstacleDetector, draw


def main(argv=None):
    parser=argparse.ArgumentParser(description="只读录像推理并导出类别/检测框证据")
    root=Path(__file__).resolve().parents[2]
    parser.add_argument("--video",type=Path,required=True)
    parser.add_argument("--config",type=Path,default=root/"config"/"culvert.yaml")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--every",type=float,default=1)
    args=parser.parse_args(argv)
    if not args.video.is_file() or args.video.suffix.lower() != ".avi":
        parser.error("--video 必须是已存在的本地AVI文件，不能是设备或流地址")
    if args.output.exists() or args.every <= 0:
        parser.error("输出目录必须为新目录，--every 必须大于零")
    capture=cv2.VideoCapture(str(args.video.resolve()))
    detector=None
    try:
        if not capture.isOpened(): raise ValueError("录像无法打开")
        fps=capture.get(cv2.CAP_PROP_FPS)
        if fps <= 0: raise ValueError("录像FPS不合法")
        detector=ObstacleDetector(args.config,root=root)
        args.output.mkdir(parents=True)
        total=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        with (args.output/"detections.jsonl").open("x",encoding="utf-8") as log:
            for index in range(0,total,max(1,round(fps*args.every))):
                capture.set(cv2.CAP_PROP_POS_FRAMES,index)
                ok,frame=capture.read()
                if not ok: break
                items,elapsed=detector.detect(frame)
                log.write(json.dumps({"frame_index":index,"video_s":index/fps,"inference_ms":elapsed,
                    "detections":[asdict(item) for item in items]},ensure_ascii=False)+"\n")
                if items:
                    path=args.output/f"frame_{index:06d}.jpg"
                    if not cv2.imwrite(str(path),draw(frame,items)): raise OSError(f"无法保存{path}")
    finally:
        capture.release()
        if detector is not None: detector.close()


if __name__ == "__main__": main()
