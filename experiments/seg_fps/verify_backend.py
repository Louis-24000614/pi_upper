"""相同输入逐元素比较 Lite/C 标准/C 输入绑定路径；不通过则不采用后端。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from road_follow.segment import RoadSegmenter, letterbox, _orient_heads, decode_road_mask


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--model",type=Path,required=True)
    p.add_argument("--videos",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    args=p.parse_args()
    sessions=[RoadSegmenter(args.model, backend=x) for x in ["lite","c-standard","c-input-zero"]]
    records=[]
    retained = None
    try:
        for path in sorted(args.videos.glob("*.avi"))[::5]:
            cap=cv2.VideoCapture(str(path))
            try:
                for index in [0, int(cap.get(cv2.CAP_PROP_FRAME_COUNT))//2]:
                    cap.set(cv2.CAP_PROP_POS_FRAMES,index)
                    ok,image=cap.read(); assert ok
                    rgb,ratio,left,top=letterbox(image)
                    tensor=np.ascontiguousarray(rgb)[None]
                    results=[s._load().inference(inputs=[tensor],data_format=["nhwc"]) for s in sessions]
                    if retained is not None:
                        originals, copies = retained
                        for a,b in zip(originals,copies):
                            for x,y in zip(a,b):
                                np.testing.assert_array_equal(x,y)
                    retained = results, [[x.copy() for x in r] for r in results]
                    heads=[_orient_heads(*r[:2]) for r in results]
                    masks=[decode_road_mask(*h,ratio,left,top,image.shape[:2],correct_nms=True) for h in heads]
                    for alternative in range(1,3):
                        differences=[]
                        for a,b in zip(heads[0],heads[alternative]):
                            differences.append(float(np.max(np.abs(a-b))))
                            np.testing.assert_allclose(a,b,atol=1e-5,rtol=1e-5)
                        np.testing.assert_array_equal(masks[0],masks[alternative])
                        records.append({"video":path.name,"frame":index,"backend":sessions[alternative].backend,
                                        "tensor_max_abs":differences,"mask_differences":0})
            finally:
                cap.release()
        args.output.write_text(json.dumps({"passed":True,"records":records},indent=2))
        print("backend equivalence passed",len(records),flush=True)
    except BaseException as exc:
        args.output.write_text(json.dumps({"passed":False,"error":repr(exc),"records":records},indent=2))
        raise
    finally:
        for session in sessions:
            session.close()


if __name__=="__main__":
    main()
