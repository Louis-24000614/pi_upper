"""九段完整录像的 latest 正确性回归；无损握手源用于覆盖每帧，不用于测 FPS。"""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import yaml
from road_follow.latest_segment import LatestSegmentStream
from road_follow.segment import RoadSegmenter
from road_follow.cpu_threads import cpu_thread_budget
from road_follow.heap_reclaim import HeapReclaimer
from experiments.seg_fps.navigation import Navigation
from experiments.seg_fps.regression_threads import InspectSession


def digest(array):
    return hashlib.sha256(array.tobytes()).hexdigest()


class AcknowledgedCapture:
    """消费者确认后才读下一帧，仅正确性测试用：latest 的跳帧由 A/B 单独统计。"""
    def __init__(self, path):
        self.cap = cv2.VideoCapture(str(path))
        self.expected = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not self.cap.isOpened() or self.expected <= 0:
            self.cap.release()
            raise RuntimeError(f'无法完整读取: {path}')
        self.permit, self.stop = threading.Event(), threading.Event()
        self.permit.set()
        self.count, self.image_digest = 0, None

    def read(self):
        self.permit.wait()
        self.permit.clear()
        if self.count == self.expected:
            # EOF 不提前触发相机故障，否则会把最后一份尚未消费结果遮住。
            self.stop.wait()
            return False, None
        if self.stop.is_set():
            return False, None
        ok, image = self.cap.read()
        if not ok:
            raise RuntimeError(f'录像中途读取失败: {self.count}/{self.expected}')
        self.count += 1
        self.image_digest = digest(image)
        return True, image

    def acknowledge(self):
        self.permit.set()

    def stop_reading(self):
        self.stop.set()
        self.permit.set()

    def release(self):
        self.cap.release()


class CachedSegment(RoadSegmenter):
    def mask(self, image):
        if not isinstance(self._session, InspectSession):
            self._session = InspectSession(self._load())
        return super().mask(image)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('videos','model','config','output'):
        p.add_argument('--'+name, type=Path, required=True)
    args = p.parse_args()
    paths = sorted(args.videos.glob('*.avi'))
    if len(paths) < 9:
        p.error('至少需要九段完整录像')
    chosen = [paths[i] for i in np.linspace(0,len(paths)-1,9,dtype=int)]
    report = dict(videos=[], total_frames=0, errors=[],
        model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        scope='9 complete videos; lossless acknowledgement fixture; cached identical NN input only; not FPS')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(args.config.read_text())
    reference = CachedSegment(args.model, core_mask=1, correct_nms=True, reuse_buffers=True)
    try:
        with cpu_thread_budget(8,1), HeapReclaimer(60):
            for path in chosen:
                capture = AcknowledgedCapture(path)
                stream = LatestSegmentStream(args.model, (1,2,4), factory=CachedSegment,
                                             correct_nms=True, reuse_buffers=True)
                navigations = [Navigation(cfg, True), Navigation(cfg, True)]
                began, retained = time.monotonic(), None
                try:
                    stream.warmup(np.zeros((720,1280,3), np.uint8))
                    for index in range(capture.expected):
                        result = stream.read(capture)
                        if result.sequence != index or result.source != 0:
                            raise AssertionError('无损源仍出现跳帧/乱序/来源变化')
                        if digest(result.image) != capture.image_digest:
                            raise AssertionError('读取线程发布的图像所有权发生变化')
                        mask = reference.mask(result.image)
                        np.testing.assert_array_equal(result.mask, mask)
                        states = [nav.process(m) for nav,m in zip(navigations,(mask,result.mask))]
                        if states[0] != states[1] or navigations[0].smoother._prev != navigations[1].smoother._prev:
                            raise AssertionError(f'EMA/控制/路口不一致: {path.name}:{index}')
                        if retained is not None:
                            old, image_digest, mask_digest = retained
                            if digest(old.image) != image_digest or digest(old.mask) != mask_digest:
                                raise AssertionError('上一帧图像或 mask 被覆盖')
                        retained = result, digest(result.image), digest(result.mask)
                        if (index+1)%200 == 0:
                            print('LATEST_REGRESSION',path.name,index+1,flush=True)
                        capture.acknowledge()
                    if capture.count != capture.expected or stream.stats['replaced_input']:
                        raise AssertionError('没有覆盖完整录像')
                    report['total_frames'] += capture.count
                    report['videos'].append(dict(video=path.name, frames=capture.count,
                        declared_frames=capture.expected, complete=True, differences=0,
                        stream_stats=dict(stream.stats), seconds=time.monotonic()-began))
                finally:
                    stream.close()
                    if not stream.owns_capture:
                        capture.release()
                args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
                print('LATEST_REGRESSION_COMPLETE',path.name,capture.count,flush=True)
    except BaseException as exc:
        report['errors'].append(repr(exc))
        raise
    finally:
        reference.close()
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
