"""逐帧对照 CPU 线程预算：调用真实 mask 入口，比较输入、输出、导航及所有权。"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from road_follow.segment import RoadSegmenter
from road_follow.cpu_threads import cpu_thread_budget
from experiments.seg_fps.navigation import Navigation


class InspectSession:
    """捕获真实入口的 tensor；相邻相同输入仅在正确性回归中允许缓存。"""
    def __init__(self, session):
        self.session, self.input_hash, self.outputs = session, None, None
        self.calls = 0

    def inference(self, inputs, data_format):
        digest = hashlib.sha256(inputs[0].tobytes()).hexdigest()
        if digest != self.input_hash:
            result = self.session.inference(inputs=inputs, data_format=data_format)
            if result is None or len(result) != 2:
                raise RuntimeError('RKNN 返回无效输出')
            # 捕获边界拥有内存，下一次真实推理不能污染回归缓存。
            self.outputs = [row.copy() for row in result]
            self.input_hash = digest
            self.calls += 1
        return self.outputs

    def release(self):
        self.session.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--videos', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--video-count', type=int, default=9)
    parser.add_argument('--opencv-threads', type=int, default=1)
    parser.add_argument('--blas-threads', type=int, default=1)
    parser.add_argument('--backend', choices=('lite', 'c-standard', 'c-input-zero'), default='lite')
    parser.add_argument('--limit', type=int, default=0)
    args = parser.parse_args()
    if args.video_count < 1 or args.limit < 0:
        parser.error('录像数必须为正、限制帧数不能为负')
    paths = sorted(args.videos.glob('*.avi'))
    if not paths:
        parser.error('没有 AVI 录像')
    chosen = [paths[i] for i in np.unique(np.linspace(0, len(paths)-1,
                                    min(len(paths), args.video_count), dtype=int))]
    segments = [RoadSegmenter(args.model, correct_nms=True, reuse_buffers=True,
                              core_mask=1, backend=backend) for backend in ('lite', args.backend)]
    report = {'videos': [], 'errors': [], 'total_frames': 0,
              'model_sha256': hashlib.sha256(args.model.read_bytes()).hexdigest(),
              'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
              'reference_threads': {'opencv': 8, 'blas': 8},
              'candidate_threads': {'opencv': args.opencv_threads, 'blas': args.blas_threads},
              'reference_backend': 'lite', 'candidate_backend': args.backend,
              'scope': 'selected complete videos; adjacent identical tensor heads cached; not FPS test'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load(args.config.read_text())
    try:
        for segment in segments:
            segment._session = InspectSession(segment._load())
        for path in chosen:
            capture = cv2.VideoCapture(str(path))
            declared = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
            navigations = [Navigation(cfg, True), Navigation(cfg, True)]
            started, count, maximum_error = time.monotonic(), 0, 0.0
            retained = None
            calls_before = segments[0]._session.calls
            try:
                if not capture.isOpened() or declared <= 0:
                    raise RuntimeError(f'无法完整解码: {path}')
                while not args.limit or count < args.limit:
                    ok, image = capture.read()
                    if not ok:
                        break
                    results = []
                    for index, (segment, navigation) in enumerate(zip(segments, navigations)):
                        threads = ((8, 8) if index == 0 else
                                   (args.opencv_threads, args.blas_threads))
                        # 回归串行执行，允许在两边切换全局预算；生产只在启动时设置一次。
                        with cpu_thread_budget(*threads):
                            mask = segment.mask(image)
                            results.append((mask, navigation.process(mask)))
                    a, b = results
                    if segments[0]._session.input_hash != segments[1]._session.input_hash:
                        raise AssertionError(f'RGB 输入不一致: {path.name}:{count}')
                    for head_a, head_b in zip(segments[0]._session.outputs, segments[1]._session.outputs):
                        np.testing.assert_allclose(head_a, head_b, atol=1e-5, rtol=1e-5)
                        maximum_error = max(maximum_error, float(np.max(np.abs(head_a-head_b))))
                    np.testing.assert_array_equal(a[0], b[0])
                    if a[1] != b[1] or navigations[0].smoother._prev != navigations[1].smoother._prev:
                        raise AssertionError(f'控制、路口或 EMA 不一致: {path.name}:{count}')
                    if retained is not None:
                        old_mask, old_digest = retained
                        if hashlib.sha256(old_mask.tobytes()).hexdigest() != old_digest:
                            raise AssertionError('上一帧 mask 所有权被覆盖')
                    retained = b[0], hashlib.sha256(b[0].tobytes()).hexdigest()
                    count += 1
                    if count % 200 == 0:
                        print('THREAD_REGRESSION', path.name, count, flush=True)
                if not args.limit and count != declared:
                    raise AssertionError(f'解码帧数 {count} != 声明帧数 {declared}')
            finally:
                capture.release()
            report['total_frames'] += count
            report['videos'].append({'video': path.name, 'frames': count, 'declared_frames': declared,
                                     'inference_frames_per_side': segments[0]._session.calls-calls_before,
                                     'tensor_max_abs_error': maximum_error, 'mask_control_event_differences': 0,
                                     'seconds': time.monotonic()-started, 'complete': count == declared})
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print('THREAD_REGRESSION_COMPLETE', path.name, count, flush=True)
    except BaseException as exc:
        report['errors'].append(repr(exc))
        raise
    finally:
        for segment in segments:
            segment.close()
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
