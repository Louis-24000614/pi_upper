"""不传优化参数启动真实入口，验证默认组合、异步录像与桌面兼容。"""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from threadpoolctl import threadpool_info
from road_follow import __main__ as entry
from road_follow.frequency_runtime import needs_frequency_guard, MANAGED_ENV


class DefaultRuntimeTest(unittest.TestCase):
    def run_default(self, model='fake.rknn', extra=()):
        segments, settings = [], []
        class Segment:
            def __init__(self, model, **kwargs):
                self.options, self.closed = kwargs, False
                segments.append(self)
            def mask(self, image):
                settings.append((cv2.getNumThreads(), [p['num_threads'] for p in
                    threadpool_info() if p['user_api']=='blas']))
                return np.zeros(image.shape[:2],np.uint8)
            def close(self):
                self.closed = True
        class Capture:
            released = False
            def read(self):
                return True,np.zeros((64,64,3),np.uint8)
            def release(self):
                self.released = True
        capture = Capture()
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)/'config.yaml'
            config.write_text('{}\n')
            record = Path(folder)/'record.avi'
            argv = ['--model',model,'--config',str(config),'--frames','2']
            if extra == ('record',):
                argv += ['--record-video',str(record)]
            else:
                argv += list(extra)
            real_async = entry.AsyncVideoRecorder
            real_projector = entry.BevProjector
            real_reclaimer = entry.HeapReclaimer
            with patch.object(entry,'RoadSegmenter',Segment), \
                    patch.object(entry,'_open_camera',return_value=capture), \
                    patch.object(entry,'needs_frequency_guard',return_value=False), \
                    patch.object(entry,'LatestSegmentStream') as latest, \
                    patch.object(entry,'BevProjector',wraps=real_projector) as bev, \
                    patch.object(entry,'AsyncVideoRecorder',wraps=real_async) as encoder, \
                    patch.object(entry,'HeapReclaimer',wraps=real_reclaimer) as heap, \
                    patch.object(entry.signal,'signal'), patch.object(entry,'_event'):
                self.assertEqual(entry.main(argv),0)
                latest.assert_not_called()
                bev.assert_called_once()
                heap.assert_called_once_with(60 if entry.sys.platform=='linux' else 0)
                if extra == ('record',):
                    encoder.assert_called_once_with(record)
                    decoded = cv2.VideoCapture(str(record))
                    try:
                        self.assertTrue(decoded.isOpened())
                        self.assertTrue(decoded.read()[0])
                    finally:
                        decoded.release()
                else:
                    encoder.assert_not_called()
        self.assertTrue(capture.released)
        self.assertTrue(all(segment.closed for segment in segments))
        self.assertTrue(all(cv==8 and all(n==1 for n in blas) for cv,blas in settings))
        self.assertTrue(all(s.options['correct_nms'] and s.options['reuse_buffers'] for s in segments))
        return segments

    def test_plain_rknn_start_uses_three_private_ordered_workers(self):
        segments = self.run_default()
        self.assertEqual([s.options['core_mask'] for s in segments],[1,2,4])

    def test_record_request_automatically_uses_async_encoder(self):
        self.run_default(extra=('record',))

    def test_onnx_and_existing_core_override_keep_single_context(self):
        self.assertEqual(len(self.run_default(model='fake.onnx')),1)
        segments = self.run_default(extra=('--npu-core-mask','2'))
        self.assertEqual(len(segments),1)
        self.assertEqual(segments[0].options['core_mask'],2)

    def test_automatic_guard_reexecs_same_arguments_without_optimization_flags(self):
        argv = ['--model','fake.rknn','--frames','2']
        with patch.object(entry,'needs_frequency_guard',return_value=True), \
                patch.object(entry,'run_guarded',return_value=17) as guard:
            self.assertEqual(entry.main(argv),17)
        guard.assert_called_once_with([entry.sys.executable,'-m','road_follow',*argv])

    def test_nested_guard_marker_prevents_double_frequency_restore(self):
        with patch.dict('os.environ',{MANAGED_ENV:'1'}):
            self.assertFalse(needs_frequency_guard(Path('fake.rknn')))


if __name__ == '__main__':
    unittest.main()
