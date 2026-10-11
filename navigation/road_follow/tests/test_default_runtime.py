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

    def test_plain_rknn_start_reserves_core_zero_for_detection(self):
        segments = self.run_default()
        self.assertEqual([s.options['core_mask'] for s in segments],[2,4])

    def assert_detector_core_zero(self, config_name):
        import hashlib
        from road_follow import segment
        from vision.obstacle import detect
        class Runtime:
            NPU_CORE_0, NPU_CORE_1, NPU_CORE_2, NPU_CORE_0_1_2 = 1, 2, 4, 7
            def __init__(self, **kwargs):
                self.selected = None
            def load_rknn(self, path):
                return 0
            def init_runtime(self, *, core_mask):
                self.selected = core_mask
                return 0
            def release(self):
                pass
        with tempfile.TemporaryDirectory() as folder:
            model = Path(folder)/'fake.rknn'
            model.write_bytes(b'fake model for loader test')
            detector = detect.ObstacleDetector(entry.ROOT/'config'/config_name, root=entry.ROOT)
            detector.model_path = model
            detector.config['model']['sha256'] = hashlib.sha256(model.read_bytes()).hexdigest()
            segs = [segment.RoadSegmenter(model, core_mask=mask) for mask in (2,4)]
            try:
                with patch.object(segment,'_import_rknn_lite',return_value=Runtime), \
                        patch.object(detect,'_import_rknn_lite',return_value=Runtime):
                    seg_masks = [s._load().selected for s in segs]
                    detection_mask = detector._load().selected
                self.assertEqual(seg_masks,[Runtime.NPU_CORE_1,Runtime.NPU_CORE_2])
                self.assertEqual(detection_mask,Runtime.NPU_CORE_0)
                self.assertTrue(all(detection_mask & mask == 0 for mask in seg_masks))
            finally:
                detector.close()
                for seg in segs:
                    seg.close()

    def test_obstacle_config_effective_core_is_zero(self):
        self.assert_detector_core_zero('obstacle.yaml')

    @unittest.skipUnless((entry.ROOT/'config/culvert.yaml').is_file(),'涵洞配置仅在板端集成目录提供')
    def test_culvert_config_effective_core_is_zero(self):
        self.assert_detector_core_zero('culvert.yaml')

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
