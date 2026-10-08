"""合成 AprilTag 与本机 HTTP 验证；不占用相机、NPU 或 UART。"""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from apriltag_calibration import load_calibration
from apriltag_web import CalibrationSession, camera_reference, handler_factory


def marker_frame():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary,0,160)
    frame = np.full((480,640),255,np.uint8)
    frame[160:320,240:400] = marker
    return cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR)


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='apriltag_web_test_')
        self.root = Path(self.tmp.name)
        self.session = CalibrationSession('synthetic-navigation-camera',self.root/'results')
        self.frame = marker_frame()
        self.session.publish(self.frame)

    def tearDown(self):
        self.tmp.cleanup()

    def capture(self):
        return self.session.capture()

    def test_capture_retains_raw_pixels_and_tag_coordinate_reference(self):
        result = self.capture()
        self.assertEqual(result['record']['tag']['outer_black_size_m'],.134)
        self.assertEqual(result['record']['coordinate_reference'],'tag_center')
        self.assertIsNone(result['record']['vehicle_reference'])
        self.assertFalse(result['record']['applied'])
        np.testing.assert_array_equal(self.session.draft['frame'],self.frame)
        self.assertFalse(np.array_equal(self.session.draft['overlay'],self.frame))

    def test_independent_save_leaves_configs_and_existing_records_unchanged(self):
        config = self.root/'config'; config.mkdir()
        files = [config/'nav_camera.yaml',config/'culvert.yaml',config/'bev_calibration.json']
        for path in files: path.write_bytes(b'KEEP ORIGINAL\n')
        first = self.session.save({'draft_id':self.capture()['draft_id']})
        original = Path(first['path']).read_bytes()
        self.session.publish(self.frame)
        second = self.session.save({'draft_id':self.capture()['draft_id']})
        self.assertNotEqual(first['path'],second['path'])
        self.assertEqual(Path(first['path']).read_bytes(),original)
        self.assertTrue(all(p.read_bytes()==b'KEEP ORIGINAL\n' for p in files))
        record = load_calibration(Path(second['path']))
        self.assertFalse(record['applied'])
        np.testing.assert_array_equal(cv2.imread(str(Path(second['path']).parent/'raw.png')),self.frame)

    def test_failed_recapture_invalidates_previous_draft(self):
        previous = self.capture()['draft_id']
        self.session.publish(np.full_like(self.frame,255))
        with self.assertRaises(ValueError): self.capture()
        self.assertIsNone(self.session.draft)
        with self.assertRaises(ValueError): self.session.save({'draft_id':previous})
        self.assertFalse(self.session.directory.exists())

    def test_stale_frame_and_camera_error_prevent_capture_and_save(self):
        draft = self.capture()['draft_id']
        with patch('apriltag_web.time.monotonic',return_value=self.session.captured_s+2):
            with self.assertRaises(ValueError): self.session.save({'draft_id':draft})
            with self.assertRaises(ValueError): self.capture()
        self.session.fail('camera unplugged')
        with self.assertRaises(ValueError): self.capture()

    def test_changed_resolution_prevents_saving_previous_draft(self):
        draft = self.capture()['draft_id']
        self.session.publish(cv2.resize(self.frame,(320,240)))
        with self.assertRaises(ValueError): self.session.save({'draft_id':draft})
        self.assertFalse(self.session.directory.exists())

    def test_changed_draft_token_rejects_old_preview_and_save(self):
        first = self.capture()['draft_id']; second = self.capture()['draft_id']
        self.assertNotEqual(first,second)
        with self.assertRaises(ValueError): self.session.preview('frozen',first)
        with self.assertRaises(ValueError): self.session.save({'draft_id':first})

    def test_save_is_idempotent_for_same_frozen_frame(self):
        values = {'draft_id':self.capture()['draft_id']}
        first = self.session.save(values); second = self.session.save(values)
        self.assertEqual(first,second)
        self.assertEqual(len(list(self.session.directory.glob('*/calibration.json'))),1)

    def test_measured_camera_reference_has_cm_conversion_and_remains_unverified(self):
        record = self.capture()['record']
        reference = camera_reference(record,{'tag_center_x_cm':3.,'tag_center_y_cm':40.,'axes_aligned':True})
        self.assertEqual(reference['tag_center_m'],[.03,.4])
        np.testing.assert_allclose(reference['ground_points_m'],np.asarray(record['ground_points'])+[.03,.4])
        self.assertFalse(reference['verified'])
        self.assertEqual(record['coordinate_reference'],'tag_center')

    def test_incomplete_or_nonfinite_camera_reference_is_rejected(self):
        record = self.capture()['record']
        self.assertIsNone(camera_reference(record,{}))
        for values in ({'tag_center_y_cm':40}, {'tag_center_x_cm':0,'tag_center_y_cm':40},
                       {'tag_center_x_cm':float('nan'),'tag_center_y_cm':40,'axes_aligned':True},
                       {'tag_center_x_cm':0,'tag_center_y_cm':-1,'axes_aligned':True}):
            with self.assertRaises(ValueError): camera_reference(record,values)

    def test_image_write_failure_does_not_claim_saved(self):
        values = {'draft_id':self.capture()['draft_id']}
        with patch('apriltag_web.cv2.imwrite',return_value=False):
            with self.assertRaises(OSError): self.session.save(values)
        self.assertIsNone(self.session.draft['path'])
        self.assertFalse(list(self.session.directory.glob('*/calibration.json')))


class HttpTest(unittest.TestCase):
    def setUp(self):
        SessionTest.setUp(self)
        self.stop = threading.Event()
        self.server = ThreadingHTTPServer(('127.0.0.1',0),handler_factory(self.session,self.stop))
        self.server.daemon_threads = True
        self.worker = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.worker.start()
        self.url = 'http://127.0.0.1:'+str(self.server.server_port)

    def tearDown(self):
        self.stop.set(); self.server.shutdown(); self.server.server_close(); self.worker.join(2)
        SessionTest.tearDown(self)

    def request(self,path,values=None,origin=None):
        headers = {'Content-Type':'application/json'}
        if origin: headers['Origin'] = origin
        data = json.dumps(values).encode() if values is not None else None
        return urlopen(Request(self.url+path,data=data,headers=headers),timeout=3)

    def test_browser_flow_and_mjpeg_headers(self):
        with self.request('/') as r: self.assertIn('13.4 cm',r.read().decode())
        with self.request('/status') as r: self.assertFalse(json.load(r)['auto_apply'])
        with self.request('/camera.mjpg') as r:
            self.assertIn('multipart/x-mixed-replace',r.headers['Content-Type'])
            self.assertIn(b'Content-Type: image/jpeg',r.read(100))
        with self.request('/capture',{}) as r: draft = json.load(r)
        with self.request('/frozen.jpg?draft='+draft['draft_id']) as r:
            self.assertEqual(r.headers['Content-Type'],'image/jpeg')
            self.assertEqual(r.read(2),b'\xff\xd8')
        with self.request('/save',{'draft_id':draft['draft_id']}) as r:
            self.assertTrue(Path(json.load(r)['path']).is_file())

    def test_cross_origin_save_is_rejected(self):
        with self.assertRaises(HTTPError) as raised:
            self.request('/capture',{},origin='http://another-host.invalid')
        self.assertEqual(raised.exception.code,403)
        self.assertIsNone(self.session.draft)


if __name__=='__main__':
    unittest.main()
