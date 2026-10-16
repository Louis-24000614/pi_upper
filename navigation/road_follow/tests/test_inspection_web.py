"""真实本机 HTTP 与合成相机；不打开任何硬件。"""
import json
from pathlib import Path
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from road_follow.inspection_config import Settings
from road_follow.inspection_io import FrameHub
from road_follow.inspection_web import InspectionWeb, PAGE


class WebTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path = Path(self.tmp.name)/"settings.json"
        path.write_bytes((Path(__file__).with_name("fixtures")/"culvert_inspection.json").read_bytes())
        self.settings, self.hub = Settings(path), FrameHub()
        self.hub.publish(np.full((100, 200, 3), 120, np.uint8), time.monotonic())
        state = {"phase": "recognizing", "side": "A", "candidate": None, "sides": [],
                 "valid_frames": 1, "remaining_s": 40, "speech_enabled": False}
        self.web = InspectionWeb(self.settings, self.hub, lambda: state,
                                 map_status=lambda: {"edge": {"status": "partial"}}, address=("127.0.0.1", 0))
        self.addCleanup(self.web.close)
        self.url = "http://127.0.0.1:"+str(self.web.server.server_port)

    def get(self, path):
        return urlopen(self.url+path, timeout=2)

    def post(self, payload, path="/settings"):
        request = Request(self.url+path, data=json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json"}, method="POST")
        return urlopen(request, timeout=2)

    def test_page_has_outer_roi_and_independent_thresholds(self):
        with self.get("/") as response:
            page = response.read().decode()
        self.assertIn("底部板卡", page)
        self.assertIn('id="face_threshold"', page)
        self.assertIn('id="knife_threshold"', page)
        self.assertIn("getBoundingClientRect", page)
        self.assertNotIn("前景提取阈值", page)

    def test_apply_save_restart_and_crop_preview(self):
        with self.post({"revision": 0, "values": {"roi": [.1, .2, .9, .8], "knife_threshold": .6}}) as response:
            self.assertEqual(json.load(response)["revision"], 1)
        with self.get("/crop.jpg") as response:
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape[:2], (60, 160))
        saved, _ = Settings(self.settings.path).snapshot()
        self.assertEqual(saved["recognition"]["knife_threshold"], .6)

    def test_saved_web_timeout_is_used_by_pair_task_and_persists(self):
        from unittest.mock import Mock
        from road_follow.inspection import SideInspection
        from road_follow.tests.test_inspection import Pool, Servo, TARGET
        for revision, seconds in ((0, 10), (1, 7)):
            with self.post({"revision": revision, "values": {"side_timeout_s": seconds}}) as response:
                self.assertEqual(json.load(response)["revision"], revision+1)
            with self.get("/status") as response:
                self.assertEqual(json.load(response)["settings"]["side_timeout_s"], seconds)
            reloaded = Settings(self.settings.path)
            task = SideInspection(reloaded, FrameHub(), Servo(), Mock(), pool=Pool(), pair_fallback=True)
            try:
                task.start(100, TARGET)
                self.assertEqual(task.deadline_s, 100+seconds)
                self.assertEqual(task.status()["effective_side_timeout_s"], seconds)
            finally:
                task.close()

    def test_invalid_input_and_stale_revision_do_not_overwrite(self):
        with self.assertRaises(HTTPError) as error:
            self.post({"revision": 0, "values": {"face_threshold": .1}})
        self.assertEqual(error.exception.code, 400)
        error.exception.close()
        with self.post({"revision": 0, "values": {"confirm_frames": 4}}):
            pass
        with self.assertRaises(HTTPError) as error:
            self.post({"revision": 0, "values": {"confirm_frames": 5}})
        self.assertEqual(error.exception.code, 409)
        error.exception.close()
        config, revision = self.settings.snapshot()
        self.assertEqual(config["recognition"]["confirm_frames"], 4)
        self.assertEqual(revision, 1)

    def test_idle_stream_client_does_not_block_status_or_settings(self):
        stream = self.get("/camera.mjpg")
        self.addCleanup(stream.close)
        started = time.monotonic()
        with self.get("/status") as response:
            payload = json.load(response)
        self.assertLess(time.monotonic()-started, 1)
        self.assertEqual(payload["map"]["edge"]["status"], "partial")
        with self.post({"revision": 0, "values": {"knife_threshold": .7}}) as response:
            self.assertEqual(json.load(response)["revision"], 1)

    def test_old_camera_frame_not_used_for_crop(self):
        self.hub.fail("camera failed")
        with self.assertRaises(HTTPError) as error:
            self.get("/crop.jpg")
        self.assertEqual(error.exception.code, 503)
        error.exception.close()

    def test_page_uses_chinese_cards_tables_and_map_instead_of_json_dump(self):
        with self.get("/") as response:page=response.read().decode()
        for text in ("当前候选目标","当前使用的识别参数","两侧检查结果","障碍判断调参","本次任务拓扑地图","侧视相机没有新鲜帧"):
            self.assertIn(text,page)
        self.assertNotIn("<pre",page)
        self.assertNotIn("JSON.stringify(s",page)
        with self.get("/topology.svg") as response:
            self.assertIn("image/svg+xml",response.headers["Content-Type"])
            self.assertIn("导航任务",response.read().decode())

    def test_obstacle_settings_save_and_conflict_are_separate_from_recognition(self):
        with self.post({"revision":0,"values":{"edge_end_margin_m":.12}},"/obstacle-settings") as response:
            self.assertEqual(json.load(response)["revision"],1)
        with self.get("/status") as response:state=json.load(response)
        self.assertEqual(state["obstacle"]["settings"]["edge_end_margin_m"],.12)
        self.assertEqual(state["revision"],0)
        with self.assertRaises(HTTPError) as error:
            self.post({"revision":0,"values":{"edge_end_margin_m":.2}},"/obstacle-settings")
        self.assertEqual(error.exception.code,409)
        error.exception.close()
        self.assertEqual(Settings(self.settings.path).obstacle_snapshot()[0]["edge_end_margin_m"],.12)

    def test_navigation_web_does_not_expose_start_action(self):
        with self.get("/status") as response:
            self.assertFalse(json.load(response)["start_task_available"])
        with self.assertRaises(HTTPError) as error:
            self.post({}, "/start-task")
        self.assertEqual(error.exception.code, 404)
        error.exception.close()

    def test_debug_button_queues_each_request_and_returns_accepted(self):
        from unittest.mock import Mock
        callback = Mock(return_value={"state": "restart_requested", "next_run": 1})
        self.web.start_task = callback
        with self.get("/status") as response:
            self.assertTrue(json.load(response)["start_task_available"])
        for count in (1, 2):
            with self.post({}, "/start-task") as response:
                self.assertEqual(response.status, 202)
                self.assertEqual(json.load(response)["state"], "restart_requested")
            self.assertEqual(callback.call_count, count)

    def test_start_rejects_foreign_origin_and_malformed_payload_without_calling_task(self):
        from unittest.mock import Mock
        self.web.start_task = Mock()
        request = Request(self.url+"/start-task", data=b"{}",
                          headers={"Content-Type": "application/json", "Origin": "http://other-host"},
                          method="POST")
        with self.assertRaises(HTTPError) as error:
            urlopen(request, timeout=2)
        self.assertEqual(error.exception.code, 403)
        error.exception.close()
        with self.assertRaises(HTTPError) as error:
            self.post({"angle": 180}, "/start-task")
        self.assertEqual(error.exception.code, 400)
        error.exception.close()
        self.web.start_task.assert_not_called()

    def test_page_script_parses_and_button_posts_then_displays_inferred_result(self):
        import shutil
        import subprocess
        if not shutil.which("node"):
            self.skipTest("Node.js unavailable")
        script = PAGE.split("<script>", 1)[1].split("</script>", 1)[0]
        harness = r"""
const vm=require('vm'),assert=require('assert');
class Element{
 constructor(){this.children=[];this.style={};this.value='0';this.disabled=false;this.clientWidth=200;this.clientHeight=100;}
 append(...children){this.children.push(...children);}
 replaceChildren(...children){this.children=children;}
 get firstChild(){return this.children[0];}
 querySelector(){return this.children[1];}
 getContext(){return {clearRect(){},strokeRect(){}};}
}
const elements={},requests=[];
const context={document:{getElementById(id){return elements[id]??=new Element();},createElement(){return new Element();}},
 window:{},setInterval(){},fetch(path,options){
  requests.push([path,options]);
  if(path==='/status')return new Promise(()=>{});
  return Promise.resolve({ok:true,json:async()=>({state:'restart_requested'})});
}};
vm.createContext(context);
vm.runInContext(SCRIPT,context);
context.s={phase:'idle',valid_frames:0,side:'A',start_task_available:true,run_number:0,
 pair_fallback:true,effective_side_timeout_s:10,inferred_announcements:[],sides:[]};
vm.runInContext("$('side_timeout_s').value='10';metrics(s);showResults(s);taskControls(s)",context);
assert.strictEqual(elements['task-controls'].hidden,false);
assert.strictEqual(elements['start-task'].textContent,'开始涵洞任务');
assert.strictEqual(elements['side_timeout_s'].disabled,false);
assert.strictEqual(elements['side_timeout_s'].value,'10');
assert(elements['pair-mode'].textContent.includes('10 秒'));
elements['side_timeout_s'].value='7';
vm.runInContext('taskControls(s)',context);
assert.strictEqual(elements['side_timeout_s'].value,'7');
context.s.phase='recognizing';context.s.run_number=1;
vm.runInContext('metrics(s);showResults(s);taskControls(s)',context);
assert.strictEqual(elements['start-task'].textContent,'重新开始涵洞任务');
context.s.phase='announcing';
context.s.pending_announcement={record:{identity:'knife_07',source:'face_pair',based_on:'suspect_08'}};
vm.runInContext('metrics(s);showResults(s);taskControls(s)',context);
assert.strictEqual(elements.results.children.length,3);
elements['start-task'].onclick().then(async()=>{
 const request=requests.find(x=>x[0]==='/start-task');
 assert(request);assert.strictEqual(request[1].method,'POST');assert.strictEqual(request[1].body,'{}');
 context.s.start_task_available=false;context.s.pair_fallback=false;
 vm.runInContext('taskControls(s)',context);
 assert.strictEqual(elements['task-controls'].hidden,true);
 assert.strictEqual(elements['side_timeout_s'].disabled,false);
 await elements['save'].onclick();
 const saved=requests.find(x=>x[0]==='/settings');
 assert(saved);assert.strictEqual(JSON.parse(saved[1].body).values.side_timeout_s,7);
 console.log('PASS button, restart, inference display, navigation hiding, editable pair timeout save');
}).catch(error=>{console.error(error);process.exitCode=1;});
"""
        harness = harness.replace("SCRIPT", json.dumps(script, ensure_ascii=True))
        result = subprocess.run(["node", "-"], input=harness, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn("PASS button", result.stdout)


if __name__ == "__main__":
    unittest.main()
