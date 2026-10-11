"""仅用于受控混合负载；识别请求不注册人脸、不写配置、不连接车辆控制。"""
import hashlib
import json
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen
import cv2
from obstacle.detect import ObstacleDetector


class MixedLoad:
    def __init__(self, road_image, root=Path("/home/orangepi/pi_upper")):
        self.stop = threading.Event()
        self.threads = []
        self.detector = ObstacleDetector(root / "config/obstacle.yaml", root=root)
        self.stats = {name: {"completed": 0, "errors": [], "period_s": period}
                      for name, period in [("obstacle", .2), ("knife", 1), ("face", .5)]}
        self.assets = {}
        for path in [root / "config/obstacle.yaml", self.detector.model_path,
                     root / "config/knife.json"]:
            self.assets[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        knife = root / "models/knife/dinov3_448_fp16_v1/references/knife_01/reference.png"
        self.assets[str(knife)] = hashlib.sha256(knife.read_bytes()).hexdigest()
        self.knife_body = self._body(knife.read_bytes(), "image/png")
        ok, jpeg = cv2.imencode(".jpg", road_image)
        if not ok:
            raise RuntimeError("混合负载画面编码失败")
        self.face_body = self._body(jpeg.tobytes(), "image/jpeg")
        self.road_image = road_image.copy()

    @staticmethod
    def _body(image, mime):
        return (b'--seg-fps-boundary\r\nContent-Disposition: form-data; name="image"; '
                b'filename="test_image"\r\nContent-Type: ' + mime.encode() +
                b'\r\n\r\n' + image + b'\r\n--seg-fps-boundary--\r\n')

    def _http(self, port, endpoint, body):
        request = Request(f"http://127.0.0.1:{port}{endpoint}", data=body,
            headers={"Content-Type": "multipart/form-data; boundary=seg-fps-boundary"})
        with urlopen(request, timeout=10) as response:
            result = json.load(response)
        if result.get("status") != "success":
            raise RuntimeError(result.get("message", "识别服务返回失败"))

    def _loop(self, name, action):
        stats = self.stats[name]
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                action()
                stats["completed"] += 1
            except Exception as exc:
                # 服务缺数据库或模型时明确记为未验证，不能把空闲服务算成有效负载。
                stats["errors"].append(repr(exc))
                return
            self.stop.wait(max(0, stats["period_s"]-(time.monotonic()-started)))

    def start(self):
        actions = {
            "obstacle": lambda: self.detector.detect(self.road_image),
            "knife": lambda: self._http(20005, "/api/v1/knife/recognize", self.knife_body),
            "face": lambda: self._http(20004, "/api/v1/recognize", self.face_body),
        }
        for name, action in actions.items():
            thread = threading.Thread(target=self._loop, args=(name, action), name=f"load-{name}")
            self.threads.append(thread)
            thread.start()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=15)
            if thread.is_alive():
                raise RuntimeError("混合负载线程未退出")
        self.detector.close()
        return {"stats": self.stats, "assets": self.assets,
                "scope": "obstacle 5Hz, knife reference 1Hz, road image face detection 2Hz; no face in input"}
