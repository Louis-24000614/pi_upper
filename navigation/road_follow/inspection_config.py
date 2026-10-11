"""侧视识别的独立 JSON 配置；网页仅可更改识别参数，不操作设备。"""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import threading

from road_follow.obstacle_edge import OBSTACLE_DEFAULTS


EDITABLE = {"face_threshold", "knife_threshold", "confirm_frames", "side_timeout_s", "roi"}
OBSTACLE_EDITABLE = set(OBSTACLE_DEFAULTS)
OBSTACLE_NAMES = {"edge_end_margin_m":"路段终点保留余量", "distance_bias_m":"投影距离修正",
                  "max_distance_m":"最大判定距离", "max_lateral_m":"横向容差",
                  "min_score":"障碍检测置信度", "min_bottom_ratio":"框底部最低位置",
                  "confirm_frames":"障碍连续确认帧数"}


def validate_obstacle(values):
    if not isinstance(values, dict) or set(values) != OBSTACLE_EDITABLE:
        raise ValueError("障碍参数字段不完整或包含未知项")
    for key in ("edge_end_margin_m", "max_distance_m", "max_lateral_m"):
        number(values[key], OBSTACLE_NAMES[key], 0 if key == "edge_end_margin_m" else .01, 5)
    number(values["distance_bias_m"], OBSTACLE_NAMES["distance_bias_m"], -1, 1)
    for key in ("min_score", "min_bottom_ratio"):
        number(values[key], OBSTACLE_NAMES[key], 0, 1)
    number(values["confirm_frames"], OBSTACLE_NAMES["confirm_frames"], 1, 100)
    if not isinstance(values["confirm_frames"], int):
        raise ValueError("障碍连续帧数必须是整数")
    return values


def number(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限数")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum}～{maximum} 之间")


def validate(config):
    if config.get("version") != 1:
        raise ValueError("不支持的侧视识别配置版本")
    config["obstacle"] = {**OBSTACLE_DEFAULTS, **config.get("obstacle", {})}
    validate_obstacle(config["obstacle"])
    rec = config["recognition"]
    number(rec["face_threshold"], "人脸阈值（服务身份门限为 0.45）", .45, 1)
    number(rec["knife_threshold"], "刀具阈值", 0, 1)
    number(rec["confirm_frames"], "连续帧数", 1, 100)
    if not isinstance(rec["confirm_frames"], int):
        raise ValueError("连续帧数必须是整数")
    number(rec["side_timeout_s"], "每侧超时", 1, 600)
    number(rec["sample_interval_s"], "采样间隔", .1, 60)
    number(rec["request_timeout_s"], "HTTP 超时", .1, 30)
    roi = rec["roi"]
    if not isinstance(roi, list) or len(roi) != 4:
        raise ValueError("ROI 必须是归一化的 [x1,y1,x2,y2]")
    for value in roi:
        number(value, "ROI", 0, 1)
    if roi[0] >= roi[2] or roi[1] >= roi[3]:
        raise ValueError("ROI 不能为空或反向")
    camera, servo, web = config["camera"], config["servo"], config["web"]
    for key in ("width", "height"):
        number(camera[key], key, 1, 8192)
        if not isinstance(camera[key], int):
            raise ValueError("相机尺寸必须是整数")
    if servo["initial_side"] not in ("A", "B") or not isinstance(servo["hardware_verified"], bool):
        raise ValueError("舵机初始侧或硬件确认项不合法")
    if servo["pwm_route"] != "PWM14_M0":
        raise ValueError("当前涵洞侧视接线已指定为 PWM14_M0")
    for value in servo["angles"].values():
        number(value, "舵机绝对角度", 0, 180)
    if set(servo["angles"]) != {"A", "B"} or abs(servo["angles"]["A"]-servo["angles"]["B"]) != 180:
        raise ValueError("两个舵机端点必须相差 180°")
    number(servo["settle_s"], "舵机稳定等待", .1, 30)
    number(servo["ack_timeout_s"], "PWM 写入回执超时", .1, 10)
    number(web["port"], "网页端口", 1, 65535)
    if not isinstance(web["port"], int) or not isinstance(web["host"], str):
        raise ValueError("网页地址或端口不合法")
    number(web["preview_fps"], "预览帧率", 1, 30)
    for endpoint in config["services"].values():
        if not isinstance(endpoint, str) or not endpoint.startswith("http://127.0.0.1:"):
            raise ValueError("识别服务须使用既有本机 HTTP 接口")
    return config


class Settings:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self._lock = threading.RLock()
        self._config = validate(json.loads(self.path.read_text(encoding="utf-8")))
        self._revision = 0
        self._obstacle_revision = 0

    def obstacle_snapshot(self):
        with self._lock:
            return deepcopy(self._config["obstacle"]), self._obstacle_revision

    def _store(self, candidate):
        fd, temporary = tempfile.mkstemp(prefix=self.path.name+".", suffix=".tmp", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(candidate, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        self._config = candidate

    def update_obstacle(self, values):
        if not isinstance(values, dict) or set(values)-OBSTACLE_EDITABLE:
            raise ValueError("网页只允许修改已列出的障碍判定参数")
        with self._lock:
            candidate = deepcopy(self._config)
            candidate["obstacle"].update(values)
            validate_obstacle(candidate["obstacle"])
            if candidate != self._config:
                self._store(candidate)
                self._obstacle_revision += 1
            return deepcopy(self._config["obstacle"]), self._obstacle_revision

    def snapshot(self):
        with self._lock:
            return deepcopy(self._config), self._revision

    def update(self, values):
        if not isinstance(values, dict) or set(values)-EDITABLE:
            raise ValueError("网页仅允许修改相似度、连续帧数、超时及外层 ROI")
        with self._lock:
            candidate = deepcopy(self._config)
            candidate["recognition"].update(values)
            validate(candidate)
            if candidate != self._config:
                self._store(candidate)
                self._revision += 1
            return deepcopy(self._config["recognition"]), self._revision


def pwm14m0_selected(chip, dt_root=Path("/sys/firmware/devicetree/base")):
    pinctrl = Path(chip)/"device/of_node/pinctrl-0"
    route = Path(dt_root)/"pinctrl/pwm14/pwm14m0-pins/phandle"
    return pinctrl.is_file() and route.is_file() and pinctrl.read_bytes() == route.read_bytes()


def preflight(settings, root, navigation_device):
    """仅检查路径与明确确认项；不打开相机、不导出 PWM、不发送命令。"""
    config, _ = settings.snapshot()
    camera, servo = config["camera"], config["servo"]
    device = camera["device"]
    if not isinstance(device, str) or not device.strip():
        raise ValueError("必须明确配置侧视相机 device，不能猜测设备号")
    if Path(device).resolve() == Path(navigation_device).resolve():
        raise ValueError("侧视相机不能与导航相机占用同一个设备")
    if not Path(device).exists():
        raise ValueError("侧视相机设备不存在")
    if not servo["hardware_verified"]:
        raise ValueError("必须先核实 PWM 接线、overlay、芯片和安全端点，再确认 hardware_verified")
    import yaml
    servo_path = Path(root)/servo["config"]
    pwm = yaml.safe_load(servo_path.read_text(encoding="utf-8"))
    chip = Path(pwm["pwmchip"])
    if not chip.is_dir():
        raise ValueError("servo.yaml 中的 PWM 芯片不存在；不会自动修改系统配置")
    if pwm.get("pwm_route") != servo["pwm_route"]:
        raise ValueError("servo.yaml 与侧视配置的 PWM14_M0 路由不一致")
    # RK3588 live DT 的 pinctrl phandle 必须指向 PWM14_M0，不按 pwmchip 编号猜控制器。
    if not pwm14m0_selected(chip):
        raise ValueError("所选 PWM 芯片的当前 pinctrl 不是 PWM14_M0")
    for key in ("channel", "period_us", "min_pulse_us", "max_pulse_us"):
        if isinstance(pwm[key], bool) or not isinstance(pwm[key], int):
            raise ValueError("PWM 配置必须使用整数")
    if pwm["channel"] < 0 or not 0 < pwm["min_pulse_us"] < pwm["max_pulse_us"] < pwm["period_us"]:
        raise ValueError("PWM 通道或脉宽配置不合法")
    if not (Path(root)/servo["binary"]).is_file():
        raise ValueError("缺少支持 --stdin 的 servo_cli；需先构建，不会自动安装")
    return config
