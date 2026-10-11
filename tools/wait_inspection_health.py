"""等待本机已有识别接口就绪，不采集相机或提交识别图片。"""
import argparse
import json
import time
from urllib.request import urlopen
from urllib.error import URLError


PORTS = {"face": 20004, "knife": 20005}
NAMES = {"face": "人脸", "knife": "刀具"}


def health_ready(service, payload):
    if not isinstance(payload, dict) or payload.get("status") != "ok":
        return False
    return payload.get("db_exists") is True if service == "face" else payload.get("ready") is True


def wait_ready(service, timeout_s):
    deadline = time.monotonic()+timeout_s
    while time.monotonic() < deadline:
        try:
            with urlopen(f"http://127.0.0.1:{PORTS[service]}/health", timeout=2) as response:
                payload = json.load(response)
            if health_ready(service, payload):
                print(f"{NAMES[service]}服务已就绪：127.0.0.1:{PORTS[service]}", flush=True)
                return
        except (OSError, URLError, ValueError):
            pass
        time.sleep(.5)
    raise RuntimeError(f"{NAMES[service]}服务未在期限内就绪，请检查模型、数据库与服务日志")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", choices=PORTS, required=True)
    parser.add_argument("--timeout", type=float, default=90)
    args = parser.parse_args()
    if not 0 < args.timeout <= 300:
        parser.error("就绪等待期限须为 0～300 秒")
    wait_ready(args.service, args.timeout)


if __name__ == "__main__":
    main()
