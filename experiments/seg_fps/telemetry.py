"""只读采样系统资源；不调整调频策略，避免把频率变化算作代码收益。"""
from pathlib import Path
import resource
import time
import numpy as np


def snapshot() -> dict:
    values = {}
    patterns = ["/sys/class/thermal/thermal_zone*/temp",
                "/sys/devices/system/cpu/cpufreq/policy*/scaling_cur_freq",
                "/sys/class/devfreq/*/cur_freq"]
    import glob
    for pattern in patterns:
        for name in glob.glob(pattern):
            try:
                values[name] = Path(name).read_text().strip()
            except OSError:
                pass
    for name in ["/sys/kernel/debug/rknpu/load", "/proc/rknpu/load", "/proc/self/status"]:
        try:
            contents = Path(name).read_text()
            values[name] = ("\n".join(x for x in contents.splitlines()
                                    if x.startswith(("VmRSS:", "VmHWM:")))
                            if name.endswith("status") else contents.strip())
        except OSError:
            values[name] = "unavailable"
    return values


def summarize(samples) -> dict:
    data = np.asarray(samples, dtype=np.float64)
    if not len(data):
        return {}
    return {"count": int(len(data)), "mean": float(data.mean()), "p50": float(np.percentile(data, 50)),
            "p95": float(np.percentile(data, 95)), "max": float(data.max())}
