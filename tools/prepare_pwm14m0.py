"""核对 live DT、导出 PWM0 并授予权限；不配置周期、角度或 enable。"""
import argparse
import errno
import os
from pathlib import Path
import pwd
import time
import uuid


CONTROL_NODES = ("period", "duty_cycle", "polarity", "enable")


def find_chip(sysfs_root, dt_root):
    route = Path(dt_root)/"pinctrl/pwm14/pwm14m0-pins/phandle"
    expected = route.read_bytes()
    if len(expected) != 4:
        raise ValueError("PWM14_M0 的设备树 phandle 无效")
    matches = []
    for chip in Path(sysfs_root).glob("pwmchip*"):
        pinctrl = chip/"device/of_node/pinctrl-0"
        if pinctrl.is_file() and pinctrl.read_bytes() == expected:
            if int((chip/"npwm").read_text().strip()) < 1:
                raise ValueError("PWM14_M0 没有通道 0")
            matches.append(chip)
    if len(matches) != 1:
        raise ValueError("必须且只能找到一个设备树指向 PWM14_M0 的芯片")
    return matches[0]


def prepare(sysfs_root, dt_root, link, username="orangepi", timeout_s=5, exporter=None):
    chip = find_chip(sysfs_root, dt_root)
    account = pwd.getpwnam(username)
    link = Path(link)
    if link.exists() and not link.is_symlink():
        raise ValueError("PWM 固定路径已被普通文件或目录占用")
    channel = chip/"pwm0"
    exported = False
    if not channel.exists():
        try:
            if exporter is None:
                (chip/"export").write_text("0", encoding="ascii")
            else:
                exporter(chip)
            exported = True
        except OSError as exc:
            if exc.errno != errno.EBUSY:
                raise
    deadline = time.monotonic()+timeout_s
    nodes = [channel/name for name in CONTROL_NODES]
    while not all(path.is_file() for path in nodes):
        if time.monotonic() >= deadline:
            raise TimeoutError("导出 PWM 通道后未出现完整的四个控制节点")
        time.sleep(.05)
    if any(path.is_symlink() for path in nodes):
        raise ValueError("PWM 控制节点不能是符号链接")
    for path in nodes:
        os.chown(path, account.pw_uid, account.pw_gid)
        os.chmod(path, 0o644)
    link.parent.mkdir(parents=True, exist_ok=True)
    if not link.is_symlink() or link.resolve() != chip.resolve():
        temporary = link.with_name("."+link.name+"-"+uuid.uuid4().hex)
        try:
            temporary.symlink_to(chip.resolve())
            os.replace(temporary, link)
        finally:
            if temporary.is_symlink():
                temporary.unlink()
    return chip, exported


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user", default="orangepi")
    parser.add_argument("--link", type=Path, default=Path("/run/pi-upper-pwm14m0"))
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error("通道导出和权限恢复需以 root 运行")
    chip, exported = prepare("/sys/class/pwm", "/sys/firmware/devicetree/base", args.link, args.user)
    print(f"已核对 PWM14_M0 → {chip}；通道0{'已导出' if exported else '已存在'}，"
          f"四个控制节点可由 {args.user} 写入；固定路径 {args.link}", flush=True)


if __name__ == "__main__":
    main()
