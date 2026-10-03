"""临时固定合法频率运行用户命令；命令保持普通用户权限，退出恢复原配置。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from road_follow.frequency import snapshot

MANAGED_ENV = 'ROAD_FREQUENCY_MANAGED'


def needs_frequency_guard(model):
    """仅实际 RK3588 的 RKNN 启动自动调频；桌面/ONNX 调试不能碰系统频率。"""
    if sys.platform != 'linux' or os.environ.get(MANAGED_ENV) == '1':
        return False
    if model.suffix != '.rknn' or not model.is_file():
        return False
    try:
        return b'rockchip,rk3588' in Path('/proc/device-tree/compatible').read_bytes()
    except OSError:
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=('ddr-max', 'all-max'), default='all-max')
    parser.add_argument('--state', type=Path,
                        help='新的快照文件，禁止覆盖；不指定时在 /tmp 生成唯一文件')
    parser.add_argument('--sudo-stdin', action='store_true',
                        help='批处理时从标准输入读取一次 sudo 密码，密码不进入参数或日志')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('在 -- 后指定普通用户运行的命令')
    return run_guarded(command, policy=args.policy, state=args.state, sudo_stdin=args.sudo_stdin)


def run_guarded(command, *, policy='all-max', state=None, sudo_stdin=None):
    """父进程负责调频与恢复，子进程负责模型/相机/串口；信号也覆盖初始化阶段。"""
    if sys.platform != 'linux' or os.geteuid() == 0:
        raise RuntimeError('请在板端以普通用户启动；仅调频子命令使用 sudo')
    state = state or Path(f'/tmp/road_frequency_{time.time_ns()}_{os.getpid()}.json')
    if state.exists():
        raise RuntimeError('快照已存在，不能覆盖原配置')
    authenticated = False
    if sudo_stdin is None:
        # 非终端 sudo 票据可能按父 PID 隔离，不能假设父 shell 已认证就能复用。
        # 优先无交互验证；需要认证时，终端沿用 sudo 提示，管道则从 stdin 读一次。
        # 只读本次系统认证，密码不会传入命令参数、环境变量或保存到磁盘。
        probe = subprocess.run(['sudo','-n','-v'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        authenticated = probe.returncode == 0
        sudo_stdin = not sys.stdin.isatty()
    if authenticated:
        pass
    elif sudo_stdin:
        password = sys.stdin.buffer.readline(4096)
        if not password:
            raise RuntimeError('没有读取到 sudo 密码')
        subprocess.run(['sudo', '-S', '-p', '', '-v'], input=password, check=True)
        password = None
    else:
        subprocess.run(['sudo', '-v'], check=True)
    state.parent.mkdir(parents=True, exist_ok=True)
    with state.open('x') as target:
        json.dump(snapshot(), target, indent=2)
    utility = Path(__file__).with_name('frequency.py').resolve()
    stopped, renewal_errors = threading.Event(), []
    child, received_signals = None, []

    def renew():
        # 只续期本进程的 sudo 票据；避免长跑后无法非交互恢复原频率。
        while not stopped.wait(30):
            try:
                result = subprocess.run(['sudo', '-n', '-v'], capture_output=True, timeout=5)
            except (OSError, subprocess.TimeoutExpired) as exc:
                renewal_errors.append(str(exc))
                return
            if result.returncode:
                renewal_errors.append(result.stderr.decode(errors='replace'))
                return

    def root_frequency(action, suffix):
        argv = ['sudo', '-n', sys.executable, str(utility), action, '--state', str(state)]
        if action == 'apply':
            argv.extend(['--policy', policy])
        with state.with_suffix(suffix).open('w') as target:
            subprocess.run(argv, stdout=target, check=True)

    def forward(signum, _frame):
        received_signals.append(signum)
        if child is not None:
            try:
                # HUP 转成 TERM，让道路主程序正常执行停止串口及模型清理。
                os.killpg(child.pid, signal.SIGTERM if signum == signal.SIGHUP else signum)
            except ProcessLookupError:
                pass

    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {signum: signal.signal(signum, forward) for signum in signals}
    keeper = threading.Thread(target=renew, name='frequency-sudo-renew', daemon=True)
    keeper.start()
    try:
        root_frequency('apply', '.applied.json')
        if received_signals:
            return 128+received_signals[-1]
        # 新进程组便于将停止请求转交整条命令；相机/NPU/串口仍由原用户程序管理。
        # 自动启动与旧实验启动器共用这层保护。标记只在本次子进程继承，防止
        # road_follow 自动入口再次启动保护器、重复调频或错误覆盖恢复快照。
        child_env = os.environ.copy()
        child_env[MANAGED_ENV] = '1'
        child = subprocess.Popen(command, start_new_session=True, env=child_env)
        if received_signals:
            forward(received_signals[-1], None)
        code = child.wait()
        return code if code >= 0 else 128-code
    finally:
        try:
            root_frequency('restore', '.restored.json')
            before = json.loads(state.read_text())['domains']
            after = json.loads(state.with_suffix('.restored.json').read_text())['domains']
            fields = ('path', 'kind', 'prefix', 'min', 'max', 'governor')
            if len(before) != len(after) or any(any(a[k] != b[k] for k in fields)
                                               for a, b in zip(before, after)):
                raise RuntimeError('频率恢复读回不一致，请检查快照')
            print('FREQUENCY_RESTORED', state, flush=True)
        finally:
            stopped.set()
            keeper.join(timeout=10)
            for signum, handler in previous.items():
                signal.signal(signum, handler)
            if renewal_errors:
                print('sudo 续期失败记录:', renewal_errors, file=sys.stderr)


if __name__ == '__main__':
    raise SystemExit(main())
