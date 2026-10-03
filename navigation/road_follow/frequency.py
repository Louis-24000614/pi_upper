"""保存、对照、恢复板端合法频率；只操作 CPU、DDR、NPU 已有 sysfs 接口。"""
import argparse
import json
from pathlib import Path
import time


def read(path):
    return path.read_text().strip()


def discover():
    """不写死设备地址；GPU 不参与本轮对照，避免引入额外变量。"""
    domains = []
    for path in sorted(Path('/sys/devices/system/cpu/cpufreq').glob('policy*')):
        domains.append((path, 'cpu', 'scaling_'))
    for path in sorted(Path('/sys/class/devfreq').glob('*')):
        name = read(path/'name')
        if name == 'dmc' or name.endswith('.npu'):
            domains.append((path, 'ddr' if name == 'dmc' else 'npu', ''))
    if not domains or not any(kind == 'npu' for _, kind, _ in domains):
        raise RuntimeError('未发现完整的 CPU/DDR/NPU 调频接口')
    return domains


def snapshot():
    rows = []
    for path, kind, prefix in discover():
        available = 'scaling_available_frequencies' if kind == 'cpu' else 'available_frequencies'
        values = sorted({int(value) for value in read(path/available).split()})
        if not values:
            raise RuntimeError(f'没有可确认的合法频率: {path}')
        rows.append({'path': str(path), 'kind': kind, 'prefix': prefix,
                     'min': int(read(path/(prefix+'min_freq'))),
                     'max': int(read(path/(prefix+'max_freq'))),
                     'governor': read(path/(prefix+'governor')),
                     'available': values})
    return {'schema': 1, 'saved_epoch_s': time.time(), 'domains': rows}


def validate(state):
    expected = {(str(p), kind, prefix) for p, kind, prefix in discover()}
    actual = {(row['path'], row['kind'], row['prefix']) for row in state['domains']}
    if state.get('schema') != 1 or actual != expected:
        raise RuntimeError('快照不属于当前板端调频接口，拒绝写入')
    for row in state['domains']:
        if not 0 <= row['min'] <= row['max'] or not row['available']:
            raise RuntimeError('快照频率区间无效')


def set_domain(row, minimum, maximum):
    path, prefix = Path(row['path']), row['prefix']
    min_path, max_path = path/(prefix+'min_freq'), path/(prefix+'max_freq')
    # 先降低下限，再调整上限，最后提高下限；全过程避免 min > max。
    current_min = int(read(min_path))
    min_path.write_text(str(min(current_min, minimum)))
    max_path.write_text(str(maximum))
    min_path.write_text(str(minimum))
    if int(read(min_path)) != minimum or int(read(max_path)) != maximum:
        raise RuntimeError(f'频率写入未生效: {path}')


def restore(state):
    """即使一个域恢复失败也继续其他域，最后明确报告，不能静默忽略。"""
    validate(state)
    errors = []
    for row in state['domains']:
        try:
            set_domain(row, row['min'], row['max'])
            path = Path(row['path'])/(row['prefix']+'governor')
            path.write_text(row['governor'])
            if read(path) != row['governor']:
                raise RuntimeError(f'governor 未恢复: {path}')
        except OSError as exc:
            errors.append(str(exc))
        except RuntimeError as exc:
            errors.append(str(exc))
    if errors:
        raise RuntimeError('恢复失败: ' + '; '.join(errors))


def apply(state, policy):
    # 每一轮都从同一原配置开始；DDR 对照不能夹带 CPU/NPU 策略变化。
    restore(state)
    if policy == 'original':
        return
    try:
        for row in state['domains']:
            if policy == 'ddr-max' and row['kind'] != 'ddr':
                continue
            # 只取系统暴露且不超过原 max 限额的合法值，不改 OPP/电压/温控。
            choices = [value for value in row['available'] if value <= row['max']]
            if not choices:
                raise RuntimeError('合法频率列表与原上限没有交集')
            maximum = max(choices)
            set_domain(row, maximum, maximum)
    except BaseException:
        restore(state)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('save', 'apply', 'restore', 'show'))
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--policy', choices=('original', 'ddr-max', 'all-max'), default='original')
    args = parser.parse_args()
    if args.action == 'save':
        state = snapshot()
        with args.state.open('x', encoding='utf-8') as target:
            json.dump(state, target, indent=2)
    else:
        state = json.loads(args.state.read_text())
        if args.action == 'apply':
            apply(state, args.policy)
        elif args.action == 'restore':
            restore(state)
    print(json.dumps(snapshot(), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
