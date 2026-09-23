"""五张实拍图板端验收：固定模板，记录逐次结果、耗时与三核原始占用。"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import statistics
import sys
import threading
import time

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    sys.path.insert(0, str(root))
    from vision.knife.engine import KnifeRecognizer
    samples = []
    stop = threading.Event()
    phase = {'name': 'load'}
    started = time.monotonic()

    def sample_load():
        # proc值本身是驱动的统计窗口，不是瞬时利用率；保留原文供复核。
        while not stop.is_set():
            try:
                raw = Path('/proc/rknpu/load').read_text().strip()
                cores = {f'core{k}': int(v) for k, v in re.findall(r'Core(\d+):\s*(\d+)%', raw)}
                samples.append({'elapsed_s': time.monotonic()-started,
                                'phase': phase['name'], 'raw': raw, **cores})
            except OSError as exc:
                samples.append({'error': str(exc), 'phase': phase['name']})
            stop.wait(0.2)

    thread = threading.Thread(target=sample_load, daemon=True)
    thread.start()
    rows = []
    load_started = time.perf_counter()
    try:
        with KnifeRecognizer(root/'config/knife.json') as engine:
            load_ms = (time.perf_counter()-load_started)*1000
            paths = sorted((root/'images').glob('*.png'), key=lambda p: int(p.stem))
            phase['name'] = 'warmup'
            engine.warmup(cv2.imread(str(paths[0]), cv2.IMREAD_UNCHANGED), count=2)
            # 首次计入准确率；后续重复仅检查稳定性和测时，不增加独立样本数。
            for path in paths:
                read_start = time.perf_counter()
                frame = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                decode_ms = (time.perf_counter()-read_start)*1000
                expected = f'knife_{int(path.stem):02d}'
                trials = []
                phase['name'] = path.name
                for i in range(5):
                    begin = time.perf_counter()
                    try:
                        result = engine.recognize(frame, request_id=f'{path.stem}-{i}')
                        result['correct'] = result['top1_class'] == expected
                    except Exception as exc:
                        result = {'correct': False, 'error': repr(exc)}
                    result['wall_ms'] = (time.perf_counter()-begin)*1000
                    trials.append(result)
                rows.append({'file': path.name, 'expected': expected,
                             'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                             'shape': list(frame.shape) if frame is not None else None,
                             'decode_ms': decode_ms, 'trials': trials})
    finally:
        stop.set()
        thread.join()
    active = [s for s in samples if s.get('phase', '').endswith('.png')]
    utilization = {}
    for k in ('core0', 'core1', 'core2'):
        values = [s[k] for s in active if k in s]
        utilization[k] = {'mean_percent': statistics.mean(values) if values else None,
                          'max_percent': max(values) if values else None,
                          'samples': len(values)}
    correct = sum(r['trials'][0]['correct'] for r in rows)
    payload = {'model': 'DINOv3 448 FP16', 'core_mask': '0_1_2',
               'model_sha256': engine.config['model_sha256'],
               'templates_sha256': hashlib.sha256(engine.templates_path.read_bytes()).hexdigest(),
               'load_ms': load_ms, 'warmup_count': 2, 'repeats': 5,
               'correct': correct, 'total': len(rows), 'accuracy': correct/len(rows),
               'npu_active_summary': utilization, 'images': rows, 'npu_samples': samples}
    (root/'result.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'correct': correct, 'total': len(rows), 'load_ms': load_ms,
                      'npu': utilization, 'images': [{'file':r['file'],
                       'predicted':r['trials'][0].get('top1_class'),
                       'correct':r['trials'][0]['correct'],
                       'mean_ms':statistics.mean(t['wall_ms'] for t in r['trials'])}
                       for r in rows]}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
