"""对真实流水线做交替 A/B；每轮独立进程，保留原始结果和配对变化。"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('video','model','config','output-dir'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--pairs', type=int, default=3)
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--source-fps', type=float, default=30)
    p.add_argument('--driver-capacity', type=int, default=4)
    p.add_argument('--strategies', nargs=2, choices=('ordered','latest','latest-ordered'),
                   default=('ordered','latest'), help='A/B 两种策略；默认保持原对照')
    args = p.parse_args()
    if args.pairs < 1 or args.seconds <= 0:
        p.error('配对数和时长必须大于零')
    if args.strategies[0] == args.strategies[1]:
        p.error('A/B 必须选择不同策略')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    pairs = []
    for index in range(args.pairs):
        pair = {}
        for strategy in (args.strategies if index%2 == 0 else args.strategies[::-1]):
            output = args.output_dir/f'{index+1}_{strategy}.json'
            command = [sys.executable, '-m', 'experiments.seg_fps.latest_benchmark',
                       '--strategy', strategy, '--output', str(output),
                       '--seconds', str(args.seconds), '--source-fps', str(args.source_fps),
                       '--driver-capacity', str(args.driver_capacity)]
            for name in ('video','model','config'):
                command.extend(['--'+name, str(getattr(args,name))])
            with output.with_suffix('.log').open('w') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            data = json.loads(output.read_text())
            if not data['passed']:
                raise RuntimeError('对照子进程记录异常')
            pair[strategy] = {name:data[name] for name in ('fps','consumed_fps','metrics',
                'cpu_percent','rss_peak_kib','camera_dropped','stream_stats','rejections')}
            print('PAIR', args.source_fps, index+1, strategy,
                  round(data['consumed_fps'],3), 'fresh FPS',
                  round(data['metrics']['age_ms']['p95'],2),'age P95', flush=True)
        pairs.append(pair)
    baseline, candidate = args.strategies
    gains = [(row[candidate]['consumed_fps']/row[baseline]['consumed_fps']-1)*100
             for row in pairs if row[baseline]['consumed_fps']]
    age_changes = [(row[candidate]['metrics']['age_ms']['p95']/
                   row[baseline]['metrics']['age_ms']['p95']-1)*100 for row in pairs]
    report = dict(source_fps=args.source_fps, seconds_per_side=args.seconds,
        baseline=baseline, candidate=candidate,
        driver_capacity=args.driver_capacity, pairs=pairs,
        fresh_fps_gain_pct=dict(mean=float(np.mean(gains)), std=float(np.std(gains)),
                               min=float(np.min(gains)), max=float(np.max(gains))),
        age_p95_change_pct=dict(mean=float(np.mean(age_changes)),std=float(np.std(age_changes))))
    (args.output_dir/'summary.json').write_text(json.dumps(report, indent=2))
    print('SUMMARY', args.source_fps, report['fresh_fps_gain_pct'],
          report['age_p95_change_pct'], flush=True)


if __name__ == '__main__':
    main()
