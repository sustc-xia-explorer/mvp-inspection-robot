#!/usr/bin/env python3
"""Compare two independent closed-loop runs; this is not identical-input replay."""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path

import yaml


def main():
    root = Path(__file__).resolve().parent.parent
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('cpp', type=Path)
    p.add_argument('python', type=Path)
    p.add_argument('--config', type=Path, default=root/'neupan_cpp_ros1/config/fw_mini.yaml')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    left = {x['case']: x for x in json.loads((args.cpp/'summary.json').read_text())}
    right = {x['case']: x for x in json.loads((args.python/'summary.json').read_text())}
    common = [name for name in left if name in right]
    if not common:
        raise ValueError('no common cases')
    table, traces = [], {}
    for name in common:
        a = list(csv.DictReader((args.cpp/name/'trajectory.csv').open()))
        b = list(csv.DictReader((args.python/name/'trajectory.csv').open()))
        pairs = list(zip(a, b))
        if not pairs or any(abs(float(x['time'])-float(y['time'])) > 1e-6 for x, y in pairs):
            raise ValueError('different sample times; resample before comparing '+name)
        position = [math.hypot(float(x['x'])-float(y['x']), float(x['y'])-float(y['y'])) for x, y in pairs]
        angles = [abs(math.atan2(math.sin(float(x['yaw'])-float(y['yaw'])),
                                math.cos(float(x['yaw'])-float(y['yaw'])))) for x, y in pairs]
        table.append(dict(case=name, cpp_pass=left[name]['passed'], python_pass=right[name]['passed'],
                          cpp_goal_error=left[name]['goal_error'], python_goal_error=right[name]['goal_error'],
                          cpp_collision=left[name]['collision'], python_collision=right[name]['collision'],
                          max_position_separation=max(position), max_heading_separation_rad=max(angles)))
        traces[name] = a, b
    with (args.output/'comparison.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        w.writerows(table)
    (args.output/'comparison.json').write_text(json.dumps(table, indent=2)+'\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
    except ImportError:
        print('CSV/JSON written; install matplotlib to generate the figure.')
        return
    spec = importlib.util.spec_from_file_location('simulation', root/'neupan_cpp_ros1/scripts/simulation_test.py')
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    cfg = yaml.safe_load(args.config.read_text())
    scenes = sim.scenarios(cfg)
    fig, axes = plt.subplots(math.ceil(len(common)/3), 3, figsize=(12, 3.4*math.ceil(len(common)/3)), squeeze=False)
    for ax, name in zip(axes.flat, common):
        a, b = traces[name]
        scene = scenes[name]
        ax.plot(*zip(*scene['path']), ':', color='gray', label='reference')
        ax.plot([float(x['x']) for x in a], [float(x['y']) for x in a], label='C++')
        ax.plot([float(x['x']) for x in b], [float(x['y']) for x in b], '--', label='Python')
        for x, y, radius in scene['circles']:
            ax.add_patch(Circle((x, y), radius, color='tab:red', alpha=.5))
        for x, y, xx, yy in scene['walls']:
            ax.plot([x, xx], [y, yy], color='tab:red')
        visible = scene['path']+[(float(x['x']), float(x['y'])) for x in a+b]
        xs, ys = zip(*visible)
        margin = max(.5, cfg['robot']['width']/2+.5)
        ax.set_xlim(min(xs)-margin, max(xs)+margin)
        ax.set_ylim(min(ys)-margin, max(ys)+margin)
        ax.set_aspect('equal', adjustable='box')
        status = '{} / {}'.format('PASS' if left[name]['passed'] else 'FAIL',
                                  'PASS' if right[name]['passed'] else 'FAIL')
        ax.set_title(name+'  '+status, fontsize=10)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
    for ax in list(axes.flat)[len(common):]:
        ax.set_visible(False)
    axes.flat[0].legend()
    fig.suptitle('FW-mini: independent ideal closed-loop runs (no hardware)')
    fig.tight_layout()
    fig.savefig(args.output/'comparison.png', dpi=170)
    plt.close(fig)
    print('Comparison:', args.output)


if __name__ == '__main__':
    main()
