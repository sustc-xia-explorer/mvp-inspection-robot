#!/usr/bin/env python3
"""Validate turn-angle and corridor perturbations with the unchanged C++ core."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path

import yaml


def scenarios():
    def scene(path, yaw=0.0, walls=()):
        return dict(path=path, yaw=yaw, circles=[], walls=list(walls), blocked=False)

    result = {}
    for degrees in [-120, -90, -60, 60, 120]:
        angle = math.radians(degrees)
        name = 'angle_' + ('right' if degrees < 0 else 'left') + str(abs(degrees))
        result[name] = scene([(0, 0), (1, 0),
                              (1 + math.cos(angle), math.sin(angle))])
    result['rotated_left90'] = scene([(0, 0), (0, 1), (-1, 1)], math.pi / 2)
    for width in [.86, 1.06, 1.26]:
        walls = [(-2, -width / 2, 5, -width / 2),
                 (-2, width / 2, 5, width / 2)]
        for degrees in [-5, 5]:
            name = 'corridor_w{:03d}_yaw{:+d}'.format(round(width * 100), degrees)
            result[name] = scene([(0, 0), (3, 0)], math.radians(degrees), walls)
    for offset in [-.05, .05]:
        # Shift both walls together; the reference remains y=0, so it is offset
        # from the corridor centre. This is not an initial path-tracking error.
        walls = [(-2, -.53 + offset, 5, -.53 + offset),
                 (-2, .53 + offset, 5, .53 + offset)]
        name = 'corridor_offset{:+03d}cm'.format(round(offset * 100))
        result[name] = scene([(0, 0), (3, 0)], walls=walls)
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'simulation_test', root / 'neupan_cpp_ros1/scripts/simulation_test.py')
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=root / 'neupan_cpp_ros1/config/fw_mini_balanced.yaml')
    parser.add_argument('--model', type=Path,
                        default=root / 'neupan_cpp_ros1/models/fw_mini.bin')
    parser.add_argument('--binary', type=Path, default=Path(os.environ.get(
        'NEUPAN_BUILD_ROOT', Path.home() / 'neupan_noetic_build')) /
        'libneupan/neupan_sim_backend')
    parser.add_argument('--output', type=Path,
                        default=Path.home() / 'neupan_sim_results/balanced_extended')
    parser.add_argument('--cases', default='all')
    parser.add_argument('--sensor', type=float, nargs=3, default=[.2, .05, .35])
    parser.add_argument('--goal-tolerance', type=float, default=.15)
    parser.add_argument('--max-cross-track', type=float, default=.35)
    parser.add_argument('--rms-cross-track', type=float, default=.15)
    parser.add_argument('--max-overshoot', type=float, default=.15)
    parser.add_argument('--min-clearance', type=float, default=.03)
    args = parser.parse_args()
    for key in ('config', 'model', 'binary', 'output'):
        setattr(args, key, getattr(args, key).expanduser().resolve())
    for key in ('config', 'model', 'binary'):
        if not getattr(args, key).is_file():
            parser.error('missing {}: {}'.format(key, getattr(args, key)))
    cfg = yaml.safe_load(args.config.read_text())
    args.dt = float(cfg['step_time'])
    if cfg['robot']['kinematics'] != 'diff' or cfg.get('ipath', {}).get('loop', False):
        parser.error('requires diff and loop=false')
    if not 0 < args.dt <= .5:
        parser.error('step_time must be in (0, 0.5]')
    cases = scenarios()
    names = list(cases) if args.cases == 'all' else args.cases.split(',')
    if not names or any(name not in cases for name in names):
        parser.error('valid cases: ' + ','.join(cases))
    args.backend, args.checkpoint, args.timeout = 'core', None, None
    args.output = sim.create_output_directory(args.output)
    print('Writing results to:', args.output, flush=True)
    run = {key: str(value) if isinstance(value, Path) else value
           for key, value in vars(args).items()}
    for key in ('config', 'model', 'binary'):
        run[key + '_sha256'] = hashlib.sha256(getattr(args, key).read_bytes()).hexdigest()
    (args.output / 'run.json').write_text(json.dumps(run, indent=2) + '\n')
    (args.output / 'config_snapshot.yaml').write_text(args.config.read_text())
    (args.output / 'scenes.json').write_text(json.dumps(
        {name: cases[name] for name in names}, indent=2) + '\n')
    results = []
    for name in names:
        results.append(sim.run_case(args, cfg, name, cases[name]))
        (args.output / 'summary.json').write_text(json.dumps(
            sim.clean_json(results), indent=2, allow_nan=False) + '\n')
    with (args.output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    print('Extended cases: {}/{} passed'.format(
        sum(result['passed'] for result in results), len(results)))
    return 0 if all(result['passed'] for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
