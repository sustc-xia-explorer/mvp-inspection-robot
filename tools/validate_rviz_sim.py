#!/usr/bin/env python3
"""Exercise the interactive world's route, scans and C++ planner without ROS."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import time


def main():
    root = Path(__file__).resolve().parent.parent
    scripts = root/'neupan_cpp_ros1/scripts'
    spec = importlib.util.spec_from_file_location('rviz_sim', scripts/'rviz_sim.py')
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    from simulation_test import CoreBackend, clean_json, path_error, plot_case
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', type=Path, default=root/'neupan_cpp_ros1/config/scenario_corridor.yaml')
    parser.add_argument('--config', type=Path, default=root/'neupan_cpp_ros1/config/fw_mini_balanced.yaml')
    parser.add_argument('--model', type=Path, default=root/'neupan_cpp_ros1/models/fw_mini.bin')
    parser.add_argument('--binary', type=Path, default=Path(os.environ.get('NEUPAN_BUILD_ROOT', Path.home()/'neupan_noetic_build'))/'libneupan/neupan_sim_backend')
    parser.add_argument('--goal', type=float, nargs=2)
    parser.add_argument('--seconds', type=float, default=420.)
    parser.add_argument('--output', type=Path, default=Path.home()/'neupan_sim_results/rviz_core')
    args = parser.parse_args()
    world = sim.World(args.scenario, args.config)
    goal = args.goal or world.config['default_goal']
    path = world.route(world.config['initial_pose'], goal)
    scene = dict(world.scene, path=path)
    folder = sim.create_output_directory(args.output.expanduser())
    print('Writing results to:', folder, flush=True)
    (folder/'planner.yaml').write_text(args.config.read_text())
    (folder/'reference.json').write_text(json.dumps(path)+'\n')
    (folder/'manifest.json').write_text(json.dumps(dict(
        config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
        scenario=str(args.scenario.resolve()), goal=goal), indent=2)+'\n')
    state, rows, settled = list(world.config['initial_pose']), [], 0.
    args.dt = float(world.planner['step_time'])
    backend = CoreBackend(args, scene, folder)
    minimum, arrived, collision, started = float('inf'), False, False, time.monotonic()
    with (folder/'trajectory.csv').open('w', newline='') as stream:
        writer = None
        try:
            for index in range(math.ceil(args.seconds/args.dt)):
                ranges, points = world.scan(state)
                v, w, info = backend.step(state, ranges, points)
                if not all(math.isfinite(x) for x in (v, w)):
                    raise RuntimeError('Non-finite planner command')
                for _ in range(5):
                    state = sim.integrate(state, v, w, args.dt/5)
                    gap = world.clearance(state)
                    minimum = min(minimum, gap)
                    collision |= gap <= 0
                arrived = bool(info['arrive'])
                settled = settled+args.dt if arrived and abs(v) < .01 and abs(w) < .02 else 0.
                row = dict(time=(index+1)*args.dt, x=state[0], y=state[1], yaw=state[2], v=v, w=w,
                           clearance=gap, cross_track=path_error(state, path),
                           goal_error=math.dist(state[:2], goal), planner_ms=info['planner_ms'],
                           solved=info['solved'], arrive=arrived)
                rows.append(row)
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                stream.flush()
                if index % 100 == 0:
                    print('t={:.1f}s pose=({:.2f},{:.2f}) goal_error={:.3f} clearance={:.3f}'.format(
                        row['time'], *state[:2], row['goal_error'], gap), flush=True)
                if collision or settled >= 1:
                    break
        finally:
            backend.close()
    summary = dict(passed=arrived and settled >= 1 and not collision and minimum >= .03,
                   arrived=arrived, collision=collision, min_clearance=minimum,
                   goal_error=math.dist(state[:2], goal), final_pose=state,
                   simulated_seconds=len(rows)*args.dt, wall_seconds=time.monotonic()-started)
    (folder/'summary.json').write_text(json.dumps(clean_json(summary), indent=2)+'\n')
    plot_case(folder, scene, rows, world.planner)
    print(json.dumps(clean_json(summary), indent=2), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
