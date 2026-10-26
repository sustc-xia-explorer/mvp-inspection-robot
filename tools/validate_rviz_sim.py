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
    from simulation_test import CoreBackend, clean_json, path_error, plot_case, limit_command
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
    applied, max_rates, max_target_rates, last_target = (0., 0.), [0., 0.], [0., 0.], (0., 0.)
    robot = world.robot
    emergency_stops = 0
    minimum, arrived, collision, started = float('inf'), False, False, time.monotonic()
    with (folder/'trajectory.csv').open('w', newline='') as stream:
        writer = None
        try:
            for index in range(math.ceil(args.seconds/args.dt)):
                ranges, points = world.scan(state)
                v, w, info = backend.step(state, ranges, points)
                if not all(math.isfinite(x) for x in (v, w)):
                    raise RuntimeError('Non-finite planner command')
                target = (v, w)
                emergency_stops += bool(info.get('stop'))
                for j in range(2):
                    max_target_rates[j] = max(max_target_rates[j], abs(target[j]-last_target[j])/args.dt)
                last_target = target
                rates = [0., 0.]
                for _ in range(5):
                    previous = applied
                    applied = limit_command(target, applied, robot['max_speed'], robot['max_acce'], args.dt/5)
                    for j in range(2):
                        rates[j] = max(rates[j], abs(applied[j]-previous[j])/(args.dt/5))
                        max_rates[j] = max(max_rates[j], rates[j])
                    state = sim.integrate(state, *applied, args.dt/5)
                    gap = world.clearance(state)
                    minimum = min(minimum, gap)
                    collision |= gap <= 0
                backend.set_applied(*applied)
                v, w = applied
                arrived = bool(info['arrive'])
                settled = settled+args.dt if arrived and abs(v) < .01 and abs(w) < .02 else 0.
                row = dict(time=(index+1)*args.dt, x=state[0], y=state[1], yaw=state[2], v=v, w=w,
                           requested_v=target[0], requested_w=target[1], raw_v=info['raw_v'], raw_w=info['raw_w'],
                           acceleration=rates[0], angular_acceleration=rates[1],
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
    limits_passed = all(max_rates[j] <= robot['max_acce'][j]+1e-8 and
                        max_target_rates[j] <= robot['max_acce'][j]+1e-8 for j in range(2))
    summary = dict(passed=arrived and settled >= 1 and not collision and minimum >= .03 and limits_passed,
                   acceleration_passed=limits_passed, max_command_acceleration=max_rates[0],
                   max_command_angular_acceleration=max_rates[1],
                   max_planner_command_acceleration=max_target_rates[0],
                   max_planner_command_angular_acceleration=max_target_rates[1], emergency_stops=emergency_stops,
                   arrived=arrived, collision=collision, min_clearance=minimum,
                   goal_error=math.dist(state[:2], goal), final_pose=state,
                   simulated_seconds=len(rows)*args.dt, wall_seconds=time.monotonic()-started)
    (folder/'summary.json').write_text(json.dumps(clean_json(summary), indent=2)+'\n')
    plot_case(folder, scene, rows, world.planner)
    print(json.dumps(clean_json(summary), indent=2), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
