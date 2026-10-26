#!/usr/bin/env python3
"""Closed-loop regression: same 2-D world through ROS1, C++ core or Python.

ROS mode uses unique topics/TF frames and never publishes chassis commands.
The virtual plant is ideal diff kinematics, not a FW-mini dynamics simulator.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import threading
import time

import yaml
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from command_guard import limit_command

TAU = 2 * math.pi


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x - s*y, s*x + c*y


def sensor_pose(state, offset):
    dx, dy = rotate(offset[0], offset[1], state[2])
    return state[0]+dx, state[1]+dy, state[2]+offset[2]


def ray_range(origin, direction, circles, walls, limit):
    """Nearest positive ray intersection, with independent analytic geometry."""
    ox, oy = origin
    dx, dy = direction
    best = float('inf')
    for cx, cy, radius in circles:
        vx, vy = ox-cx, oy-cy
        b = vx*dx + vy*dy
        discriminant = b*b - (vx*vx+vy*vy-radius*radius)
        if discriminant >= 0:
            root = math.sqrt(discriminant)
            for value in (-b-root, -b+root):
                if value >= 0:
                    best = min(best, value)
    for ax, ay, bx, by in walls:
        sx, sy = bx-ax, by-ay
        denominator = dx*sy - dy*sx
        if abs(denominator) < 1e-12:
            continue
        qx, qy = ax-ox, ay-oy
        distance = (qx*sy-qy*sx)/denominator
        segment = (qx*dy-qy*dx)/denominator
        if distance >= 0 and 0 <= segment <= 1:
            best = min(best, distance)
    return best if best <= limit else float('inf')


def make_scan(state, scene, offset, beams=360, limit=5.0):
    sx, sy, yaw = sensor_pose(state, offset)
    ranges, points = [], []
    for i in range(beams):
        angle = -math.pi + TAU*i/(beams-1)
        direction = (math.cos(yaw+angle), math.sin(yaw+angle))
        distance = ray_range((sx, sy), direction, scene['circles'], scene['walls'], limit)
        # Inside the minimum range is unknown, not a fabricated distant return.
        value = distance if distance >= 0.1 else float('nan')
        ranges.append(value)
        if math.isfinite(value):
            points.append((sx+value*direction[0], sy+value*direction[1]))
    return ranges, points


def point_segment(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    square = dx*dx+dy*dy
    t = max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/square)) if square else 0.0
    return math.hypot(px-ax-t*dx, py-ay-t*dy)


def cross(ax, ay, bx, by):
    return ax*by-ay*bx


def segment_distance(a, b, c, d):
    ab, cd = (b[0]-a[0], b[1]-a[1]), (d[0]-c[0], d[1]-c[1])
    denominator = cross(*ab, *cd)
    if abs(denominator) > 1e-12:
        ac = (c[0]-a[0], c[1]-a[1])
        t, u = cross(*ac, *cd)/denominator, cross(*ac, *ab)/denominator
        if 0 <= t <= 1 and 0 <= u <= 1:
            return 0.0
    return min(point_segment(*a, *c, *d), point_segment(*b, *c, *d),
               point_segment(*c, *a, *b), point_segment(*d, *a, *b))


def clearance(state, scene, length, width, origin_offset=0.0):
    """True rectangle/obstacle distance, independent of learned DUNE distance."""
    def local(x, y):
        lx, ly = rotate(x-state[0], y-state[1], -state[2])
        return lx-origin_offset, ly
    best = float('inf')
    for cx, cy, radius in scene['circles']:
        x, y = local(cx, cy)
        qx, qy = abs(x)-length/2, abs(y)-width/2
        signed = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)
        best = min(best, signed-radius)
    vertices = [(-length/2, -width/2), (length/2, -width/2),
                (length/2, width/2), (-length/2, width/2)]
    for ax, ay, bx, by in scene['walls']:
        a, b = local(ax, ay), local(bx, by)
        inside = any(abs(x) <= length/2 and abs(y) <= width/2 for x, y in (a, b))
        value = 0.0 if inside else min(segment_distance(a, b, vertices[i], vertices[(i+1)%4])
                                      for i in range(4))
        best = min(best, value)
    return best


def path_error(state, path):
    return min(point_segment(state[0], state[1], *a, *b) for a, b in zip(path, path[1:]))


def integrate(state, v, w, dt):
    if abs(w) < 1e-10:
        return [state[0]+v*math.cos(state[2])*dt, state[1]+v*math.sin(state[2])*dt, state[2]]
    yaw = state[2]+w*dt
    return [state[0]+v/w*(math.sin(yaw)-math.sin(state[2])),
            state[1]-v/w*(math.cos(yaw)-math.cos(state[2])), math.atan2(math.sin(yaw), math.cos(yaw))]


def scenarios(cfg):
    width, length = cfg['robot']['width'], cfg['robot']['length']
    y = width/2 + 0.3 + 0.5
    wall_x = max(1.5, length/2+0.8)
    def scene(path, yaw=0.0, circles=(), walls=(), blocked=False):
        return dict(path=path, yaw=yaw, circles=list(circles), walls=list(walls), blocked=blocked)
    return {
        'straight': scene([(0, 0), (2, 0)]),
        'heading_north': scene([(0, 0), (0, 2)], math.pi/2),
        'turn_left': scene([(0, 0), (1, 0), (1, 1)]),
        'sparse_stop': scene([(0, 0), (2, 0), (4, 0)]),
        'corridor': scene([(0, 0), (3, 0)], walls=[(-2, -width/2-.35, 5, -width/2-.35),
                                                                 (-2, width/2+.35, 5, width/2+.35)]),
        'detour': scene([(0, 0), (1, 0), (1.8, y), (3.2, y), (4, 0), (5, 0)],
                        circles=[(2.5, 0, .3)]),
        'blocked_wall': scene([(0, 0), (wall_x+1, 0)], walls=[(wall_x, -20, wall_x, 20)], blocked=True),
    }


class CoreBackend:
    def __init__(self, args, scene, output):
        self.log = (output/'backend.log').open('w')
        self.proc = subprocess.Popen([str(args.binary), str(args.config), str(args.model)],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
                                     text=True, bufsize=1)
        path = scene['path']
        text = 'PATH {}\n'.format(len(path))
        for i, (x, y) in enumerate(path):
            a, b = (path[i], path[i+1]) if i+1 < len(path) else (path[i-1], path[i])
            text += '{} {} {}\n'.format(x, y, math.atan2(b[1]-a[1], b[0]-a[0]))
        self.proc.stdin.write(text)
        self.proc.stdin.flush()
        try:
            if self.read() != 'READY':
                raise RuntimeError('backend did not load path; see backend.log')
        except Exception:
            self.close()
            raise

    def read(self):
        if not select.select([self.proc.stdout], [], [], 20)[0]:
            raise RuntimeError('C++ backend timeout')
        line = self.proc.stdout.readline().strip()
        if not line:
            raise RuntimeError('C++ backend exited; see backend.log')
        return line

    def step(self, state, ranges, points):
        text = 'STEP {} {} {} {}\n'.format(*state, len(points))
        text += ''.join('{} {}\n'.format(x, y) for x, y in points)
        self.proc.stdin.write(text)
        self.proc.stdin.flush()
        values = list(map(float, self.read().split()))
        v, w, solved, stop, arrive, distance, ms = values[:7]
        self.supports_applied = len(values) == 10
        raw_v, raw_w, braking = values[7:] if len(values) == 10 else (v, w, 0)
        return v, w, dict(solved=bool(solved), stop=bool(stop), arrive=bool(arrive),
                          dune_distance=distance, planner_ms=ms, raw_v=raw_v, raw_w=raw_w, braking=bool(braking))

    def set_applied(self, v, w):
        if not self.supports_applied:
            return  # archived seven-field backends have no APPLIED protocol
        self.proc.stdin.write("APPLIED {} {}\n".format(v, w))
        self.proc.stdin.flush()
        if self.read() != "ACK":
            raise RuntimeError("applied command feedback failed")

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.log.close()


class PythonBackend:
    def __init__(self, args, scene, output):
        import numpy as np
        from neupan import neupan
        self.np = np
        cfg = yaml.safe_load(args.config.read_text())
        cfg['device'] = 'cpu'
        cfg.setdefault('pan', {})['dune_checkpoint'] = str(args.checkpoint)
        path = scene['path']
        waypoints = []
        for i, (x, y) in enumerate(path):
            a, b = (path[i], path[i+1]) if i+1 < len(path) else (path[i-1], path[i])
            waypoints.append([float(x), float(y), math.atan2(b[1]-a[1], b[0]-a[0])])
        cfg.setdefault('ipath', {})['waypoints'] = waypoints
        self.config = output/'python_config.yaml'
        self.config.write_text(yaml.safe_dump(cfg))
        self.planner = neupan.init_from_yaml(str(self.config))

    def step(self, state, ranges, points):
        points_np = self.np.array(points, dtype=float).T if points else None
        begin = time.perf_counter()
        action, info = self.planner(self.np.array(state, dtype=float).reshape(3, 1), points_np)
        ms = (time.perf_counter()-begin)*1000
        # Python's original solver/fallback policy is retained for comparison.
        return float(action[0, 0]), float(action[1, 0]), dict(
            solved=None, stop=bool(info['stop']), arrive=bool(info['arrive']),
            dune_distance=float(self.planner.min_distance), planner_ms=ms)

    def close(self):
        pass


class ROSBackend:
    def __init__(self, args, scene, output):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
        from nav_msgs.msg import Path as ROSPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool
        from diagnostic_msgs.msg import DiagnosticArray
        if not rospy.core.is_initialized():
            rospy.init_node('neupan_simulation_suite', anonymous=True, disable_signals=True)
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError('This headless ROS backend requires use_sim_time=false.')
        self.rospy, self.Transform, self.Scan = rospy, TransformStamped, LaserScan
        self.args = args
        self.ns = '/neupan_sim_{}_{}'.format(os.getpid(), time.monotonic_ns())
        self.map, self.base, self.laser = [self.ns[1:]+'/'+x for x in ('map', 'base', 'laser')]
        self.lock, self.latest = threading.Lock(), (0.0, 0.0, 0.0)
        self.info = dict(solved=None, stop=False, arrive=False, dune_distance=None, planner_ms=None)
        def on_cmd(msg):
            with self.lock:
                self.latest = (msg.linear.x, msg.angular.z, time.monotonic())
        def on_arrive(msg):
            with self.lock:
                self.info['arrive'] = msg.data
        def on_diag(msg):
            if msg.status:
                data = {v.key: v.value for v in msg.status[0].values}
                with self.lock:
                    self.info['solved'] = data.get('solved') == 'true'
                    self.info['dune_distance'] = float(data.get('min_distance', 'nan'))
        self.subs = [rospy.Subscriber(self.ns+'/cmd', Twist, on_cmd, queue_size=10),
                     rospy.Subscriber(self.ns+'/arrive', Bool, on_arrive, queue_size=10),
                     rospy.Subscriber(self.ns+'/diag', DiagnosticArray, on_diag, queue_size=10)]
        self.scan_pub = rospy.Publisher(self.ns+'/scan', LaserScan, queue_size=1)
        self.path_pub = rospy.Publisher(self.ns+'/path', ROSPath, queue_size=1, latch=True)
        self.tf, self.static = tf2_ros.TransformBroadcaster(), tf2_ros.StaticTransformBroadcaster()
        extrinsic = TransformStamped()
        extrinsic.header.stamp, extrinsic.header.frame_id = rospy.Time.now(), self.base
        extrinsic.child_frame_id = self.laser
        extrinsic.transform.translation.x, extrinsic.transform.translation.y = args.sensor[:2]
        extrinsic.transform.rotation.z = math.sin(args.sensor[2]/2)
        extrinsic.transform.rotation.w = math.cos(args.sensor[2]/2)
        self.static.sendTransform(extrinsic)
        self.log = (output/'backend.log').open('w')
        rate = 1/args.dt
        node_command = [os.environ['NEUPAN_ROS_NODE']] if os.environ.get('NEUPAN_ROS_NODE') else ['rosrun', 'neupan_cpp_ros1', 'neupan_node']
        command = node_command + [ '__name:=planner', '__ns:='+self.ns,
                   '_config_file:='+str(args.config), '_dune_checkpoint:='+str(args.model),
                   '_map_frame:='+self.map, '_base_frame:='+self.base, '_lidar_frame:='+self.laser,
                   '_control_rate:='+str(rate), '_solver_fail_grace:=0',
                   '/scan:='+self.ns+'/scan', '/initial_path:='+self.ns+'/path',
                   '/neupan_cmd_vel:='+self.ns+'/cmd', '/neupan_arrive:='+self.ns+'/arrive',
                   '/neupan_diagnostics:='+self.ns+'/diag']
        for name in ['neupan_goal', 'neupan_waypoints', 'neupan_plan', 'neupan_ref_state',
                     'neupan_initial_path', 'dune_point_markers', 'nrmp_point_markers', 'robot_marker', 'neupan_emergency_stop']:
            command.append('/{}:={}/{}'.format(name, self.ns, name))
        self.proc = subprocess.Popen(command, stdout=self.log, stderr=self.log, start_new_session=True)
        initial = [*scene['path'][0], scene['yaw']]
        ranges, points = make_scan(initial, scene, args.sensor)
        # Allow TF subscriptions and initial sensor data to become available.
        self.warming_up = True
        try:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                self.step(initial, ranges, points)
                with self.lock:
                    fresh = time.monotonic()-self.latest[2] < .5
                if fresh and self.path_pub.get_num_connections() > 0:
                    break
            else:
                raise RuntimeError('ROS planner did not become ready in 5 s; see backend.log')
        except BaseException:
            self.close()
            raise
        self.warming_up = False
        path = ROSPath()
        path.header.frame_id, path.header.stamp = self.map, rospy.Time.now()
        for x, y in scene['path']:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = x, y, 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def step(self, state, ranges, points):
        end = time.monotonic()+self.args.dt
        while time.monotonic() < end:
            if self.proc.poll() is not None:
                raise RuntimeError('ROS planner exited; see backend.log')
            if self.rospy.is_shutdown():
                raise RuntimeError('ROS shutdown')
            tf = self.Transform()
            tf.header.stamp, tf.header.frame_id = self.rospy.Time.now(), self.map
            tf.child_frame_id = self.base
            tf.transform.translation.x, tf.transform.translation.y = state[:2]
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(state[2]/2), math.cos(state[2]/2)
            self.tf.sendTransform(tf)
            scan = self.Scan()
            scan.header.stamp, scan.header.frame_id = tf.header.stamp, self.laser
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment, scan.scan_time = TAU/(len(ranges)-1), self.args.dt
            scan.range_min, scan.range_max, scan.ranges = 0.1, 5.0, ranges
            self.scan_pub.publish(scan)
            time.sleep(min(.02, self.args.dt/4))
        with self.lock:
            v, w, receipt = self.latest
            info = dict(self.info)
        if time.monotonic()-receipt > 0.5 and not self.warming_up:
            raise RuntimeError('no fresh planner command for 0.5 s; see backend.log')
        return v, w, info

    def close(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        for sub in self.subs:
            sub.unregister()
        self.scan_pub.unregister()
        self.path_pub.unregister()
        self.log.close()


def percentile(values, fraction):
    if not values:
        return None
    data = sorted(values)
    return data[min(len(data)-1, math.ceil(fraction*len(data))-1)]


def clean_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean_json(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean_json(x) for x in value]
    return value


def plot_case(folder, scene, rows, cfg):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
    except ImportError:
        return
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    ax = axes[0]
    ax.plot(*zip(*scene['path']), '--', color='gray', label='reference')
    half_l, half_w = cfg['robot']['length']/2, cfg['robot']['width']/2
    if rows:
        ax.plot([r['x'] for r in rows], [r['y'] for r in rows], label='robot centre')
        offset = cfg['robot'].get('wheelbase', 0)/2
        for row in rows[::max(1, len(rows)//12)]:
            vertices = []
            for x, y in [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]:
                dx, dy = rotate(x+offset, y, row['yaw'])
                vertices.append((row['x']+dx, row['y']+dy))
            ax.add_patch(Polygon(vertices, fill=False, alpha=.25, color='tab:blue'))
    for x, y, radius in scene['circles']:
        ax.add_patch(Circle((x, y), radius, color='tab:red', alpha=.6))
    for x1, y1, x2, y2 in scene['walls']:
        ax.plot([x1, x2], [y1, y2], color='tab:red')
    # Crop the blocked-wall plot to the trajectory/goal rather than the whole wall.
    visible = scene['path'] + [(r['x'], r['y']) for r in rows]
    xs, ys = zip(*visible)
    ax.set_xlim(min(xs)-1.5, max(xs)+1.5)
    ax.set_ylim(min(ys)-max(1.5, half_w+1), max(ys)+max(1.5, half_w+1))
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend()
    if rows:
        axes[1].plot([r['time'] for r in rows], [r['v'] for r in rows], label='v [m/s]')
        axes[1].plot([r['time'] for r in rows], [r['w'] for r in rows], label='yaw rate [rad/s]')
        axes[1].plot([r['time'] for r in rows], [r['cross_track'] for r in rows], label='cross-track [m]')
    axes[1].set_xlabel('simulation time [s]')
    axes[1].legend()
    fig.suptitle(folder.name)
    fig.tight_layout()
    fig.savefig(folder/'trajectory.png', dpi=160)
    plt.close(fig)


def run_case(args, cfg, name, scene):
    folder = args.output/name
    folder.mkdir(parents=True)
    print('START {} ({})'.format(name, args.backend), flush=True)
    robot = cfg['robot']
    length, width = robot['length'], robot['width']
    state = [*scene['path'][0], scene['yaw']]
    path_length = sum(math.dist(a, b) for a, b in zip(scene['path'], scene['path'][1:]))
    speed = max(0.02, abs(cfg['ref_speed']))
    timeout = args.timeout or (min(30, path_length/speed+8) if scene['blocked'] else 3*path_length/speed+10)
    rows, reasons, backend = [], [], None
    trajectory_stream = (folder/'trajectory.csv').open('w', newline='')
    trajectory_writer = None
    settled, ever_arrived, collided = 0.0, False, False
    applied = (0., 0.)
    max_rates, max_target_rates, last_target = [0., 0.], [0., 0.], (0., 0.)
    emergency_stops = 0
    min_clearance, overshoot = float('inf'), 0.0
    start = time.monotonic()
    try:
        factory = {'core': CoreBackend, 'ros': ROSBackend, 'python': PythonBackend}[args.backend]
        backend = factory(args, scene, folder)
        for index in range(math.ceil(timeout/args.dt)):
            ranges, points = make_scan(state, scene, args.sensor)
            v, w, info = backend.step(state, ranges, points)
            if not math.isfinite(v) or not math.isfinite(w):
                reasons.append('non-finite command')
                break
            if abs(v) > robot['max_speed'][0]+.01 or abs(w) > robot['max_speed'][1]+.01:
                reasons.append('velocity bound exceeded')
                break
            target = (v, w)
            emergency_stops += bool(info.get('stop'))
            for j in range(2):
                max_target_rates[j] = max(max_target_rates[j], abs(target[j]-last_target[j])/args.dt)
            last_target = target
            # The actual actuator ramps at 50 Hz; these commands drive the plant.
            rates = [0., 0.]
            for _ in range(5):
                previous = applied
                applied = limit_command(target, applied, robot['max_speed'], robot['max_acce'], args.dt/5)
                for j in range(2):
                    rates[j] = max(rates[j], abs(applied[j]-previous[j])/(args.dt/5))
                    max_rates[j] = max(max_rates[j], rates[j])
                state = integrate(state, *applied, args.dt/5)
                gap = clearance(state, scene, length, width, robot.get('wheelbase', 0)/2)
                min_clearance = min(min_clearance, gap)
                collided |= gap <= 0
            if hasattr(backend, 'set_applied'):
                backend.set_applied(*applied)
            v, w = applied
            error = path_error(state, scene['path'])
            goal_error = math.dist(state[:2], scene['path'][-1])
            a, b = scene['path'][-2:]
            dx, dy = b[0]-a[0], b[1]-a[1]
            norm = math.hypot(dx, dy)
            overshoot = max(overshoot, ((state[0]-b[0])*dx+(state[1]-b[1])*dy)/norm)
            arrived = bool(info['arrive'])
            ever_arrived |= arrived
            stopped = abs(v) < .01 and abs(w) < .02
            settled = settled+args.dt if stopped and (arrived or scene['blocked']) else 0.0
            rows.append(dict(time=(index+1)*args.dt, x=state[0], y=state[1], yaw=state[2], v=v, w=w,
                             requested_v=target[0], requested_w=target[1], raw_v=info.get('raw_v', target[0]),
                             raw_w=info.get('raw_w', target[1]), acceleration=rates[0], angular_acceleration=rates[1],
                             emergency_stop=bool(info.get('stop')),
                             cross_track=error, goal_error=goal_error, clearance=gap,
                             obstacle_points=len(points), solved=info['solved'], arrive=arrived,
                             dune_distance=info['dune_distance'], planner_ms=info['planner_ms']))
            if trajectory_writer is None:
                trajectory_writer = csv.DictWriter(trajectory_stream, fieldnames=list(rows[-1]))
                trajectory_writer.writeheader()
            trajectory_writer.writerow(rows[-1])
            trajectory_stream.flush()
            if index % max(1, round(5/args.dt)) == 0:
                print('  {} t={:.1f}s goal_error={:.3f}m solved={}'.format(
                    name, (index+1)*args.dt, goal_error, info['solved']), flush=True)
            if collided:
                reasons.append('true footprint collision')
                break
            if settled >= (2.0 if scene['blocked'] else 1.0):
                # A blocked path should have been approached, not just idle at startup.
                if not scene['blocked'] or math.dist(state[:2], scene['path'][0]) > .1:
                    break
    except Exception as ex:
        reasons.append('{}: {}'.format(type(ex).__name__, ex))
    finally:
        try:
            if backend is not None:
                backend.close()
        finally:
            trajectory_stream.close()
    max_cte = max((r['cross_track'] for r in rows), default=float('inf'))
    rms_cte = math.sqrt(sum(r['cross_track']**2 for r in rows)/len(rows)) if rows else float('inf')
    goal_error = math.dist(state[:2], scene['path'][-1])
    if scene['blocked']:
        if ever_arrived or settled < 2 or math.dist(state[:2], scene['path'][0]) <= .1:
            reasons.append('blocked path did not approach and settle without arriving')
    else:
        if not ever_arrived or settled < 1 or goal_error > args.goal_tolerance:
            reasons.append('did not arrive and stop within tolerance/time')
        if max_cte > args.max_cross_track or rms_cte > args.rms_cross_track:
            reasons.append('cross-track tolerance exceeded')
        if overshoot > args.max_overshoot:
            reasons.append('endpoint overshoot exceeded')
    if scene['circles'] or scene['walls']:
        if not rows or max((r['obstacle_points'] for r in rows), default=0) == 0:
            reasons.append('obstacle case had no sensor returns')
        if min_clearance < args.min_clearance:
            reasons.append('true clearance below required margin')
    if any(max_rates[j] > robot['max_acce'][j]+1e-8 for j in range(2)):
        reasons.append('applied command acceleration bound exceeded')
    if args.backend == 'core' and any(max_target_rates[j] > robot['max_acce'][j]+1e-8 for j in range(2)):
        reasons.append('planner command acceleration bound exceeded (including emergency stops)')
    timings = [r['planner_ms'] for r in rows if r['planner_ms'] is not None and math.isfinite(r['planner_ms'])]
    v_stream, w_stream = [0.0]+[r['v'] for r in rows], [0.0]+[r['w'] for r in rows]
    result = dict(case=name, backend=args.backend, passed=not reasons, reasons=reasons,
                  arrived=ever_arrived, collision=collided, goal_error=goal_error,
                  rms_cross_track=rms_cte, max_cross_track=max_cte, max_overshoot=overshoot,
                  min_clearance=min_clearance, simulated_seconds=len(rows)*args.dt,
                  max_command_acceleration=max_rates[0],
                  max_command_angular_acceleration=max_rates[1],
                  max_planner_command_acceleration=max_target_rates[0],
                  max_planner_command_angular_acceleration=max_target_rates[1], emergency_stops=emergency_stops,
                  wall_seconds=time.monotonic()-start, planner_mean_ms=sum(timings)/len(timings) if timings else None,
                  planner_p95_ms=percentile(timings, .95))
    (folder/'summary.json').write_text(json.dumps(clean_json(result), indent=2, allow_nan=False)+'\n')
    plot_case(folder, scene, rows, cfg)
    print('{} {}: goal={:.3f} m, max_cte={:.3f} m, clearance={}{}'.format(
        'PASS' if result['passed'] else 'FAIL', name, goal_error, max_cte,
        '{:.3f} m'.format(min_clearance) if math.isfinite(min_clearance) else 'open field',
        '; '+ '; '.join(reasons) if reasons else ''), flush=True)
    return result


def create_output_directory(output):
    """Reserve a fresh run directory without overwriting previous results."""
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        output.mkdir()
        return output
    except FileExistsError:
        stamp = time.strftime('%Y%m%d_%H%M%S')
    index = 0
    while True:
        suffix = '_' + stamp + ('_{:02d}'.format(index) if index else '')
        candidate = output.with_name(output.name + suffix)
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            index += 1


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['ros', 'core', 'python'], default='ros')
    parser.add_argument('--config', type=Path, default=root/'config/fw_mini.yaml')
    parser.add_argument('--model', type=Path, default=root/'models/fw_mini.bin')
    parser.add_argument('--checkpoint', type=Path, help='Python .pth from the SAME training as .bin')
    parser.add_argument('--binary', type=Path, default=Path(os.environ.get('NEUPAN_BUILD_ROOT', Path.home()/'neupan_noetic_build'))/'libneupan/neupan_sim_backend')
    parser.add_argument('--cases', default='all', help='comma-separated case names, or all')
    parser.add_argument('--output', type=Path, default=Path.home()/'neupan_sim_results'/time.strftime('%Y%m%d_%H%M%S'),
                        help='result directory; adds a timestamp if it already exists')
    parser.add_argument('--sensor', type=float, nargs=3, default=[.2, .05, .35], metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal-tolerance', type=float, default=.15)
    parser.add_argument('--max-cross-track', type=float, default=.35)
    parser.add_argument('--rms-cross-track', type=float, default=.15)
    parser.add_argument('--max-overshoot', type=float, default=.15)
    parser.add_argument('--min-clearance', type=float, default=.03)
    parser.add_argument('--timeout', type=float, help='override each case timeout in simulated seconds')
    args = parser.parse_args()
    args.config, args.model, args.output, args.binary = [p.expanduser().resolve() for p in (args.config, args.model, args.output, args.binary)]
    cfg = yaml.safe_load(args.config.read_text())
    args.dt = float(cfg['step_time'])
    if cfg['robot']['kinematics'] != 'diff' or cfg.get('ipath', {}).get('loop', False):
        parser.error('requires diff and loop=false')
    if not 0 < args.dt <= .5:
        parser.error('step_time must be in (0, 0.5]')
    if args.backend == 'core' and not args.binary.is_file():
        parser.error('build neupan_sim_backend first, or pass --binary')
    if args.backend == 'python':
        if args.checkpoint is None or not args.checkpoint.is_file():
            parser.error('Python backend requires --checkpoint /path/model_5000.pth')
        args.checkpoint = args.checkpoint.resolve()
    elif not args.model.is_file():
        parser.error('missing model: train/export FW-mini model, or pass matching --config/--model')
    all_scenes = scenarios(cfg)
    names = list(all_scenes) if args.cases == 'all' else args.cases.split(',')
    if not names or any(n not in all_scenes for n in names):
        parser.error('valid cases: '+','.join(all_scenes))
    requested_output = args.output
    args.output = create_output_directory(requested_output)
    if args.output != requested_output:
        print('Output path already exists; keeping previous results.', flush=True)
    print('Writing results to:', args.output, flush=True)
    run = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
    run['config_sha256'] = hashlib.sha256(args.config.read_bytes()).hexdigest()
    if args.backend != 'python':
        run['model_sha256'] = hashlib.sha256(args.model.read_bytes()).hexdigest()
    else:
        run['checkpoint_sha256'] = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
    (args.output/'config_snapshot.yaml').write_text(args.config.read_text())
    (args.output/'run.json').write_text(json.dumps(run, indent=2)+'\n')
    results = [run_case(args, cfg, name, all_scenes[name]) for name in names]
    (args.output/'summary.json').write_text(json.dumps(clean_json(results), indent=2, allow_nan=False)+'\n')
    with (args.output/'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    print('Results:', args.output)
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
