#!/usr/bin/env python3
"""Interactive ROS1 world for the real neupan_node; no chassis connection."""
import csv
import hashlib
import heapq
import json
import math
from pathlib import Path
import sys
import threading
import time

import yaml

# Also works through catkin's devel-space relay and after catkin install.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from simulation_test import (clearance, create_output_directory, integrate,
                             ray_range, rotate, sensor_pose)


class World:
    """Analytic obstacles, conservative circular A*, rectangular collision check."""
    def __init__(self, scenario_file, robot_file):
        self.scenario_file, self.robot_file = Path(scenario_file), Path(robot_file)
        self.config = yaml.safe_load(self.scenario_file.read_text())
        self.planner = yaml.safe_load(self.robot_file.read_text())
        self.robot = self.planner['robot']
        if self.robot['kinematics'] != 'diff':
            raise ValueError('This virtual plant supports diff kinematics only')
        self.map_file = self.scenario_file.parent/self.config['map_file']
        environment = yaml.safe_load(self.map_file.read_text())
        self.width, self.height = [float(environment['world'][x]) for x in ('width', 'height')]
        self.resolution = float(self.config.get('resolution', .05))
        self.radius = math.hypot(self.robot['length'], self.robot['width'])/2
        self.radius += abs(self.robot.get('wheelbase', 0))/2
        self.margin = float(self.config.get('safety_margin', .04))
        self.rectangles, self.scene = [], dict(circles=[], walls=[])
        for obstacle in environment['obstacles']:
            shape, pose = obstacle['shape'], obstacle['pose']
            if shape['type'] == 'circle':
                self.scene['circles'].append((pose['x'], pose['y'], shape['radius']))
            elif shape['type'] == 'rectangle':
                rectangle = (pose['x'], pose['y'], pose.get('yaw', 0), shape['length'], shape['width'])
                self.rectangles.append(rectangle)
                x, y, yaw, length, width = rectangle
                corners = []
                for dx, dy in [(-length/2, -width/2), (length/2, -width/2),
                               (length/2, width/2), (-length/2, width/2)]:
                    rx, ry = rotate(dx, dy, yaw)
                    corners.append((x+rx, y+ry))
                for a, b in zip(corners, corners[1:]+corners[:1]):
                    self.scene['walls'].append((*a, *b))
            else:
                raise ValueError('Unsupported obstacle: '+str(shape['type']))
        self.nx, self.ny = math.ceil(self.width/self.resolution), math.ceil(self.height/self.resolution)
        self.occupied, self.blocked = [], set()
        for j in range(self.ny):
            for i in range(self.nx):
                x, y = self.point((i, j))
                gap = self.point_clearance(x, y)
                self.occupied.append(100 if gap <= self.resolution/math.sqrt(2) else 0)
                # Include cell half-diagonal so every point in a free cell is safe.
                if gap <= self.radius+self.margin+self.resolution/math.sqrt(2):
                    self.blocked.add((i, j))

    def point_clearance(self, x, y):
        result = min(self.width/2-abs(x), self.height/2-abs(y))
        for cx, cy, yaw, length, width in self.rectangles:
            px, py = rotate(x-cx, y-cy, -yaw)
            qx, qy = abs(px)-length/2, abs(py)-width/2
            gap = math.hypot(max(qx, 0), max(qy, 0))+min(max(qx, qy), 0)
            result = min(result, gap)
        for cx, cy, radius in self.scene['circles']:
            result = min(result, math.hypot(x-cx, y-cy)-radius)
        return result

    def clearance(self, state):
        if self.point_clearance(*state[:2]) <= 0:
            return min(0, self.point_clearance(*state[:2]))
        return clearance(state, self.scene, self.robot['length'], self.robot['width'],
                         self.robot.get('wheelbase', 0)/2)

    def scan(self, state):
        sensor = self.config['sensor']
        sx, sy, yaw = sensor_pose(state, sensor['offset'])
        beams, limit = int(sensor['beams']), float(sensor['range_max'])
        ranges, points = [], []
        for i in range(beams):
            angle = -math.pi+2*math.pi*i/(beams-1)
            direction = (math.cos(yaw+angle), math.sin(yaw+angle))
            distance = ray_range((sx, sy), direction, self.scene['circles'], self.scene['walls'], limit)
            if distance < sensor['range_min']:
                distance = float('nan')
            ranges.append(distance)
            if math.isfinite(distance):
                points.append((sx+distance*direction[0], sy+distance*direction[1]))
        return ranges, points

    def point(self, cell):
        return (-self.width/2+(cell[0]+.5)*self.resolution,
                -self.height/2+(cell[1]+.5)*self.resolution)

    def cell(self, point):
        return (math.floor((point[0]+self.width/2)/self.resolution),
                math.floor((point[1]+self.height/2)/self.resolution))

    def free(self, cell):
        return 0 <= cell[0] < self.nx and 0 <= cell[1] < self.ny and cell not in self.blocked

    def segment_free(self, a, b):
        count = max(1, math.ceil(math.dist(a, b)/(self.resolution/2)))
        return all(self.point_clearance(a[0]+(b[0]-a[0])*k/count, a[1]+(b[1]-a[1])*k/count)
                   > self.radius+self.margin for k in range(count+1))

    def route(self, start, goal):
        start, goal = tuple(start[:2]), tuple(goal[:2])
        source, target = self.cell(start), self.cell(goal)
        if not self.free(source) or not self.free(target):
            raise ValueError('Start/goal is outside the map or inside the inflated obstacle margin')
        queue, cost, parent = [(math.dist(source, target), 0., source)], {source: 0.}, {}
        while queue:
            _, g, current = heapq.heappop(queue)
            if g > cost[current]+1e-10:
                continue
            if current == target:
                break
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                neighbour = current[0]+dx, current[1]+dy
                if not self.free(neighbour):
                    continue
                if dx and dy and (not self.free((current[0]+dx, current[1])) or
                                  not self.free((current[0], current[1]+dy))):
                    continue
                candidate = g+math.hypot(dx, dy)
                if candidate < cost.get(neighbour, float('inf')):
                    cost[neighbour], parent[neighbour] = candidate, current
                    heapq.heappush(queue, (candidate+math.dist(neighbour, target), candidate, neighbour))
        else:
            raise ValueError('No route for the inflated FW-mini footprint')
        cells = [target]
        while cells[-1] != source:
            cells.append(parent[cells[-1]])
        raw = [start]+[self.point(c) for c in reversed(cells)]+[goal]
        raw = [p for i, p in enumerate(raw) if not i or math.dist(p, raw[i-1]) > 1e-8]
        sparse, index = [raw[0]], 0
        while index < len(raw)-1:
            end = len(raw)-1
            while end > index+1 and not self.segment_free(raw[index], raw[end]):
                end -= 1
            if not self.segment_free(raw[index], raw[end]):
                raise ValueError('Start/goal cannot connect safely to the grid')
            sparse.append(raw[end])
            index = end
        # Round corners with quadratic Bezier arcs; retain the original corner
        # if any sampled arc segment leaves the inflated free space.
        rounded = [sparse[0]]
        for a, b, c in zip(sparse, sparse[1:], sparse[2:]):
            cut = min(.45, .3*math.dist(a, b), .3*math.dist(b, c))
            p = tuple(b[k]+(a[k]-b[k])*cut/math.dist(a, b) for k in (0, 1))
            q = tuple(b[k]+(c[k]-b[k])*cut/math.dist(b, c) for k in (0, 1))
            arc = [tuple((1-t)**2*p[k]+2*t*(1-t)*b[k]+t*t*q[k] for k in (0, 1))
                   for t in [i/12 for i in range(13)]]
            chain = [rounded[-1]]+arc
            if all(self.segment_free(x, y) for x, y in zip(chain, chain[1:])):
                rounded.extend(arc)
            else:
                rounded.append(b)
        rounded.append(sparse[-1])
        path = [rounded[0]]
        for a, b in zip(rounded, rounded[1:]):
            count = max(1, math.ceil(math.dist(a, b)/.04))
            path.extend(tuple(a[k]+(b[k]-a[k])*i/count for k in (0, 1)) for i in range(1, count+1))
        if len(path) < 2:
            path.append(goal)
        return path


def run_ros():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as ROSPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from visualization_msgs.msg import Marker, MarkerArray

    class Simulation:
        def __init__(self):
            if rospy.get_param('/use_sim_time', False):
                raise RuntimeError('This simulator uses wall time; use a ROS master with /use_sim_time=false')
            self.world = World(rospy.get_param('~scenario_file'), rospy.get_param('~config_file'))
            self.frame = rospy.get_param('~map_frame', 'neupan_sim/map')
            self.odom_frame = rospy.get_param('~odom_frame', 'neupan_sim/odom')
            self.base = rospy.get_param('~base_frame', 'neupan_sim/base_link')
            self.laser = rospy.get_param('~laser_frame', 'neupan_sim/laser')
            self.state = list(self.world.config['initial_pose'])
            if self.world.clearance(self.state) <= 0:
                raise ValueError('Initial pose collides with an obstacle')
            self.lock = threading.RLock()
            self.command, self.receipt = (0., 0.), 0.
            self.goal, self.pending_goal, self.pending_pose = None, None, None
            self.active, self.acknowledged, self.arrived, self.collided = False, False, False, False
            self.min_clearance, self.collisions = float('inf'), 0
            self.goal_history = []
            self.history = ROSPath()
            self.tf, self.static = tf2_ros.TransformBroadcaster(), tf2_ros.StaticTransformBroadcaster()
            self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
            self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1, latch=True)
            self.obstacles_pub = rospy.Publisher('obstacles', MarkerArray, queue_size=1, latch=True)
            self.markers_pub = rospy.Publisher('vehicle', MarkerArray, queue_size=1)
            self.path_pub = rospy.Publisher('initial_path', ROSPath, queue_size=1, latch=True)
            self.trace_pub = rospy.Publisher('actual_path', ROSPath, queue_size=1)
            self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
            self.subs = [rospy.Subscriber('cmd_vel', Twist, self.on_command, queue_size=1),
                         rospy.Subscriber('goal', PoseStamped, self.on_goal, queue_size=1),
                         rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.on_pose, queue_size=1),
                         rospy.Subscriber('neupan_arrive', Bool, self.on_arrive, queue_size=1),
                         rospy.Subscriber('neupan_initial_path', ROSPath, self.on_reference, queue_size=1)]
            output = Path(rospy.get_param('~output', str(Path.home()/'neupan_sim_results/rviz'))).expanduser()
            self.output = create_output_directory(output)
            self.log = (self.output/'trajectory.csv').open('w', newline='')
            self.writer = csv.writer(self.log)
            self.writer.writerow(['wall_seconds', 'x', 'y', 'yaw', 'raw_v', 'raw_w',
                                  'applied_v', 'applied_w', 'true_clearance', 'arrived', 'collision'])
            (self.output/'planner.yaml').write_text(self.world.robot_file.read_text())
            (self.output/'scenario.yaml').write_text(self.world.scenario_file.read_text())
            (self.output/'map.yaml').write_text(self.world.map_file.read_text())
            model = Path(rospy.get_param('~dune_checkpoint'))
            (self.output/'run.json').write_text(json.dumps(dict(
                backend='ros_rviz', model=str(model),
                model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                config_sha256=hashlib.sha256(self.world.robot_file.read_bytes()).hexdigest(),
                scenario_file=str(self.world.scenario_file), output=str(self.output),
                command_topic=rospy.resolve_name('cmd_vel')), indent=2)+'\n')
            self.started = time.monotonic()
            self.publish_static()
            rospy.on_shutdown(self.close)
            rospy.loginfo('FW-mini simulation ready. Use RViz 2D Nav Goal. Results: %s', self.output)

        def on_command(self, msg):
            with self.lock:
                if self.acknowledged:
                    self.command = (msg.linear.x, msg.angular.z)
                    self.receipt = time.monotonic()

        def on_reference(self, msg):
            with self.lock:
                if self.active and self.goal and msg.poses:
                    end = msg.poses[-1].pose.position
                    self.acknowledged = math.hypot(end.x-self.goal[0], end.y-self.goal[1]) < .001

        def on_arrive(self, msg):
            with self.lock:
                if (self.active and self.acknowledged and self.goal and msg.data and
                        math.dist(self.state[:2], self.goal) < self.world.planner['ipath']['arrive_threshold']):
                    if not self.arrived and self.goal_history:
                        self.goal_history[-1].update(status='arrived', final_pose=list(self.state),
                            goal_error=math.dist(self.state[:2], self.goal),
                            arrived_wall_seconds=time.monotonic()-self.started)
                    self.arrived = True

        def on_goal(self, msg):
            if msg.header.frame_id != self.frame:
                rospy.logwarn('Goal frame must be %s', self.frame)
                return
            with self.lock:
                self.active, self.acknowledged = False, False
                self.command = (0., 0.)
                self.pending_goal = (msg.pose.position.x, msg.pose.position.y)

        def on_pose(self, msg):
            if msg.header.frame_id != self.frame:
                rospy.logwarn('Initial pose frame must be %s', self.frame)
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            with self.lock:
                self.pending_pose = [p.x, p.y, yaw]

        def path_message(self, path, stamp):
            msg = ROSPath()
            msg.header.frame_id, msg.header.stamp = self.frame, stamp
            for i, (x, y) in enumerate(path):
                a, b = (path[i], path[i+1]) if i+1 < len(path) else (path[max(0, i-1)], path[i])
                yaw = math.atan2(b[1]-a[1], b[0]-a[0])
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y = x, y
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                msg.poses.append(pose)
            return msg

        def publish_static(self):
            stamp = rospy.Time.now()
            transforms = []
            offset = self.world.config['sensor']['offset']
            for parent, child, x, y, yaw in [(self.frame, self.odom_frame, 0., 0., 0.),
                                            (self.base, self.laser, *offset)]:
                tf = TransformStamped()
                tf.header.frame_id, tf.header.stamp, tf.child_frame_id = parent, stamp, child
                tf.transform.translation.x, tf.transform.translation.y = x, y
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
                transforms.append(tf)
            self.static.sendTransform(transforms)
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = stamp, self.frame
            grid.info.resolution = self.world.resolution
            grid.info.width, grid.info.height = self.world.nx, self.world.ny
            grid.info.origin.position.x, grid.info.origin.position.y = -self.world.width/2, -self.world.height/2
            grid.info.origin.orientation.w = 1.
            grid.data = self.world.occupied
            self.map_pub.publish(grid)
            array = MarkerArray()
            for i, (x, y, yaw, length, width) in enumerate(self.world.rectangles):
                marker = self.marker(i, Marker.CUBE, stamp, 'obstacles')
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, .15
                marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                marker.scale.x, marker.scale.y, marker.scale.z = length, width, .3
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = .4, .45, .5, .9
                array.markers.append(marker)
            for x, y, radius in self.world.scene['circles']:
                marker = self.marker(len(array.markers), Marker.CYLINDER, stamp, 'obstacles')
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, .15
                marker.scale.x, marker.scale.y, marker.scale.z = 2*radius, 2*radius, .3
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = .8, .35, .2, .9
                array.markers.append(marker)
            self.obstacles_pub.publish(array)

        def marker(self, number, kind, stamp, namespace='vehicle'):
            msg = Marker()
            msg.header.frame_id, msg.header.stamp = self.frame, stamp
            msg.ns, msg.id, msg.type, msg.action = namespace, number, kind, Marker.ADD
            msg.pose.orientation.w, msg.color.a = 1., 1.
            return msg

        def publish_state(self, stamp, v, w, gap):
            x, y, yaw = self.state
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.odom_frame, self.base
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.tf.sendTransform(tf)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.odom_frame, self.base
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation = tf.transform.rotation
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
            self.odom_pub.publish(odom)
            array = MarkerArray()
            body = self.marker(0, Marker.CUBE, stamp)
            body.pose = odom.pose.pose
            dx, dy = rotate(self.world.robot.get('wheelbase', 0)/2, 0, yaw)
            body.pose.position.x += dx
            body.pose.position.y += dy
            body.pose.position.z = .10
            body.scale.x, body.scale.y, body.scale.z = self.world.robot['length'], self.world.robot['width'], .20
            body.color.r, body.color.g, body.color.b = (1., .1, .1) if self.collided else (.1, .55, .95)
            array.markers.append(body)
            arrow = self.marker(1, Marker.ARROW, stamp)
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = x, y, .25
            arrow.pose.orientation = tf.transform.rotation
            arrow.scale.x, arrow.scale.y, arrow.scale.z = .4, .06, .06
            arrow.color.r, arrow.color.g = 1., .8
            array.markers.append(arrow)
            status = self.marker(2, Marker.TEXT_VIEW_FACING, stamp)
            status.pose.position.x, status.pose.position.y, status.pose.position.z = x, y, .75
            status.scale.z = .15
            status.color.r, status.color.g, status.color.b = 1., 1., 1.
            label = ('COLLISION: RESET POSE' if self.collided else 'ARRIVED' if self.arrived else
                     'TRACKING' if self.active else 'SET 2D NAV GOAL')
            status.text = '{}\nv={:.2f} w={:.2f} clearance={:.2f}m'.format(label, v, w, gap)
            array.markers.append(status)
            goal = self.marker(3, Marker.SPHERE, stamp)
            if self.goal:
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = *self.goal, .12
                goal.scale.x, goal.scale.y, goal.scale.z = .18, .18, .18
                goal.color.r, goal.color.g, goal.color.b = 1., .4, .1
            else:
                goal.action = Marker.DELETE
            array.markers.append(goal)
            self.markers_pub.publish(array)

        def close(self):
            with self.lock:
                if self.log.closed:
                    return
                self.log.flush()
                self.log.close()
                summary = dict(collision_count=self.collisions, min_true_clearance=self.min_clearance,
                               goals=self.goal_history, final_pose=self.state, arrived=self.arrived,
                               note='Ideal diff plant. Collision is latched and requires a pose reset.')
                (self.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')

        def run(self):
            hz, scan_hz = float(self.world.config.get('update_rate', 50)), float(self.world.config['sensor']['rate'])
            rate, previous, last_scan = rospy.Rate(hz), time.monotonic(), 0.
            auto_goal = rospy.get_param('~auto_goal', False)
            while not rospy.is_shutdown():
                now = time.monotonic()
                dt, previous = min(now-previous, .1), now
                with self.lock:
                    pending, pose = self.pending_goal, self.pending_pose
                    self.pending_goal, self.pending_pose = None, None
                    if pose is not None:
                        if self.world.clearance(pose) > 0:
                            if self.goal_history and self.goal_history[-1].get('status') == 'tracking':
                                self.goal_history[-1]['status'] = 'pose_reset'
                            self.state, self.goal = pose, None
                            self.active, self.acknowledged, self.arrived, self.collided = False, False, False, False
                            self.command, self.receipt = (0., 0.), 0.
                            self.history.poses = []
                            self.path_pub.publish(self.path_message([], rospy.Time.now()))
                            pending = None
                        else:
                            rospy.logwarn('Rejected initial pose: footprint overlaps an obstacle')
                    if auto_goal and now-self.started > 2:
                        pending = self.world.config['default_goal']
                        auto_goal = False
                    if pending is not None:
                        if self.goal_history and self.goal_history[-1].get('status') == 'tracking':
                            self.goal_history[-1]['status'] = 'replaced'
                        self.active, self.acknowledged, self.arrived = False, False, False
                        self.command, self.receipt = (0., 0.), 0.
                        try:
                            if self.collided:
                                raise ValueError('Reset pose after collision before selecting another goal')
                            route = self.world.route(self.state, pending)
                            self.goal, self.active = tuple(pending), True
                            self.goal_history.append(dict(goal=self.goal, wall_seconds=now-self.started, status='tracking'))
                            (self.output/'goal_{:03d}_path.json'.format(len(self.goal_history))).write_text(
                                json.dumps(route)+'\n')
                            self.path_pub.publish(self.path_message(route, rospy.Time.now()))
                            rospy.loginfo('Global path: %d poses to (%.2f, %.2f)', len(route), *pending)
                        except ValueError as ex:
                            rospy.logwarn('Goal rejected: %s', ex)
                            self.goal = None
                            self.path_pub.publish(self.path_message([], rospy.Time.now()))
                    raw_v, raw_w = self.command
                    v, w = raw_v, raw_w
                    if (not self.active or not self.acknowledged or self.arrived or self.collided or
                            now-self.receipt > self.world.config.get('command_timeout', .5)):
                        v, w = 0., 0.
                    if not math.isfinite(v) or not math.isfinite(w):
                        v, w = 0., 0.
                    v = max(-self.world.robot['max_speed'][0], min(self.world.robot['max_speed'][0], v))
                    w = max(-self.world.robot['max_speed'][1], min(self.world.robot['max_speed'][1], w))
                    for _ in range(5):
                        candidate = integrate(self.state, v, w, dt/5)
                        gap = self.world.clearance(candidate)
                        self.min_clearance = min(self.min_clearance, gap)
                        if gap <= 0:
                            if not self.collided:
                                self.collisions += 1
                                if self.goal_history:
                                    self.goal_history[-1]['status'] = 'collision'
                                rospy.logerr('TRUE FOOTPRINT COLLISION: simulation halted; use 2D Pose Estimate to reset')
                            self.collided, v, w = True, 0., 0.
                            break
                        self.state = candidate
                    stamp = rospy.Time.now()
                    self.publish_state(stamp, v, w, self.world.clearance(self.state))
                    if now-last_scan >= 1/scan_hz:
                        last_scan = now
                        ranges, _ = self.world.scan(self.state)
                        scan = LaserScan()
                        scan.header.stamp, scan.header.frame_id = stamp, self.laser
                        scan.angle_min, scan.angle_max = -math.pi, math.pi
                        scan.angle_increment, scan.scan_time = 2*math.pi/(len(ranges)-1), 1/scan_hz
                        scan.range_min, scan.range_max = self.world.config['sensor']['range_min'], self.world.config['sensor']['range_max']
                        scan.ranges = ranges
                        self.scan_pub.publish(scan)
                        pose_msg = self.path_message([self.state[:2]], stamp).poses[0]
                        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = math.sin(self.state[2]/2), math.cos(self.state[2]/2)
                        self.history.header = pose_msg.header
                        self.history.poses = (self.history.poses+[pose_msg])[-6000:]
                        self.trace_pub.publish(self.history)
                        if not self.log.closed:
                            self.writer.writerow([now-self.started, *self.state, raw_v, raw_w, v, w,
                                                  self.world.clearance(self.state), self.arrived, self.collided])
                            self.log.flush()
                rate.sleep()

    rospy.init_node('world')
    Simulation().run()


if __name__ == '__main__':
    run_ros()
