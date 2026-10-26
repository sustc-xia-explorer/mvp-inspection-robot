#!/usr/bin/env python3
"""Isolated ROS1 interface test with a virtual diff robot; never publishes /ctrl_cmd."""
import math
import os
import signal
import subprocess
import threading
import time

import rospkg
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('neupan_noetic_smoke', anonymous=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Run this self-test with use_sim_time=false and a running roscore.')
    root = rospkg.RosPack().get_path('neupan_cpp_ros1')
    ns = '/neupan_selftest_{}'.format(os.getpid())
    frame_map, frame_base = ns[1:] + '/map', ns[1:] + '/base_link'
    state, received = [0.0, 0.0, 0.0], []
    lock = threading.Lock()
    latest = [0.0, 0.0, 0.0]

    def on_cmd(msg):
        with lock:
            latest[:] = [msg.linear.x, msg.angular.z, time.monotonic()]
            received.append((time.monotonic(), msg.linear.x, msg.angular.z))

    sub = rospy.Subscriber(ns + '/cmd_vel', Twist, on_cmd, queue_size=20)
    scan_pub = rospy.Publisher(ns + '/scan', LaserScan, queue_size=1)
    path_pub = rospy.Publisher(ns + '/initial_path', Path, queue_size=1, latch=True)
    broadcaster = tf2_ros.TransformBroadcaster()
    names = ['scan', 'initial_path', 'neupan_waypoints', 'neupan_goal',
             'neupan_plan', 'neupan_ref_state', 'neupan_initial_path',
             'dune_point_markers', 'nrmp_point_markers', 'robot_marker',
             'neupan_arrive', 'neupan_diagnostics', 'neupan_emergency_stop']
    node_command = [os.environ['NEUPAN_ROS_NODE']] if os.environ.get('NEUPAN_ROS_NODE') else ['rosrun', 'neupan_cpp_ros1', 'neupan_node']
    args = node_command + [ '__name:=planner', '__ns:=' + ns,
            '_config_file:=' + root + '/config/smoke.yaml',
            '_dune_checkpoint:=' + root + '/models/diff_default.bin',
            '_map_frame:=' + frame_map, '_base_frame:=' + frame_base,
            '_lidar_frame:=' + frame_base, '_control_rate:=10.0',
            '_scan_timeout:=0.5', '_solver_fail_grace:=0',
            '/neupan_cmd_vel:=' + ns + '/cmd_vel']
    args.extend('/{}:={}/{}'.format(n, ns, n) for n in names)
    proc = subprocess.Popen(args, start_new_session=True)

    def advance(seconds, scans=False):
        deadline, last = time.monotonic() + seconds, time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if proc.poll() is not None:
                raise RuntimeError('planner exited with code {}'.format(proc.returncode))
            current = time.monotonic()
            dt, last = min(current - last, 0.1), current
            with lock:
                v, w, receipt = latest
            if current - receipt < 0.5:
                state[0] += v * math.cos(state[2]) * dt
                state[1] += v * math.sin(state[2]) * dt
                state[2] += w * dt
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id = rospy.Time.now(), frame_map
            tf.child_frame_id = frame_base
            tf.transform.translation.x, tf.transform.translation.y = state[:2]
            tf.transform.rotation.z = math.sin(state[2] / 2)
            tf.transform.rotation.w = math.cos(state[2] / 2)
            broadcaster.sendTransform(tf)
            if scans:
                msg = LaserScan()
                msg.header = tf.header
                msg.header.frame_id = frame_base
                msg.angle_min, msg.angle_max = -math.pi, math.pi
                msg.angle_increment, msg.scan_time = 2 * math.pi / 359, 0.05
                msg.range_min, msg.range_max = 0.1, 5.0
                msg.ranges = [float('inf')] * 360  # Valid unobstructed synthetic scan.
                scan_pub.publish(msg)
            time.sleep(0.05)

    try:
        path = Path()
        path.header.stamp, path.header.frame_id = rospy.Time.now(), frame_map
        for x in (0.0, 2.0):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.orientation.w = x, 1.0
            path.poses.append(pose)
        path_pub.publish(path)
        advance(3.0)
        with lock:
            before = list(received)
        if not before or any(abs(v) + abs(w) > 1e-6 for _, v, w in before):
            raise AssertionError('expected zero commands before the first valid scan')
        advance(6.0, scans=True)
        if state[0] < 0.05:
            raise AssertionError('virtual robot did not move forward: {}'.format(state))
        cutoff = time.monotonic()
        advance(2.0)
        with lock:
            stopped = [x for x in received if x[0] > cutoff + 1.0]
            all_commands = list(received)
        if not stopped or any(abs(v) + abs(w) > 1e-6 for _, v, w in stopped):
            raise AssertionError('scan timeout did not produce zero commands')
        if any(not math.isfinite(v) or not math.isfinite(w) for _, v, w in all_commands):
            raise AssertionError('non-finite command')
        print('PASS: no-scan stop, virtual tracking, scan-loss stop; x={:.3f} m'.format(state[0]))
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        sub.unregister()


if __name__ == '__main__':
    main()
