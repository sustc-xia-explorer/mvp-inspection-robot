#!/usr/bin/env python3
"""ROS1 Twist -> FW-mini overall-control driver; no motor-enable operations."""
import csv
import json
import math
from pathlib import Path
import sys
import threading
import time
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from command_guard import CommandGuard, fw_fields


def main():
    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool, SetBoolResponse
    from yhs_can_msgs.msg import ctrl_cmd

    rospy.init_node('fw_mini_bridge')
    cfg_path = Path(rospy.get_param('~config_file')).expanduser()
    cfg = yaml.safe_load(cfg_path.read_text())
    if cfg['robot']['kinematics'] != 'diff':
        raise ValueError('This interface supports diff commands only (vy=0).')
    gear = rospy.get_param('~ctrl_gear')  # required, never inferred from RC mode
    fw_fields((0., 0.), gear)
    hz = float(rospy.get_param('~publish_rate', 50.))
    if not math.isfinite(hz) or hz < 10 or hz > 200:
        raise ValueError('publish_rate must be in [10,200]')
    guard = CommandGuard(cfg['robot'], float(rospy.get_param('~command_timeout', .3)))
    base = rospy.get_param('~base_frame', 'base_link')
    lock = threading.RLock()
    output = Path(rospy.get_param('~output', str(Path.home()/'neupan_hardware_results'))).expanduser()
    output = output / time.strftime('bridge_%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    (output/'planner.yaml').write_text(cfg_path.read_text())
    (output/'run.json').write_text(json.dumps(dict(gear=gear, rate=hz, timeout=guard.timeout,
        angular_driver_unit='deg/s', feedback='command after guard, not measured velocity'), indent=2)+'\n')
    stream = (output/'commands.csv').open('w', newline='')
    writer = csv.writer(stream)
    writer.writerow(['wall_seconds', 'dt', 'target_v', 'target_w', 'applied_v', 'applied_w',
                     'acceleration', 'angular_acceleration', 'enabled', 'emergency', 'fresh', 'stop_override',
                     'gear', 'driver_x_mps', 'driver_z_degps', 'driver_y_mps'])
    pub = rospy.Publisher('ctrl_cmd', ctrl_cmd, queue_size=1)
    feedback = rospy.Publisher('applied_cmd_vel', TwistStamped, queue_size=1)

    def command(msg):
        with lock:
            if not guard.receive(msg.linear.x, msg.angular.z, time.monotonic(), msg.linear.y):
                rospy.logerr('Invalid or lateral command: bridge disabled; inspect then enable again.')

    def emergency(msg):
        with lock:
            guard.set_emergency(msg.data)

    def enable(req):
        with lock:
            accepted = guard.enable(req.data)
            return SetBoolResponse(accepted, 'enabled; waiting for a fresh command' if accepted and req.data
                                   else ('disabled' if accepted else 'emergency active'))

    subs = [rospy.Subscriber('cmd_vel', Twist, command, queue_size=1),
            rospy.Subscriber('emergency_stop', Bool, emergency, queue_size=1)]
    service = rospy.Service('~enable', SetBool, enable)

    def publish(command):
        msg = ctrl_cmd()
        msg.ctrl_cmd_gear, msg.ctrl_cmd_x_linear, msg.ctrl_cmd_z_angular, msg.ctrl_cmd_y_linear = fw_fields(command, gear)
        pub.publish(msg)
        stamped = TwistStamped()
        stamped.header.stamp, stamped.header.frame_id = rospy.Time.now(), base
        stamped.twist.linear.x, stamped.twist.angular.z = command
        feedback.publish(stamped)
        return msg

    rospy.loginfo('FW-mini bridge DISABLED. Gear=%d, output=%s. Enable via ~enable after checks.', gear, output)
    last_sent = (0., 0.)
    started = previous = time.monotonic()
    try:
        while not rospy.is_shutdown():
            tick = time.monotonic()
            elapsed, previous = max(1e-6, tick-previous), tick
            with lock:
                before = last_sent
                applied = guard.tick(tick, min(elapsed, 1/hz))
                msg = publish(applied)
                last_sent = applied
                fresh = guard.receipt is not None and 0 <= tick-guard.receipt <= guard.timeout
                writer.writerow([tick-started, elapsed, *guard.target, *applied,
                    abs(applied[0]-before[0])/elapsed, abs(applied[1]-before[1])/elapsed,
                    guard.enabled, guard.emergency, fresh, not guard.enabled or guard.emergency, msg.ctrl_cmd_gear,
                    msg.ctrl_cmd_x_linear, msg.ctrl_cmd_z_angular, msg.ctrl_cmd_y_linear])
                stream.flush()
            time.sleep(max(0., 1/hz-(time.monotonic()-tick)))
    finally:
        with lock:
            guard.enable(False)
            msg = publish((0., 0.))  # shutdown/operator/emergency stop can bypass ramp
            tick = time.monotonic()
            elapsed = max(1e-6, tick-previous)
            writer.writerow([tick-started, elapsed, *guard.target, 0., 0.,
                abs(last_sent[0])/elapsed, abs(last_sent[1])/elapsed,
                False, guard.emergency, False, True, msg.ctrl_cmd_gear,
                msg.ctrl_cmd_x_linear, msg.ctrl_cmd_z_angular, msg.ctrl_cmd_y_linear])
            stream.flush()
        stream.close()


if __name__ == '__main__':
    main()
