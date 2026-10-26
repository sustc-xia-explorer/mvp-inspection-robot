#!/usr/bin/env python3
"""Real ROS1 transport test using isolated topics; never addresses the driver."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from yhs_can_msgs.msg import ctrl_cmd
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('fw_bridge_test', anonymous=True, disable_signals=True)
    ns = '/fw_bridge_selftest_'+str(os.getpid())
    values = []
    sub = rospy.Subscriber(ns+'/ctrl_cmd', ctrl_cmd, lambda m: values.append(m), queue_size=100)
    pub = rospy.Publisher(ns+'/cmd', Twist, queue_size=1)
    emergency = rospy.Publisher(ns+'/emergency', Bool, queue_size=1, latch=True)
    log = (args.output/'bridge.log').open('w')
    proc = subprocess.Popen([sys.executable, str(root/'scripts/fw_mini_bridge.py'),
        '__name:=bridge', '__ns:='+ns, '_ctrl_gear:=2',
        '_config_file:='+str(root/'config/fw_mini_balanced.yaml'),
        '_output:='+str(args.output), 'cmd_vel:='+ns+'/cmd',
        'ctrl_cmd:='+ns+'/ctrl_cmd', 'emergency_stop:='+ns+'/emergency',
        'applied_cmd_vel:='+ns+'/applied'], stdout=log, stderr=log, start_new_session=True)

    def advance(seconds, cmd=None):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            if proc.poll() is not None: raise RuntimeError('bridge exited; see bridge.log')
            if cmd is not None: pub.publish(cmd)
            time.sleep(.02)

    def stopped():
        assert values and abs(values[-1].ctrl_cmd_x_linear)<1e-6 and abs(values[-1].ctrl_cmd_z_angular)<1e-6

    try:
        rospy.wait_for_service(ns+'/bridge/enable', timeout=10)
        enable=rospy.ServiceProxy(ns+'/bridge/enable',SetBool)
        cmd=Twist();cmd.linear.x=.2;cmd.angular.z=.4
        advance(.5,cmd);stopped()
        assert enable(True).success
        advance(.2);stopped()  # cached disabled commands cannot start motion
        emergency.publish(Bool(False))
        advance(1.2,cmd)
        assert values[-1].ctrl_cmd_x_linear>.19
        assert abs(values[-1].ctrl_cmd_z_angular-math.degrees(.4))<1e-4
        assert all(m.ctrl_cmd_y_linear==0 and m.ctrl_cmd_gear==2 for m in values)
        advance(1.2);stopped()  # upstream command loss ramps to zero
        advance(.6,cmd)
        emergency.publish(Bool(True));advance(.15,cmd);stopped()
        assert not enable(True).success
        emergency.publish(Bool(False));advance(.15,cmd);stopped()
        assert enable(True).success
        invalid=Twist();invalid.linear.y=.1
        advance(.1,invalid);stopped()
        assert not enable(True).success
        enable(False)
        # Stop the process normally so the log is complete.
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
        rows=list(csv.DictReader(next(args.output.glob('bridge_*/commands.csv')).open()))
        normal=[r for r in rows if r['stop_override']=='False']
        maximum=max(float(r['acceleration']) for r in normal)
        angular=max(float(r['angular_acceleration']) for r in normal)
        assert maximum<=.3+1e-8 and angular<=.6+1e-8
        result=dict(passed=True, isolated_namespace=ns, samples=len(rows),
            disabled_stop=True, fresh_after_enable=True, timeout_braking=True,
            emergency_stop=True, rejects_lateral=True, angular_units='deg/s',
            max_normal_acceleration=maximum, max_normal_angular_acceleration=angular,
            note='Test gear 2 is arbitrary for transport validation, not a firmware recommendation.')
        (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
        log.close()


if __name__=='__main__': main()
