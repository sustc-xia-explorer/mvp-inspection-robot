#!/usr/bin/env python3
"""Actual ROS1 multi-goal corridor regression, using sim_complete.launch.
Uses only /neupan_sim virtual topics on an isolated ROS master. No CAN/chassis.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from simulation_test import create_output_directory


def metrics(directory):
    rows=list(csv.DictReader((directory/'trajectory.csv').open()))
    stats={};previous=(0.,0.);peak_accel=0.;plan_ms=[]
    for row in rows:
        velocity=float(row['applied_vx']),float(row['applied_vy']);dt=float(row['dt'])
        peak_accel=max(peak_accel,math.dist(velocity,previous)/max(dt,1e-6));previous=velocity
        if row.get('planner_ms'):plan_ms.append(float(row['planner_ms']))
        if row.get('active')!='True' or row.get('acknowledged')!='True':continue
        key=row['goal_x']+','+row['goal_y']
        s=stats.setdefault(key,dict(min_clearance=math.inf,max_stall_seconds=0.,
            stall=0.,samples=0,unsolved_tracking_samples=0))
        s['samples']+=1;s['min_clearance']=min(s['min_clearance'],float(row['true_clearance']))
        low=(row['arrived']!='True' and float(row['goal_distance'])>.2 and math.hypot(*velocity)<.02)
        s['stall']=s['stall']+dt if low else 0.
        s['max_stall_seconds']=max(s['max_stall_seconds'],s['stall'])
        if float(row['goal_distance'])>.2 and row.get('solved')=='false':s['unsolved_tracking_samples']+=1
    for s in stats.values():s.pop('stall')
    plan_ms.sort()
    return dict(goals=stats,max_actual_xy_acceleration=peak_accel,
        planner_p95_ms=plan_ms[min(len(plan_ms)-1,int(.95*len(plan_ms)))] if plan_ms else None,
        planner_max_ms=max(plan_ms) if plan_ms else None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'config/fw_mini_omni.yaml')
    p.add_argument('--scenario',type=Path,default=ROOT/'config/scenario_corridor.yaml')
    p.add_argument('--timeout',type=float,default=150.)
    p.add_argument('--scenario-goal',action='store_true',help='Test just the supplied scenario default goal')
    a=p.parse_args();a.output=create_output_directory(a.output.resolve())
    with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(port)
    os.environ['ROS_IP']='127.0.0.1'
    os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='neupan_smooth_roslogs_')
    masterlog=(a.output/'roscore.log').open('w')
    master=subprocess.Popen(['roscore','-p',str(port)],stdout=masterlog,stderr=subprocess.STDOUT)
    launch=None
    try:
        for _ in range(100):
            try:urllib.request.urlopen(os.environ['ROS_MASTER_URI'],timeout=.1);break
            except urllib.error.HTTPError:break
            except OSError:time.sleep(.1)
        else:raise RuntimeError('ROS master did not start')
        time.sleep(.8)
        if master.poll() is not None:raise RuntimeError('ROS master exited')
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool
        rospy.init_node('smooth_corridor_regression',anonymous=True,disable_signals=True)
        pose=[-4.,0.];done=[False]
        def on_odom(msg):pose[:]=[msg.pose.pose.position.x,msg.pose.pose.position.y]
        subs=[rospy.Subscriber('/neupan_sim/odom',Odometry,on_odom,queue_size=1),
              rospy.Subscriber('/neupan_sim/neupan_arrive',Bool,lambda m:done.__setitem__(0,m.data),queue_size=1)]
        publisher=rospy.Publisher('/neupan_sim/goal',PoseStamped,queue_size=1)
        log=(a.output/'roslaunch.log').open('w')
        launch=subprocess.Popen(['roslaunch',str(ROOT/'launch/sim_complete.launch'),'use_rviz:=false',
            'config_file:='+str(a.config.resolve()),'scenario_file:='+str(a.scenario.resolve()),
            'output:='+str(a.output/'world')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rospy.wait_for_message('/neupan_sim/odom',Odometry,timeout=15.)
        time.sleep(.8)
        reports=[]
        import yaml
        targets=([yaml.safe_load(a.scenario.read_text())['default_goal']] if a.scenario_goal else
                 [(4.4,-.5),(-4.,0.),(-1.65,.65)])
        for goal in targets:
            done[0]=False;message=PoseStamped();message.header.frame_id='neupan_sim/map'
            message.header.stamp=rospy.Time.now();message.pose.orientation.w=1.
            message.pose.position.x,message.pose.position.y=goal
            publisher.publish(message);began=time.monotonic();next_update=began+20.
            while not (done[0] and math.dist(pose,goal)<.1):
                if launch.poll() is not None:raise RuntimeError('sim_complete exited')
                if time.monotonic()-began>a.timeout:raise RuntimeError('Goal timed out: '+str(goal)+', current='+str(pose))
                if time.monotonic()>next_update:
                    print(json.dumps(dict(goal=goal,seconds=time.monotonic()-began,pose=pose,remaining=math.dist(pose,goal))),flush=True)
                    next_update+=20.
                time.sleep(.1)
            result=dict(goal=goal,seconds=time.monotonic()-began,error=math.dist(pose,goal),arrived=True)
            reports.append(result);print(json.dumps(result),flush=True);time.sleep(.6)
        rospy.signal_shutdown('test complete')
    finally:
        if launch:
            os.killpg(launch.pid,signal.SIGINT)
            try:launch.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
        master.send_signal(signal.SIGINT)
        try:master.wait(timeout=5)
        except subprocess.TimeoutExpired:master.kill();master.wait()
        masterlog.close()
    summary=json.loads((a.output/'world/summary.json').read_text())
    detail=metrics(a.output/'world')
    report=dict(sequence=reports,world=summary,metrics=detail)
    passed=(len(reports)==len(targets) and all(x['status']=='arrived' for x in summary['goals']) and
            summary['collision_count']==0 and summary['min_true_clearance']>=.15 and
            detail['max_actual_xy_acceleration']<=.30001 and
            all(s['max_stall_seconds']<3. for s in detail['goals'].values()))
    report['passed']=passed
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
