#!/usr/bin/env python3
"""Real ROS1 node interface regression, with an isolated optional ROS master.
Only namespaced virtual TF/scan/path/cmd topics are used; no chassis messages.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--node',type=Path,required=True)
    p.add_argument('--with-master',action='store_true')
    p.add_argument('--rviz-corridor',action='store_true',help='also run sim_complete.launch headlessly with omni')
    p.add_argument('--output',type=Path,default=ROOT/'test/results/ros1_models')
    a=p.parse_args()
    sys.path.insert(0,str(ROOT/'scripts'))
    from simulation_test import create_output_directory
    a.output=create_output_directory(a.output.expanduser().resolve())
    print('Results:',a.output,flush=True)
    master=None
    try:
        if a.with_master:
            with socket.socket() as s:
                s.bind(('127.0.0.1',0));port=s.getsockname()[1]
            os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(port)
            os.environ['ROS_IP']='127.0.0.1'
            os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='neupan_omni_roslogs_')
            masterlog=(a.output/'roscore.log').open('w')
            master=subprocess.Popen(['roscore','-p',str(port)],stdout=masterlog,stderr=subprocess.STDOUT)
            for _ in range(100):
                try:
                    urllib.request.urlopen(os.environ['ROS_MASTER_URI'],timeout=.1)
                    break
                except urllib.error.HTTPError:
                    break  # XML-RPC server is ready; GET is intentionally unsupported.
                except (OSError,ValueError):time.sleep(.1)
            else:raise RuntimeError('roscore failed to start')
            time.sleep(.8)
            if master.poll() is not None:raise RuntimeError('roscore exited; see roscore.log')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist,TwistStamped,PoseStamped,TransformStamped
        from nav_msgs.msg import Path as ROSPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool
        rospy.init_node('omni_port_regression',anonymous=True,disable_signals=True)
        results=[]
        for name,model,yaw,goal in [('omni_lateral','omni',0.,(0.,.8)),
                                     ('omni_rotated','omni',math.pi/2,(.8,0.)),
                                     ('diff_regression','diff',0.,(.8,0.))]:
            namespace='/port_test_'+name
            frame=name+'/map';base=name+'/base_link'
            config=ROOT/'config'/('fw_mini_omni.yaml' if model=='omni' else 'fw_mini_balanced.yaml')
            state=[0.,0.,yaw];commands=[];applied=[0.,0.];emergency=[False];arrived=[False]
            scans=[True];transforms=[True];active=[True];mutex=threading.RLock()
            tf=tf2_ros.TransformBroadcaster()
            scanpub=rospy.Publisher(namespace+'/scan',LaserScan,queue_size=1)
            pathpub=rospy.Publisher(namespace+'/path',ROSPath,queue_size=1,latch=True)
            feedback=rospy.Publisher(namespace+'/applied',TwistStamped,queue_size=1)
            def on_cmd(msg):
                with mutex:
                    commands.append((time.monotonic(),msg.linear.x,msg.linear.y,msg.angular.z))
                    applied[:]=[msg.linear.x,msg.linear.y if model=='omni' else msg.angular.z]
            subs=[rospy.Subscriber(namespace+'/cmd',Twist,on_cmd,queue_size=1),
                  rospy.Subscriber(namespace+'/stop',Bool,lambda m: emergency.__setitem__(0,m.data),queue_size=1),
                  rospy.Subscriber(namespace+'/arrive',Bool,lambda m: arrived.__setitem__(0,m.data),queue_size=1)]
            args=[str(a.node),'__name:='+name,'_config_file:='+str(config),
                  '_dune_checkpoint:='+str(ROOT/'models/fw_mini.bin'),'_map_frame:='+frame,
                  '_base_frame:='+base,'_lidar_frame:='+base,'_control_rate:=10.',
                  '_scan_timeout:=0.3','_pose_timeout:=0.3','_applied_cmd_topic:='+namespace+'/applied']
            for src,dst in [('scan','scan'),('initial_path','path'),('neupan_cmd_vel','cmd'),
                            ('neupan_emergency_stop','stop'),('neupan_arrive','arrive'),
                            ('neupan_plan','plan'),('neupan_diagnostics','diagnostics'),
                            ('neupan_initial_path','initial_path'),('neupan_ref_state','ref_state'),
                            ('neupan_waypoints','waypoints'),('neupan_goal','goal'),
                            ('dune_point_markers','dune'),('nrmp_point_markers','nrmp'),('robot_marker','robot')]:
                args.append('/'+src+':='+namespace+'/'+dst)
            log=(a.output/(name+'.log')).open('w')
            node=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
            from sys import path as syspath
            syspath.insert(0,str(ROOT/'scripts'))
            from planar_kinematics import PlanarPlant
            plant=PlanarPlant(__import__('yaml').safe_load(config.read_text())['robot'])
            def publish():
                last=time.monotonic()
                while active[0] and not rospy.is_shutdown():
                    now=time.monotonic();dt=min(now-last,.04);last=now
                    with mutex:
                        if not emergency[0]:state[:]=plant.integrate(state,applied,dt)
                        stamp=rospy.Time.now()
                        if transforms[0]:
                            t=TransformStamped();t.header.frame_id=frame;t.child_frame_id=base;t.header.stamp=stamp
                            t.transform.translation.x,t.transform.translation.y=state[:2]
                            t.transform.rotation.z,t.transform.rotation.w=math.sin(yaw/2),math.cos(yaw/2)
                            tf.sendTransform(t)
                        if scans[0]:
                            s=LaserScan();s.header.stamp=stamp;s.header.frame_id=base
                            s.angle_min,s.angle_max,s.angle_increment=-math.pi,math.pi,math.pi/90
                            s.range_min,s.range_max=.1,5.;s.ranges=[float('inf')]*181;scanpub.publish(s)
                        fb=TwistStamped();fb.header.stamp=stamp;fb.header.frame_id=base
                        fb.twist.linear.x=applied[0]
                        if model=='omni':fb.twist.linear.y=applied[1]
                        else:fb.twist.angular.z=applied[1]
                        feedback.publish(fb)
                    time.sleep(.02)
            worker=threading.Thread(target=publish,daemon=True);worker.start()
            try:
                time.sleep(1.)
                path=ROSPath();path.header.frame_id=frame
                for x,y in [(0.,0.),goal]:
                    pose=PoseStamped();pose.header=path.header
                    pose.pose.position.x,pose.pose.position.y=x,y;pose.pose.orientation.w=1.
                    path.poses.append(pose)
                pathpub.publish(path)
                deadline=time.monotonic()+18
                while not arrived[0] and time.monotonic()<deadline:
                    if node.poll() is not None:raise RuntimeError('node exited: '+name)
                    time.sleep(.05)
                distance=math.dist(state[:2],goal)
                assert arrived[0] and distance<.12,(name,state,distance)
                if model=='omni':
                    assert max(abs(c[3]) for c in commands)<1e-10
                    assert max(abs(c[2]) for c in commands)>.02
                    assert abs(state[2]-yaw)<1e-10
                else:assert max(abs(c[2]) for c in commands)<1e-10
                # New path must move before testing dropout, so a stationary goal
                # cannot produce a false-positive stop result.
                for pose in path.poses:pose.pose.position.x+=1.
                pathpub.publish(path);arrived[0]=False
                time.sleep(1.2)
                assert math.hypot(*applied)>.01,(name,'did not resume after replan')
                scans[0]=False;time.sleep(.65)
                assert emergency[0] and max(map(abs,applied))<1e-10,(name,'scan loss did not stop')
                scans[0]=True;time.sleep(1.)
                assert math.hypot(*applied)>.01,(name,'did not resume with fresh scan')
                transforms[0]=False;time.sleep(.7)
                assert emergency[0] and max(map(abs,applied))<1e-10,(name,'TF loss did not stop')
                report=dict(case=name,passed=True,arrival_distance=distance,
                            max_body_vx=max(abs(c[1]) for c in commands),
                            max_body_vy=max(abs(c[2]) for c in commands),
                            max_yawrate=max(abs(c[3]) for c in commands),
                            scan_loss_stop=True,tf_loss_stop=True,commands=len(commands))
                results.append(report);print(json.dumps(report),flush=True)
            finally:
                active[0]=False;worker.join(timeout=2)
                node.send_signal(signal.SIGINT)
                try:node.wait(timeout=4)
                except subprocess.TimeoutExpired:node.kill();node.wait()
                log.close()
                for sub in subs:sub.unregister()
                for pub in (scanpub,pathpub,feedback):pub.unregister()
        if a.rviz_corridor:
            done=[False]
            sub=rospy.Subscriber('/neupan_sim/neupan_arrive',Bool,
                                 lambda msg: done.__setitem__(0,msg.data),queue_size=1)
            sim_output=(a.output/'rviz_corridor').resolve()
            log=(a.output/'rviz_corridor.log').open('w')
            launch=subprocess.Popen(['roslaunch',str(ROOT/'launch/sim_complete.launch'),
                'use_rviz:=false','auto_goal:=true',
                'config_file:='+str(ROOT/'config/fw_mini_omni.yaml'),
                'output:='+str(sim_output)],stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                deadline=time.monotonic()+150
                while not done[0] and time.monotonic()<deadline:
                    if launch.poll() is not None:raise RuntimeError('sim_complete exited; see log')
                    time.sleep(.2)
                assert done[0],'RViz corridor did not arrive within 150 seconds'
            finally:
                os.killpg(launch.pid,signal.SIGINT)
                try:launch.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(launch.pid,signal.SIGKILL);launch.wait()
                log.close();sub.unregister()
            summary=json.loads((sim_output/'summary.json').read_text())
            assert summary['arrived'] and summary['collision_count']==0,summary
            assert summary['min_true_clearance']>0,summary
            assert abs(summary['final_pose'][2])<1e-9,summary
            assert summary['max_normal_acceleration']<=.300001,summary
            assert summary['max_normal_direction_rate']<=3.000001,summary
            report=dict(case='ros1_rviz_corridor',passed=True,**summary)
            results.append(report);print(json.dumps(report),flush=True)
        (a.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
        rospy.signal_shutdown('test complete')
        return 0
    finally:
        if master:
            master.send_signal(signal.SIGINT)
            try:master.wait(timeout=5)
            except subprocess.TimeoutExpired:master.kill();master.wait()
            masterlog.close()


if __name__=='__main__':raise SystemExit(main())
