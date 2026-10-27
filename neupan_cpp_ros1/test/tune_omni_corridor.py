#!/usr/bin/env python3
"""Reproducible corridor tuning with the actual C++ core and a 50 Hz plant.
Defaults include reverse travel and goals near the screenshots, not only the
single default forward goal. Metrics are independent of the planner's pass flag.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from rviz_sim import World
from planar_kinematics import PlanarPlant,rotate_vector,control_difference
from simulation_test import create_output_directory


def cases(world, suite):
    forward=dict(name='forward',start=world.config['initial_pose'][:],goal=world.config['default_goal'][:])
    reverse=dict(name='reverse',start=[*world.config['default_goal'],0.],goal=world.config['initial_pose'][:2])
    middle=dict(name='middle_left',start=[.5,-.55,0.],goal=[-1.65,.65])
    rotated=dict(name='rotated_short',start=[.6,-.55,1.88],goal=[1.28,.484])
    if suite=='forward':return [forward]
    if suite=='reverse':return [reverse]
    if suite=='screen':return [middle,rotated]
    if suite=='sequence':return [forward,reverse,dict(name='third_goal',start=[-4.,0.,0.],goal=[-1.65,.65])]
    if suite=='hot_replan':return [dict(name='hot_reverse',start=forward['start'],goal=forward['goal'],
        switch_at=30.,next_goal=[-4.,0.])]
    if suite=='density':return [dict(name='density_'+str(spacing),start=[0.,0.,0.],goal=[1.,0.],empty=True,spacing=spacing)
                                for spacing in (.005,.01,.04,1.)]
    result=[forward,reverse,middle,rotated]
    if suite=='all':result += [dict(name='open_reverse',start=[0.,0.,0.],goal=[-1.,.5],empty=True),
                              dict(name='open_lateral',start=[0.,0.,math.pi/2],goal=[0.,-1.],empty=True)]
    return result


def run_case(binary, planner, model, world, case, output, timeout=160., session=None):
    state=case['start'][:];goal=case['goal'];initial_yaw=state[2]
    path=case.get('path') or ([state[:2],goal] if case.get('empty') else world.route(state,goal))
    if 'spacing' in case:
        count=math.ceil(math.dist(state[:2],goal)/case['spacing'])
        path=[tuple(state[j]+(goal[j]-state[j])*i/count for j in (0,1)) for i in range(count+1)]
    proc=(session[0] if session else subprocess.Popen([str(binary),str(planner),str(model)],stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,stderr=(output/(case['name']+'.log')).open('w'),text=True,bufsize=1))
    def query(line):
        proc.stdin.write(line+'\n');proc.stdin.flush();answer=proc.stdout.readline().strip()
        if not answer:raise RuntimeError('C++ core terminated, see case log')
        return answer
    angles=[math.atan2(path[min(i+1,len(path)-1)][1]-p[1],path[min(i+1,len(path)-1)][0]-p[0]) for i,p in enumerate(path)]
    command=('REPATH '+' '.join(map(str,state))+' ') if session else 'PATH '
    query(command+str(len(path))+' '+' '.join(str(x) for i,p in enumerate(path) for x in (*p,angles[i])))
    (output/(case['name']+'_path.json')).write_text(json.dumps(path)+'\n')
    plant=session[1] if session else PlanarPlant(world.robot)
    dt=world.planner['step_time'];rows=[];arrived=False
    min_gap=math.inf;low_time=0.;max_low_time=0.;total_low=0.;speed_tv=0.;direction_tv=0.
    max_accel=0.;sum_accel2=0.;sum_jerk2=0.;max_jerk=0.;previous_velocity=(0.,0.);previous_accel=(0.,0.)
    solve_times=[];solved=0;unsolved=0;stops=0;previous_speed=0.;previous_control=(0.,0.)
    began=time.monotonic()
    try:
        for tick in range(math.ceil(timeout/dt)):
            if case.get('switch_at') is not None and tick==round(case['switch_at']/dt):
                goal=case['next_goal']
                path=world.route(state,goal)
                angles=[math.atan2(path[min(i+1,len(path)-1)][1]-p[1],path[min(i+1,len(path)-1)][0]-p[0]) for i,p in enumerate(path)]
                query('REPATH '+' '.join(map(str,state))+' '+str(len(path))+' '+
                      ' '.join(str(x) for i,p in enumerate(path) for x in (*p,angles[i])))
                (output/(case['name']+'_replanned_path.json')).write_text(json.dumps(path)+'\n')
            points=[] if case.get('empty') else world.scan(state)[1]
            values=list(map(float,query('STEP '+' '.join(map(str,state))+' '+str(len(points))+' '+
                            ' '.join(str(x) for p in points for x in p)).split()))
            action=values[:2];stop=bool(values[3]);arrived=bool(values[4]);braking=bool(values[9])
            solved += bool(values[2]);unsolved += not bool(values[2]) and not arrived and not braking
            solve_times.append(values[6]);stops+=stop
            body=rotate_vector(action,-state[2])
            for _ in range(5):
                actual=plant.apply(body,state[2],dt/5,emergency=stop)
                state=plant.integrate(state,actual,dt/5)
                gap=math.inf if case.get('empty') else world.clearance(state)
                min_gap=min(min_gap,gap)
            query('APPLIED '+' '.join(map(str,rotate_vector(actual,state[2]))))
            distance=math.dist(state[:2],goal);speed=math.hypot(*actual)
            accel=tuple((actual[j]-previous_velocity[j])/dt for j in (0,1))
            jerk=math.dist(accel,previous_accel)/dt
            max_accel=max(max_accel,math.hypot(*accel));sum_accel2+=sum(x*x for x in accel)
            max_jerk=max(max_jerk,jerk);sum_jerk2+=jerk*jerk
            speed_tv+=abs(speed-previous_speed)
            direction_tv+=abs(control_difference(plant.control,previous_control,world.robot)[1])
            low=(speed<.02 and distance>.2 and tick*dt>2.)
            low_time=low_time+dt if low else 0.;total_low+=dt if low else 0.
            max_low_time=max(max_low_time,low_time)
            rows.append([tick*dt,*state,*action,*actual,*plant.control,gap,distance,speed,
                         values[2],stop,arrived,values[6],*accel,jerk,values[5]])
            previous_speed=speed;previous_velocity=actual;previous_accel=accel;previous_control=plant.control
            if arrived or min_gap<=0 or max_low_time>6.:break
    finally:
        if not session:proc.stdin.close();proc.wait(timeout=10)
    solve_times.sort()
    result=dict(case=case['name'],arrived=arrived,collision=min_gap<=0,
                min_clearance=None if not math.isfinite(min_gap) else min_gap,
                goal_error=math.dist(state[:2],goal),final_pose=state,seconds=len(rows)*dt,steps=len(rows),
                max_stall_seconds=max_low_time,total_low_speed_seconds=total_low,
                max_xy_acceleration=max_accel,rms_xy_acceleration=math.sqrt(sum_accel2/len(rows)),
                max_xy_jerk=max_jerk,rms_xy_jerk=math.sqrt(sum_jerk2/len(rows)),
                speed_total_variation=speed_tv,direction_total_variation=direction_tv,
                solved=solved,unsolved=unsolved,emergency_stops=stops,
                solve_p95_ms=solve_times[min(len(solve_times)-1,int(len(solve_times)*.95))],
                solve_max_ms=max(solve_times),yaw_drift=abs(state[2]-initial_yaw),
                reference_min_clearance=None if case.get('empty') else min(world.clearance([*p,initial_yaw]) for p in path),
                elapsed_seconds=time.monotonic()-began)
    with (output/(case['name']+'.csv')).open('w') as f:
        w=csv.writer(f);w.writerow(['t','x','y','yaw','raw_vx','raw_vy','body_vx','body_vy',
            'control_v','control_phi','clearance','goal_distance','speed','solved','stop','arrived',
            'solve_ms','ax','ay','jerk','dune_distance']);w.writerows(rows)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--planner',type=Path,default=ROOT/'config/fw_mini_omni.yaml')
    p.add_argument('--scenario',type=Path,default=ROOT/'config/scenario_corridor.yaml')
    p.add_argument('--model',type=Path,default=ROOT/'models/fw_mini.bin')
    p.add_argument('--suite',choices=['forward','reverse','screen','corridor','density','sequence','hot_replan','all'],default='corridor')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--timeout',type=float,default=160.)
    a=p.parse_args();a.output=create_output_directory(a.output.resolve())
    world=World(a.scenario,a.planner);reports=[]
    session=None
    if a.suite=='sequence':
        process=subprocess.Popen([str(a.binary),str(a.planner),str(a.model)],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=(a.output/'core_sequence.log').open('w'),text=True,bufsize=1)
        session=(process,PlanarPlant(world.robot))
    for c in cases(world,a.suite):
        if session and reports and 'final_pose' in reports[-1]:c['start']=reports[-1]['final_pose'][:]
        try:r=run_case(a.binary,a.planner,a.model,world,c,a.output,a.timeout,session)
        except ValueError as e:r=dict(case=c['name'],arrived=False,error=str(e))
        reports.append(r);print(json.dumps(r),flush=True)
        (a.output/'summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    if session:session[0].stdin.close();session[0].wait(timeout=10)
    return 0 if all(x.get('arrived') and not x.get('collision') for x in reports) else 1


if __name__=='__main__':raise SystemExit(main())
