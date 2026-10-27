#!/usr/bin/env python3
"""Closed-loop exercise of the package-local C++ core, diff or omni.
No ROS master or hardware. The plant executes body-frame commands; the core
returns omni velocities in world coordinates. Rectangle clearance is analytic.
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from rviz_sim import World
from planar_kinematics import PlanarPlant, rotate_vector, control_difference
from simulation_test import clearance, create_output_directory


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--config', type=Path, default=ROOT/'config/fw_mini_omni.yaml')
    p.add_argument('--model', type=Path, default=ROOT/'models/fw_mini.bin')
    p.add_argument('--output', type=Path, default=ROOT/'test/results/omni_closed_loop')
    a = p.parse_args()
    cfg = yaml.safe_load(a.config.read_text()); model = cfg['robot']['kinematics']
    a.output=create_output_directory(a.output.expanduser().resolve())
    print('Results:',a.output,flush=True)
    # Lateral movement, diagonal at rotated body yaw, reversal, hard right angle,
    # narrow corridor and an A* detour around an obstacle exercise direction changes.
    cases = [dict(name='lateral', start=[0.,0.,0.], path=[[0.,0.],[0.,1.2]]),
             dict(name='diagonal_rotated', start=[0.,0.,math.pi/2], path=[[0.,0.],[1.,1.]]),
             dict(name='reverse', start=[0.,0.,-.8], path=[[0.,0.],[-1.,.7]]),
             dict(name='right_angle', start=[0.,0.,0.], path=[[0.,0.],[1.,0.],[1.,1.]]),
             dict(name='narrow_corridor', start=[0.,0.,0.], path=[[0.,0.],[1.5,0.]],
                  scene=dict(circles=[], walls=[[-1.,-.45,2.,-.45],[-1.,.45,2.,.45]]))]
    w=World(ROOT/'config/scenario_corridor.yaml',a.config)
    start=list(w.config['initial_pose']); goal=w.config['default_goal']
    cases.append(dict(name='rviz_corridor_astar',start=start,path=w.route(start,goal),world=w))
    results=[]
    for case in cases:
        proc=subprocess.Popen([str(a.binary),str(a.config),str(a.model)], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
        def query(line):
            proc.stdin.write(line+'\n');proc.stdin.flush()
            answer=proc.stdout.readline().strip()
            if not answer:
                raise RuntimeError(proc.stderr.read())
            return answer
        path=case['path']
        points=[[x,y,math.atan2(path[min(i+1,len(path)-1)][1]-y,
                               path[min(i+1,len(path)-1)][0]-x)] for i,(x,y) in enumerate(path)]
        assert query('PATH '+str(len(points))+' '+' '.join(str(x) for pose in points for x in pose))=='READY'
        plant=PlanarPlant(cfg['robot']); state=case['start'][:]; yaw0=state[2]
        scene=case.get('scene',dict(circles=[],walls=[])); min_gap=math.inf; solved=0; arrived=False
        max_rates=[0.,0.]; max_speed=0.; max_yaw_drift=0.; rows=[]; dt=cfg['step_time']
        began=time.monotonic()
        for tick in range(1600):
            obs=[]
            if 'world' in case:
                _,obs=case['world'].scan(state)
            else:
                for x1,y1,x2,y2 in scene['walls']:
                    for i in range(81): obs.append((x1+(x2-x1)*i/80,y1+(y2-y1)*i/80))
            response=query('STEP '+' '.join(map(str,state))+' '+str(len(obs))+' '+
                           ' '.join(str(x) for xy in obs for x in xy))
            values=list(map(float,response.split())); action=values[:2]; solved+=bool(values[2])
            stop=bool(values[3]); arrived=bool(values[4]); previous=plant.control
            body=rotate_vector(action,-state[2]) if model=='omni' else action
            actual=plant.apply(body,state[2],dt,emergency=stop)
            for _ in range(5):
                state=plant.integrate(state,actual,dt/5)
                gap=case['world'].clearance(state) if 'world' in case else clearance(
                    state,scene,cfg['robot']['length'],cfg['robot']['width'])
                min_gap=min(min_gap,gap)
            world=rotate_vector(actual,state[2]) if model=='omni' else actual
            assert query('APPLIED '+' '.join(map(str,world)))=='ACK'
            rates=[abs(x)/dt for x in control_difference(plant.control,previous,cfg['robot'])]
            if not stop: max_rates=[max(x,y) for x,y in zip(max_rates,rates)]
            max_speed=max(max_speed,abs(plant.control[0]))
            max_yaw_drift=max(max_yaw_drift,abs(state[2]-yaw0))
            rows.append([tick*dt,*state,*action,*actual,*plant.control,gap,arrived,stop,*rates])
            if arrived or min_gap<=0: break
        proc.stdin.close(); proc.wait(timeout=5)
        distance=math.dist(state[:2],path[-1])
        passed=(arrived and distance<=cfg['ipath']['arrive_threshold']+.02 and min_gap>0 and
                max_speed<=cfg['robot']['max_speed'][0]+1e-6 and
                all(r<=bound+1e-6 for r,bound in zip(max_rates,cfg['robot']['max_acce'])) and
                (model!='omni' or max_yaw_drift<1e-9))
        report=dict(case=case['name'],kinematics=model,passed=passed,arrived=arrived,
                    steps=len(rows),simulated_seconds=len(rows)*dt,goal_distance=distance,
                    min_true_clearance=min_gap if math.isfinite(min_gap) else None,
                    max_speed=max_speed,max_control_change_rates=max_rates,
                    max_yaw_drift=max_yaw_drift,solved_cycles=solved,elapsed_seconds=time.monotonic()-began)
        results.append(report); print(json.dumps(report),flush=True)
        with (a.output/(case['name']+'.csv')).open('w') as f:
            writer=csv.writer(f);writer.writerow(['t','x','y','yaw','action0','action1','body0','body1',
                'speed','direction_or_yawrate','clearance','arrived','stop','rate0','rate1']);writer.writerows(rows)
    summary=dict(config=str(a.config),model=model,cases=results,passed=sum(r['passed'] for r in results),total=len(results))
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':
    raise SystemExit(main())
