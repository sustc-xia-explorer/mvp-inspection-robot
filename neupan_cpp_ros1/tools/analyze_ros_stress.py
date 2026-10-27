#!/usr/bin/env python3
"""Independent path-error and overshoot measurements from the ROS1 50 Hz log."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'test'))
from stress_omni_navigation import metrics,plot_case
from rviz_sim import World


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True);p.add_argument('--case',required=True)
    a=p.parse_args();worldlog=a.results/'world';out=a.results/'plots';out.mkdir(exist_ok=True)
    world=World(a.scenario,worldlog/'planner.yaml');path=json.loads((worldlog/'goal_001_path.json').read_text())
    raw=list(csv.DictReader((worldlog/'trajectory.csv').open()))
    rows=[r for r in raw if r.get('active')=='True' and r.get('acknowledged')=='True']
    start=float(rows[0]['wall_seconds']);previous=(0.,0.);previous_a=(0.,0.);converted=[]
    max_acc=0.;jerks=[]
    for r in rows:
        velocity=tuple(float(r['applied_'+k]) for k in ('vx','vy'));dt=float(r['dt'])
        acceleration=tuple((velocity[i]-previous[i])/dt for i in (0,1))
        jerk=math.dist(acceleration,previous_a)/dt;previous=velocity;previous_a=acceleration
        max_acc=max(max_acc,math.hypot(*acceleration));jerks.append(jerk)
        converted.append(dict(t=float(r['wall_seconds'])-start,x=r['x'],y=r['y'],yaw=r['yaw'],
            body_vx=velocity[0],body_vy=velocity[1],speed=math.hypot(*velocity),clearance=r['true_clearance'],
            ax=acceleration[0],ay=acceleration[1],jerk=jerk))
    file=out/(a.case+'.csv')
    with file.open('w') as f:
        w=csv.DictWriter(f,fieldnames=converted[0].keys());w.writeheader();w.writerows(converted)
    (out/(a.case+'_path.json')).write_text(json.dumps(path)+'\n')
    m=metrics(file,path);summary=json.loads((a.results/'summary.json').read_text())
    m.update(case=a.case,passed=summary['passed'],min_clearance=summary['world']['min_true_clearance'],
             max_xy_acceleration=max_acc,seconds=float(rows[-1]['wall_seconds'])-start,
             rms_xy_jerk=math.sqrt(sum(x*x for x in jerks)/len(jerks)),
             note='ROS samples at variable ~50 Hz; moving-average ripple uses 11 samples (~0.22 s), not core 1.1 s.')
    if a.case=='hard_90_walled':m['corner_overshoot_m']=max(0.,max(float(r['x']) for r in rows)-2.)
    plot_case(out,dict(name=a.case,start=world.config['initial_pose'],goal=world.config['default_goal']),world,dict(m,**{'pass':m['passed']}))
    (a.results/'path_metrics.json').write_text(json.dumps(m,indent=2)+'\n');print(json.dumps(m,indent=2))

if __name__=='__main__':main()
