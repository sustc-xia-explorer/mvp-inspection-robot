#!/usr/bin/env python3
"""Fixed-seed omni navigation stress suite; real C++ planner, 50 Hz plant.
Target cases precede generated maps. All maps, routes, samples and failed cases
are retained. No per-map controller tuning is allowed.
"""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import sys
import yaml

from tune_omni_corridor import ROOT, run_case
sys.path.insert(0, str(ROOT/'scripts'))
from rviz_sim import World
from simulation_test import create_output_directory


def rect(x,y,length,width,yaw=0.):
    return dict(shape=dict(type='rectangle',length=length,width=width),pose=dict(x=x,y=y,yaw=yaw))


def room(width=8.,height=8.):
    return [rect(0.,-height/2,width+.2,.2),rect(0.,height/2,width+.2,.2),
            rect(-width/2,0.,.2,height),rect(width/2,0.,.2,height)]


def scenarios(output):
    """Seeds and geometry are fixed independently of controller outcomes."""
    template=yaml.safe_load((ROOT/'config/scenario_corridor.yaml').read_text())
    specs=[
        dict(name='bar_auto_180',obstacles=room()+[rect(0.,0.,.24,2.2)],
             start=[-.75,.1,0.],goal=[.75,.1]),
        dict(name='bar_retarget_moving',obstacles=room()+[rect(0.,0.,.24,2.2)],
             start=[-1.15,-1.75,0.],goal=[-1.15,.75],switch_at=8.,next_goal=[1.15,.75]),
        dict(name='bar_wrap_below',obstacles=room()+[rect(0.,0.,.24,2.2)],
             start=[-1.15,.75,0.],goal=[1.15,.75],via=[[-1.15,.75],[-.65,-1.75],[.65,-1.75],[1.15,.75]]),
        dict(name='bar_wrap_above',obstacles=room()+[rect(0.,0.,.24,2.2)],
             start=[1.15,-.75,0.],goal=[-1.15,-.75],via=[[1.15,-.75],[.65,1.75],[-.65,1.75],[-1.15,-.75]]),
        dict(name='hard_90_open',obstacles=room(),start=[-1.5,-1.,0.],goal=[0.,1.5],
             path=[[-1.5,-1.],[0.,-1.],[0.,1.5]]),
        dict(name='hard_90_double',obstacles=room(),start=[-2.,-1.5,0.],goal=[2.,1.],
             path=[[-2.,-1.5],[0.,-1.5],[0.,1.],[2.,1.]]),
        dict(name='short_acute_zigzag',obstacles=room(),start=[-1.5,-1.,0.],goal=[1.,1.],
             path=[[-1.5,-1.],[0.,-1.],[-.4,-.6],[.4,-.6],[0.,-.2],[1.,1.]]),
        # Free L is 1.8 m wide: horizontal y[-2.9,-1.1], vertical x[1.1,2.9].
        dict(name='hard_90_walled',obstacles=room()+[rect(-1.4,1.4,5.,5.),
             rect(0.,-3.45,8.,1.1),rect(3.45,0.,1.1,8.)],
             start=[-2.4,-2.,0.],goal=[2.,2.5],path=[[-2.4,-2.],[2.,-2.],[2.,2.5]]),
    ]
    for seed,count in [(1103,6),(2207,10),(3301,14),(4409,18),(5519,22),(6619,12),
                       (7103,5),(8209,9),(9311,13),(10427,17),
                       (11549,21),(12671,11),(13781,15),(14891,19)]:
        rng=random.Random(seed);width,height=(8.,8.) if seed!=6619 else (10.,6.)
        start=[-width/2+.85,-height/2+.85,math.pi/2 if seed==4409 else 0.]
        goal=[width/2-.85,height/2-.85]
        obstacles=room(width,height)
        attempts=0
        while len(obstacles)<count+4:
            attempts+=1
            x,y=rng.uniform(-width/2+.8,width/2-.8),rng.uniform(-height/2+.8,height/2-.8)
            if min(math.dist([x,y],p[:2]) for p in (start,goal))<1.05:continue
            if rng.random()<.3:
                obstacles.append(dict(shape=dict(type='circle',radius=rng.uniform(.16,.32)),pose=dict(x=x,y=y)))
            else:obstacles.append(rect(x,y,rng.uniform(.35,1.05),rng.uniform(.22,.55),rng.uniform(-math.pi,math.pi)))
        specs.append(dict(name='random_'+str(seed),seed=seed,obstacle_count=count,
            map_set='holdout' if seed>=7000 else 'diagnostic',
            width=width,height=height,obstacles=obstacles,start=start,goal=goal,generation_attempts=attempts))
    result=[]
    maps=output/'maps';maps.mkdir(exist_ok=True)
    for spec in specs:
        name=spec['name'];mapfile=maps/(name+'.yaml');scenario=maps/(name+'_scenario.yaml')
        environment=dict(world=dict(width=spec.get('width',8.),height=spec.get('height',8.)),obstacles=spec['obstacles'])
        mapfile.write_text(yaml.safe_dump(environment,sort_keys=False))
        config=copy.deepcopy(template);config.update(map_file=mapfile.name,initial_pose=spec['start'],default_goal=spec['goal'])
        if spec.get('path'):config['test_reference_path']=spec['path']
        if spec.get('via'):config['test_via_points']=spec['via']
        scenario.write_text(yaml.safe_dump(config,sort_keys=False))
        case={k:v for k,v in spec.items() if k not in ('obstacles','width','height')}
        case['scenario']=str(scenario);result.append(case)
    (output/'cases.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def route_via(world,case):
    if 'via' not in case:return
    path=[]
    for a,b in zip(case['via'],case['via'][1:]):
        part=world.route([*a,case['start'][2]],b)
        path.extend(part if not path else part[1:])
    case['path']=path


def metrics(csvfile,path,replanned=None,switch_at=None):
    import numpy as np
    rows=list(csv.DictReader(csvfile.open()))
    xy=np.array([[float(r['x']),float(r['y'])] for r in rows]);p=np.array(path)
    a=p[:-1];d=np.diff(p,axis=0);span=np.linalg.norm(d,axis=1);good=span>1e-10
    a,d,span=a[good],d[good],span[good];arc=np.r_[0.,np.cumsum(span)]
    progress=[];errors=[]
    for q in xy:
        f=np.clip(np.sum((q-a)*d,axis=1)/span**2,0.,1.)
        dist=np.linalg.norm(q-a-f[:,None]*d,axis=1);j=np.argmin(dist)
        errors.append(float(dist[j]));progress.append(float(arc[j]+f[j]*span[j]))
    if replanned is not None:
        for i,(q,row) in enumerate(zip(xy,rows)):
            if float(row['t'])<switch_at:continue
            route=np.array(replanned);start=route[:-1];delta=np.diff(route,axis=0)
            lengths=np.linalg.norm(delta,axis=1);valid=lengths>1e-10
            start,delta,lengths=start[valid],delta[valid],lengths[valid]
            fraction=np.clip(np.sum((q-start)*delta,axis=1)/lengths**2,0.,1.)
            errors[i]=float(np.min(np.linalg.norm(q-start-fraction[:,None]*delta,axis=1)))
    # Deadband removes floating noise; net travel backwards along the reference.
    regress=np.diff(progress)
    speed=np.array([float(r['speed']) for r in rows])
    residual=speed[5:-5]-np.convolve(speed,np.ones(11)/11,mode='valid') if len(speed)>10 else np.zeros(1)
    return dict(max_path_error=max(errors),rms_path_error=float(np.sqrt(np.mean(np.array(errors)**2))),
        speed_high_frequency_rms=float(np.sqrt(np.mean(residual**2))),
        backwards_progress_m=None if replanned is not None else float(-sum(regress[regress<-.001])),
        hard_corner_overshoot_m=max(0.,float(xy[:,0].max())) if csvfile.stem=='hard_90_open' else None)


def plot_case(out,case,world,result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle,Circle
    import matplotlib.transforms as transforms
    import numpy as np
    rows=list(csv.DictReader((out/(case['name']+'.csv')).open()));v=lambda key:np.array([float(r[key]) for r in rows])
    path=json.loads((out/(case['name']+'_path.json')).read_text());xy=np.array(path)
    fig,axes=plt.subplots(2,2,figsize=(13,9));ax=axes[0,0]
    for x,y,yaw,length,width in world.rectangles:
        patch=Rectangle((-length/2,-width/2),length,width,color='#333b44')
        patch.set_transform(transforms.Affine2D().rotate(yaw).translate(x,y)+ax.transData);ax.add_patch(patch)
    for x,y,r in world.scene['circles']:ax.add_patch(Circle((x,y),r,color='#333b44'))
    ax.plot(xy[:,0],xy[:,1],'--',color='#78ad43',label='Global/reference')
    if (out/(case['name']+'_replanned_path.json')).exists():
        new=json.loads((out/(case['name']+'_replanned_path.json')).read_text())
        ax.plot(*zip(*new),'--',color='#ba8d27',label='Replanned')
    ax.plot(v('x'),v('y'),color='#1976d2',label='Actual')
    for i in range(0,len(rows),max(1,len(rows)//12)):
        yaw=float(rows[i]['yaw']);length=world.robot['length'];width=world.robot['width']
        patch=Rectangle((-length/2,-width/2),length,width,fill=False,color='#1976d2',alpha=.4)
        patch.set_transform(transforms.Affine2D().rotate(yaw).translate(float(rows[i]['x']),float(rows[i]['y']))+ax.transData);ax.add_patch(patch)
    ax.scatter(*case['start'][:2],marker='o',color='green');ax.scatter(*case.get('next_goal',case['goal']),marker='*',s=120,color='red')
    ax.set(xlim=(-world.width/2,world.width/2),ylim=(-world.height/2,world.height/2),xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal');ax.legend()
    t=v('t');ax=axes[0,1];ax.plot(t,v('speed'),label='speed');ax.plot(t,v('body_vx'),label='body vx',alpha=.7);ax.plot(t,v('body_vy'),label='body vy',alpha=.7)
    ax.set(xlabel='time (s)',ylabel='velocity (m/s)');ax.legend();ax.grid(alpha=.25)
    ax=axes[1,0];ax.plot(t,v('clearance'),label='True footprint clearance');ax.axhline(.12,color='r',linestyle='--',label='0.12 m acceptance');ax.set(xlabel='time (s)',ylabel='clearance (m)');ax.legend();ax.grid(alpha=.25)
    ax=axes[1,1];ax.plot(t,np.hypot(v('ax'),v('ay')),label='XY acceleration');ax.plot(t,v('jerk'),alpha=.5,label='XY jerk');ax.set(xlabel='time (s)',ylabel='m/s^2; m/s^3');ax.legend();ax.grid(alpha=.25)
    fig.suptitle(f"{case['name']} | {'PASS' if result.get('pass') else 'FAIL'} | "
        f"clearance={result['min_clearance']:.3f} m | path error={result['max_path_error']:.3f} m | "
        f"time={result['seconds']:.1f} s | jerk RMS={result['rms_xy_jerk']:.3f}")
    fig.tight_layout();fig.savefig(out/(case['name']+'.png'),dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--planner',type=Path,default=ROOT/'config/fw_mini_omni.yaml')
    parser.add_argument('--model',type=Path,default=ROOT/'models/fw_mini.bin');parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--cases',default='all',help='all, targeted, random, holdout, or comma separated case names')
    parser.add_argument('--timeout',type=float,default=180.);args=parser.parse_args();out=create_output_directory(args.output.resolve())
    specs=scenarios(out);reports=[]
    (out/'planner.yaml').write_bytes(args.planner.read_bytes())
    for case in specs:
        name=case['name']
        if args.cases=='targeted' and name.startswith('random'):continue
        if args.cases=='random' and not name.startswith('random'):continue
        if args.cases=='holdout' and case.get('map_set')!='holdout':continue
        if args.cases not in ('all','targeted','random','holdout') and name not in args.cases.split(','):continue
        world=World(case['scenario'],args.planner)
        try:
            route_via(world,case)
            report=run_case(args.binary,args.planner,args.model,world,case,out,args.timeout)
            path=json.loads((out/(name+'_path.json')).read_text())
            replanned=(json.loads((out/(name+'_replanned_path.json')).read_text()) if case.get('switch_at') is not None else None)
            report.update(metrics(out/(name+'.csv'),path,replanned,case.get('switch_at')))
            report['pass']=bool(report['arrived'] and not report['collision'] and report['min_clearance']>=.12
                and report['max_path_error']<=.10 and report['max_stall_seconds']<=2.
                and report['max_xy_acceleration']<=.30 and report['unsolved']==0)
            plot_case(out,case,world,report)
        except ValueError as e:
            no_route=str(e).startswith('No route for the inflated')
            report=dict(case=name,**{'pass':False},error=str(e),geometric_no_path=no_route,
                arrived=False,outcome='global_no_route' if no_route else 'invalid_test_or_route')
        report.update(seed=case.get('seed'),obstacle_count=case.get('obstacle_count'),
            planner_sha256=hashlib.sha256(args.planner.read_bytes()).hexdigest())
        reports.append(report);(out/'summary.json').write_text(json.dumps(reports,indent=2)+'\n');print(json.dumps(report),flush=True)
    # A global no-route refusal is retained, never counted as an arrival or a
    # successful controller run. It is a valid safe outcome of the full stack.
    return 0 if (any(r.get('pass') for r in reports) and
                 all(r.get('pass') or r.get('geometric_no_path') for r in reports)) else 1

if __name__=='__main__':raise SystemExit(main())
