#!/usr/bin/env python3
"""Compare copied IPC omni recordings; never starts ROS or chassis commands."""
from pathlib import Path
import argparse,csv,hashlib,json,math,sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from audit_ipc_results import scene,geom_gap

def load(path):return json.loads(path.read_text())
def rows(path):
    with path.open() as stream:return list(csv.DictReader(stream))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def yes(v):return str(v).lower() in ('true','1','1.0')
def vec(trace,key):return np.array([float(r[key]) for r in trace])
def timing(values):
    a=np.asarray(values)
    return dict(samples=len(a),mean_ms=float(a.mean()),p95_ms=float(np.percentile(a,95)),
                max_ms=float(a.max()),above_100ms=int(sum(a>100))) if len(a) else None
def distance_to_path(xy,path):
    path=np.asarray(path);d=np.diff(path,axis=0);l2=np.sum(d*d,axis=1);good=l2>1e-20
    start,d,l2=path[:-1][good],d[good],l2[good]
    return np.array([np.min(np.linalg.norm(q-start-np.clip(np.sum((q-start)*d,axis=1)/l2,0,1)[:,None]*d,axis=1)) for q in xy])
def errors(directory,name,trace):
    xy=np.c_[vec(trace,'x'),vec(trace,'y')];t=vec(trace,'t')
    result=distance_to_path(xy,load(directory/(name+'_path.json')))
    other=directory/(name+'_replanned_path.json')
    if other.exists():result[t>=8.]=distance_to_path(xy[t>=8.],load(other))
    return result
def longest(mask,dt):
    current=peak=0.
    for flag,span in zip(mask,np.broadcast_to(dt,len(mask))):
        current=current+span if flag else 0.;peak=max(current,peak)
    return float(peak)

def core(directory,reference,geometry=False,physical_limits=None):
    config=yaml.safe_load((directory/'planner.yaml').read_text());robot=config['robot'];dt=config['step_time']
    reports=[];times=[];issues=[];count=0
    for original in load(directory/'summary.json'):
        r=dict(original);name=r['case'];problems=[]
        if r.get('planner_sha256') and r['planner_sha256']!=sha(directory/'planner.yaml'):problems.append('saved planner hash mismatch')
        if r.get('geometric_no_path'):
            r.update(audit_issues=problems);reports.append(r);issues+=problems;continue
        trace=rows(directory/(name+'.csv'));count+=len(trace);error=errors(directory,name,trace)
        velocity=np.c_[vec(trace,'body_vx'),vec(trace,'body_vy')]
        accel=np.diff(np.vstack([np.zeros(2),velocity]),axis=0)/dt
        jerk=np.linalg.norm(np.diff(np.vstack([np.zeros(2),accel]),axis=0)/dt,axis=1)
        low=(vec(trace,'speed')<.02)&(vec(trace,'goal_distance')>.2)&(vec(trace,'t')>2.)
        computed=dict(max_path_error=float(error.max()),rms_path_error=float(np.sqrt(np.mean(error**2))),
                      rms_xy_jerk=float(np.sqrt(np.mean(jerk**2))),max_stall_seconds=longest(low,dt),
                      goal_error=float(trace[-1]['goal_distance']))
        for key,value in computed.items():
            if abs(r[key]-value)>1e-8:problems.append('summary/CSV mismatch: '+key)
        if len(trace)!=r['steps'] or math.dist(r['final_pose'][:2],[float(trace[-1][k]) for k in ['x','y']])>1e-8:
            problems.append('final state/length mismatch')
        speeds=vec(trace,'speed');control=np.c_[vec(trace,'control_v'),vec(trace,'control_phi')]
        rates=np.diff(np.vstack([np.zeros(2),control]),axis=0)/dt
        rates[:,1]=np.arctan2(np.sin(rates[:,1]*dt),np.cos(rates[:,1]*dt))/dt
        limits=physical_limits if physical_limits is not None else robot
        if speeds.max()>limits['max_speed'][0]+1e-6:problems.append('speed exceeded')
        if np.any(np.max(np.abs(rates),axis=0)>np.array(limits['max_acce'])+1e-6):problems.append('control rate exceeded')
        if np.linalg.norm(accel,axis=1).max()>.300001:problems.append('XY acceleration exceeded')
        if geometry:
            env=scene(yaml.safe_load((directory/'maps'/(name+'.yaml')).read_text()))
            gaps=np.array([geom_gap(row,env,robot) for row in trace])
            deviation=float(np.max(np.abs(gaps-vec(trace,'clearance'))))
            if deviation>1e-7:problems.append('footprint geometry mismatch')
            r['geometry_max_error_m']=deviation
        solved=[float(row['solve_ms']) for row in trace if yes(row['solved']) and not yes(row['stop'])]
        times+=solved;r.update(computed,planner_timing=timing(solved),audit_issues=problems)
        if reference is not None:
            local=rows(reference/(name+'.csv'))
            r['development_trace_max_xy_difference_m']=max(math.dist([float(a[k]) for k in ['x','y']],[float(b[k]) for k in ['x','y']]) for a,b in zip(trace,local)) if len(local)==len(trace) else None
        r['target_pass']=bool(r['pass'] and r['max_stall_seconds']<=1.5+1e-9 and not problems)
        r['speed_at_arrival_mps']=float(trace[-1]['speed'])
        reports.append(r);issues += [name+': '+v for v in problems]
    feasible=[r for r in reports if not r.get('geometric_no_path')]
    result=dict(total_maps=len(reports),feasible=len(feasible),arrived=sum(r['arrived'] for r in feasible),
        original_pass=sum(r['pass'] for r in feasible),target_pass=sum(r['target_pass'] for r in feasible),
        no_route=[r['case'] for r in reports if r.get('geometric_no_path')],
        collisions=sum(r['collision'] for r in feasible),unsolved=sum(r['unsolved'] for r in feasible),
        min_clearance_m=min(r['min_clearance'] for r in feasible),
        mean_max_path_error_m=float(np.mean([r['max_path_error'] for r in feasible])),
        mean_rms_path_error_m=float(np.mean([r['rms_path_error'] for r in feasible])),
        mean_goal_error_m=float(np.mean([r['goal_error'] for r in feasible])),
        mean_jerk_rms=float(np.mean([r['rms_xy_jerk'] for r in feasible])),
        max_stall_s=max(r['max_stall_seconds'] for r in feasible),
        sum_seconds=sum(r['seconds'] for r in feasible),timing=timing(times),csv_rows=count,
        max_path_error_m=max(r['max_path_error'] for r in feasible),issues=issues,cases=reports)
    return result

def ros(directory,expected,model):
    trace=rows(directory/'trajectory.csv');summary=load(directory/'summary.json');run=load(directory/'run.json')
    config=yaml.safe_load((directory/'planner.yaml').read_text());robot=config['robot'];issues=[]
    if config!=yaml.safe_load(expected.read_text()):issues.append('profile values mismatch')
    if run['config_sha256']!=sha(directory/'planner.yaml'):issues.append('saved profile hash mismatch')
    if run['model_sha256']!=sha(model):issues.append('model hash mismatch')
    env=scene(yaml.safe_load((directory/'map.yaml').read_text()));gaps=np.array([geom_gap(r,env,robot) for r in trace])
    gap_error=float(np.max(np.abs(gaps-vec(trace,'true_clearance'))))
    if gap_error>1e-7:issues.append('footprint geometry mismatch')
    velocity=np.c_[vec(trace,'applied_vx'),vec(trace,'applied_vy')];dt=vec(trace,'dt')
    accel=np.diff(np.vstack([np.zeros(2),velocity]),axis=0)/dt[:,None]
    jerk=np.linalg.norm(np.diff(np.vstack([np.zeros(2),accel]),axis=0)/dt[:,None],axis=1)
    wall=vec(trace,'wall_seconds');speeds=np.linalg.norm(velocity,axis=1)
    valid=np.array([yes(r['active']) and yes(r['acknowledged']) for r in trace]);times=[];previous=None
    for r in trace:
        if yes(r['solved']) and r['planner_ms'] and r['planner_ms']!=previous:
            times.append(float(r['planner_ms']));previous=r['planner_ms']
    goals=[];post_startup=np.zeros(len(trace),dtype=bool)
    for i,goal in enumerate(summary['goals']):
        end=goal.get('arrived_wall_seconds',summary['goals'][i+1]['wall_seconds'] if i+1<len(summary['goals']) else wall[-1])
        mask=(wall>=goal['wall_seconds'])&(wall<=end)&valid
        post_startup |= mask&(wall-goal['wall_seconds']>2.)
        low=mask&(wall-goal['wall_seconds']>2.)&np.array([float(r['goal_distance'])>.2 if r['goal_distance'] else False for r in trace])&(speeds<.02)
        stall=longest(low,dt)
        path=load(directory/f'goal_{i+1:03d}_path.json')
        err=distance_to_path(np.c_[vec(trace,'x'),vec(trace,'y')][mask],path)
        if goal['status']!='arrived':issues.append('goal not arrived: '+str(i+1))
        goals.append(dict(index=i+1,status=goal['status'],goal_error_m=math.dist(goal['final_pose'][:2],goal['goal']),
            seconds=end-goal['wall_seconds'],max_stall_s=stall,target_pass=stall<=1.5+1e-9,
            max_path_error_m=float(err.max()),rms_path_error_m=float(np.sqrt(np.mean(err**2))),
            rms_jerk_50hz=float(np.sqrt(np.average(jerk[mask]**2,weights=dt[mask])))))
    rates=np.array([[float(r['acceleration']),float(r['direction_rate'])] for r in trace if not yes(r['stop_override'])])
    if np.any(rates.max(axis=0)>np.array(robot['max_acce'])+1e-6):issues.append('control rate exceeded')
    if speeds.max()>robot['max_speed'][0]+1e-6:issues.append('speed exceeded')
    if np.linalg.norm(accel,axis=1).max()>.30001:issues.append('XY acceleration exceeded')
    failure=sum(yes(r['active']) and yes(r['acknowledged']) and r['solved']=='false' and r['solver_status'] not in ('','-1','1','2') for r in trace)
    unexpected=sum(post_startup[i] and yes(r['stop_override']) and not yes(r['arrived']) for i,r in enumerate(trace))
    startup_stops=sum(valid[i] and not post_startup[i] and yes(r['stop_override']) and not yes(r['arrived']) for i,r in enumerate(trace))
    yaw=float(trace[0]['yaw']);yaw_drift=float(max(abs(math.atan2(math.sin(float(r['yaw'])-yaw),math.cos(float(r['yaw'])-yaw))) for r in trace))
    if failure or unexpected or yaw_drift>1e-6:issues.append('tracking failure/emergency/yaw drift')
    if gaps.min()<.12 or summary['collision_count']:issues.append('clearance/collision')
    return dict(goals=goals,goal_count=len(goals),arrived=sum(g['status']=='arrived' for g in goals),
        collisions=summary['collision_count'],min_clearance_m=float(gaps.min()),max_stall_s=max(g['max_stall_s'] for g in goals),
        max_goal_error_m=max(g['goal_error_m'] for g in goals),max_speed_mps=float(speeds.max()),max_control_rates=rates.max(axis=0).tolist(),
        max_xy_acceleration=float(np.linalg.norm(accel,axis=1).max()),yaw_drift=yaw_drift,geometry_max_error_m=gap_error,
        solver_failure_samples=int(failure),unexpected_emergency_samples=int(unexpected),startup_or_goal_change_stop_samples=int(startup_stops),
        timing=timing(times),csv_rows=len(trace),issues=issues)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--developer-data',type=Path,required=True);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    source=args.input;dev=args.developer_data;cases={};sessions={};inventory=[];hashes={}
    for file in sorted(source.rglob('*')):
        if file.is_file():hashes[str(file.relative_to(source))]=sha(file)
    for file in sorted(source.rglob('planner.yaml')):
        config=yaml.safe_load(file.read_text())
        if config.get('robot',{}).get('kinematics')!='omni':continue
        directory=file.parent;name=str(directory.relative_to(source));is_v2=name.startswith('light_turn_v2')
        profile=ROOT/'config/profiles/ros1'/('fw_mini_omni_light_turn_v2.yaml' if is_v2 else 'fw_mini_omni_full_v1.yaml')
        assert config==yaml.safe_load(profile.read_text()),name
        inventory.append(dict(path=name,profile='FW-OMNI-LIGHT-TURN-V2' if is_v2 else 'FW-OMNI-FULL-V1',planner_sha256=sha(file)))
        if (directory/'trajectory.csv').is_file():sessions[name]=ros(directory,profile,ROOT/'models/fw_mini.bin')
        elif isinstance(load(directory/'summary.json'),list):
            reference=dev/'local_v2' if is_v2 else ROOT/'test/results/omni_turns_20261027/final_core'
            cases[name]=core(directory,reference,True)
    plant=yaml.safe_load((dev/'local_v2/planner.yaml').read_text())['robot']
    development={name:core(dev/name,None,False,plant) for name in ['local_v1','local_v2','upstream_tuned']}
    # The upstream planner predicts more conservatively; physical plant limits
    # in that experiment are [0.2,0.8]. Do not reject its physical trace using
    # its narrower prediction limits [0.12,0.4]. Keep that fact explicit.
    development['upstream_tuned']['prediction_vs_plant_note']='Prediction [0.12,0.4]; physical plant [0.2,0.8].'
    issues=[name+': '+s for group in [cases,sessions,development] for name,r in group.items() for s in r['issues']]
    for old in source.rglob('maps/short_acute_zigzag.yaml'):
        assert yaml.safe_load(old.read_text())==yaml.safe_load((ROOT/'config/stress/short_acute_zigzag.yaml').read_text())
    result=dict(passed=not issues,inventory=inventory,ipc_core=cases,ipc_ros=sessions,development=development,
        csv_rows=sum(r['csv_rows'] for group in [cases,sessions] for r in group.values()),issues=issues,
        scope='Recorded IPC static-map ideal-plant simulations; upstream comparison is developer-host data, not IPC.')
    (args.output/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (args.output/'INPUT_SHA256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],omni_inputs=len(inventory),ipc_csv_rows=result['csv_rows'],issues=issues),ensure_ascii=False))
    for group in ['ipc_core','ipc_ros','development']:
        for name,r in result[group].items():print(json.dumps(dict(group=group,name=name,**{k:v for k,v in r.items() if k not in ['cases','goals']}),ensure_ascii=False))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
