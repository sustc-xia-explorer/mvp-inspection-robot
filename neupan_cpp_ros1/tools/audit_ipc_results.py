#!/usr/bin/env python3
"""Audit copied IPC evidence offline, without publishing ROS/chassis commands."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import yaml
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from simulation_test import clearance,rotate


def read_json(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def delta(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
def scene(env):
    out=dict(circles=[],walls=[])
    for obstacle in env['obstacles']:
        s,p=obstacle['shape'],obstacle['pose']
        if s['type']=='circle':out['circles'].append((p['x'],p['y'],s['radius']));continue
        corners=[]
        for dx,dy in [(-1,-1),(1,-1),(1,1),(-1,1)]:
            x,y=rotate(dx*s['length']/2,dy*s['width']/2,p.get('yaw',0.))
            corners.append((p['x']+x,p['y']+y))
        out['walls'].extend((*a,*b) for a,b in zip(corners,corners[1:]+corners[:1]))
    return out

def geom_gap(row,env,robot):
    return clearance([float(row[k]) for k in ('x','y','yaw')],env,robot['length'],robot['width'],robot.get('wheelbase',0.)/2)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    expected=sha(ROOT/'config/fw_mini_omni.yaml');model=sha(ROOT/'models/fw_mini.bin')
    core=a.input/'omni_turns_ipc/stress_core';cfg=yaml.safe_load((core/'planner.yaml').read_text());robot=cfg['robot']
    summaries=read_json(core/'summary.json');local={r['case']:r for r in read_json(ROOT/'test/results/omni_turns_20261027/final_core/summary.json')}
    reports=[];errors=[];manifest={str(f.relative_to(a.input)):sha(f) for f in sorted(a.input.rglob('*')) if f.is_file()}
    for item in summaries:
        name=item['case'];issues=[];record=dict(item)
        if item['planner_sha256']!=expected:issues.append('profile hash mismatch')
        if yaml.safe_load((core/'maps'/(name+'.yaml')).read_text()) != yaml.safe_load((ROOT/'config/stress'/(name+'.yaml')).read_text()):issues.append('map mismatch')
        if item.get('geometric_no_path'):
            record.update(audited=True,issues=issues)
        else:
            trace=rows(core/(name+'.csv'));geometry=scene(yaml.safe_load((core/'maps'/(name+'.yaml')).read_text()))
            gaps=[geom_gap(r,geometry,robot) for r in trace];saved_gaps=[float(r['clearance']) for r in trace]
            max_gap_error=max(abs(x-y) for x,y in zip(gaps,saved_gaps))
            dt=float(cfg['step_time']);previous=(0.,0.);rates=[0.,0.];peak_speed=0.
            for r in trace:
                now=(float(r['control_v']),float(r['control_phi']))
                rates[0]=max(rates[0],abs(now[0]-previous[0])/dt)
                rates[1]=max(rates[1],abs(delta(now[1],previous[1]))/dt);previous=now
                peak_speed=max(peak_speed,math.hypot(float(r['body_vx']),float(r['body_vy'])))
            goal_error=math.dist([float(trace[-1][k]) for k in ('x','y')],item['final_pose'][:2])
            if len(trace)!=item['steps']:issues.append('CSV length differs from summary')
            if max_gap_error>1e-7:issues.append('independent footprint distance mismatch')
            if min(gaps)<.12:issues.append('clearance below 0.12 m')
            if peak_speed>robot['max_speed'][0]+1e-6:issues.append('speed bound exceeded')
            if any(rates[i]>robot['max_acce'][i]+1e-6 for i in (0,1)):issues.append('control rate bound exceeded')
            if goal_error>1e-6 or trace[-1]['arrived'] not in ('True','true','1','1.0'):issues.append('final state/arrival mismatch')
            if item['goal_error']>=cfg['ipath']['arrive_threshold']:issues.append('goal error exceeds threshold')
            if item['unsolved'] or item['emergency_stops'] or item['collision']:issues.append('solver/emergency/collision reported')
            local_trace=rows(ROOT/'test/results/omni_turns_20261027/final_core'/(name+'.csv'))
            xy_difference=(max(math.dist([float(r[k]) for k in ('x','y')],[float(s[k]) for k in ('x','y')])
                for r,s in zip(trace,local_trace)) if len(trace)==len(local_trace) else None)
            record.update(audited=True,csv_rows=len(trace),independent_gap_max_error=max_gap_error,
                max_recorded_speed=peak_speed,max_control_rates=rates,local_trace_max_xy_difference=xy_difference,
                local_steps=local[name].get('steps'),issues=issues)
        reports.append(record);errors.extend(name+': '+v for v in issues)
    sessions=[]
    for tracefile in sorted(a.input.rglob('trajectory.csv')):
        directory=tracefile.parent;relative=str(directory.relative_to(a.input));trace=rows(tracefile)
        params=yaml.safe_load((directory/'planner.yaml').read_text());body=params['robot'];summary=read_json(directory/'summary.json');run=read_json(directory/'run.json')
        env=scene(yaml.safe_load((directory/'map.yaml').read_text()));gap_error=0.;minimum=math.inf;rates=[0.,0.];peak_speed=0.;peak_acc=0.;previous=(0.,0.)
        failures=0;unexpected_emergency=0;times=[];previous_ms=None;goals=[];issues=[]
        if sha(directory/'planner.yaml')!=expected or run['config_sha256']!=expected:issues.append('profile hash mismatch')
        if run['model_sha256']!=model:issues.append('DUNE model hash mismatch')
        for row in trace:
            velocity=tuple(float(row['applied_'+k]) for k in ('vx','vy'));dt=float(row['dt'])
            peak_acc=max(peak_acc,math.dist(velocity,previous)/dt);previous=velocity
            peak_speed=max(peak_speed,math.hypot(*velocity))
            gap=geom_gap(row,env,body);minimum=min(minimum,gap);gap_error=max(gap_error,abs(gap-float(row['true_clearance'])))
            if row['stop_override']!='True':
                rates=[max(rates[i],float(row[k])) for i,k in enumerate(('acceleration','direction_rate'))]
            # OSQP -1 means no solve (startup/arrival/braking), not a failed QP.
            if row['active']=='True' and row['acknowledged']=='True' and row['solved']=='false' and row['solver_status'] not in ('','-1','1','2'):failures+=1
            if row['solved']=='true' and row['planner_ms'] and row['planner_ms']!=previous_ms:
                times.append(float(row['planner_ms']));previous_ms=row['planner_ms']
        for i,g in enumerate(summary['goals']):
            end=summary['goals'][i+1]['wall_seconds'] if i+1<len(summary['goals']) else math.inf
            active=[r for r in trace if g['wall_seconds']<=float(r['wall_seconds'])<end and r['acknowledged']=='True' and r['active']=='True']
            stall=0.;max_stall=0.;not_run=0.;max_not_run=0.
            for row in active:
                distance=float(row['goal_distance']);elapsed=float(row['wall_seconds'])-g['wall_seconds'];dt=float(row['dt'])
                low=row['arrived']!='True' and distance>.2 and elapsed>2. and math.hypot(float(row['applied_vx']),float(row['applied_vy']))<.02
                stall=stall+dt if low else 0.;max_stall=max(max_stall,stall)
                no_solve=row['solved']=='false' and distance>.2 and elapsed>2. and row['arrived']!='True'
                not_run=not_run+dt if no_solve else 0.;max_not_run=max(max_not_run,not_run)
                unexpected_emergency+=elapsed>2. and row['stop_override']=='True' and row['arrived']!='True'
            error=math.dist(g['final_pose'][:2],g['goal'])
            goals.append(dict(index=i+1,goal=g['goal'],status=g['status'],goal_error=error,
                seconds=g.get('arrived_wall_seconds',end)-g['wall_seconds'],max_stall_seconds=max_stall,max_no_solve_seconds=max_not_run))
            if g['status']!='arrived' or error>=params['ipath']['arrive_threshold']:issues.append('goal did not arrive within threshold: '+str(i+1))
            if max_stall>2.:issues.append('stall exceeded 2 s at goal '+str(i+1))
            if max_not_run>.5:issues.append('prolonged unsolved tracking at goal '+str(i+1))
        if minimum<.12 or summary['collision_count']:issues.append('unsafe clearance/collision')
        if gap_error>1e-7:issues.append('independent footprint distance mismatch')
        if peak_speed>body['max_speed'][0]+1e-6 or any(rates[i]>body['max_acce'][i]+1e-6 for i in (0,1)):issues.append('speed/control rate bound exceeded')
        if peak_acc>.30001:issues.append('actual XY acceleration above 0.30 m/s^2')
        if failures or unexpected_emergency:issues.append('solver failure or unexpected tracking emergency')
        yaw0=float(trace[0]['yaw']);yaw_drift=max(abs(delta(float(r['yaw']),yaw0)) for r in trace)
        if yaw_drift>1e-6:issues.append('omni body yaw changed')
        s=dict(session=relative,passed=not issues,goals=goals,csv_rows=len(trace),min_clearance=minimum,
            independent_gap_max_error=gap_error,max_speed=peak_speed,max_control_rates=rates,
            max_xy_acceleration=peak_acc,yaw_drift=yaw_drift,solver_failure_samples=failures,
            unexpected_emergency_samples=unexpected_emergency,
            planner_p95_ms=float(np.percentile(times,95)),planner_max_ms=max(times),
            planner_cycles=len(times),planner_over_100ms=sum(x>100 for x in times),
            initial_emergency_samples=sum(r['stop_override']=='True' for r in trace),issues=issues)
        if '90' in relative:
            first=[r for r in trace if goals and summary['goals'][0]['wall_seconds']<=float(r['wall_seconds'])<=summary['goals'][0]['arrived_wall_seconds']]
            s['first_corner_overshoot_m']=max(0.,max(float(r['x']) for r in first)-2.)
        sessions.append(s);errors.extend(relative+': '+v for v in issues)
    export_file=a.input/'fw_dune_export.json'
    export=read_json(export_file) if export_file.exists() else None
    if export is not None and (not export.get('passed') or export.get('model_sha256')!=model or
            export['max_absolute_error']>export['tolerance']):errors.append('DUNE export report/model mismatch')
    result=dict(passed=not errors,profile_sha256=expected,model_sha256=model,dune_export_report=export,
        core_cases=len(reports),core_arrived=sum(bool(r.get('arrived')) for r in reports),
        core_no_route=[r['case'] for r in reports if r.get('geometric_no_path')],
        ros_sessions=len(sessions),ros_goals=sum(len(s['goals']) for s in sessions),
        core=reports,ros=sessions,issues=errors,
        scope='Offline reanalysis of user-supplied IPC simulations; not real chassis validation.')
    (a.output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.output/'INPUT_SHA256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('core','ros')},indent=2),flush=True)
    for s in sessions:print(json.dumps(s),flush=True)
    return 0 if not errors else 1

if __name__=='__main__':raise SystemExit(main())
