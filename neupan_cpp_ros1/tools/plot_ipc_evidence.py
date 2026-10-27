#!/usr/bin/env python3
"""Plot recorded IPC ROS1 sessions without replaying or controlling hardware."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from matplotlib.transforms import Affine2D
import numpy as np
import yaml


def plot(directory,output,title):
    with (directory/'trajectory.csv').open() as f:rows=list(csv.DictReader(f))
    summary=json.loads((directory/'summary.json').read_text());env=yaml.safe_load((directory/'map.yaml').read_text())
    cfg=yaml.safe_load((directory/'planner.yaml').read_text());v=lambda key:np.array([float(r[key]) for r in rows])
    fig,axes=plt.subplots(2,2,figsize=(13,8.5),constrained_layout=True);ax=axes[0,0]
    for o in env['obstacles']:
        s,p=o['shape'],o['pose']
        if s['type']=='circle':ax.add_patch(Circle((p['x'],p['y']),s['radius'],color='#41474e'))
        else:
            patch=Rectangle((-s['length']/2,-s['width']/2),s['length'],s['width'],color='#41474e')
            patch.set_transform(Affine2D().rotate(p.get('yaw',0.)).translate(p['x'],p['y'])+ax.transData);ax.add_patch(patch)
    colors=plt.get_cmap('tab10');times=v('wall_seconds')
    for i,goal in enumerate(summary['goals']):
        path=json.loads((directory/f'goal_{i+1:03d}_path.json').read_text())
        ax.plot(*zip(*path),'--',color=colors(i),alpha=.6,lw=1)
        end=summary['goals'][i+1]['wall_seconds'] if i+1<len(summary['goals']) else times[-1]
        mask=(times>=goal['wall_seconds'])&(times<end)
        ax.plot(v('x')[mask],v('y')[mask],color=colors(i),lw=1.7,label=f'Goal {i+1}')
        ax.scatter(*goal['goal'],marker='*',s=90,color=colors(i),zorder=4)
        for j in [0,1,2]:axes.flat[j+1].axvline(goal['wall_seconds'],color=colors(i),alpha=.3,lw=.8)
    x,y=v('x'),v('y');mx,my=cfg['robot']['length']/2+.5,cfg['robot']['width']/2+.5
    ax.set(xlim=(max(-env['world']['width']/2,x.min()-mx),min(env['world']['width']/2,x.max()+mx)),
           ylim=(max(-env['world']['height']/2,y.min()-my),min(env['world']['height']/2,y.max()+my)),
           xlabel='x (m)',ylabel='y (m)',title='Dashed: global path; solid: actual; stars: goals')
    ax.set_aspect('equal');ax.legend(fontsize=8)
    ax=axes[0,1];ax.plot(times,np.hypot(v('applied_vx'),v('applied_vy')),label='Speed',color='#17858b')
    ax.plot(times,v('applied_vx'),label='Body vx',alpha=.65);ax.plot(times,v('applied_vy'),label='Body vy',alpha=.65)
    ax.set(xlabel='Wall time (s)',ylabel='Velocity (m/s)',title='Actual 50 Hz execution, including waits');ax.legend(fontsize=8)
    ax=axes[1,0];ax.plot(times,v('true_clearance'),color='#17858b');ax.axhline(.12,color='#b76565',ls='--',label='Acceptance: 0.12 m')
    ax.set(xlabel='Wall time (s)',ylabel='True footprint clearance (m)',title=f"Minimum clearance: {summary['min_true_clearance']:.3f} m");ax.legend(fontsize=8)
    ax=axes[1,1];sel=[r for r in rows if r['solved']=='true' and r['planner_ms']]
    ax.plot([float(r['wall_seconds']) for r in sel],[float(r['planner_ms']) for r in sel],color='#17858b',lw=.7)
    ax.axhline(100,color='#b76565',ls='--',label='10 Hz period: 100 ms');ax.set(xlabel='Wall time (s)',ylabel='Planner duration (ms)',title='Recorded IPC planner timings');ax.legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.18)
    count=sum(g['status']=='arrived' for g in summary['goals'])
    fig.suptitle(f'{title} | {count}/{len(summary["goals"])} goals arrived | collisions: {summary["collision_count"]}',fontsize=13)
    fig.savefig(output,dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    for folder,name,title in [('omni_turns_rviz','ipc_corridor','IPC original corridor: six interactive goals'),
        ('omni_turns_rviz_bar','ipc_bar','IPC U-shaped obstacle wrap: two interactive goals'),
        ('omni_turns_rviz_90','ipc_hard90_interactive','IPC walled right-angle: three interactive goals'),
        ('omni_turns_ros1_90/world','ipc_hard90_auto','IPC walled right-angle: automatic verification')]:
        plot(a.input/folder,a.output/(name+'.png'),title)

if __name__=='__main__':main()
