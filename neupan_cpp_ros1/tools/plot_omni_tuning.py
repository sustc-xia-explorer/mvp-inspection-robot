#!/usr/bin/env python3
"""Plot same-map corridor tuning traces; requires matplotlib only for plotting."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import yaml


def load(path):
    with path.open() as f:rows=list(csv.DictReader(f))
    fields=('t','x','y','speed','clearance','ax','ay')
    return {k:[float(r[k]) for r in rows] for k in fields}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True)
    p.add_argument('--path',type=Path,required=True);p.add_argument('--map',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    before,after=load(a.before),load(a.after)
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    colors=['#b45b60','#007f87'];labels=['Baseline','Tuned']
    ax=axes[0,0]
    environment=yaml.safe_load(a.map.read_text())
    for item in environment['obstacles']:
        shape,pose=item['shape'],item['pose'];x,y,angle=pose['x'],pose['y'],pose.get('yaw',0)
        if shape['type']!='rectangle':continue
        corners=[]
        for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]:
            dx,dy=sx*shape['length']/2,sy*shape['width']/2
            corners.append((x+math.cos(angle)*dx-math.sin(angle)*dy,
                            y+math.sin(angle)*dx+math.cos(angle)*dy))
        ax.add_patch(Polygon(corners,facecolor='#666b70',edgecolor='none',alpha=.8))
    path=json.loads(a.path.read_text());ax.plot(*zip(*path),'--',color='#7d9873',lw=1,label='Tuned A* reference')
    for trace,color,label in zip([before,after],colors,labels):ax.plot(trace['x'],trace['y'],color=color,lw=1.7,label=label)
    ax.set(xlim=(-4.7,4.8),ylim=(-1.5,1.5),xlabel='x [m]',ylabel='y [m]',title='Actual trajectories: fixed body heading')
    ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=8,loc='upper left')
    for trace,color,label in zip([before,after],colors,labels):
        axes[0,1].plot(trace['t'],trace['speed'],color=color,label=label,lw=1)
        axes[1,0].plot(trace['t'],trace['clearance'],color=color,label=label,lw=1)
        axes[1,1].plot(trace['t'],[math.hypot(x,y) for x,y in zip(trace['ax'],trace['ay'])],color=color,label=label,lw=1)
    axes[0,1].axhline(.15,color='#999999',ls=':',label='Reference speed 0.15 m/s')
    axes[0,1].set(xlabel='Simulation time [s]',ylabel='Speed [m/s]',title='Cruising speed and oscillation')
    axes[1,0].axhline(.16,color='#999999',ls=':',label='Configured planning margin 0.16 m')
    axes[1,0].set(xlabel='Simulation time [s]',ylabel='True footprint clearance [m]',title='Clearance to obstacles')
    axes[1,1].axhline(.3,color='#999999',ls=':',label='0.30 m/s² reference')
    axes[1,1].set(xlabel='Simulation time [s]',ylabel='XY acceleration [m/s²]',title='Actual Cartesian acceleration')
    for ax in axes.flat:ax.grid(alpha=.2)
    for ax in [axes[0,1],axes[1,0],axes[1,1]]:ax.legend(fontsize=8)
    fig.suptitle('NeuPAN omni corridor tuning — same map and 50 Hz virtual plant',fontsize=14)
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=170);plt.close(fig)


if __name__=='__main__':main()
