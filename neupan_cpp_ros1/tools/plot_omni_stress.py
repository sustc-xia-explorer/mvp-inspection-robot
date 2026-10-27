#!/usr/bin/env python3
"""Render recorded stress results, including failures and no-route maps."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from matplotlib.transforms import Affine2D
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'test'))
from stress_omni_navigation import metrics


def load_rows(file):return list(csv.DictReader(file.open()))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    results=json.loads((a.results/'summary.json').read_text());cases={c['name']:c for c in json.loads((a.results/'cases.json').read_text())}
    base={}
    for directory in [a.baseline/'targeted',a.baseline/'extra']:
        for r in json.loads((directory/'summary.json').read_text()):
            name=r['case'];case=cases[name]
            replanned=(json.loads((directory/(name+'_replanned_path.json')).read_text()) if case.get('switch_at') is not None else None)
            r.update(metrics(directory/(name+'.csv'),json.loads((directory/(name+'_path.json')).read_text()),replanned,case.get('switch_at')))
            base[r['case']]=r
    for name,selection in [('targeted_overview',[r for r in results if not r['case'].startswith('random')]),
                           ('random_overview',[r for r in results if r['case'].startswith('random')])]:
        columns=4;count=math.ceil(len(selection)/columns)
        fig,axes=plt.subplots(count,columns,figsize=(columns*4,count*4.1),squeeze=False)
        for ax,r in zip(axes.flat,selection):
            case=cases[r['case']];env=yaml.safe_load((a.results/'maps'/(r['case']+'.yaml')).read_text())
            for obstacle in env['obstacles']:
                shape,pose=obstacle['shape'],obstacle['pose'];x,y=pose['x'],pose['y']
                if shape['type']=='circle':ax.add_patch(Circle((x,y),shape['radius'],color='#41474e'))
                else:
                    patch=Rectangle((-shape['length']/2,-shape['width']/2),shape['length'],shape['width'],color='#41474e')
                    patch.set_transform(Affine2D().rotate(pose.get('yaw',0.)).translate(x,y)+ax.transData);ax.add_patch(patch)
            trace=a.results/(r['case']+'.csv')
            if trace.exists():
                rows=load_rows(trace);path=json.loads((a.results/(r['case']+'_path.json')).read_text())
                ax.plot(*zip(*path),'--',color='#75a546',lw=1,label='Reference')
                if (a.results/(r['case']+'_replanned_path.json')).exists():
                    new=json.loads((a.results/(r['case']+'_replanned_path.json')).read_text());ax.plot(*zip(*new),'--',color='#ba8d27',lw=1)
                ax.plot([float(x['x']) for x in rows],[float(x['y']) for x in rows],color='#1976d2',lw=1.4,label='Actual')
            ax.scatter(*case['start'][:2],color='green',s=16);ax.scatter(*case.get('next_goal',case['goal']),marker='*',s=50,color='red')
            width,height=env['world']['width'],env['world']['height'];ax.set(xlim=(-width/2,width/2),ylim=(-height/2,height/2));ax.set_aspect('equal');ax.grid(alpha=.15)
            if r.get('geometric_no_path'):title=r['case']+' | GLOBAL NO ROUTE\nNo controller run (retained)'
            else:title=(f"{r['case']} | {'PASS' if r['pass'] else 'FAIL'}\n"
                f"gap {r['min_clearance']:.3f} m | error {r['max_path_error']:.3f} m\n"
                f"{r['seconds']:.1f} s | jerk RMS {r['rms_xy_jerk']:.3f}")
            ax.set_title(title,fontsize=9)
        for ax in list(axes.flat)[len(selection):]:ax.axis('off')
        fig.suptitle('Same frozen omni controller | 0.495 x 0.360 m body | 50 Hz kinematic plant')
        fig.tight_layout(rect=(0,0,1,.975));fig.savefig(a.output/(name+'.png'),dpi=150);plt.close(fig)
    targets=[r for r in results if r['case'] in base];labels=[r['case'].replace('bar_','').replace('hard_','').replace('_','\n') for r in targets]
    fig,axes=plt.subplots(2,2,figsize=(15,9));x=np.arange(len(targets));comparison=[]
    for r in targets:
        before=base[r['case']];comparison.append(dict(case=r['case'],baseline=before,tuned=r))
    for ax,(key,title,scale) in zip(axes.flat,[('max_path_error','Maximum path error (cm)',100),
            ('rms_xy_jerk','XY jerk RMS (m/s^3)',1),('seconds','Simulation duration (s)',1),
            ('speed_high_frequency_rms','Speed ripple RMS, 1.1 s moving average (mm/s)',1000)]):
        ax.bar(x-.19,[base[r['case']][key]*scale for r in targets],.38,color='#b76565',label='Previous smooth profile')
        ax.bar(x+.19,[r[key]*scale for r in targets],.38,color='#17858b',label='Turn-aware profile')
        ax.set_xticks(x);ax.set_xticklabels(labels,fontsize=8);ax.set_title(title);ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(a.output/'targeted_comparison.png',dpi=150);plt.close(fig)
    feasible=[r for r in results if not r.get('geometric_no_path')]
    report=dict(total_maps=len(results),controller_runs=len(feasible),controller_passed=sum(r.get('pass',False) for r in feasible),
        global_no_route=[r['case'] for r in results if r.get('geometric_no_path')],
        controller_failures=[r['case'] for r in feasible if not r.get('pass')],
        min_clearance=min(r['min_clearance'] for r in feasible),max_path_error=max(r['max_path_error'] for r in feasible),
        max_stall_seconds=max(r['max_stall_seconds'] for r in feasible),max_xy_acceleration=max(r['max_xy_acceleration'] for r in feasible),
        all_same_config=len({r['planner_sha256'] for r in results})==1,planner_sha256=results[0]['planner_sha256'],
        targeted_comparison=comparison,results=results)
    (a.output/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('targeted_comparison','results')},indent=2))

if __name__=='__main__':main()
