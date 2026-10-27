#!/usr/bin/env python3
"""Independent unweighted flood fill of the configured inflated A* grid."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from rviz_sim import World


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True);a=p.parse_args()
    reports=json.loads((a.results/'summary.json').read_text());proof=[]
    for r in reports:
        if not r.get('geometric_no_path'):continue
        world=World(a.results/'maps'/(r['case']+'_scenario.yaml'),a.results/'planner.yaml')
        source,target=world.cell(world.config['initial_pose']),world.cell(world.config['default_goal'])
        visited={source};queue=deque([source])
        while queue:
            x,y=queue.popleft()
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                v=x+dx,y+dy
                if v in visited or not world.free(v):continue
                if dx and dy and (not world.free((x+dx,y)) or not world.free((x,y+dy))):continue
                visited.add(v);queue.append(v)
        proof.append(dict(case=r['case'],source_free=world.free(source),target_free=world.free(target),
            target_reachable=target in visited,reachable_cells=len(visited),blocked_cells=len(world.blocked),
            total_cells=world.nx*world.ny,note='No path in this conservative inflated grid; not a proof of physical impossibility.'))
        if target in visited:raise AssertionError('A* wrongly rejected a connected grid: '+r['case'])
    (a.results/'no_route_proof.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof,indent=2))

if __name__=='__main__':main()
