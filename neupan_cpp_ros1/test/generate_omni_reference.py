#!/usr/bin/env python3
"""Generate a golden QP fixture using the ORIGINAL Python NeuPAN Robot methods.
Run with the NeuPAN training venv (torch, cvxpy, neupan); not needed at runtime.
"""
import hashlib
import json
import math
from pathlib import Path
import inspect

import cvxpy as cp
import numpy as np
import torch
from neupan.robot.robot import robot as Robot
from neupan.blocks.initial_path import InitialPath


def main():
    cases = []
    for index, (yaw, phi, speed) in enumerate([(0., 0., .12), (1.57, .9, .15),
                                               (-.7, -1.2, .18), (.4, 2.6, .12),
                                               (2., -2.8, -.1)]):
        horizon, dt, count = 6, .1, 3
        r = Robot(receding=horizon, step_time=dt, kinematics='omni',
                  max_speed=[.4, math.pi], max_acce=[1.2, 4.], length=.495, width=.36)
        r.define_variable(indep_dis=cp.Variable((1, horizon)))
        r.state_parameter_define()
        r.coefficient_parameter_define(max_num=count)
        nom_u = np.tile([[speed], [phi]], (1, horizon))
        nom_s = np.zeros((3, horizon+1)); nom_s[:, 0] = [.1, -.2, yaw]
        for t in range(horizon):
            nom_s[:, t+1:t+2] = InitialPath.omni_model(None, nom_s[:, t:t+1], nom_u[:, t:t+1], dt)
        ref_s = nom_s.copy()
        ref_s[0] += np.linspace(0., .08, horizon+1)
        ref_s[1] += np.linspace(0., .06*(-1 if index % 2 else 1), horizon+1)
        ref_s[2] = -2.5  # Deliberately different: omni must ignore yaw tracking.
        ref_us = np.full(horizon, .16)
        qs = np.array([[4.], [3.], [8.]])
        pu, bk, eta, rho = 1., .2, .7, 80.
        r.para_s.value = nom_s
        r.para_gamma_a.value = qs*ref_s
        r.para_gamma_b.value = pu*ref_us
        linear = []
        for t in range(horizon):
            abc = r.linear_omni_model(torch.tensor(nom_u[:, t:t+1]), dt)
            a, b, c = [x.detach().cpu().numpy().astype(float) for x in abc]
            r.para_A_list[t].value, r.para_B_list[t].value, r.para_C_list[t].value = a, b, c
            linear.append(dict(A=a.tolist(), B=b.tolist(), C=c[:, 0].tolist()))
        fa = np.tile(np.array([[1., 0.], [0., -1.], [-.6, .8]])[None, :, :], (horizon, 1, 1))
        fb = np.tile(np.array([-.08, -.02, -.3])[None, :], (horizon, 1))
        for t in range(horizon):
            r.para_gamma_c[t].value = fa[t]
            r.para_zeta_a[t].value = fb[t, :, None]
        d = cp.Variable((1, horizon))
        objective = (r.C0_cost(cp.Constant(pu), cp.Constant(qs)) + .5*bk*r.proximal_cost()
                     - eta*cp.sum(d) + r.I_cost(d, cp.Constant(rho)))
        problem = cp.Problem(cp.Minimize(objective), r.dynamics_constraint() +
                             r.bound_su_constraints() + [d >= .05, d <= .2])
        problem.solve(solver='OSQP', eps_abs=1e-9, eps_rel=1e-9, max_iter=100000, polishing=True)
        assert problem.status == 'optimal', problem.status
        cases.append(dict(yaw=yaw, phi=phi, nom_s=nom_s.tolist(), nom_u=nom_u.tolist(),
                          ref_s=ref_s.tolist(), ref_us=ref_us.tolist(), linear=linear,
                          fa=fa.tolist(), fb=fb.tolist(), opt_s=r.indep_s.value.tolist(),
                          opt_u=r.indep_u.value.tolist(), opt_d=d.value[0].tolist(),
                          objective=problem.value))
    sources = [Path(inspect.getfile(Robot)), Path(inspect.getfile(InitialPath))]
    data = dict(source='hanruihua/NeuPAN', reference_commit='d225c3862654ade30ccd19455997f615a689a670',
                source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                cvxpy=cp.__version__, torch=torch.__version__, T=6, dt=.1, max_num=3,
                q_s=[4., 3., 8.], p_u=1., bk=.2, eta=.7, ro_obs=80.,
                max_speed=[.4, math.pi], max_acce=[1.2, 4.], d_min=.05, d_max=.2, cases=cases)
    dest = Path(__file__).parent/'data/omni_python_reference.json'
    dest.write_text(json.dumps(data, indent=2)+'\n')
    print('Original Python omni QP fixtures:', len(cases), dest)


if __name__ == '__main__':
    main()
