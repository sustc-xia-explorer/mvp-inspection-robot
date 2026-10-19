#!/usr/bin/env python3
"""Compare the actual .pth network to Eigen MLP on identical float32 points."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
import torch
from neupan.blocks import ObsPointNet


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', required=True, type=Path)
    p.add_argument('--config', required=True, type=Path)
    p.add_argument('--model', required=True, type=Path)
    p.add_argument('--checkpoint', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--edge-dim', type=int, default=4)
    p.add_argument('--tolerance', type=float, default=1e-4)
    args = p.parse_args()
    rng = np.random.default_rng(42)
    # Cover near-body and normal lidar range inputs; this checks implementation,
    # not the learned model's geometric accuracy.
    points = np.vstack([rng.uniform(-.8, .8, (500, 2)), rng.uniform(-5, 5, (500, 2))]).astype(np.float32)
    net = ObsPointNet(2, args.edge_dim)
    net.load_state_dict(torch.load(args.checkpoint, map_location='cpu'))
    net.eval()
    with torch.no_grad():
        reference = net(torch.from_numpy(points)).numpy().T
    payload = 'MLP {}\n'.format(len(points))+''.join('{} {}\n'.format(x, y) for x, y in points)
    result = subprocess.run([str(args.binary.resolve()), str(args.config.resolve()), str(args.model.resolve())],
                            input=payload, text=True, capture_output=True, timeout=30, check=True)
    fields = result.stdout.split()
    if fields[:1] != ['MLP']:
        raise RuntimeError('unexpected backend response: '+result.stdout[:200])
    rows, columns = int(fields[1]), int(fields[2])
    actual = np.array(fields[3:], dtype=np.float64).reshape(rows, columns, order='F')
    if actual.shape != reference.shape:
        raise RuntimeError('network output shape mismatch')
    error = np.abs(actual-reference)
    passed = bool(np.isfinite(error).all() and error.max() <= args.tolerance)
    report = dict(passed=passed, points=len(points), max_absolute_error=float(error.max()),
                  rms_error=float(np.sqrt(np.mean(error**2))), tolerance=args.tolerance,
                  checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                  model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
