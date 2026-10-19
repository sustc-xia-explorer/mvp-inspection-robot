#!/usr/bin/env python3
"""Run in the Python NeuPAN environment, not the IPC ROS environment."""
from pathlib import Path
from neupan import neupan

if __name__ == '__main__':
    root = Path(__file__).resolve().parent.parent
    cfg = root / 'neupan_cpp_ros1/config/fw_mini_dune_train.yaml'
    planner = neupan.init_from_yaml(str(cfg))
    planner.train_dune()
