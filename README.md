# mvp-inspection-robot platform
## 1.Hardware
BOM: 

|  Material   |    Spec     |
| ----------- | ----------- |
| Computer    | M20       |
| Vehicle     | FW-mini     |
| Lidar       | TM-16       |
| IMU         | WT901WIFI   |

## 2.Enviroment:
  (1) Ubuntu 20.04
  (2) ROS1 noetic
  (3) Docker

## 3.Framework:
### 3.1 Autoware.ai-mvp
  （1）SLAM：ndt_mapping
  （2）Localization: ndt_matching
  （3）global planner: record path
  （4）local planner: astar_avoid+velocity_set
  （5）control: pure pursuit
### 3.2 Autoware.ai-livox-sustc
  （1）SLAM：ndt_mapping
  （2）Localization: ndt_matching
  （3）global planner: record path
  （4）local planner: astar_avoid+velocity_set
  （5）control: pure pursuit
### 3.3 neupan_cpp
  （1）SLAM：fastlio2
  （2）Localization: fastlio2
  （3）global planner: record path
  （4）local planner: neupan
  （5）control: pure pursuit
