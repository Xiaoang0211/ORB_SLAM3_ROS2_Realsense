![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![Version](https://img.shields.io/badge/Version-1.5.0-blue.svg)

# ROS2 ORB SLAM3 package 

This package originates from ![ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3/tree/main).
Please refere to their repository for the environment settings

In Addition to the handshake example, we added the support for realsense d435i. Currently it is only supported for monocular mode (without IMU)

```bibtex
@INPROCEEDINGS{kamal2024solving,
  author={Kamal, Azmyin Md. and Dadson, Nenyi Kweku Nkensen and Gegg, Donovan and Barbalata, Corina},
  booktitle={2024 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)}, 
  title={Solving Short-Term Relocalization Problems In Monocular Keyframe Visual SLAM Using Spatial And Semantic Data}, 
  year={2024},
  volume={},
  number={},
  pages={615-622},
  keywords={Visualization;Simultaneous localization and mapping;Accuracy;Three-dimensional displays;Semantics;Robot vision systems;Pipelines},
  doi={10.1109/AIM55361.2024.10637187}}
```

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
 }
```

## 0. Preamble

* This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) V1.0 as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]
* g2o used is an older version and is incompatible with the latest release found here [g2o github page](https://github.com/RainerKuemmerle/g2o).
* This package differs from other ROS1 wrappers, thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is purely a design choice.
  * At least C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV 4.2, latest release of Pangolin
* Comes with a small test image sequence from EuRoC MAV dataset (MH05) to quickly test installation
* For newcomers in ROS2 ecosystem, this package serves as an example of building a shared cpp library and also a package with both cpp and python nodes.
* May not build or work correctly in **resource constrainted hardwares** such as Raspberry Pi 4, Jetson Nano

## Testing platforms

* AMD® Ryzen 9 5900hx, x86_64 bit architecture, Ubuntu 22.04 LTS (Jammy Jellyfish) and RO2 Humble Hawksbill (LTS)

## 2. Installation

Follow the steps below to create the ```orbslam3_ws``` workspace, install dependencies and build the package. Note, the workspace must be named ```orbslam3_ws``` due to a HARDCODED path in the python node. I leave it to the developers to change this behavior as they see fit.

```bash
cd ~
mkdir -p ~/orbslam3_ws/src
cd ~/orbslam3_ws/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
cd .. # make sure you are in ~/ros2_ws root directory
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Usage example

First, run the cpp subscriber node:
```bash
ros2 run ros2_orb_slam3 replica_subscriber
```
With the default settings it runs in RGBD mode

Then run the python publisher node:
```bash
ros2 run ros2_orb_slam3 replica_driver_node.py --ros-args -p replica_path:=<path_to_the_replica_v1_dataset> -p scene_name:=<target_room>
```
An example with room_0:
s
```bash
ros2 run ros2_orb_slam3 replica_driver_node.py --ros-args -p replica_path:=/mnt/replica_v1/ -p scene_name:=room_0/
```
