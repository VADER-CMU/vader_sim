# VADER SIM

## Attribution

This repository belongs to Carnegie Mellon University, Masters of Science - Robotic Systems Development (MRSD) Team E - VADER

Team Members: Tom Gao, Abhishek Mathur, Rohit Satishkumar, Kshitij Bhat, Keerthi GV 

Special thanks to the authors at UFactory Inc. of the original xarm_ros repository. Much of the work in this repository were taken in reference to their work. Their original work is referenced from https://github.com/xArm-Developer/xarm_ros/tree/master.

First Revision: March 2025

## Introduction and Overview

This is the VADER Simulator repository, which houses all pertinent packages required to create a simulation of the Spring Validation Demonstration (SVD) scene. This repository excludes the planner, state machine, common messages, and end effector control software.

`xarm_controller` sets up the navigation/control behavior of the arms. `xarm_description` holds the URDF/SRDF/mesh/xacro files that describe the hardware of the robot, and publishes the robot description. `xarm_gazebo` sets up the Gazebo simulation environment with the robot description. `sim_fake_perception` publishes fake pepper data (coarse and fine estimates) to support the simulation in lieu of actual perception.

`xarm_api`, `xarm_msgs`, and `xarm_sdk` are supporting packages required by the xarm repository infrastructure and should be left in the workspace as-is during build. `gazebo-pkgs` and `general-message-pkgs` are third party software that creates libgazebo-grasp-fix, a plugin used to make grasping objects in simulation significantly easier.

## Installation

Make sure you do recursive cloning when cloning this repository:

```bash
git clone --recursive git@github.com:VADER-CMU/vader_sim.git
```

Currently, due to an issue when committing files, the xarm_sdk repository is faulty when cloned as-is from this repository. To fix this, do the following:

```bash
# REMOVE the xarm_sdk folder currently in this repository
rm -rf vader_sim/src/xarm_sdk
# clone the xarm_ros repository in full, next to this repository
git clone --recursive git@github.com:VADER-CMU/xarm_ros.git
# navigate to their source and locate the xarm_sdk folder
cd xarm_ros # or wherever the SDK is
# clone the SDK over to this repository
cp -r < xarm_ros-copy-of-sdk > < your_path_to_vader_sim/src/ >
```

Verify that the workspace builds via catkin_make successfully.

## Launch file usage

The simulator has several canned launch files designed to represent preset scenes for various arm setups.

* VADER_vertical_2arms.launch: Defines dual arms with end effectors mounted vertically (as if mounted on table).

<image src="images/vertical.png" width=512>

* VADER_horizontal_2arms.launch: Defines dual arms with end effectors mounted horizontally (as if mounted on the Warthog platform). The assembly is rotated so that the arms face the +x direction, and offset from the ground by 0.5m (default).

<image src="images/horizontal.png" width=512>

* xarm7_single.launch / xarm7_beside_table.launch: Both launch files are single vertical xarm mounted vertically.

<image src="images/single.png" width=300>

For both dual-arm setups, the end effectors are included, and a storage bin is added at a default location. The inter-arm distance as well as the distance from the ground (in the horizontal case) are defaulted to 0.5m. These arguments are defined inside dual_xarm_device.urdf.xacro, so go check it out and modify it there!

For both dual-arm setups, a breakable pepper spawns at the origin by default. This can be changed via an argument when launching the respective launch files.