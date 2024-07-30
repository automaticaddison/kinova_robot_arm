# kinova_robot_arm #

Automatic Addison ROS 2 support for the Gen3 Lite Robotic arm by Kinova Robotics.

![gen3_lite_kinova](./kinova_robot_arm_description/urdf/gen3-lite-robot-urdf-xacro-rviz.jpg)

## Overview

This package provides ROS 2 support for the Kinova Gen3 Lite robotic arm. It includes URDF/Xacro descriptions.

## Contents

- `kinova_robot_arm_description`: Contains URDF/Xacro files for the Gen3 Lite arm
  - `urdf/gen3_lite_gen3_lite_2f.xacro`: Main Xacro file describing the complete arm with gripper

## Features

- 6 degrees of freedom (DOF) arm configuration
- Integrated two-finger gripper (Gen3 Lite 2F)
- Configurable joint limits and physical parameters

## Usage

To use this description in your ROS 2 projects:

1. Clone this repository into your ROS 2 workspace
2. Build your workspace
3. Launch the robot model in RViz visualization.

## Dependencies

- ROS 2 (tested on ROS 2 Iron)
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2` (for visualization)
- `gazebo_ros` (for simulation)

## License

This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.

## Acknowledgements

This package is based on the work by Kinova Robotics and has been adapted for ROS 2 use.

