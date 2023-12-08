# Differential Drive Robot Navigation with ROS

This project focuses on creating a ROS2-based navigation system for a differential drive robot within a simulated environment. The system enables the robot to intelligently navigate towards specified goal poses while avoiding collisions with obstacles.

## Overview

The project consists of the following key components:

- **simulator.py**: A ROS2 node for simulating the motion of a differential drive robot within a given environment.
- **velocity_translator.py**: Node responsible for translating linear and angular velocities to left and right wheel velocities.
- **prm_controller.py**: Implementation of a proportional controller for navigating the robot to move from start configuration to goal configuration.
- **prm_planner.py**: A module for PRM-based path planning utilized by the prm_controller.py.
- **launch.py**: Launch script to start the simulation environment by launching the nodes and configuring the robot parameters.



## Usage

- **Prerequisites**: Make sure you have ROS2 installed and set up properly.
- Clone this repositior in src folder of ROS2 workspace
- Build the package using:

  ```bash
  colcon build --packages-select special_implemenatation
  ```

- Launch the project using:
  ```bash
  ros2 launch special_implementation robot_name:=ideal.robot world_name:=bricks.world
  ```

- Change robot file names and world file names as per your requirement or you can create new worlds

