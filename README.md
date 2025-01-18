# Moveo Robotic Arm with ROS2 and MoveIt

![Robotic Arm](assets/moveo1.png) <!-- Add an image or GIF of your project if available -->

## Overview
This repository contains the ROS2 and MoveIt configuration for the Moveo robotic arm. It includes:
- A URDF/Xacro model of the Moveo arm.
- ROS2 control integration.
- MoveIt configuration for motion planning and execution.
- Simulation and visualization in RViz.

## Features
- Fully articulated 5-DOF robotic arm.
- Gripper support for object manipulation.
- Motion planning with OMPL and custom controllers.
- Compatible with Gazebo simulation.

## Table of Contents
1. [Installation](#installation)
2. [Usage](#usage)
3. [Configuration](#configuration)
4. [Contributing](#contributing)
5. [License](#license)

## Installation
### Prerequisites
- Ubuntu 24.04
- ROS2 (Jazzy Jalisco)
- MoveIt2
- Python 3.8+
- colcon build tools

### Steps
1. Clone this repository:
   ```bash
   git clone https://github.com/your_username/moveo_robotic_arm.git
   cd moveo_robotic_arm
   colcon build

2. Souce the workspace 
   ```bash
   echo source

## Usage

1. Launch Gazebo sim
   ```bash
   ros2 launch moveo_description gazebo.launch.py
2. Launch rviz with moveit
   ```bash
   ros2 launch moveo_moveit_config demo.launch.py
3. Start planing the robot movements
4. Execute to move the robot


