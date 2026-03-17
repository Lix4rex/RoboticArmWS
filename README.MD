# ROS 2 & Gazebo Simulation for a Robotic Arm

## Overview

This project focuses on simulating and controlling a robotic arm using **ROS 2** and **Gazebo**.

The main objective is to enable intuitive manipulation of the robotic arm through **inverse kinematics**, allowing users to control the end-effector position in 3D space.

## Features

- Robotic arm simulation in Gazebo  
- Inverse kinematics control  
- End-effector positioning using Cartesian coordinates  
- Visualization with RViz (planned)  

## Concept

Instead of manually controlling each joint, the user provides a **target position (x, y, z)** in 3D space.  
The system then computes the corresponding **joint angles** required for the robotic arm to reach that position.

## Roadmap

- [x] Basic robotic arm model  
- [ ] Inverse kinematics implementation  
- [ ] ROS 2 integration for control  
- [ ] RViz visualization  
- [ ] Real-world robotic arm control  

## Tech Stack

- ROS 2  
- Gazebo  
- RViz (upcoming)  
- Python / C++  

## Installation

```bash
# Clone the repository
git clone https://github.com/Lix4rex/RoboticArmWS.git
```

### Build the workspace
./build.sh

### Source the workspace
source install/setup.bash

### Usage

### Launch the simulation
ros2 launch robotic_arm sim.launch.py

## Future Work

The next step is to integrate full ROS 2 control pipelines and enable:

- Real-time visualization in RViz
- Interaction with real robotic hardware

## Made BY

Axel PARIS - Student at Centrale Lille - France
