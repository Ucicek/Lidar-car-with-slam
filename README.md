# Autonomous Car Navigation System

Using an **NVIDIA Jetson** and **ROS Melodic**, this project aims to develop an autonomous car capable of navigating its environment using LIDAR as its primary sensor.

## Table of Contents

- [Objective](#objective)
- [Key Components](#key-components)
  - [Hardware](#hardware)
  - [Software](#software)
- [System Workflow](#system-workflow)
- [Visualization and Debugging](#visualization-and-debugging)
- [Challenges](#challenges)
- [Demo](#demo)
- [Outcome](#outcome)

## Objective

Develop an autonomous car that can interpret its surroundings using LIDAR. It should localize itself in real-time and move from point A to point B autonomously, all while avoiding any obstacles in its path.

## Key Components

### Hardware

- **NVIDIA Jetson**: A powerful computational platform for robot algorithms. 
- **YDLIDAR**: A LIDAR sensor responsible for 360-degree distance measurements.

### Software

- **ROS Melodic**: The backbone of the robot's operations.
- **Hector SLAM**: Used for simultaneous mapping and localization without odometry data.
- **Move_Base**: Responsible for path planning and guiding the car to its destination.
- **URDF (Unified Robot Description Format)**: An XML-based description of the robot's model, including its structure and joint dynamics. In this project, it defines the robot's physical properties, including its wheels and LIDAR.
- **Custom Node (Motor Command Converter)**: A ROS node developed specifically for this project to translate `move_base` commands into instructions that the car's motor controller can execute. This bridge ensures that high-level navigation commands result in physical movement of the car.

## System Workflow

1. **Sensing**: YDLIDAR scans the environment, producing a point cloud.
2. **Mapping and Localization**: Hector SLAM uses the LIDAR data to create a map and determine the car's position (`slam_out_pose`).
3. **Path Planning**: On setting a navigation goal, Move_Base generates a path from the car's current location to its destination.
4. **Command Conversion**: The custom node interprets `move_base` commands and sends the appropriate instructions to the motor controller.
5. **Execution**: The car moves according to the translated commands, ensuring real-world navigation matches the planned path.

## Visualization and Debugging

- **Rviz**: An integral tool in ROS for visualizing robot data like paths, localization estimates, and the generated map.

## Challenges

- **LIDAR Limitations**: Relying solely on LIDAR data demands accurate interpretation and response mechanisms.
- **Tuning**: Parameters for Hector SLAM and Move_Base need fine-tuning for precise localization and efficient navigation.
- **Motor Control**: Working with DC motors and the L298N motor driver presents challenges due to their inherent imprecision. The motors may not provide exact movement as expected, leading to deviations from the desired path.
- **Command Conversion**: Ensuring seamless translation of high-level navigation commands to motor commands without delays or errors.

## Demo

Demo video will be uploaded within the next few days

## Outcome

The outcome is a car capable of understanding its position relative to a map and navigating to specified destinations autonomously.

