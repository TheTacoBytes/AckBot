# AckBot

## Overview
AckBot is a ROS 2 robot platform that handles core functionalities such as `cmd_vel`, `scan`, `odom`, and more. This repository contains the ROS 2 packages needed to control AckBot and interface with its sensors.


## Features
- Handles the core control topics for the Ackbot robot.
- Interfaces with various sensors (LiDAR, IMU, Camera, etc.).
- Publishes essential topics for robot state, velocity, and sensor data.

## ROS Nodes
### Available Nodes
- **/LD19**: LiDAR node.
- **/base_link_to_base_laser_ld19**: Transform node for LiDAR to base link.
- **/battery_display**: Publishes battery voltage information.
- **/odom_pub**: Publishes odometry data.
- **/robot_state_publisher**: Publishes robot state.
- **/usb_cam**: Camera node.
- **/vel_raw_pub**: Publishes raw velocity data.

## Installation
### Prerequisites
- ROS 2 Humble
- A Raspberry Pi 4

### Clone the Repository
```bash
git clone https://github.com/thetacobytes/ackbot.git
cd ackbot
