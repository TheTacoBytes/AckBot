# AckBot

https://github.com/user-attachments/assets/03323889-74ca-40c9-b6ba-a75158bd82f6

## Overview
AckBot is a ROS 2 robot platform that handles core functionalities such as `cmd_vel`, `scan`, `odom`, and more. This repository contains the ROS 2 packages needed to control AckBot and interface with its sensors.

## Features
- Handles the core control topics for the AckBot robot.
- Interfaces with various sensors (LiDAR, IMU, Camera, etc.).
- Publishes essential topics for robot state, velocity, and sensor data.

### Required Components
You can find all the necessary parts in this Amazon wishlist:
[Amazon List for AckBot](https://www.amazon.com/hz/wishlist/ls/1HPBW0ZJIIN79?ref_=wl_share)


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
- A Raspberry Pi 4 With Ubuntu 22.04
- Colcon build system

### Clone the Repository and Set Up the Workspace

1. **Create a new workspace**:
   First, create a workspace for ROS 2 if you don’t have one already:
   ```bash
   mkdir -p ~/ackbot_ws/src
   cd ~/ackbot_ws/src
   ```
2. **Clone the repository into the workspace**:
   ```bash
   git clone https://github.com/thetacobytes/ackbot.git
   ```
3. **Go to the workspace root and build**:
   Navigate back to the workspace root and build the workspace using `colcon`:
   ```bash
   cd ~/ackbot_ws
   colcon build
   ```
4. **Source the workspace**:
   After building, source the setup file:
   ```bash
   source install/setup.bash
   ```
### Running the Nodes
You can now run the various ROS 2 nodes provided in this workspace with one launch command.
   ```bash
   ros2 launch ackbot_description start_nodes_launch.py
  ```
### Setting the `ROS_DOMAIN_ID` for Multi-Machine Communication
If you are using multiple machines and want them to see each other's topics, you need to set the `ROS_DOMAIN_ID` to the same value on all machines. For example, you can set it to `1` by running:
   ```bash
   export ROS_DOMAIN_ID=1
   ```


