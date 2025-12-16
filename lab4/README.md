# Lab 4 - Sensor Fusion Controller

## Overview
This package implements a sensor fusion controller combining LIDAR and camera data for TurtleBot4 navigation.

## Dependencies
- ROS2 Humble
- Python 3.10+
- rclpy
- geometry_msgs
- sensor_msgs

## Installation
```bash
cd ~/ros2_ws
colcon build --packages-select lab4
source install/setup.bash
```

## Running the Nodes
```bash
ros2 launch lab4 lab4_launch.py
```

## Individual Nodes
- **Node 1**: LIDAR Avoidance - `ros2 run lab4 lidar_avoidance`
- **Node 2**: Color Tracker - `ros2 run lab4 color_tracker`
- **Node 3**: Fusion Node - `ros2 run lab4 fusion_node`

## Author
Isaac Premkumar (premkumar.i@northeastern.edu)
