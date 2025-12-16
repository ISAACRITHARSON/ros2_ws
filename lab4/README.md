# Lab 4: Perception and Reactive Navigation

**Author:** Isaac Premkumar  
**Course:** EECE 5554 - Robotics Sensing and Navigation  
**Date:** December 2025  
**Institution:** Northeastern University - Seattle

## Overview

This package implements real-time perception and reactive navigation for the TurtleBot4, combining LIDAR-based obstacle avoidance with vision-based object tracking. The system demonstrates sensor fusion for adaptive, environment-aware autonomous navigation.

## Table of Contents

- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Nodes](#nodes)
- [Launch Files](#launch-files)
- [Usage](#usage)
- [Algorithm Details](#algorithm-details)
- [Demonstrations](#demonstrations)
- [Troubleshooting](#troubleshooting)
- [Future Improvements](#future-improvements)

## Features

- **LIDAR-Based Obstacle Avoidance**: Real-time obstacle detection and reactive collision avoidance
- **Vision-Based Color Tracking**: OpenCV-powered colored object detection and following
- **Sensor Fusion Controller**: Hierarchical behavior system combining vision and LIDAR inputs
- **Safety-First Design**: Obstacle avoidance overrides tracking behavior for safe operation

## System Requirements

### Hardware
- TurtleBot4 with OAKD camera
- RPLIDAR A1/A2 or equivalent
- Ubuntu 24.04 (native or VM)

### Software
- ROS2 Jazzy
- Python 3.10+
- OpenCV 4.x
- NumPy

### ROS2 Dependencies
```bash
sudo apt install ros-jazzy-nav2-bringup \
                 ros-jazzy-vision-msgs \
                 ros-jazzy-cv-bridge \
                 ros-jazzy-image-transport \
                 ros-jazzy-sensor-msgs \
                 ros-jazzy-geometry-msgs
```

## Installation

### 1. Clone and Build
```bash
cd ~/ros2_ws
git clone <your-repo-url> src/lab4
cd ~/ros2_ws
colcon build --packages-select lab4
source install/setup.bash
```

### 2. Verify Installation
```bash
ros2 pkg list | grep lab4
```

## Package Structure
```
lab4/
├── lab4/
│   ├── package.xml                          # Package metadata
│   ├── setup.py                             # Python package setup
│   ├── setup.cfg                            # Setup configuration
│   ├── resource/
│   │   └── lab4                             # Resource marker
│   ├── lab4/
│   │   ├── __init__.py                      # Package initializer
│   │   ├── lab4_node1_lidar_avoidance.py   # LIDAR obstacle avoidance
│   │   ├── lab4_node2_color_tracker.py     # Vision-based tracking
│   │   └── lab4_node3_fusion_node.py       # Combined sensor fusion
│   └── launch/
│       └── lab4_launch.py                   # Launch all nodes
├── submissions/
│   ├── lab4_report_premkumar.pdf           # Technical report
│   ├── lab4_demo_video1_lidar_avoidance.mov
│   ├── lab4_demo_video2_color_tracker.mov
│   └── lab4_demo_video3_fusion_node.mov
└── README.md                                # This file
```

## Nodes

### Node 1: LIDAR Avoidance (`lidar_avoidance`)

**Purpose:** Reactive obstacle avoidance using RPLIDAR scan data

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan) - LIDAR measurements

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Parameters:**
- `obstacle_threshold`: 0.5m (distance to trigger avoidance)
- `front_sector_size`: 20 degrees
- `linear_speed`: 0.2 m/s
- `angular_speed`: 0.5 rad/s

**Behavior:**
- Monitors front sector (±10 degrees from robot's front)
- Stops and rotates when obstacle < 0.5m
- Resumes forward motion when clear

### Node 2: Color Tracker (`color_tracker`)

**Purpose:** Vision-based colored object detection and tracking

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Parameters:**
- `lower_red`: [0, 120, 70] (HSV)
- `upper_red`: [10, 255, 255] (HSV)
- `linear_speed`: 0.15 m/s
- `angular_gain`: 300.0

**Behavior:**
- Detects red objects using HSV color segmentation
- Computes centroid of detected region
- Proportional control to center object in frame

### Node 3: Fusion Controller (`fusion_node`)

**Purpose:** Hierarchical sensor fusion combining vision and LIDAR

**Subscribed Topics:**
- `/scan` (sensor_msgs/LaserScan)
- `/camera/image_raw` (sensor_msgs/Image)

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist)

**Control Hierarchy:**
1. **Priority 1 - Safety**: Obstacle detected → Stop/rotate
2. **Priority 2 - Mission**: Target visible → Track object
3. **Priority 3 - Search**: No target → Rotate to search

## Launch Files

### Launch All Nodes
```bash
ros2 launch lab4 lab4_launch.py
```

Launches all three nodes simultaneously for integrated testing.

## Usage

### Running Individual Nodes

**LIDAR Avoidance Only:**
```bash
ros2 run lab4 lidar_avoidance
```

**Color Tracking Only:**
```bash
ros2 run lab4 color_tracker
```

**Fusion Controller:**
```bash
ros2 run lab4 fusion_node
```

### Visualization in RViz2
```bash
rviz2
```

Add displays:
- `/scan` → LaserScan
- `/camera/image_raw` → Image
- `/odom` → Odometry
- `/map` → Map

### Testing Procedure

1. **Test LIDAR Avoidance:**
```bash
   ros2 run lab4 lidar_avoidance
```
   - Move obstacles in front of robot
   - Observe reactive turning behavior

2. **Test Color Tracking:**
```bash
   ros2 run lab4 color_tracker
```
   - Show red object to camera
   - Move object left/right
   - Observe tracking behavior

3. **Test Fusion:**
```bash
   ros2 run lab4 fusion_node
```
   - Place red target in environment
   - Introduce obstacles while tracking
   - Verify safety override

## Algorithm Details

### LIDAR Processing
```python
# Extract front sector (90° ± 10°)
front_index = len(ranges) // 4
sector = ranges[front_index-10:front_index+10]
min_distance = min(valid_readings)

if min_distance < 0.5:
    # Obstacle detected - avoid
    cmd.linear.x = 0.0
    cmd.angular.z = 0.5
```

### Color Segmentation
```python
# Convert BGR → HSV
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Threshold red color
mask = cv2.inRange(hsv, lower_red, upper_red)

# Compute centroid
M = cv2.moments(mask)
cx = int(M['m10'] / M['m00'])

# Proportional control
error = cx - frame_width/2
angular_z = -error / 300.0
```

### Sensor Fusion Logic
```
IF obstacle_detected:
    → AVOID (Priority 1)
ELSE IF target_visible:
    → TRACK (Priority 2)
ELSE:
    → SEARCH (Priority 3)
```

## Demonstrations

Video demonstrations are available in `submissions/`:

1. **LIDAR Avoidance** - Robot navigating with obstacle avoidance
2. **Color Tracking** - Following a red object
3. **Fusion Controller** - Combined behavior with safety override

## Troubleshooting

### Camera Not Detected
```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
```

### LIDAR Not Publishing
```bash
# Check LIDAR connection
ros2 topic list | grep scan
ros2 topic hz /scan

# Restart LIDAR
ros2 launch turtlebot4_bringup rplidar.launch.py
```

### Robot Not Moving
```bash
# Check cmd_vel publication
ros2 topic echo /cmd_vel

# Verify node is running
ros2 node list

# Check for controller conflicts
ros2 topic info /cmd_vel
```

### Color Detection Issues

- **Adjust HSV thresholds** in `color_tracker.py`
- Use `cv2.imshow()` to visualize mask
- Ensure proper lighting conditions
- Consider different colored objects

## Future Improvements

- [ ] Implement PID control for smoother tracking
- [ ] Add multiple color detection
- [ ] Integrate ArUco marker detection
- [ ] Add obstacle avoidance trajectory planning
- [ ] Implement Kalman filtering for target prediction
- [ ] Add ROS parameters for runtime tuning
- [ ] Create configuration YAML files
- [ ] Add unit tests for perception modules

## References

- [TurtleBot4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [OpenCV Python Tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [MediaPipe Hand Tracking](https://google.github.io/mediapipe/solutions/hands)

## License

MIT License - See course policies for academic integrity requirements

## Contact

**Isaac Premkumar**  
Email: premkumar.i@northeastern.edu  
GitHub: [Your GitHub Profile]

---

*This package was developed as part of EECE 5554 - Robotics Sensing and Navigation at Northeastern University, Seattle Campus, Fall 2025.*
