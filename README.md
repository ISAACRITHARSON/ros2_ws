# EECE 5554 - Robotics Sensing and Navigation
## Northeastern University Seattle - Fall 2025

**Student:** Isaac Premkumar  
**Email:** premkumar.i@northeastern.edu  
**Program:** Master's in Data Analytics Engineering  

## Repository Overview

This repository contains all lab assignments and projects for EECE 5554 - Robotics Sensing and Navigation, focusing on autonomous mobile robotics using ROS2 Jazzy and TurtleBot4 platform.

## Course Topics Covered

- **Mobile Robot Control**: Teleoperation, velocity commands, trajectory planning
- **SLAM & Mapping**: 2D environment mapping using slam_toolbox
- **Sensor Feedback Control**: IMU and odometry-based closed-loop control
- **Autonomous Navigation**: Nav2 stack, path planning, and localization
- **Perception Systems**: LIDAR processing, computer vision, sensor fusion
- **Human-Robot Interaction**: Gesture-based control using MediaPipe

## Repository Structure
```
ros2_ws/
├── lab1/           # Basic TurtleBot4 Control & ROS2 Fundamentals
├── lab2/           # SLAM Mapping & Autonomous Path Planning
├── lab3/           # Sensor Feedback Control & Nav2 Navigation
├── lab4/           # Perception & Reactive Navigation (LIDAR + Vision)
├── lab5/           # AI-Assisted Gesture-Based Control (MediaPipe)
└── README.md       # This file
```

---

## Lab 1: Driving Your TurtleBot

**Objectives:** Introduction to ROS2, TurtleBot4 teleoperation, and basic node programming

### Topics Covered
- ROS2 Jazzy installation and setup
- TurtleBot4 connection via Discovery Server
- Teleoperation (joystick, keyboard, command velocity)
- Writing custom ROS2 nodes for motion control

### Key Deliverables
- `node1.py` - Basic motion control node
- Trajectory execution: straight lines, circles, arcs
- Custom path: Letter "P" shape

### Technologies
- ROS2 Jazzy
- Python 3.10+
- TurtleBot4 Platform
- Twist messages (geometry_msgs)

---

## Lab 2: Programming & Building Maps

**Objectives:** Structured ROS2 programming, SLAM mapping, exploration strategies

### Topics Covered
- Timed callbacks and modular motion functions
- SLAM with slam_toolbox
- Map generation and quality analysis
- Systematic vs. random exploration

### Key Deliverables
- `drive_node.py` - Enhanced motion controller with square path
- Generated maps (`.pgm` + `.yaml`)
- Comparative analysis of exploration strategies
- RViz2 visualization and rqt_graph

### Technologies
- ROS2 Jazzy
- slam_toolbox
- RViz2
- Nav2 stack

---

## Lab 3: Feedback Control & Autonomous Navigation

**Objectives:** Sensor-based motion correction, localization, autonomous navigation

### Part 1: Sensor-Based Motion Correction

#### Topics Covered
- Odometry and IMU data processing
- Quaternion to Euler angle conversion
- PID controller implementation
- Drift correction for straight-line motion

#### Key Deliverables
- `straight_drive_controller.py` - P/PD/PID feedback controller
- Performance comparison: open-loop vs. closed-loop
- Trajectory analysis and error plots

### Part 2: Autonomous Navigation with Nav2

#### Topics Covered
- AMCL localization
- Nav2 path planning
- Goal-directed navigation
- Programmatic goal submission via action clients

#### Key Deliverables
- `send_nav2_goal.py` - Autonomous navigation node
- Integration with Lab 2 maps
- Multi-goal waypoint navigation

### Technologies
- ROS2 Jazzy
- Nav2 stack
- AMCL (Adaptive Monte Carlo Localization)
- PID control
- ROS2 Actions

---

## Lab 4: Perception & Reactive Navigation

**Objectives:** Real-time perception, sensor fusion, reactive control

### Topics Covered
- LIDAR-based obstacle avoidance
- Computer vision with OpenCV
- Color-based object tracking
- Hierarchical behavior arbitration
- Sensor fusion (vision + LIDAR)

### Key Nodes

#### Node 1: LIDAR Avoidance
- Subscribes: `/scan` (LaserScan)
- Publishes: `/cmd_vel` (Twist)
- Behavior: Reactive obstacle avoidance

#### Node 2: Color Tracker
- Subscribes: `/camera/image_raw` (Image)
- Publishes: `/cmd_vel` (Twist)
- Behavior: HSV-based color segmentation and tracking

#### Node 3: Fusion Controller
- Subscribes: `/scan`, `/camera/image_raw`
- Publishes: `/cmd_vel`
- Behavior: Safety-first hierarchical control
  1. Priority 1: Obstacle avoidance
  2. Priority 2: Target tracking
  3. Priority 3: Search behavior

### Key Deliverables
- `lab4_node1_lidar_avoidance.py`
- `lab4_node2_color_tracker.py`
- `lab4_node3_fusion_node.py`
- Demonstration videos showing integrated behavior

### Technologies
- OpenCV (cv2)
- cv_bridge
- HSV color space
- Sensor fusion algorithms
- Proportional control

---

## Lab 5: AI-Assisted Gesture-Based Control

**Objectives:** Vision-driven HRI, MediaPipe integration, AI-assisted development

### Topics Covered
- MediaPipe hand tracking pipeline
- Real-time gesture recognition
- Gesture-to-velocity mapping
- AI-assisted code development (ChatGPT/Claude/Copilot)
- Integration with Nav2 for high-level control

### Tasks

#### Task 1: Turtlesim Control
- Hand gesture control in simulation
- Drawing shapes with hand movements

#### Task 2: TurtleBot4 Teleoperation
- Adapt turtlesim controller to TurtleBot4
- Safety limits and deadzone implementation

#### Task 3: Room Navigation
- Complete loop navigation using gestures
- LIDAR and OAKD camera integration

#### Task 4: Gesture-Guided Autonomous Navigation
- High-level gesture commands
- Integration with Nav2 waypoint navigation
- Gesture vocabulary:
  - Open Palm → STOP
  - Closed Fist → Teleop mode
  - Point gestures → Navigate to goals

### Key Deliverables
- `hand_control_turtlesim.py`
- `hand_control_tb4_task2.py`
- `hand_control_tb4_task3.py`
- `hand_control_tb4_task4.py`

### Technologies
- MediaPipe
- OpenCV
- ROS2 Actions (Nav2)
- Real-time computer vision
- AI code generation tools

---

## Installation & Setup

### System Requirements

**Hardware:**
- TurtleBot4 with OAKD camera
- RPLIDAR A1/A2
- Laptop with Ubuntu 24.04 (native or VM recommended)
- Webcam (for Lab 5)

**Software:**
- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Python 3.10+

### Quick Start
```bash
# Install ROS2 Jazzy
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    python3-pip \
    git

# Python dependencies
pip install opencv-python mediapipe numpy

# Clone this repository
cd ~/ros2_ws/src
git clone <your-repo-url>

# Build workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### TurtleBot4 Connection
```bash
# Set up Discovery Server (on your laptop)
# Follow: https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html

# Or use ROS_DOMAIN_ID
export ROS_DOMAIN_ID=<your_id>
```

---

## Running the Labs

### Lab 1
```bash
ros2 run node1pkg node1
```

### Lab 2
```bash
# SLAM mapping
ros2 launch turtlebot4_navigation slam.launch.py

# Run drive node
ros2 run lab2_pkg drive_node
```

### Lab 3
```bash
# Feedback control
ros2 run lab3_pkg straight_drive_controller

# Navigation
ros2 launch turtlebot4_navigation nav_bringup.launch.py map:=~/lab2_map.yaml
ros2 run lab3_pkg send_nav2_goal
```

### Lab 4
```bash
# Launch all nodes
ros2 launch lab4 lab4_launch.py

# Or individual nodes
ros2 run lab4 lidar_avoidance
ros2 run lab4 color_tracker
ros2 run lab4 fusion_node
```

### Lab 5
```bash
# Gesture control
ros2 run lab5 hand_control_turtlesim
ros2 run lab5 hand_control_tb4_task2
ros2 run lab5 hand_control_tb4_task3
ros2 run lab5 hand_control_tb4_task4
```

---

## Key Concepts & Skills Developed

### ROS2 Architecture
- Publisher/Subscriber pattern
- Action clients and servers
- Launch files and parameters
- TF transforms
- Message types and interfaces

### Control Theory
- PID control
- Feedback loops
- Dead reckoning vs. sensor-based control
- Reactive vs. deliberative control
- Behavior hierarchies

### Perception
- LIDAR processing
- Computer vision (OpenCV)
- Color segmentation (HSV)
- Hand tracking (MediaPipe)
- Sensor fusion

### Navigation & Planning
- SLAM (Simultaneous Localization and Mapping)
- AMCL localization
- Costmap generation
- Path planning (Nav2)
- Waypoint navigation

### Software Engineering
- ROS2 package structure
- Modular code design
- Version control (Git)
- AI-assisted development
- Testing and debugging

---

## Academic Integrity

All work in this repository was completed in accordance with Northeastern University's Academic Integrity Policy. Where AI tools (ChatGPT, Claude, Copilot, etc.) were used, they served as assistive tools for debugging, code generation suggestions, and learning - with all final implementations understood, evaluated, and modified by the student.

Reference: [Northeastern Academic Integrity Policy](https://catalog.northeastern.edu/handbook/policies-regulations/academic-integrity/)

---

## Resources & References

### Official Documentation
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [TurtleBot4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [OpenCV Python Tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [MediaPipe Solutions](https://google.github.io/mediapipe/solutions/solutions.html)

### Course Materials
- Lab assignments and rubrics
- Lecture slides and notes
- TA office hours and lab sessions

### Additional Learning
- *Programming Robots with ROS* by Morgan Quigley et al.
- *Probabilistic Robotics* by Sebastian Thrun et al.
- ROS2 tutorials on ros.org

---

## Project Timeline

| Week | Lab | Focus Area |
|------|-----|------------|
| 1-2  | Lab 1 | ROS2 fundamentals, basic control |
| 3-4  | Lab 2 | SLAM, mapping, structured programming |
| 5-6  | Lab 3 | Feedback control, autonomous navigation |
| 7-8  | Lab 4 | Perception, sensor fusion, reactive control |
| 9-10 | Lab 5 | HRI, gesture control, AI-assisted development |

---

## Contact & Support

**Student:** Isaac Premkumar  
**Email:** premkumar.i@northeastern.edu  
**Program:** MS Data Analytics Engineering  
**Expected Graduation:** May 2026

**Instructor:** Dr. Xian Li  
**Course:** EECE 5554 - Robotics Sensing and Navigation  
**Semester:** Fall 2025  
**Location:** Northeastern University - Seattle Campus

---

## License

This repository contains academic coursework. Code may be referenced for educational purposes with proper attribution. Please respect academic integrity policies when using this code.

---

*Last Updated: December 2025*
