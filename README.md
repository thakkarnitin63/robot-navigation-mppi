Robot Navigation System
A complete ROS2 navigation system for TurtleBot3 with path smoothing, trajectory generation, and obstacle avoidance.

Overview
This project implements autonomous navigation for differential drive robots through three core tasks:

Path Smoothing - Convert discrete waypoints into smooth curves using Catmull-Rom splines
Trajectory Generation - Add velocity and timing constraints using trapezoidal profiles
Trajectory Tracking - Follow paths using Pure Pursuit and MPPI controllers

Features

Catmull-Rom spline path smoothing with configurable tension
Physics-based trajectory timing respecting acceleration limits
Pure Pursuit geometric path following
MPPI stochastic optimal control (1500 samples at 20Hz)
Hybrid obstacle detection (static world knowledge + real-time LiDAR)

Two Controllers
ControllerUse CaseSpeedObstaclesPure PursuitClean environments50 HzNoMPPIComplex environments20 HzYes

Installation
Prerequisites

Ubuntu 22.04
ROS2 Humble
TurtleBot3 packages
Gazebo simulator

Step 1: Install ROS2 Humble
If not installed, follow: ROS2 Humble Installation
Step 2: Install Dependencies
bashsudo apt update
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-gazebo-ros-pkgs
pip3 install matplotlib pandas numpy
Step 3: Build Package
bash# Create workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone repository
git clone https://github.com/YOUR_USERNAME/robot-navigation-mppi.git

# Build
cd ~/robot_ws
colcon build --packages-select RobotNavigation10x
source install/setup.bash

# Set robot model
export TURTLEBOT3_MODEL=burger

Quick Start
Run Pure Pursuit (Clean Environment)
bashcd ~/robot_ws/src/robot-navigation-mppi/RobotNavigation10x/src
ros2 launch RobotNavigation10x run_tracker.launch.py
What happens:

Robot spawns in 10×10m environment
Generates and follows smooth trajectory
Reaches goal in ~45 seconds

Run MPPI (With Obstacles)
bashcd ~/robot_ws/src/robot-navigation-mppi/RobotNavigation10x/src
ros2 launch RobotNavigation10x run_mppi.launch.py
What happens:

Environment has 7 obstacles
Robot automatically avoids all obstacles
Reaches goal in ~60 seconds

Generate Plots
bashpython3 plot_path.py
Creates:

task_1_smoothing.png
task_2_timestamp_path.png


Configuration
Pure Pursuit Parameters
Edit config/tracker_params.yaml:
yamlmax_linear_velocity: 0.5      # Maximum speed (m/s)
lookahead_distance: 0.75      # Tracking distance (m)
num_waypoints: 6              # Path complexity
MPPI Parameters
Edit config/mppi_params.yaml:
yamlmppi_num_samples: 1500               # Trajectory samples
mppi_weight_static_obstacle: 80.0    # Obstacle avoidance strength
mppi_weight_goal: 100.0              # Goal reaching priority

Results
MetricPure PursuitMPPIControl Rate50 Hz20 HzComputation<1 ms30-50 msSuccess Rate100%95%Completion Time45s60sObstacles❌✅

Troubleshooting
Robot doesn't move
bash# Check odometry
ros2 topic hz /odom

# Check nodes
ros2 node list
No CSV files
Run from correct directory:
bashcd ~/robot_ws/src/robot-navigation-mppi/RobotNavigation10x/src
MPPI too slow
Reduce samples in config/mppi_params.yaml:
yamlmppi_num_samples: 1000
Robot collides
Increase obstacle weights:
yamlmppi_weight_static_obstacle: 120.0

Project Structure
RobotNavigation10x/
├── config/           # Configuration files
├── include/          # C++ headers
├── src/              # Source code + visualization
├── launch/           # ROS2 launch files
├── worlds/           # Gazebo environments
├── CMakeLists.txt
└── package.xml

Development
Built in 2 days using Claude AI for rapid prototyping and algorithm implementation.