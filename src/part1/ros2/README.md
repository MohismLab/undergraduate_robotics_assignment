# Part 1: Obstacle Avoidance & Localization

This package implements obstacle avoidance algorithms (Braitenberg, rule-based) and localization using ROS 2 and Webots.

## Requirements

```bash
sudo apt install ros-jazzy-webots-ros2
sudo apt install ros-jazzy-turtlebot3-description
```

## Setup the repo
1. Open a terminal and create directory in Documents:
```bash
cd ~/Documents
mkdir -p course/robotics_ws/src
cd ~/Documents/course/robotics_ws/src
git clone https://github.com/MohismLab/undergraduate_robotics_assignment.git
```

## Build

```bash
cd ~/Documents/course/robotics_ws
colcon build
source install/setup.bash
```

## Running

**Terminal 1: Start Webots simulator**
```bash
ros2 launch part1_ros2 webots_world.launch.py
```

**Terminal 2: Run obstacle avoidance**
```bash
# Braitenberg mode
ros2 launch part1_ros2 obstacle_avoidance.launch.py mode:=braitenberg

# Or rule-based mode
ros2 launch part1_ros2 obstacle_avoidance.launch.py mode:=rule_based
```


## Utilities

```bash
# Plot logged trajectories
ros2 run part1_ros2 plot_trajectory_part1
```

