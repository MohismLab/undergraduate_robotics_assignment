# Part 1: Obstacle Avoidance & Localization

This package implements obstacle avoidance algorithms (Braitenberg, rule-based) and localization using ROS 2 and Webots.

## Requirements
All computers at E11-1046 shall have following packages. If you want to install them on your own computer, you can run following command.  
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
or you can download the repository as zip file, and extrat the zip file  to obtain following files:
```bash
images/
src/
install_dependencies.sh
README.md
```

## Quick Start
Make sure that your terminal is within `~/Documents/course/robotics_ws`, otherwise:
```bash
cd ~/Documents/course/robotics_ws
```
deactivate conda to avoid python version conflict and source ros2.
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash 
```
**Build everything:**
```bash
colcon build
source install/setup.bash
```

## Running

**Terminal 1: Start Webots simulator**
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash 
source install/setup.bash
ros2 launch part1_ros2 webots_world.launch.py
```

**Terminal 2: Run obstacle avoidance**
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash 
source install/setup.bash

# Braitenberg mode
ros2 launch part1_ros2 obstacle_avoidance.launch.py mode:=braitenberg

# Or rule-based mode
ros2 launch part1_ros2 obstacle_avoidance.launch.py mode:=rule_based
```

**Terminal 3: Run localization (optional)**
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash 
source install/setup.bash
ros2 launch part1_ros2 localization.launch.py
```

## Utilities

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash 
source install/setup.bash
# Plot logged trajectories
ros2 run part1_ros2 plot_trajectory_part1
```

