# Undergraduate Robotics Assignment (ROS 2 + Webots 2025a)

![Assignment](images/.assignment.jpg)

## Requirements 

All computers at E11-1046 shall have following packages. If you want to install them on your own computer, you can run following command.  
- **Webots R2025a** installed (`webots` available on your PATH)
- **ROS 2 Jazzy** sourced in your shell (`source /opt/ros/jazzy/setup.bash`). You can find installation tutorial via [this link](https://docs.ros.org/en/jazzy/Installation.html).
- **Python dependencies**: `numpy`, `matplotlib`, `scipy`, `PyYAML`

```bash
# Install ROS 2 dependencies
sudo apt install ros-jazzy-webots-ros2
sudo apt install ros-jazzy-turtlebot3-description
sudo apt install ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-tf-transformations
```

## Setup the repo
1. Open a terminal and create directory in Documents:
```bash
cd ~/Documents
mkdir -p course/robotics_ws/src
cd ~/Documents/course/robotics_ws/src
```

2. Clone down this repository:
```bash
git clone https://github.com/MohismLab/undergraduate_robotics_assignment.git
```
or you can download the repository as zip file, and extrat the zip file  to obtain following files:
```bash
images/
src/
install_dependencies.sh
README.md
```
move all these files within `~/Documents/course/robotics_ws`.

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

## Parts

| Part | Description | Details |
|------|-------------|---------|
| **Part 1** | Obstacle Avoidance & Localization | [src/part1/ros2/README.md](src/part1/ros2/README.md) |
| **Part 2** | RRT Navigation with Pre-built Map | [src/part2/ros2/README.md](src/part2/ros2/README.md) |


