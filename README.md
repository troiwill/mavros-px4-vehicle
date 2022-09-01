# MAVROS PX4 Vehicle, A ROS Package

## Overview

This codebase is a ROS package that allows a user to communicate with [PX4 autopilot](https://docs.px4.io/master/en/) using MAVROS. This package was developed for [Ubuntu 18.04 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04/) and [ROS Melodic](http://wiki.ros.org/melodic). However, this pacakge also works with [Ubuntu 20.04 LTS (Focal Fossa)](https://releases.ubuntu.com/20.04/) and [ROS Noetic](http://wiki.ros.org/noetic).

Users can control a PX4-enabled vehicle within Python ROS nodes with this API. Currently, the API can control a multi-rotor aircraft.


## Installation

These instructions assume you already have the appropriate version of Gazebo, ROS, and the PX4 autopilot. You will also need to install QCGroundControl. If not, please follow [these instructions](https://github.com/troiwill/px4-mavros-gazebo-sim/tree/alpha). To add this package to your catkin workspace, simply clone this repository, navigate to your catkin workspace, create a link to the ROS package, and then go through your catkin build process.

```
# Clone this repository.
mkdir -p ${HOME}/repos && cd ${HOME}/repos
git clone https://github.com/troiwill/mavros-px4-vehicle.git

# Make the Python scripts executable.
chmod +x mavros-px4-vehicle/scripts/*.py
chmod +x mavros-px4-vehicle/test/*.py

# Go to the src directory of your catkin_ws root dir.
cd ${HOME}/catkin_ws
cd src

# Link the ROS package from this repository.
ln -s ${HOME}/repos/mavros-px4-vehicle mavros-px4-vehicle

# Go to your catkin_ws root dir and build the environment.
cd ${HOME}/catkin_ws
catkin build
source devel/setup.bash
```

# Quick start

Perform the following tasks to ensure this package it working with your setup.

1) Start QCGroundControl and enable the joysticks via Settings. If you do not enable the joysticks, the vehicle will not arm or take off.

2) Set up your environment to use PX4 autopilot. You will always have to run these following when you open a new terminal.
```
cd ${HOME}/repos/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

3) Run the test launch file.
```
roslaunch mavros_px4_vehicle test_vehicle.test
```

4) Assuming all the tests are successful, you should now run the offboard hover Python script. You must use two (2) terminal windows to run the script.
The first terminal window should run MAVROS, Gazebo, and PX4.
```
roslaunch px4 mavros_posix_sitl.launch
```

The second terminal window should run the hover script.
```
rosrun mavros_px4_vehicle offboard_hover.py
```
If the script is successful, the drone should hover a few meters above the ground.
