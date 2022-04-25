# MAVROS PX4 Vehicle, A ROS Package

## Overview

This codebase is a ROS package that allows a user to communicate with a PX4 autopilot using MAVROS. This package was developed using [Ubuntu 18.04 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04/), [ROS Melodic](http://wiki.ros.org/melodic), and the [PX4 autopilot](https://docs.px4.io/master/en/). Users can control a PX4-enabled vehicle using an easy-to-use Python API. Essentially, users can control PX4-enabled vehicles within Python ROS nodes with this API. Currently, the API can control a multi-rotor aircraft. However, we are developing controls for (land-based) rovers too.


## Installation

These instructions assume you already have the appropriate version of Gazebo, ROS, and the PX4 autopilot. If not, please follow [these instructions](https://github.com/troiwill/px4-mavros-gazebo-sim/tree/alpha). To add this package to your catkin workspace, simply clone this repository, navigate to your catkin workspace, create a link to the ROS package, and then go through your catkin build process.

```
# Clone this repository.
mkdir -p ${HOME}/repos && cd ${HOME}/repos
git clone https://github.com/troiwill/mavros-px4-vehicle.git

# Go to the src directory of your catkin_ws root dir.
cd ${HOME}/catkin_ws
cd src

# Link the ROS package from this repository.
ln -s ${HOME}/repos/mavros-px4-vehicle mavros-px4-vehicle

# Go to your catkin_ws root dir and build the environment.
cd ${HOME}/catkin_ws
catkin build
source devel/setup.bash

# Make the Python scripts executable.

# Start writing Python scripts to control the vehicles.
```
