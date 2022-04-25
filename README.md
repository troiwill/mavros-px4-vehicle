# MAVROS PX4 Vehicle, A ROS Package

## Overview

This codebase is a ROS package that allows a user to communicate with a PX4 autopilot using MAVROS. This package was developed using [Ubuntu 18.04 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04/), [ROS Melodic](http://wiki.ros.org/melodic), and the [PX4 autopilot](https://docs.px4.io/master/en/). Users can control a PX4-enabled vehicle using an easy-to-use Python API. Essentially, users can control PX4-enabled vehicles within Python ROS nodes with this API. Currently, the API can control a multi-rotor aircraft. However, we are developing controls for (land-based) rovers too.


## Quick Start: Adding This Package to Your Workspace

These instructions assume you already have the appropriate version of Gazebo, ROS, and the PX4 autopilot. If not, please follow these instructions. To add this package to your catkin workspace, simply navigate to your catkin workspace, clone this repository, and then go through your catkin build process.
```
# Go to your catkin_ws root dir.
cd <catkin_ws root>
cd src

# Clone this repository using HTTP or SSH.
git clone https://github.com/troiwill/mavros-px4-vehicle.git

# Go to your catkin_ws root dir and build the environment.
cd <catkin_ws root>
catkin build
source devel/setup.bash

# Make the Python scripts executable.

# Start writing Python scripts to control the vehicles.
```
