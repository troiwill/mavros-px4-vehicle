# Prerequisites

These instructions are for installing required dependencies on Ubuntu 18.04.

## Installing ROS and Gazebo
1) Go [here](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) and follow the instructions below ROS/Gazebo

2) Run the following
```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

3) Build the catkin workspace
```
cd ${HOME}/catkin_ws
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```

## Installing PX4
1) Create a repos folder and clone PX4 (the v1.12.3 branch is required)
```
cd
mkdir repos
cd repos
git clone -b v1.12.3 https://github.com/PX4/Firmware.git --recursive
```

2) Run the bash script to set up PX4
```
cd Firmware
bash ./Tools/setup/ubuntu.sh
```

3) To test PX4 alone, run
```
make px4_sitl gazebo
```