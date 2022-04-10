---
layout: default
title: Running the Robot
parent: Usage
nav_order: 3
---

# Running the Robot

## SSH
The easiest way to access the Jetson is with SSH. Make sure you know the username and password for your Jetson.

- `ssh username@192.168.55.1` is the default way to SSH through a micro-USB cable
- `ssh username@jetsonname` can be used if connected to same network

## Running Gym2Real 
- `cd` to `workspace` folder
- `colcon build --symlink-install` to build packages (only necessary the first time)
- `sudo su` to start root shell (required for some features like setting real-time priority)
- `source install/setup.bash`
- `export ROS_DOMAIN_ID=1` allows exchange of messages to other PC on the same network with the same ROS_DOMAIN_ID (use your favourite number)
- `ros2 launch controller controller.launch.py` starts the nodes (controller, IMU driver, motor driver)

## Uploading Model
- Place trained model in `.onnx` format in `workspace/cfg`
- Modify `workspace/cfg/config.yaml` to add your controller (if the model has different observation size or action size, modifications will have to be made to `workspace/controller/src/controller_node.cpp` to handle the changes)

## Optional Steps
- `sudo crontab -e` can be used to add a startup script that runs whenever the Jetson boots