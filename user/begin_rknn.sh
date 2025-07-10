#!/bin/bash

cd /home/elf/slam/MVS_ROS2
source install/setup.bash
ros2 launch hikrobot_camera rknn_inference.launch.py