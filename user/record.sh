#!/bin/bash

# 录制压缩图像的脚本

echo "启动压缩图像录制..."
cd /home/elf/ros_bag/

# 设置环境
source /home/elf/slam/MVS_ROS2/install/setup.bash
source /home/elf/slam/livox/livox_ws_1/install/setup.sh --extend

# record following topics
    #   lid_topic: "/livox/lidar_converted"
    #   imu_topic: "/livox/imu"
    # /compressed_image/compressed
ros2 bag record /livox/lidar /livox/imu /compressed_image/compressed  

