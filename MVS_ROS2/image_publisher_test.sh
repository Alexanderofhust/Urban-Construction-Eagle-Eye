#!/bin/bash

# image_publisher使用脚本 - 循环播放图片代替rosbag

echo "Image Publisher 使用指南"
echo "======================"
echo ""

# 设置环境
source /opt/ros/humble/setup.bash

IMAGE_PATH="/home/elf/slam/test.jpg"
TARGET_TOPIC="/hikrobot_camera/rgb"

echo "目标图像: $IMAGE_PATH"
echo "发布话题: $TARGET_TOPIC"
echo ""

echo "可用的命令选项："
echo ""

echo "1. 基本循环播放 (1Hz频率):"
echo "ros2 run image_publisher image_publisher_node $IMAGE_PATH --ros-args -r image:=$TARGET_TOPIC"
echo ""

echo "2. 指定频率播放 (10Hz):"
echo "ros2 run image_publisher image_publisher_node $IMAGE_PATH --ros-args -r image:=$TARGET_TOPIC -p frequency:=10.0"
echo ""

echo "3. 设置frame_id:"
echo "ros2 run image_publisher image_publisher_node $IMAGE_PATH --ros-args -r image:=$TARGET_TOPIC -p frame_id:='hikrobot_camera'"
echo ""

echo "4. 完整配置 (推荐使用):"
echo "ros2 run image_publisher image_publisher_node $IMAGE_PATH --ros-args \\"
echo "  -r image:=$TARGET_TOPIC \\"
echo "  -p frequency:=10.0 \\"
echo "  -p frame_id:='hikrobot_camera' \\"
echo "  -p loop:=true"
echo ""

echo "5. 启动发布器 (后台运行):"
read -p "是否现在启动图像发布器? (y/n): " start_publisher

if [ "$start_publisher" = "y" ] || [ "$start_publisher" = "Y" ]; then
    echo "启动图像发布器..."
    echo "使用 Ctrl+C 停止发布"
    
    ros2 run image_publisher image_publisher_node $IMAGE_PATH --ros-args \
      -r image:=$TARGET_TOPIC \
      -p frequency:=10.0 \
      -p frame_id:='hikrobot_camera' \
      -p loop:=true
else
    echo "脚本结束。您可以手动运行上述命令。"
fi
