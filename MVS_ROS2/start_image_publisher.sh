#!/bin/bash

# Image Publisher 完整使用指南
# 用于循环播放图片代替rosbag进行测试

echo "==================================="
echo "Image Publisher 使用指南"
echo "==================================="
echo ""

IMAGE_PATH="/home/elf/slam/test.jpg"
TARGET_TOPIC="/hikrobot_camera/rgb"

echo "配置信息："
echo "- 源图像: $IMAGE_PATH"
echo "- 目标话题: $TARGET_TOPIC"
echo ""

# 检查图像文件是否存在
if [ ! -f "$IMAGE_PATH" ]; then
    echo "错误: 图像文件不存在: $IMAGE_PATH"
    exit 1
fi

echo "图像文件存在，大小: $(du -h $IMAGE_PATH | cut -f1)"
echo ""

echo "使用方法："
echo ""

echo "1. 基本命令 (推荐):"
echo "ros2 run image_publisher image_publisher_node $IMAGE_PATH \\"
echo "  --ros-args -r image_raw:=$TARGET_TOPIC \\"
echo "  -p frequency:=10.0 \\"
echo "  -p frame_id:=hikrobot_camera"
echo ""

echo "2. 不同频率选项:"
echo "  - 低频率 (1Hz): -p frequency:=1.0"
echo "  - 中频率 (5Hz): -p frequency:=5.0"  
echo "  - 高频率 (30Hz): -p frequency:=30.0"
echo ""

echo "3. 验证命令:"
echo "  - 检查话题: ros2 topic list | grep hikrobot"
echo "  - 检查频率: ros2 topic hz $TARGET_TOPIC"
echo "  - 查看图像: ros2 topic echo $TARGET_TOPIC --once"
echo ""

echo "4. 与推理节点配合使用:"
echo "  终端1: 启动image_publisher (上述命令)"
echo "  终端2: ros2 launch hikrobot_camera unet_inference.launch.py"
echo "  终端3: ros2 topic echo /inference/results"
echo ""

# 提供交互选项
echo "操作选项:"
echo "1) 立即启动 image_publisher (10Hz)"
echo "2) 启动 image_publisher (自定义频率)"
echo "3) 启动 image_publisher + UNet推理节点"
echo "4) 只显示命令不执行"
echo ""

read -p "请选择操作 (1-4): " choice

case $choice in
    1)
        echo "启动 image_publisher (10Hz)..."
        source /opt/ros/humble/setup.bash
        ros2 run image_publisher image_publisher_node $IMAGE_PATH \
          --ros-args -r image_raw:=$TARGET_TOPIC \
          -p frequency:=10.0 \
          -p frame_id:=hikrobot_camera
        ;;
    2)
        read -p "请输入发布频率 (Hz): " freq
        echo "启动 image_publisher (${freq}Hz)..."
        source /opt/ros/humble/setup.bash
        ros2 run image_publisher image_publisher_node $IMAGE_PATH \
          --ros-args -r image_raw:=$TARGET_TOPIC \
          -p frequency:=$freq \
          -p frame_id:=hikrobot_camera
        ;;
    3)
        echo "启动 image_publisher + UNet推理节点..."
        echo "请在新终端中运行推理节点: ros2 launch hikrobot_camera unet_inference.launch.py"
        source /opt/ros/humble/setup.bash
        ros2 run image_publisher image_publisher_node $IMAGE_PATH \
          --ros-args -r image_raw:=$TARGET_TOPIC \
          -p frequency:=10.0 \
          -p frame_id:=hikrobot_camera
        ;;
    4)
        echo "命令已显示，请手动执行所需命令。"
        ;;
    *)
        echo "无效选择。"
        ;;
esac
