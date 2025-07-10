#!/bin/bash

# 图像压缩节点测试脚本

echo "启动图像压缩节点测试..."

# 设置环境
source /home/elf/slam/MVS_ROS2/install/setup.bash

echo "可用的命令："
echo "1. 启动图像压缩节点: ros2 launch hikrobot_camera image_compression.launch.py"
echo "2. 直接运行节点: ros2 run hikrobot_camera image_compression_node"
echo "3. 查看话题: ros2 topic list | grep compressed"
echo "4. 查看压缩图像信息: ros2 topic echo /compressed_image/compressed --once"
echo "5. 录制压缩图像: ros2 bag record /compressed_image/compressed"
echo ""
echo "参数说明："
echo "- input_topic: 输入图像话题 (默认: /hikrobot_camera/rgb)"
echo "- output_topic: 输出话题前缀 (默认: /compressed_image)"  
echo "- compression_quality: 压缩质量 1-100 (默认: 80)"
echo "- compression_format: 压缩格式 jpg/png (默认: jpg)"
echo ""
echo "输出话题："
echo "- /compressed_image/compressed (压缩图像)"
echo "- /compressed_image (解压验证图像，仅在有订阅者时发布)"
