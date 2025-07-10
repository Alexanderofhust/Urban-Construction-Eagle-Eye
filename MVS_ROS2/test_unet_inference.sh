#!/bin/bash

# UNet推理节点测试脚本

echo "UNet分割推理节点测试"
echo "===================="

# 设置环境
source /home/elf/slam/MVS_ROS2/install/setup.bash

echo "可用的命令："
echo "1. 启动UNet推理节点: ros2 launch hikrobot_camera unet_inference.launch.py"
echo "2. 直接运行节点: ros2 run hikrobot_camera image_inference_node --ros-args --params-file /home/elf/slam/MVS_ROS2/install/hikrobot_camera/share/hikrobot_camera/config/unet_inference_params.yaml"
echo "3. 查看推理结果: ros2 topic echo /inference/results"
echo "4. 查看可视化: ros2 topic echo /inference/visualization"
echo "5. 录制推理结果: ros2 bag record /inference/results /inference/visualization"
echo ""
echo "UNet模型参数："
echo "- 输入尺寸: 480x480"
echo "- 输出尺寸: 480x480"
echo "- 类别数: 3 (包括背景)"
echo "- 输入格式: 1x3x480x480"
echo "- 输出格式: 1x3x480x480"
echo ""
echo "注意事项："
echo "1. 请确保best.onnx模型文件路径正确"
echo "2. 根据实际情况调整config/unet_inference_params.yaml中的参数"
echo "3. 类别名称可以在代码中的initializeClassNames()函数中修改"
