#!/bin/bash

# RKNN推理节点测试脚本

echo "RKNN分割推理节点测试"
echo "===================="

# 设置环境
source /home/elf/slam/MVS_ROS2/install/setup.bash

echo "可用的命令："
echo "1. 启动RKNN推理节点: ros2 launch hikrobot_camera rknn_inference.launch.py"
echo "2. 直接运行节点: ros2 run hikrobot_camera image_inference_node --ros-args --params-file /home/elf/slam/MVS_ROS2/install/hikrobot_camera/share/hikrobot_camera/config/rknn_inference_params.yaml"
echo "3. 查看推理结果: ros2 topic echo /inference/results"
echo "4. 查看可视化: ros2 topic echo /inference/visualization"
echo "5. 录制推理结果: ros2 bag record /inference/results /inference/visualization"
echo ""
echo "RKNN模型参数："
echo "- 输入尺寸: 480x480"
echo "- 输出尺寸: 480x480"
echo "- 类别数: 3 (包括背景)"
echo "- 模型文件: segmentation_model.rknn"
echo ""
echo "注意事项："
echo "1. 请确保segmentation_model.rknn模型文件路径正确"
echo "2. 根据实际情况调整config/rknn_inference_params.yaml中的参数"
echo "3. 类别名称可以在代码中的initializeClassNames()函数中修改"
echo "4. RKNN推理需要RK3588硬件支持"
echo ""
echo "编译信息："
echo "- RKNN头文件: /home/elf/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include"
echo "- RKNN库文件: /home/elf/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
echo ""

# 检查模型文件
MODEL_FILE="/home/elf/slam/MVS_ROS2/segmentation_model.rknn"
if [ -f "$MODEL_FILE" ]; then
    echo "✓ 找到RKNN模型文件: $MODEL_FILE"
    echo "  文件大小: $(du -h $MODEL_FILE | cut -f1)"
else
    echo "✗ 未找到RKNN模型文件: $MODEL_FILE"
    echo "  请确保模型文件存在于正确的路径"
fi

echo ""
echo "准备启动RKNN推理节点..."
