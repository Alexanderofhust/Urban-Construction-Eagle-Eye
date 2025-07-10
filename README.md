# Urban-Construction-Eagle-Eye

## 2025年全国大学生嵌入式芯片与系统设计竞赛



### 城建鹰眼——基于搭载 RK3588 芯片的飞凌嵌入式 ELF 2 开发板的基建三维重建自动检测与数字孪生系统



### 无人机旋转30秒 

## 项目概述

Urban Construction Eagle Eye 是一个用于基建缺陷检测和相关数据处理的综合性项目。该项目结合了多种技术，包括基于 RK3588 NPU 的基建缺陷检测模型、Livox 激光雷达驱动、Hikrobot 相机驱动、图像推理和压缩节点，以及基于 ROS 2 的 SLAM 功能。项目旨在提供一个高效、准确的解决方案，用于城市建设中的数据采集、处理和分析。

## 主要特性

- **基于 RK3588 NPU 的基建缺陷检测**：利用 RK3588 的强大计算能力，实现高效的基建缺陷检测。
- **多传感器融合**：集成 Livox 激光雷达和 Hikrobot 相机，提供丰富的环境感知数据。
- **图像推理和压缩**：对相机采集的图像进行推理和压缩处理，减少数据传输和存储成本。
- **SLAM 功能**：实现实时的激光雷达里程计和建图，为城市建设提供精确的地图信息。

## 代码结构



```plaintext
Urban-Construction-Eagle-Eye/
├── QianTrain/              # 模型训练与转换
├── .gitignore              # Git忽略文件配置
├── livox_ros_driver2/      # Livox激光雷达ROS 2驱动
├── MVS_ROS2/               # Hikrobot相机ROS 2驱动与模型推理代码
├── user/                   # 用户脚本，用于启动项目和录制数据
├── fast_livo2_ws/          # 基于ROS 2的SLAM功能包
```

## 安装与使用

### 依赖安装

确保你已经安装了以下依赖：

- ROS 2
- OpenCV
- RKNN Toolkit 2
- Livox SDK

### 编译项目



```bash
# 创建工作空间
mkdir -p ~/slam/Urban-Construction-Eagle-Eye_ws/src
cd ~/slam/Urban-Construction-Eagle-Eye_ws/src

# 克隆项目代码
git clone https://github.com/Alexanderofhust/Urban-Construction-Eagle-Eye.git

# 编译项目
cd ~/slam/Urban-Construction-Eagle-Eye_ws
colcon build
```

### 启动项目



```bash
# 进入工作空间
cd ~/slam/Urban-Construction-Eagle-Eye_ws

# 加载环境变量
source install/setup.bash

# 启动项目脚本
./user/start_slam.sh
```

### 录制数据

```bash
# 启动录制脚本
./user/record.sh
```

### 可视化

```bash
# 启动RViz
./user/rviz_begin.sh
```

## 贡献指南

欢迎提交 Issue 和 Pull Request 来改进这个项目！如果你有任何问题或建议，请随时在 GitHub 上提出。

## 版权信息

本项目使用了多个开源库和工具，具体版权信息请参考各个文件中的 LICENSE 文件。

- Livox ROS Driver2 使用了 MIT 许可证。
- RapidJSON 使用了 MIT 许可证。
- ROS 使用了 3-Clause-BSD 许可证。
- ROS2-rclcpp 使用了 Apache License 2.0。

## 联系方式

如果你有任何问题或建议，请联系项目维护者：2356296594@qq.com

## 致谢

感谢所有为这个项目做出贡献的人！