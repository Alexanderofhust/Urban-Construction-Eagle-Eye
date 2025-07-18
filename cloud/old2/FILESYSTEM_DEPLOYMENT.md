# 文件系统数据处理流程部署指南

## 📋 概述

本指南介绍了Urban Construction Eagle Eye系统的新数据处理流程。我们已经从基于ROS2 rosbag的处理方式改为基于文件系统的处理方式，这样可以避免在Python中直接使用ROS2，同时减少网络传输负载。

## 🔄 新的数据流程

### 1. 数据流设计

```
ROS节点 → 文件系统 → Python处理器 → Web界面
    ↓           ↓            ↓           ↓
  图像/mask    自动保存      监控处理      结果展示
   点云PCD     定时写入      实时分析      AI分析
```

### 2. 文件组织结构

```
data/
├── {session_id}/
│   ├── images/                 # 原始图像
│   │   ├── 20240718_143025_000001.jpg
│   │   ├── 20240718_143026_000002.jpg
│   │   └── ...
│   ├── masks/                  # 分割mask
│   │   ├── 20240718_143025_000001_mask.png
│   │   ├── 20240718_143026_000002_mask.png
│   │   └── ...
│   ├── pointclouds/            # 点云数据
│   │   ├── final_pointcloud_20240718_143530.pcd
│   │   ├── incremental_pointcloud_20240718_143030_0001.pcd
│   │   └── ...
│   └── metadata/               # 元数据
│       ├── 20240718_143025_000001_metadata.json
│       ├── pointcloud_session_info.json
│       └── session_summary.json
```

## 🚀 部署步骤

### 步骤1: 准备文件系统数据处理器

1. **复制文件到目标系统**
   ```bash
   # 复制以下文件到Ubuntu系统
   cp filesystem_data_processor.py /path/to/target/
   cp main_filesystem.py /path/to/target/
   cp requirements.txt /path/to/target/
   cp -r ros_nodes/ /path/to/target/
   ```

2. **安装Python依赖**
   ```bash
   pip install -r requirements.txt
   ```

### 步骤2: 配置ROS节点

1. **复制ROS节点文件**
   ```bash
   mkdir -p ~/ros2_ws/src/eagle_eye_nodes/scripts
   cp ros_nodes/*.py ~/ros2_ws/src/eagle_eye_nodes/scripts/
   chmod +x ~/ros2_ws/src/eagle_eye_nodes/scripts/*.py
   ```

2. **创建ROS2包**
   ```bash
   cd ~/ros2_ws/src/eagle_eye_nodes
   
   # 创建package.xml
   cat > package.xml << EOF
   <?xml version="1.0"?>
   <package format="3">
     <name>eagle_eye_nodes</name>
     <version>1.0.0</version>
     <description>Eagle Eye data saving nodes</description>
     <maintainer email="your@email.com">Your Name</maintainer>
     <license>MIT</license>
     <depend>rclpy</depend>
     <depend>sensor_msgs</depend>
     <depend>std_msgs</depend>
     <depend>cv_bridge</depend>
     <depend>sensor_msgs_py</depend>
   </package>
   EOF
   
   # 创建CMakeLists.txt
   cat > CMakeLists.txt << EOF
   cmake_minimum_required(VERSION 3.5)
   project(eagle_eye_nodes)
   
   find_package(ament_cmake REQUIRED)
   
   install(PROGRAMS
     scripts/image_saver_node.py
     scripts/pointcloud_saver_node.py
     DESTINATION lib/\${PROJECT_NAME}
   )
   
   ament_package()
   EOF
   ```

3. **编译ROS2包**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select eagle_eye_nodes
   source install/setup.bash
   ```

### 步骤3: 启动数据处理系统

1. **启动Python处理器**
   ```bash
   cd /path/to/target
   python main_filesystem.py --host 0.0.0.0 --port 8000
   ```

2. **启动ROS节点**
   ```bash
   # 使用启动脚本
   ./ros_nodes/start_nodes.sh my_session_id /tmp/eagle_eye_data
   
   # 或手动启动
   ros2 run eagle_eye_nodes image_saver_node.py --ros-args -p session_id:=my_session_id
   ros2 run eagle_eye_nodes pointcloud_saver_node.py --ros-args -p session_id:=my_session_id
   ```

## ⚙️ 配置说明

### 1. 数据处理器配置

编辑`.env`文件：
```env
# 数据目录
DATA_DIR=./data
IMAGE_DIR=./data/images
MASK_DIR=./data/masks
POINTCLOUD_DIR=./data/pointclouds
METADATA_DIR=./data/metadata

# 监控配置
MONITOR_INTERVAL=2.0
MAX_FILE_AGE_HOURS=24
AUTO_CLEANUP_ENABLED=true

# 处理配置
POINTCLOUD_DOWNSAMPLE_RATIO=0.1
MAX_POINTCLOUD_SIZE=1000000
```

### 2. ROS节点配置

**图像保存节点参数：**
- `session_id`: 会话ID
- `image_topic`: 图像话题 (默认: `/camera/image_raw`)
- `mask_topic`: mask话题 (默认: `/segmentation/mask`)
- `output_dir`: 输出目录 (默认: `/tmp/eagle_eye_data`)
- `save_interval`: 保存间隔秒数 (默认: 1.0)
- `image_quality`: JPEG质量 (默认: 95)
- `max_images_per_session`: 每会话最大图像数 (默认: 1000)

**点云保存节点参数：**
- `session_id`: 会话ID
- `pointcloud_topic`: 点云话题 (默认: `/mapping/pointcloud`)
- `mapping_status_topic`: 建图状态话题 (默认: `/mapping/status`)
- `output_dir`: 输出目录 (默认: `/tmp/eagle_eye_data`)
- `auto_save_interval`: 自动保存间隔秒数 (默认: 30.0)
- `max_points_per_cloud`: 每个点云最大点数 (默认: 1000000)
- `downsample_ratio`: 降采样比例 (默认: 0.1)

## 🔍 监控和调试

### 1. 查看处理状态

```bash
# 通过API查看会话状态
curl http://localhost:8000/api/session/my_session_id/status

# 查看数据文件
curl http://localhost:8000/api/data?session_id=my_session_id
```

### 2. 检查文件系统

```bash
# 查看保存的文件
ls -la /tmp/eagle_eye_data/my_session_id/

# 查看图像
ls -la /tmp/eagle_eye_data/my_session_id/images/

# 查看点云
ls -la /tmp/eagle_eye_data/my_session_id/pointclouds/
```

### 3. 查看日志

```bash
# ROS节点日志
ros2 node info /image_saver_node
ros2 node info /pointcloud_saver_node

# Python处理器日志
tail -f ./logs/eagle_eye.log
```

## 🛠️ 故障排除

### 1. 常见问题

**问题1: 文件没有被保存**
- 检查ROS话题是否发布数据
- 检查输出目录权限
- 查看节点日志

**问题2: Python处理器检测不到文件**
- 检查目录路径配置
- 确认文件监控间隔设置
- 验证文件权限

**问题3: 点云文件过大**
- 调整降采样比例
- 设置最大点数限制
- 增加磁盘空间

### 2. 性能优化

**存储优化：**
```bash
# 使用SSD存储
sudo mount -t tmpfs -o size=2G tmpfs /tmp/eagle_eye_data

# 设置自动清理
echo "0 2 * * * find /tmp/eagle_eye_data -type f -mtime +7 -delete" | crontab -
```

**处理优化：**
```env
# 增加监控间隔
MONITOR_INTERVAL=5.0

# 减少并发处理
MAX_CONCURRENT_SESSIONS=5
```

## 📊 数据格式说明

### 1. 图像元数据格式

```json
{
  "filename": "20240718_143025_000001.jpg",
  "mask_filename": "20240718_143025_000001_mask.png",
  "timestamp": "2024-07-18T14:30:25.123456",
  "ros_timestamp": {
    "sec": 1721290225,
    "nanosec": 123456789
  },
  "image_size": {
    "width": 1920,
    "height": 1080,
    "channels": 3
  },
  "mask_info": {
    "has_mask": true,
    "mask_area": 15420,
    "total_area": 2073600,
    "coverage_ratio": 0.0074
  },
  "header": {
    "frame_id": "camera_frame",
    "seq": 1
  }
}
```

### 2. 点云元数据格式

```json
{
  "filename": "final_pointcloud_20240718_143530.pcd",
  "timestamp": "2024-07-18T14:35:30.789012",
  "ros_timestamp": {
    "sec": 1721290530,
    "nanosec": 789012345
  },
  "point_count": 125000,
  "original_point_count": 1250000,
  "is_final": true,
  "mapping_status": "completed",
  "header": {
    "frame_id": "map",
    "seq": 1
  },
  "file_size": 15728640
}
```

## 🎯 优势总结

1. **简化部署**: 避免了Python中的ROS2依赖问题
2. **减少带宽**: 不需要传输大型rosbag文件
3. **实时处理**: 文件一旦保存就立即开始处理
4. **灵活配置**: 可以根据需求调整保存策略
5. **易于扩展**: 可以轻松添加新的数据类型

这个新的数据处理流程完全解决了您提到的ROS安装和传输带宽问题，同时提供了更灵活和高效的数据处理能力。
