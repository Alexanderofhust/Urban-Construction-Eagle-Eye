#!/bin/bash
# Ubuntu一键部署脚本

set -e

echo "========================================="
echo "Urban Construction Eagle Eye"
echo "Ubuntu一键部署脚本"
echo "========================================="

# 检查是否为root用户
if [[ $EUID -eq 0 ]]; then
   echo "请不要使用root用户运行此脚本"
   exit 1
fi

# 检查Ubuntu版本
echo "正在检查系统版本..."
if ! command -v lsb_release &> /dev/null; then
    echo "错误: 无法检测系统版本"
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs)
echo "检测到Ubuntu版本: $UBUNTU_VERSION"

# 更新系统包
echo "正在更新系统包..."
sudo apt update

# 安装基础依赖
echo "正在安装基础依赖..."
sudo apt install -y curl gnupg lsb-release python3 python3-pip python3-venv build-essential

# 安装ROS2
echo "正在安装ROS2..."
if ! command -v ros2 &> /dev/null; then
    echo "安装ROS2 Humble..."
    
    # 设置locale
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    
    # 添加ROS2仓库
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # 安装ROS2
    sudo apt update
    sudo apt install -y ros-humble-desktop python3-rosdep python3-rosbag2 python3-rclpy python3-sensor-msgs python3-std-msgs python3-cv-bridge
    
    # 初始化rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # 设置环境变量
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
    echo "ROS2安装完成"
else
    echo "ROS2已安装"
fi

# 创建项目目录
PROJECT_DIR="$HOME/eagle_eye_system"
echo "创建项目目录: $PROJECT_DIR"
mkdir -p $PROJECT_DIR
cd $PROJECT_DIR

# 创建Python虚拟环境
echo "创建Python虚拟环境..."
python3 -m venv venv
source venv/bin/activate

# 检查是否有项目文件
if [ ! -f "main_ubuntu.py" ]; then
    echo "错误: 项目文件不存在"
    echo "请将项目文件复制到: $PROJECT_DIR"
    echo "包括以下文件:"
    echo "- main_ubuntu.py"
    echo "- ubuntu_rosbag_processor.py"
    echo "- requirements_ubuntu.txt"
    echo "- 其他项目文件"
    exit 1
fi

# 安装Python依赖
echo "安装Python依赖..."
if [ -f "requirements_ubuntu.txt" ]; then
    pip install -r requirements_ubuntu.txt
elif [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
else
    echo "错误: 找不到requirements文件"
    exit 1
fi

# 创建配置文件
echo "创建配置文件..."
if [ ! -f ".env" ]; then
    cat > .env << EOF
# 高德地图API密钥
AMAP_API_KEY=your_amap_api_key
AMAP_SECURITY_CODE=your_amap_security_code

# Google Gemini API密钥
GEMINI_API_KEY=your_gemini_api_key

# 服务器配置
SERVER_HOST=0.0.0.0
SERVER_PORT=8000
DEBUG=false

# ROS2配置
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0

# 数据路径
DATA_DIR=./data
ROSBAG_PATH=/home/\$USER/rosbags

# 处理配置
POINTCLOUD_DOWNSAMPLE_RATIO=0.1
MAX_POINTCLOUD_SIZE=1000000
REVIEW_IMAGE_COUNT=10
REVIEW_IMAGE_SIZE=800

# AI分析配置
AI_ANALYSIS_TIMEOUT=60
AI_RETRY_COUNT=3
EOF
    echo "配置文件已创建: .env"
    echo "请编辑.env文件配置API密钥"
fi

# 创建数据目录
echo "创建数据目录..."
mkdir -p data/{rosbags,pointclouds,images,cache}
mkdir -p logs

# 创建rosbag目录
mkdir -p ~/rosbags

# 配置防火墙
echo "配置防火墙..."
if command -v ufw &> /dev/null; then
    sudo ufw allow 8000/tcp
    echo "已开放8000端口"
fi

# 创建系统服务
echo "创建系统服务..."
sudo tee /etc/systemd/system/eagle-eye.service > /dev/null << EOF
[Unit]
Description=Urban Construction Eagle Eye System
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PROJECT_DIR
ExecStart=$PROJECT_DIR/start_ubuntu.sh
Restart=always
RestartSec=10
Environment=ROS_DOMAIN_ID=0
Environment=PATH=$PROJECT_DIR/venv/bin:/usr/bin:/bin
Environment=VIRTUAL_ENV=$PROJECT_DIR/venv

[Install]
WantedBy=multi-user.target
EOF

# 启用服务
sudo systemctl daemon-reload
sudo systemctl enable eagle-eye.service

# 设置启动脚本权限
chmod +x start_ubuntu.sh

# 获取本机IP地址
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "========================================="
echo "部署完成！"
echo "========================================="
echo "项目目录: $PROJECT_DIR"
echo "配置文件: $PROJECT_DIR/.env"
echo "数据目录: $PROJECT_DIR/data"
echo "rosbag目录: $HOME/rosbags"
echo ""
echo "下一步："
echo "1. 编辑配置文件: nano .env"
echo "2. 配置API密钥"
echo "3. 启动系统: ./start_ubuntu.sh"
echo "4. 或启动服务: sudo systemctl start eagle-eye.service"
echo ""
echo "访问地址: http://$LOCAL_IP:8000"
echo "健康检查: http://$LOCAL_IP:8000/api/health"
echo "API文档: http://$LOCAL_IP:8000/api/docs"
echo "========================================="
