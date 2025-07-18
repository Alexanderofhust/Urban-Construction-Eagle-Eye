#!/bin/bash
# Ubuntu下位机启动脚本

set -e

echo "========================================="
echo "Urban Construction Eagle Eye (Ubuntu版)"
echo "建筑缺陷检测可视化系统"
echo "版本: v1.2.0 (Ubuntu部署版)"
echo "========================================="

# 检查是否在正确的目录
if [ ! -f "main_stable.py" ]; then
    echo "错误: 请在项目根目录下运行此脚本"
    exit 1
fi

# 激活ROS2环境
echo "正在激活ROS2环境..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS2 Humble环境已激活"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "ROS2 Foxy环境已激活"
else
    echo "警告: 未找到ROS2环境，某些功能可能不可用"
fi

# 激活Python虚拟环境
if [ -d "venv" ]; then
    echo "正在激活Python虚拟环境..."
    source venv/bin/activate
    echo "Python虚拟环境已激活"
elif [ -d "~/eagle_eye_env" ]; then
    echo "正在激活Python虚拟环境..."
    source ~/eagle_eye_env/bin/activate
    echo "Python虚拟环境已激活"
else
    echo "警告: 未找到Python虚拟环境"
fi

# 检查Python环境
echo "正在检查Python环境..."
python3 --version
if [ $? -ne 0 ]; then
    echo "错误: Python3未安装"
    exit 1
fi

# 检查ROS2环境
echo "正在检查ROS2环境..."
if command -v ros2 &> /dev/null; then
    echo "ROS2版本: $(ros2 --version)"
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=0
else
    echo "警告: ROS2未正确安装，rosbag处理功能将不可用"
fi

# 检查依赖库
echo "正在检查依赖库..."
pip3 install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "错误: 依赖库安装失败"
    exit 1
fi

# 检查配置文件
echo "正在检查配置文件..."
if [ ! -f ".env" ]; then
    echo "警告: 配置文件(.env)不存在，将使用默认配置"
    cp .env.example .env 2>/dev/null || echo "请手动创建.env配置文件"
fi

# 创建必要的目录
echo "正在创建必要目录..."
mkdir -p data/rosbags
mkdir -p data/pointclouds
mkdir -p data/images
mkdir -p data/cache
mkdir -p logs

# 检查网络端口
echo "正在检查网络端口..."
if netstat -tuln | grep -q ":8000 "; then
    echo "警告: 端口8000已被占用"
    echo "请修改配置文件中的SERVER_PORT设置"
fi

# 检查防火墙
echo "正在检查防火墙设置..."
if command -v ufw &> /dev/null; then
    if ufw status | grep -q "Status: active"; then
        if ! ufw status | grep -q "8000/tcp"; then
            echo "警告: 防火墙已启用但未开放8000端口"
            echo "请运行: sudo ufw allow 8000/tcp"
        fi
    fi
fi

# 显示系统信息
echo "========================================="
echo "系统信息:"
echo "操作系统: $(lsb_release -d | cut -f2)"
echo "Python版本: $(python3 --version)"
if command -v ros2 &> /dev/null; then
    echo "ROS2版本: $(ros2 --version)"
fi
echo "工作目录: $(pwd)"
echo "数据目录: $(pwd)/data"
echo "========================================="

# 显示网络信息
echo "网络信息:"
echo "本机IP地址:"
hostname -I | tr ' ' '\n' | grep -v '^$' | head -5
echo "访问地址: http://$(hostname -I | awk '{print $1}'):8000"
echo "========================================="

# 启动系统
echo "正在启动系统..."
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# 启动主程序
if [ "$1" = "dev" ]; then
    echo "开发模式启动..."
    DEBUG=true python3 main_stable.py
else
    echo "生产模式启动..."
    python3 main_stable.py
fi
