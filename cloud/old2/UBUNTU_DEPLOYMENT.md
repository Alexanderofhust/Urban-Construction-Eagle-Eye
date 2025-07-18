# Ubuntu下位机部署方案

## 系统要求

- Ubuntu 20.04 或更高版本
- ROS2 Humble 或更高版本
- Python 3.8+
- 至少 4GB RAM
- 至少 20GB 可用磁盘空间

## 1. 环境准备

### 1.1 安装ROS2
```bash
# 设置locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 安装ROS2 Humble
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-pip

# 初始化rosdep
sudo rosdep init
rosdep update

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.2 安装Python依赖
```bash
# 安装系统依赖
sudo apt install python3-pip python3-venv build-essential

# 创建虚拟环境
python3 -m venv ~/eagle_eye_env
source ~/eagle_eye_env/bin/activate

# 安装Python包
pip install -r requirements.txt
```

### 1.3 安装ROS2 Python包
```bash
# 激活ROS2环境
source /opt/ros/humble/setup.bash

# 安装ROS2 Python包
pip install rosbag2-py rclpy
sudo apt install python3-sensor-msgs python3-std-msgs python3-cv-bridge
```

## 2. 项目部署

### 2.1 创建项目目录
```bash
# 创建项目目录
mkdir -p ~/eagle_eye_system
cd ~/eagle_eye_system

# 复制项目文件
# 将cloud目录下的所有文件复制到这里
```

### 2.2 配置环境变量
```bash
# 编辑配置文件
nano .env

# 添加以下内容：
# 高德地图API密钥
AMAP_API_KEY=your_amap_api_key
AMAP_SECURITY_CODE=your_amap_security_code

# Google Gemini API密钥
GEMINI_API_KEY=your_gemini_api_key

# 服务器配置
SERVER_HOST=0.0.0.0
SERVER_PORT=8000

# ROS2配置
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0

# 数据路径
DATA_DIR=./data
ROSBAG_PATH=/home/user/rosbags
```

### 2.3 配置防火墙
```bash
# 开放端口
sudo ufw allow 8000/tcp
sudo ufw enable

# 检查防火墙状态
sudo ufw status
```

## 3. 启动服务

### 3.1 创建启动脚本
```bash
# 创建启动脚本
nano start_server.sh

# 添加以下内容：
#!/bin/bash
set -e

echo "========================================="
echo "Urban Construction Eagle Eye (Ubuntu版)"
echo "建筑缺陷检测可视化系统"
echo "========================================="

# 激活ROS2环境
source /opt/ros/humble/setup.bash

# 激活Python虚拟环境
source ~/eagle_eye_env/bin/activate

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo "错误: ROS2未正确安装"
    exit 1
fi

# 启动系统
echo "正在启动系统..."
python3 main_stable.py

# 赋予执行权限
chmod +x start_server.sh
```

### 3.2 创建系统服务
```bash
# 创建系统服务文件
sudo nano /etc/systemd/system/eagle-eye.service

# 添加以下内容：
[Unit]
Description=Urban Construction Eagle Eye System
After=network.target

[Service]
Type=simple
User=user
WorkingDirectory=/home/user/eagle_eye_system
ExecStart=/home/user/eagle_eye_system/start_server.sh
Restart=always
RestartSec=10
Environment=ROS_DOMAIN_ID=0

[Install]
WantedBy=multi-user.target

# 启用服务
sudo systemctl daemon-reload
sudo systemctl enable eagle-eye.service
sudo systemctl start eagle-eye.service
```

## 4. 网络配置

### 4.1 端口转发（从Windows上位机访问）
```bash
# 在Windows上位机上，通过SSH端口转发访问
ssh -L 8000:localhost:8000 user@<下位机IP>

# 然后在Windows浏览器中访问：
# http://localhost:8000
```

### 4.2 直接网络访问
```bash
# 在下位机上获取IP地址
ip addr show

# 在Windows上位机浏览器中访问：
# http://<下位机IP>:8000
```

## 5. 日志和监控

### 5.1 查看系统日志
```bash
# 查看服务状态
sudo systemctl status eagle-eye.service

# 查看实时日志
sudo journalctl -u eagle-eye.service -f

# 查看应用日志
tail -f ~/eagle_eye_system/logs/eagle_eye_*.log
```

### 5.2 系统监控
```bash
# 监控系统资源
htop

# 监控网络连接
sudo netstat -tulnp | grep 8000

# 监控磁盘使用
df -h
```

## 6. 故障排除

### 6.1 ROS2环境问题
```bash
# 检查ROS2环境
printenv | grep ROS

# 重新source环境
source /opt/ros/humble/setup.bash
```

### 6.2 权限问题
```bash
# 检查文件权限
ls -la ~/eagle_eye_system/

# 修复权限
chmod -R 755 ~/eagle_eye_system/
```

### 6.3 网络问题
```bash
# 检查端口占用
sudo lsof -i :8000

# 测试网络连接
curl http://localhost:8000/api/health
```

## 7. 性能优化

### 7.1 系统优化
```bash
# 调整系统参数
echo 'net.core.rmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
echo 'net.core.wmem_max = 134217728' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 7.2 ROS2优化
```bash
# 设置ROS2参数
export ROS_DOMAIN_ID=0
export RCUTILS_LOGGING_SEVERITY=WARN
export RCUTILS_COLORIZED_OUTPUT=1
```

## 8. 备份和恢复

### 8.1 数据备份
```bash
# 备份数据
tar -czf eagle_eye_backup_$(date +%Y%m%d).tar.gz ~/eagle_eye_system/data/

# 备份配置
cp ~/eagle_eye_system/.env ~/eagle_eye_system/.env.backup
```

### 8.2 系统恢复
```bash
# 恢复数据
tar -xzf eagle_eye_backup_*.tar.gz -C ~/

# 恢复配置
cp ~/eagle_eye_system/.env.backup ~/eagle_eye_system/.env
```

## 9. 开发调试

### 9.1 开发模式启动
```bash
# 开发模式
DEBUG=true python3 main_stable.py
```

### 9.2 远程调试
```bash
# 在Windows上位机通过SSH连接下位机进行调试
ssh -X user@<下位机IP>
```

这个方案将系统完全部署在Ubuntu下位机上，通过网络端口让Windows上位机访问Web界面，是最优的架构选择。
