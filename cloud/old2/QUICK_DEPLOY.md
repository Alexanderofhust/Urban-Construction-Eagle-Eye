# Ubuntu部署快速指南

本指南帮助您在Ubuntu系统上快速部署Urban Construction Eagle Eye系统。

## 系统要求

- **操作系统**: Ubuntu 20.04 LTS 或更高版本
- **内存**: 至少4GB RAM
- **存储**: 至少20GB可用空间
- **网络**: 稳定的网络连接

## 快速部署

### 1. 下载部署脚本

```bash
# 克隆项目或下载项目文件
git clone [项目地址]
cd Urban-Construction-Eagle-Eye

# 或者手动下载以下文件到Ubuntu系统:
# - cloud/deploy_ubuntu.sh
# - cloud/start_ubuntu.sh
# - cloud/main_ubuntu.py
# - cloud/ubuntu_rosbag_processor.py
# - cloud/requirements_ubuntu.txt
# - cloud/amap_client.py
# - cloud/templates/
# - cloud/static/
```

### 2. 运行一键部署脚本

```bash
cd cloud
chmod +x deploy_ubuntu.sh
./deploy_ubuntu.sh
```

部署脚本会自动：
- 检查系统版本
- 安装ROS2 Humble
- 创建Python虚拟环境
- 安装依赖包
- 创建配置文件
- 设置系统服务

### 3. 配置API密钥

编辑配置文件：
```bash
nano .env
```

填入您的API密钥：
```env
AMAP_API_KEY=your_amap_api_key
AMAP_SECURITY_CODE=your_amap_security_code
GEMINI_API_KEY=your_gemini_api_key
```

### 4. 启动系统

```bash
# 手动启动
./start_ubuntu.sh

# 或使用系统服务
sudo systemctl start eagle-eye.service
sudo systemctl status eagle-eye.service
```

### 5. 访问系统

- **主界面**: http://[Ubuntu_IP]:8000
- **API文档**: http://[Ubuntu_IP]:8000/api/docs
- **健康检查**: http://[Ubuntu_IP]:8000/api/health

## 服务管理

### 查看服务状态
```bash
sudo systemctl status eagle-eye.service
```

### 启动/停止服务
```bash
sudo systemctl start eagle-eye.service
sudo systemctl stop eagle-eye.service
sudo systemctl restart eagle-eye.service
```

### 查看日志
```bash
sudo journalctl -u eagle-eye.service -f
```

## 网络配置

### 防火墙设置
```bash
sudo ufw allow 8000/tcp
sudo ufw enable
```

### 从Windows访问
在Windows上位机上，通过浏览器访问：
```
http://[Ubuntu下位机IP]:8000
```

## 故障排除

### 检查ROS2环境
```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### 检查Python环境
```bash
cd ~/eagle_eye_system
source venv/bin/activate
python --version
pip list
```

### 检查服务日志
```bash
sudo journalctl -u eagle-eye.service --since "1 hour ago"
```

### 重新部署
```bash
sudo systemctl stop eagle-eye.service
rm -rf ~/eagle_eye_system
./deploy_ubuntu.sh
```

## 性能优化

### 1. 系统优化
```bash
# 增加文件句柄限制
echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf

# 优化网络参数
echo "net.core.rmem_max = 134217728" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max = 134217728" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 2. ROS2优化
```bash
# 设置ROS2环境变量
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/ros/humble/share/fastrtps/xml/DEFAULT_FASTRTPS_PROFILES.xml
```

### 3. 存储优化
```bash
# 创建专用存储目录
sudo mkdir -p /opt/eagle_eye_data
sudo chown $USER:$USER /opt/eagle_eye_data
ln -s /opt/eagle_eye_data ~/eagle_eye_system/data
```

## 开发环境设置

### 1. 安装开发工具
```bash
sudo apt install -y git vim code
```

### 2. 配置开发环境
```bash
cd ~/eagle_eye_system
source venv/bin/activate
pip install -r requirements_ubuntu.txt
```

### 3. 运行开发服务器
```bash
python main_ubuntu.py --debug
```

## 维护

### 定期更新
```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 更新Python依赖
cd ~/eagle_eye_system
source venv/bin/activate
pip install --upgrade -r requirements_ubuntu.txt
```

### 备份
```bash
# 备份配置
cp .env .env.backup

# 备份数据
tar -czf data_backup_$(date +%Y%m%d).tar.gz data/
```

## 技术支持

如果遇到问题：
1. 检查系统日志
2. 查看服务状态
3. 验证网络连接
4. 检查API密钥配置

部署完成后，您就可以在Windows上位机通过浏览器访问Ubuntu下位机上的系统了！
