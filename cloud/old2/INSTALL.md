# Urban Construction Eagle Eye - 安装和部署指南

## 系统要求

### 硬件要求
- RAM: 最少 8GB，推荐 16GB
- 存储: 最少 50GB 可用空间
- 网络: 稳定的网络连接

### 软件要求
- Python 3.8+
- Windows 10/11 或 Ubuntu 20.04+
- Chrome/Firefox 浏览器（用于访问Web界面）

## 安装步骤

### 1. 准备Python环境

```bash
# 创建虚拟环境
python -m venv eagle_eye_env

# 激活虚拟环境
# Windows:
eagle_eye_env\Scripts\activate
# Linux/Mac:
source eagle_eye_env/bin/activate
```

### 2. 安装依赖包

```bash
# 安装基础依赖
pip install -r requirements.txt

# 如果需要ROS2支持（仅在Ubuntu上）
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
pip install rosbag2_py rclpy
```

### 3. 配置环境变量

复制 `.env.example` 为 `.env` 并配置以下变量：

```env
# 高德地图API密钥
AMAP_API_KEY=your_amap_api_key
AMAP_SECURITY_CODE=your_amap_security_code

# Google Gemini API密钥
GEMINI_API_KEY=your_gemini_api_key

# 服务器配置
SERVER_HOST=0.0.0.0
SERVER_PORT=8000
DEBUG=true

# 数据存储配置
DATA_DIR=./data
MAX_ROSBAG_SIZE=10GB
MAX_SESSION_COUNT=100

# 处理配置
POINTCLOUD_DOWNSAMPLE_RATIO=0.1
MAX_POINTCLOUD_SIZE=1000000
REVIEW_IMAGE_COUNT=10
REVIEW_IMAGE_SIZE=800

# AI分析配置
AI_ANALYSIS_TIMEOUT=60
AI_RETRY_COUNT=3
```

### 4. 启动系统

#### Windows系统
```bash
# 使用批处理文件启动
start.bat

# 或者手动启动
python main_stable.py
```

#### Linux/Mac系统
```bash
# 手动启动
python main_stable.py

# 或者使用nohup后台运行
nohup python main_stable.py > eagle_eye.log 2>&1 &
```

### 5. 访问Web界面

打开浏览器访问：`http://localhost:8000`

## 配置说明

### API密钥申请

#### 高德地图API密钥
1. 访问 [高德开放平台](https://lbs.amap.com/)
2. 注册账号并实名认证
3. 创建Web服务应用并获取API密钥
4. 申请安全密钥（2021年12月02日后申请的密钥需要）
5. 配置域名白名单和访问配额

#### Google Gemini API密钥
1. 访问 [Google AI Studio](https://makersuite.google.com/app/apikey)
2. 创建新的API密钥
3. 设置使用配额和限制

### 网络配置

#### 端口配置
- 默认端口：8000
- 如需更改，修改 `.env` 文件中的 `SERVER_PORT`

#### 防火墙配置
```bash
# Windows防火墙
netsh advfirewall firewall add rule name="Eagle Eye" dir=in action=allow protocol=TCP localport=8000

# Ubuntu防火墙
sudo ufw allow 8000/tcp
```

### 数据存储配置

#### 目录结构
```
data/
├── session_20240101_123456/
│   ├── rosbags/
│   ├── pointclouds/
│   ├── images/
│   └── cache/
├── session_20240101_234567/
└── eagle_eye.db
```

#### 数据库配置
- 默认使用SQLite数据库
- 数据库文件：`data/eagle_eye.db`
- 支持自动备份和清理

## 使用指南

### 1. 连接Ubuntu系统
1. 在连接界面输入Ubuntu系统的IP地址
2. 输入SSH用户名和密码
3. 点击"连接"按钮

### 2. 处理rosbag数据
1. 连接成功后，系统会自动从 `/home/user/rosbags` 下载数据
2. 等待处理完成，查看点云数据

### 3. 查看审核图像
1. 切换到"审核"选项卡
2. 查看按缺陷严重程度排序的图像
3. 对比原始图像和叠加图像

### 4. AI分析
1. 切换到"分析"选项卡
2. 选择需要分析的图像
3. 启动AI分析并查看结果

## 故障排除

### 常见问题

#### 1. 连接失败
- 检查网络连接
- 验证SSH凭据
- 确认Ubuntu系统可访问

#### 2. 依赖包错误
```bash
# 更新pip
pip install --upgrade pip

# 重新安装依赖
pip install -r requirements.txt --force-reinstall
```

#### 3. 内存不足
- 调整点云采样率：`POINTCLOUD_DOWNSAMPLE_RATIO=0.05`
- 减少最大点云大小：`MAX_POINTCLOUD_SIZE=500000`

#### 4. API调用失败
- 检查API密钥是否正确
- 验证网络连接
- 查看API配额限制

### 日志查看

#### 日志位置
```
logs/
└── eagle_eye_20240101.log
```

#### 日志级别
- INFO: 正常操作信息
- WARNING: 警告信息
- ERROR: 错误信息
- DEBUG: 调试信息

#### 修改日志级别
```python
# 在main_stable.py中修改
from error_handler import setup_logging
setup_logging("DEBUG")
```

### 性能优化

#### 1. 处理速度优化
- 使用SSD存储
- 增加内存配置
- 调整处理并发数

#### 2. 网络优化
- 使用有线网络连接
- 配置网络缓存
- 优化SSH连接参数

#### 3. 存储优化
- 定期清理旧数据
- 压缩存储文件
- 使用外部存储

## 开发指南

### 代码结构
```
cloud/
├── main_stable.py          # 主程序入口
├── config.py               # 配置管理
├── data_manager.py         # 数据管理
├── error_handler.py        # 错误处理
├── ssh_client.py           # SSH客户端
├── rosbag_processor.py     # rosbag处理
├── pointcloud_analyzer.py  # 点云分析
├── gemini_client.py        # AI客户端
├── baidu_map_client.py     # 地图客户端
├── templates/              # 网页模板
├── static/                 # 静态文件
├── data/                   # 数据存储
└── logs/                   # 日志文件
```

### 扩展功能

#### 添加新的AI模型
1. 创建新的客户端类
2. 实现分析接口
3. 注册到主程序

#### 添加新的数据源
1. 扩展SSH客户端
2. 添加新的处理器
3. 更新数据管理器

#### 自定义Web界面
1. 修改HTML模板
2. 添加CSS样式
3. 扩展JavaScript功能

## 联系支持

如果遇到问题，请查看：
1. 日志文件中的错误信息
2. 系统健康检查：`http://localhost:8000/api/health`
3. API文档：`http://localhost:8000/api/docs`

技术支持：
- 邮箱：support@eagle-eye.com
- 文档：https://docs.eagle-eye.com
- GitHub：https://github.com/eagle-eye/urban-construction
