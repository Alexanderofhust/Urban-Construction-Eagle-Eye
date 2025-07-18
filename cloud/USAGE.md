# 城市建设鹰眼系统 - 使用说明

## 🎯 系统概述
城市建设鹰眼系统是一个基于点云数据的建筑缺陷检测可视化平台，集成了3D点云处理、AI分析、人工复检等功能。

## 🚀 快速启动

### 方法1：使用start.bat（推荐）
```bash
# 在cloud目录中双击或运行
start.bat
```

### 方法2：使用Python启动脚本
```bash
# 在cloud目录中运行
python run.py
```

### 方法3：直接运行main.py
```bash
# 确保在cloud目录中
cd "d:\Desktop\Github_Repo\Urban-Construction-Eagle-Eye\cloud"
python main.py
```

## 📋 配置说明

### 必需配置项
在`.env`文件中设置以下配置：

```properties
# Gemini AI API密钥（用于AI分析）
GEMINI_API_KEY=your_gemini_api_key_here

# 高德地图API密钥（用于地图显示）
AMAP_API_KEY=your_amap_api_key_here
AMAP_SECURITY_KEY=your_amap_security_key_here

# SSH连接信息（用于连接边缘设备）
SSH_HOSTNAME=192.168.2.50
SSH_PORT=22
SSH_USERNAME=elf
SSH_PASSWORD=elf
```

### 可选配置项
```properties
# 服务器配置
HOST=127.0.0.1
PORT=8000
DEBUG=true
```

## 🔧 功能模块

### 1. 总览页面
- 系统状态监控
- 会话统计
- 快速操作入口

### 2. 点云可视化
- 3D点云渲染
- 点云数据统计
- 地图位置显示

### 3. 缺陷复检
- 人工缺陷标注
- 缺陷分类管理
- 复检结果保存

### 4. AI分析
- 智能缺陷检测
- 修复建议生成
- 分析报告导出

### 5. 统计报告
- 数据统计图表
- 报告生成
- 数据导出

## 🔍 使用流程

1. **启动系统**
   - 运行start.bat或使用其他启动方式
   - 访问 http://localhost:8000

2. **连接下位机**
   - 在总览页面点击"连接下位机"
   - 填写连接信息（如果.env已配置则自动填充）
   - 选择要下载的数据路径

3. **数据处理**
   - 系统自动下载并处理点云数据
   - 查看3D点云可视化结果
   - 检查地图定位信息

4. **缺陷检测**
   - 进入AI分析页面
   - 启动自动分析
   - 查看检测结果

5. **人工复检**
   - 进入缺陷复检页面
   - 对AI检测结果进行人工确认
   - 添加或修改缺陷标注

6. **报告生成**
   - 查看统计报告
   - 导出分析结果
   - 生成完整报告

## 📊 API接口

### 主要接口
- `GET /` - 主页面
- `POST /api/connect` - 连接下位机
- `GET /api/status` - 获取系统状态
- `GET /api/sessions` - 获取会话列表
- `POST /api/analyze` - 启动AI分析
- `POST /api/review` - 提交复检结果
- `GET /api/statistics/{session_id}` - 获取统计数据

## 🛠️ 故障排除

### 常见问题

1. **无法启动系统**
   - 检查Python环境是否正确安装
   - 确保在cloud目录中运行启动脚本
   - 检查端口8000是否被占用

2. **SSH连接失败**
   - 验证网络连通性（ping测试）
   - 检查SSH服务是否运行
   - 确认用户名密码正确
   - 运行 `python test_ssh.py` 进行诊断

3. **地图无法显示**
   - 检查高德地图API密钥是否正确
   - 确认安全密钥设置
   - 查看浏览器控制台错误信息

4. **AI分析失败**
   - 检查Gemini API密钥是否有效
   - 确认网络连接正常
   - 查看系统日志文件

### 诊断工具
- `python test_environment.py` - 环境检查
- `python test_ssh.py` - SSH连接测试
- `python test_startup.py` - 启动测试

## 📁 目录结构
```
cloud/
├── main.py              # 主程序
├── config.py            # 配置管理
├── .env                 # 环境配置
├── requirements.txt     # 依赖包
├── start.bat           # 启动脚本
├── run.py              # Python启动脚本
├── services/           # 服务模块
├── utils/              # 工具模块
├── static/             # 静态资源
├── templates/          # 模板文件
├── data/               # 数据目录
└── logs/               # 日志目录
```

## 📞 技术支持
如需技术支持，请查看日志文件（logs/app.log）或联系开发团队。
