# Urban Construction Eagle Eye - 启动脚本

@echo off
echo ========================================
echo Urban Construction Eagle Eye (高德地图版本)
echo 建筑缺陷检测可视化系统
echo 版本: v1.1.0
echo 地图服务: 高德地图API
echo ========================================

echo 正在检查Python环境...
python --version
if %errorlevel% neq 0 (
    echo 错误: Python未安装或不在PATH中
    pause
    exit /b 1
)

echo 正在检查依赖库...
pip install -r requirements.txt
if %errorlevel% neq 0 (
    echo 错误: 依赖库安装失败
    pause
    exit /b 1
)

echo 正在创建必要目录...
if not exist "data" mkdir data
if not exist "data\rosbags" mkdir data\rosbags
if not exist "data\results" mkdir data\results
if not exist "data\images" mkdir data\images
if not exist "data\pointclouds" mkdir data\pointclouds

echo 正在启动服务器...
echo 请在浏览器中访问: http://localhost:8000
echo 按Ctrl+C停止服务器

python main.py

pause
