@echo off
title 城市建设鹰眼系统

echo ========================================
echo           城市建设鹰眼系统
echo ========================================
echo.

REM 确保在正确的目录中
cd /d "%~dp0"
echo 工作目录: %cd%

REM 检查关键文件是否存在
if not exist "main.py" (
    echo 错误: 找不到main.py文件
    echo 请确保在正确的目录中运行此脚本
    pause
    exit /b 1
)

REM 检查Python环境
echo 检查Python环境...
python --version
if %errorlevel% neq 0 (
    echo 错误: 未找到Python环境
    echo 请先安装Python 3.8或更高版本
    pause
    exit /b 1
)

REM 检查配置文件
if not exist ".env" (
    echo 提示: 未找到配置文件 .env
    echo 请复制 config.env.example 为 .env 并配置您的API密钥
    if exist "config.env.example" (
        echo 正在创建默认配置文件...
        copy "config.env.example" ".env"
        echo 请编辑 .env 文件设置您的API密钥
        pause
    )
)

REM 检查虚拟环境
if not exist "venv" (
    echo 创建虚拟环境...
    python -m venv venv
    if %errorlevel% neq 0 (
        echo 错误: 无法创建虚拟环境
        pause
        exit /b 1
    )
)

REM 激活虚拟环境
echo 激活虚拟环境...
call venv\Scripts\activate.bat
if %errorlevel% neq 0 (
    echo 错误: 无法激活虚拟环境
    pause
    exit /b 1
)

REM 安装依赖
echo 检查并安装依赖...
pip install -r requirements.txt
if %errorlevel% neq 0 (
    echo 警告: 依赖安装可能存在问题
)

REM 安装python-dotenv用于环境变量管理
pip install python-dotenv
echo.

REM 创建必要目录
if not exist "data" mkdir data
if not exist "data\sessions" mkdir data\sessions
if not exist "data\results" mkdir data\results
if not exist "logs" mkdir logs

REM 启动应用
echo.
echo 启动城市建设鹰眼系统...
echo 服务地址: http://localhost:8000
echo 按 Ctrl+C 停止服务
echo.
echo 当前工作目录: %cd%
echo.

REM 确保在正确的目录中运行Python
cd /d "%~dp0"
python "%~dp0main.py"

pause
