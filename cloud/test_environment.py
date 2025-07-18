#!/usr/bin/env python3
"""
环境测试脚本
检查系统环境和依赖是否正确安装
"""

import sys
import os
import subprocess
import importlib
from pathlib import Path

def check_python_version():
    """检查Python版本"""
    print("🔍 检查Python版本...")
    version = sys.version_info
    if version.major == 3 and version.minor >= 8:
        print(f"✅ Python版本: {version.major}.{version.minor}.{version.micro}")
        return True
    else:
        print(f"❌ Python版本过低: {version.major}.{version.minor}.{version.micro}")
        print("   需要Python 3.8或更高版本")
        return False

def check_dependencies():
    """检查依赖包"""
    print("\n🔍 检查依赖包...")
    
    required_packages = [
        'fastapi',
        'uvicorn',
        'open3d',
        'paramiko',
        'PIL',
        'numpy',
        'cv2',
        'google.generativeai',
        'jinja2',
        'aiofiles'
    ]
    
    success = True
    for package in required_packages:
        try:
            if package == 'PIL':
                importlib.import_module('PIL')
            elif package == 'cv2':
                importlib.import_module('cv2')
            else:
                importlib.import_module(package)
            print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package} - 未安装")
            success = False
    
    return success

def check_directories():
    """检查必要目录"""
    print("\n🔍 检查目录结构...")
    
    required_dirs = [
        'templates',
        'static',
        'static/css',
        'static/js',
        'services',
        'utils',
        'data',
        'data/sessions',
        'data/results'
    ]
    
    success = True
    for directory in required_dirs:
        dir_path = Path(directory)
        if dir_path.exists():
            print(f"✅ {directory}")
        else:
            print(f"❌ {directory} - 不存在")
            success = False
    
    return success

def check_files():
    """检查关键文件"""
    print("\n🔍 检查关键文件...")
    
    required_files = [
        'main.py',
        'requirements.txt',
        'templates/index.html',
        'static/css/style.css',
        'static/js/main.js',
        'services/ssh_client.py',
        'services/pcd_processor.py',
        'services/gemini_client.py',
        'services/data_manager.py',
        'utils/image_utils.py'
    ]
    
    success = True
    for file_path in required_files:
        if Path(file_path).exists():
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} - 不存在")
            success = False
    
    return success

def check_environment_variables():
    """检查环境变量"""
    print("\n🔍 检查环境变量...")
    
    gemini_key = os.getenv('GEMINI_API_KEY')
    if gemini_key:
        print(f"✅ GEMINI_API_KEY: {gemini_key[:10]}...")
    else:
        print("⚠️ GEMINI_API_KEY - 未设置 (AI分析功能将不可用)")
    
    return True

def check_port():
    """检查端口是否可用"""
    print("\n🔍 检查端口8000...")
    
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('localhost', 8000))
        sock.close()
        
        if result == 0:
            print("⚠️ 端口8000已被占用")
        else:
            print("✅ 端口8000可用")
        
        return True
    except Exception as e:
        print(f"❌ 端口检查失败: {e}")
        return False

def main():
    """主函数"""
    print("=" * 50)
    print("    城市建设鹰眼系统 - 环境检查")
    print("=" * 50)
    
    checks = [
        check_python_version,
        check_dependencies,
        check_directories,
        check_files,
        check_environment_variables,
        check_port
    ]
    
    results = []
    for check in checks:
        try:
            result = check()
            results.append(result)
        except Exception as e:
            print(f"❌ 检查失败: {e}")
            results.append(False)
    
    print("\n" + "=" * 50)
    print("           检查结果汇总")
    print("=" * 50)
    
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print("🎉 所有检查通过！系统就绪")
        print("\n📋 下一步操作:")
        print("1. 设置Gemini API密钥 (如果还没有)")
        print("2. 修改HTML模板中的高德地图API密钥")
        print("3. 运行 start.bat 启动系统")
        print("4. 打开浏览器访问 http://localhost:8000")
        return True
    else:
        print(f"⚠️ {total - passed} 个检查未通过")
        print("\n📋 请解决以下问题:")
        print("1. 安装缺失的依赖包: pip install -r requirements.txt")
        print("2. 确保所有文件都存在")
        print("3. 设置必要的环境变量")
        return False

if __name__ == "__main__":
    success = main()
    
    print("\n按任意键退出...")
    try:
        input()
    except KeyboardInterrupt:
        pass
    
    sys.exit(0 if success else 1)
