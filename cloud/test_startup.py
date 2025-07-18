"""
系统启动测试脚本
用于验证所有依赖是否正确安装并可以正常启动
"""

import sys
import os
import importlib

def test_imports():
    """测试所有必需的模块是否可以导入"""
    print("正在测试模块导入...")
    
    required_modules = [
        "fastapi",
        "uvicorn", 
        "open3d",
        "paramiko",
        "PIL",
        "numpy",
        "cv2",
        "google.generativeai",
        "jinja2",
        "aiofiles",
        "dotenv"
    ]
    
    failed_modules = []
    
    for module in required_modules:
        try:
            importlib.import_module(module)
            print(f"✓ {module}")
        except ImportError as e:
            print(f"✗ {module}: {e}")
            failed_modules.append(module)
    
    return failed_modules

def test_config():
    """测试配置文件"""
    print("\n正在测试配置文件...")
    
    try:
        from config import Config
        Config.load_from_env()
        print("✓ 配置文件加载成功")
        
        # 检查配置错误
        errors = Config.validate_config()
        if errors:
            print("⚠ 配置警告:")
            for error in errors:
                print(f"  - {error}")
        else:
            print("✓ 配置验证通过")
            
        return True
    except Exception as e:
        print(f"✗ 配置文件错误: {e}")
        return False

def test_directories():
    """测试目录结构"""
    print("\n正在测试目录结构...")
    
    required_dirs = [
        "data",
        "data/sessions", 
        "data/results",
        "logs",
        "static",
        "templates",
        "services",
        "utils"
    ]
    
    missing_dirs = []
    
    for dir_path in required_dirs:
        if os.path.exists(dir_path):
            print(f"✓ {dir_path}")
        else:
            print(f"✗ {dir_path} (缺失)")
            missing_dirs.append(dir_path)
    
    return missing_dirs

def test_files():
    """测试关键文件"""
    print("\n正在测试关键文件...")
    
    required_files = [
        "main.py",
        "config.py",
        ".env",
        "requirements.txt",
        "services/ssh_client.py",
        "services/pcd_processor.py",
        "services/gemini_client.py",
        "services/data_manager.py",
        "utils/image_utils.py",
        "templates/index.html",
        "static/css/style.css",
        "static/js/main.js"
    ]
    
    missing_files = []
    
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"✓ {file_path}")
        else:
            print(f"✗ {file_path} (缺失)")
            missing_files.append(file_path)
    
    return missing_files

def main():
    """主测试函数"""
    print("========================================")
    print("       城市建设鹰眼系统启动测试")
    print("========================================")
    
    # 测试模块导入
    failed_modules = test_imports()
    
    # 测试配置
    config_ok = test_config()
    
    # 测试目录
    missing_dirs = test_directories()
    
    # 测试文件
    missing_files = test_files()
    
    print("\n========================================")
    print("           测试结果汇总")
    print("========================================")
    
    if failed_modules:
        print(f"✗ 缺失模块: {', '.join(failed_modules)}")
        print("  请运行: pip install -r requirements.txt")
    else:
        print("✓ 所有模块导入成功")
    
    if not config_ok:
        print("✗ 配置文件有问题")
    else:
        print("✓ 配置文件正常")
    
    if missing_dirs:
        print(f"✗ 缺失目录: {', '.join(missing_dirs)}")
        print("  请运行: python setup_directories.py")
    else:
        print("✓ 目录结构完整")
    
    if missing_files:
        print(f"✗ 缺失文件: {', '.join(missing_files)}")
    else:
        print("✓ 关键文件完整")
    
    # 总结
    all_ok = not failed_modules and config_ok and not missing_dirs and not missing_files
    
    if all_ok:
        print("\n🎉 系统测试通过！可以启动系统了")
        print("运行命令: python main.py")
    else:
        print("\n❌ 系统测试失败，请修复上述问题后重试")
    
    return all_ok

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
